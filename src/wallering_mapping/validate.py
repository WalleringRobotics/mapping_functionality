"""Read-only integrity, continuity and timing audit of a captured session."""

import bisect
import json
from collections import Counter, defaultdict

import cv2
import numpy as np

from .dataset import jsonl, safe_path, sha256_file


def distribution(values):
    return {"min": float(np.min(values)), "median": float(np.median(values)),
            "p95": float(np.percentile(values, 95)), "max": float(np.max(values))} if values else {}


def nearest_offsets(reference, other):
    offsets = []
    for timestamp in reference:
        index = bisect.bisect_left(other, timestamp)
        candidates = other[max(0, index - 1):index + 1]
        if candidates:
            offsets.append(min(abs(timestamp - value) for value in candidates) / 1e6)
    return offsets


def validate(root, decode=True):
    errors, warnings = [], []
    streams, imu_times, imu_seq = defaultdict(list), defaultdict(list), {}
    counts, gaps, paths = Counter(), Counter(), set()
    report = {"valid": False, "errors": errors, "warnings": warnings,
              "streams": {}, "imu": {}, "nearest_frame_offset_ms": {}}
    try:
        manifest = json.loads((root / "manifest.json").read_text())
        if manifest.get("schema_version") != 1:
            raise ValueError("Unsupported dataset schema_version")
        if manifest["status"] != "complete":
            errors.append(f"Session is {manifest['status']}, not complete; preserve and inspect")
        if manifest["source"] == "synthetic":
            warnings.append("Synthetic IO fixture: not a photogrammetric accuracy test")
        if sha256_file(root / "calibration.json") != manifest["calibration_sha256"]:
            errors.append("Calibration checksum mismatch")
        if manifest["status"] == "complete":
            journals = ["frames", "imu", "events", "clock"]
            if "telemetry" in manifest:
                journals.append("telemetry")
            for name in journals:
                if sha256_file(root / f"{name}.jsonl") != manifest.get("journals_sha256", {}).get(name):
                    errors.append(f"Journal checksum mismatch: {name}")
        previous = {}
        for row in jsonl(root / "frames.jsonl"):
            stream, sequence, timestamp = row["stream"], row["sequence"], row["device_ns"]
            if stream not in manifest["config"]["streams"]:
                errors.append(f"Unexpected stream {stream}")
            if type(sequence) is not int or type(timestamp) is not int or min(sequence, timestamp) < 0:
                raise ValueError("Invalid frame sequence/timestamp")
            if stream in previous:
                seq0, time0 = previous[stream]
                if sequence <= seq0 or timestamp <= time0:
                    errors.append(f"Non-monotonic frame sequence/timestamp on {stream}")
                gaps[stream] += max(0, sequence - seq0 - 1)
            previous[stream] = sequence, timestamp
            if row["path"] in paths:
                errors.append(f"Duplicate image path: {row['path']}")
            paths.add(row["path"])
            file = safe_path(root, row["path"])
            if not file.is_file():
                errors.append(f"Missing image: {row['path']}")
            elif file.stat().st_size != row["bytes"] or sha256_file(file) != row["sha256"]:
                errors.append(f"Image checksum/size mismatch: {row['path']}")
            elif decode:
                image = cv2.imread(str(file), cv2.IMREAD_UNCHANGED)
                if image is None or image.shape[:2] != (row["height"], row["width"]):
                    errors.append(f"Image cannot be decoded at expected dimensions: {row['path']}")
            camera = row["camera"]
            k = np.asarray(camera["K"], dtype=float)
            if (k.shape != (3, 3) or not np.isfinite(k).all()
                    or k[0, 0] <= 0 or k[1, 1] <= 0):
                errors.append(f"Invalid pixel intrinsics on {stream}")
            streams[stream].append(timestamp)
            counts[stream] += 1
        for stream in manifest["config"]["streams"]:
            times = streams[stream]
            if not times:
                errors.append(f"No frames for {stream}")
            if manifest.get("counts", {}).get(stream) != counts[stream]:
                errors.append(f"Manifest count mismatch for {stream}")
            intervals = [float(t) for t in np.diff(times) / 1e6]
            if gaps[stream]:
                warnings.append(f"{stream}: {gaps[stream]} sequence gaps")
            if intervals and max(intervals) > 2500 / manifest["config"]["fps"]:
                warnings.append(f"{stream}: frame interval exceeds 2.5 requested periods")
            report["streams"][stream] = {
                "frames": counts[stream], "sequence_gaps": gaps[stream],
                "interval_ms": distribution(intervals),
                "observed_fps": ((len(times) - 1) * 1e9 / (times[-1] - times[0])
                                 if len(times) > 1 and times[-1] > times[0] else None),
            }
        for row in jsonl(root / "imu.jsonl"):
            sensor, seq, timestamp = row["sensor"], row["sequence"], row["device_ns"]
            if sensor not in {"accelerometer", "gyroscope"}:
                raise ValueError(f"Unknown IMU report {sensor}")
            if (type(seq) is not int or type(timestamp) is not int
                    or min(seq, timestamp) < 0 or len(row["xyz"]) != 3
                    or not np.isfinite(row["xyz"]).all()):
                raise ValueError(f"Invalid IMU record: {sensor}")
            if sensor in imu_seq:
                if seq <= imu_seq[sensor] or timestamp <= imu_times[sensor][-1]:
                    errors.append(f"Non-monotonic IMU sequence/timestamp: {sensor}")
                gaps[sensor] += max(0, seq - imu_seq[sensor] - 1)
            imu_seq[sensor] = seq
            imu_times[sensor].append(timestamp)
        for sensor in ("accelerometer", "gyroscope"):
            times = imu_times[sensor]
            if manifest["device"].get("imu_enabled") and not times:
                errors.append(f"IMU enabled but no {sensor} samples")
            if manifest.get("counts", {}).get(sensor, 0) != len(times):
                errors.append(f"Manifest count mismatch for {sensor}")
            if gaps[sensor]:
                warnings.append(f"{sensor}: {gaps[sensor]} sequence gaps")
            report["imu"][sensor] = {"samples": len(times), "sequence_gaps": gaps[sensor],
                                     "interval_ms": distribution(list(np.diff(times) / 1e6))}
        for a, b in (("left", "right"), ("rgb", "left")):
            if streams[a] and streams[b]:
                report["nearest_frame_offset_ms"][f"{a}_to_{b}"] = distribution(
                    nearest_offsets(sorted(streams[a]), sorted(streams[b])))
        for file in root.glob("images/**/*"):
            if file.is_file() and file.relative_to(root).as_posix() not in paths:
                warnings.append(f"Unindexed image/tail file: {file.relative_to(root)}")
        # Audit all journals, including a torn final JSON line after a power loss.
        for name in ("events", "clock"):
            list(jsonl(root / f"{name}.jsonl"))
        if "telemetry" in manifest:
            from .telemetry_audit import audit_telemetry
            telemetry = audit_telemetry(root, manifest)
            report["telemetry"] = telemetry
            errors.extend(telemetry["errors"])
            warnings.extend(telemetry["warnings"])
    except (OSError, ValueError, KeyError, TypeError, RuntimeError) as error:
        errors.append(str(error))
    report["valid"] = not errors
    return report
