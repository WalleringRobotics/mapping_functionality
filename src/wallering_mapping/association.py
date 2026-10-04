"""Offline exposure-to-PX4 association; original sensor records stay unchanged."""

import bisect
import csv
import json
import math

import numpy as np

from .dataset import jsonl, sha256_file, write_json
from .telemetry_config import TelemetryConfig
from .validate import distribution, validate


class ClockMap:
    def __init__(self, samples):
        self.samples = sorted(samples, key=lambda s: s["target_ns"])
        self.times = [s["target_ns"] for s in self.samples]
        if not self.samples or any(b <= a for a, b in zip(self.times, self.times[1:])):
            raise ValueError("Clock map requires strictly increasing observations")

    def to_monotonic(self, timestamp, max_age_ns):
        index = bisect.bisect_left(self.times, timestamp)
        if 0 < index < len(self.samples):
            left, right = self.samples[index - 1:index + 1]
            if max(timestamp - left["target_ns"], right["target_ns"] - timestamp) <= max_age_ns:
                # Only differences enter floating-point arithmetic: preserve epoch ns.
                delta = timestamp - left["target_ns"]
                mapped = left["reference_ns"] + round(delta * (
                    right["reference_ns"] - left["reference_ns"]) / (
                    right["target_ns"] - left["target_ns"]))
                budget = max(left["bracket_ns"], right["bracket_ns"]) // 2
                return mapped, budget, "interpolated_clock"
        candidates = self.samples[max(0, index - 1):index + 1]
        nearest = min(candidates, key=lambda s: abs(s["target_ns"] - timestamp))
        if abs(nearest["target_ns"] - timestamp) > max_age_ns:
            raise ValueError("Stale clock bridge")
        return (timestamp - nearest["target_ns"] + nearest["reference_ns"],
                nearest["bracket_ns"] // 2, "fresh_offset_hold")


def slerp_xyzw(a, b, weight):
    a, b = np.asarray(a, float), np.asarray(b, float)
    if (a.shape != (4,) or b.shape != (4,) or not np.isfinite(a).all() or not np.isfinite(b).all()
            or abs(np.linalg.norm(a) - 1) > .01 or abs(np.linalg.norm(b) - 1) > .01):
        raise ValueError("Invalid pose orientation quaternion")
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    dot = np.dot(a, b)
    if dot < 0:
        b, dot = -b, -dot
    dot = float(np.clip(dot, -1, 1))
    if dot > .9995:
        result = a + weight * (b - a)
        return (result / np.linalg.norm(result)).tolist()
    angle = math.acos(dot)
    return ((math.sin((1 - weight) * angle) * a + math.sin(weight * angle) * b) / math.sin(angle)).tolist()


def pose_at(rows, times, timestamp, max_bracket_ns):
    index = bisect.bisect_left(times, timestamp)
    if index < len(times) and times[index] == timestamp:
        left = right = rows[index]
        weight = 0
    elif 0 < index < len(times):
        left, right = rows[index - 1:index + 1]
        before, after = times[index - 1], times[index]
        if max(timestamp - before, after - timestamp) > max_bracket_ns:
            raise ValueError("Pose bracket exceeds configured limit")
        weight = (timestamp - before) / (after - before)
    else:
        raise ValueError("No bracketing pose; extrapolation is refused")
    a, b = left["fields"]["pose"], right["fields"]["pose"]
    if left["fields"]["header"]["frame_id"] != right["fields"]["header"]["frame_id"]:
        raise ValueError("Pose reference frame changed")
    xyz_a, xyz_b = ([p["position"][axis] for axis in "xyz"] for p in (a, b))
    if not np.isfinite(xyz_a + xyz_b).all():
        raise ValueError("Invalid pose position")
    xyz = (np.asarray(xyz_a) + weight * (np.asarray(xyz_b) - xyz_a)).tolist()
    quaternions = [[p["orientation"][axis] for axis in ("x", "y", "z", "w")] for p in (a, b)]
    return {"position_enu_m": xyz, "orientation_body_flu_to_enu_xyzw": slerp_xyzw(*quaternions, weight),
            "frame_id": left["fields"]["header"]["frame_id"], "weight": weight,
            "before_ordinal": left["ordinal"], "after_ordinal": right["ordinal"],
            "before_source_ros_ns": left["source_stamp_ros_ns"],
            "after_source_ros_ns": right["source_stamp_ros_ns"],
            "clock_budget_ns": max(left.get("clock_budget_ns", 0), right.get("clock_budget_ns", 0))}


def nearest_record(rows, times, timestamp, max_age_ns):
    index = bisect.bisect_left(times, timestamp)
    candidates = rows[max(0, index - 1):index + 1]
    if not candidates:
        return None
    nearest = min(candidates, key=lambda r: abs(r["mapped_monotonic_ns"] - timestamp))
    age = abs(nearest["mapped_monotonic_ns"] - timestamp)
    return {"ordinal": nearest["ordinal"], "source_ros_ns": nearest["source_stamp_ros_ns"],
            "age_ms": age / 1e6, "fields": nearest["fields"]} if age <= max_age_ns else None


def associate(root, output, stream="rgb", min_fraction=.9):
    root, output = root.resolve(), output.resolve()
    if output.is_relative_to(root) or root.is_relative_to(output):
        raise ValueError("Write association results outside the immutable source session")
    if not math.isfinite(min_fraction) or not 0 <= min_fraction <= 1:
        raise ValueError("min_fraction must be finite in [0,1]")
    audit = validate(root)
    if not audit["valid"]:
        raise ValueError(f"Dataset failed validation: {audit['errors']}")
    manifest = json.loads((root / "manifest.json").read_text())
    if manifest.get("source") == "rosbag2":
        raise ValueError("ROS image import has no qualified clock bridge; audit the original bag timing first")
    if "telemetry" not in manifest:
        raise ValueError("Session contains no MAVROS telemetry")
    if stream not in manifest["config"]["streams"]:
        raise ValueError("Requested camera stream was not captured")
    values = dict(manifest["telemetry"]["config"])
    values["required"] = tuple(values["required"])
    config = TelemetryConfig(**values)
    sdk = ClockMap([s for s in jsonl(root / "clock.jsonl") if s.get("target") == "depthai_steady"])
    ros = ClockMap([s for s in jsonl(root / "telemetry.jsonl") if s["record_type"] == "clock"])
    inverse_ros = ClockMap([{**s, "target_ns": s["reference_ns"], "reference_ns": s["target_ns"]}
                            for s in ros.samples])
    max_clock_age = int(config.max_clock_age_ms * 1e6)
    syncs = []
    role_rows = {role: [] for role in ("pose", "imu_raw", "gnss")}
    # Do not load CDR or duplicate attitude data into RAM for a long survey.
    for row in jsonl(root / "telemetry.jsonl"):
        if row["record_type"] != "message":
            continue
        if row["role"] == "timesync":
            syncs.append({k: row[k] for k in ("fields", "sync_quality", "received_monotonic_ns")})
        elif row["role"] in role_rows and row["source_stamp_ros_ns"]:
            role_rows[row["role"]].append({k: row[k] for k in ("fields", "ordinal", "source_stamp_ros_ns")})
    sync_times = [s["received_monotonic_ns"] for s in syncs]
    mapped = {}
    for role in ("pose", "imu_raw", "gnss"):
        rows = []
        for row in role_rows[role]:
            try:
                stamp, budget, _ = ros.to_monotonic(row["source_stamp_ros_ns"], max_clock_age)
            except ValueError:
                continue
            rows.append({**row, "mapped_monotonic_ns": stamp, "clock_budget_ns": budget})
        rows.sort(key=lambda r: r["mapped_monotonic_ns"])
        times = [r["mapped_monotonic_ns"] for r in rows]
        if any(b <= a for a, b in zip(times, times[1:])):
            raise ValueError(f"Duplicate/nonmonotonic source timestamps: {role}")
        mapped[role] = rows, times
    output.mkdir(parents=True, exist_ok=False)
    summary = {"status": "running", "source_manifest_sha256": sha256_file(root / "manifest.json"),
               "stream": stream, "min_associated_fraction": min_fraction,
               "coordinate_frame": "Vehicle pose in local ENU; body FLU, not calibrated camera pose",
               "accuracy_claim": "Estimated temporal association only; no metric accuracy claim",
               "camera_extrinsics": "Not supplied; do not treat body/GNSS position as camera center",
               "altitude": manifest["telemetry"]["altitude"]}
    write_json(output / "report.json", summary)
    matched, rejected, budgets = [], [], []
    try:
        with (output / "associations.jsonl").open("x") as decisions:
            for frame in jsonl(root / "frames.jsonl"):
                if frame["stream"] != stream:
                    continue
                record = {"image": frame["path"], "sequence": frame["sequence"],
                          "device_exposure_ns": frame["device_ns"], "sdk_exposure_ns": frame["host_synced_ns"]}
                try:
                    mono, sdk_bracket, method = sdk.to_monotonic(frame["host_synced_ns"], max_clock_age)
                    index = bisect.bisect_right(sync_times, mono) - 1
                    if index < 0 or mono - sync_times[index] > config.max_sync_age_ms * 1e6:
                        raise ValueError("Missing/stale MAVROS TIMESYNC evidence")
                    sync = syncs[index]
                    if sync["sync_quality"]["qualification"] != "qualified":
                        raise ValueError("MAVROS timing not qualified at exposure")
                    ros_exposure, ros_bracket, ros_method = inverse_ros.to_monotonic(mono, max_clock_age)
                    pose = pose_at(*mapped["pose"], mono, int(config.max_pose_bracket_ms * 1e6))
                    budget = (sdk_bracket + max(ros_bracket, pose["clock_budget_ns"])
                              + sync["sync_quality"]["timesync_budget_ns"]
                              + int((config.sdk_sync_budget_ms + config.px4_timestamp_budget_ms) * 1e6))
                    if budget > config.max_alignment_budget_ms * 1e6:
                        raise ValueError("Estimated clock-alignment budget exceeds configured limit")
                    imu = nearest_record(*mapped["imu_raw"], mono, int(config.max_pose_bracket_ms * 1e6))
                    gnss = nearest_record(*mapped["gnss"], mono, int(config.max_gnss_age_ms * 1e6))
                    if gnss:
                        fields = gnss["fields"]
                        try:
                            valid_fix = (fields["status"]["status"] >= 0 and np.isfinite(
                                [fields["latitude"], fields["longitude"], fields["altitude"]]).all()
                                and -90 <= fields["latitude"] <= 90 and -180 <= fields["longitude"] <= 180)
                        except (KeyError, TypeError):
                            valid_fix = False
                        if not valid_fix:
                            gnss = None
                    record.update(status="associated", exposure_monotonic_ns=mono,
                                  exposure_ros_ns=ros_exposure,
                                  estimated_px4_boot_ns=ros_exposure - sync["fields"]["estimated_offset_ns"],
                                  clock_method=method, ros_clock_method=ros_method,
                                  estimated_alignment_budget_ms=budget / 1e6,
                                  body_pose=pose, imu=imu, gnss=gnss,
                                  gnss_reference="MAVROS vehicle/global fix; no camera lever arm applied")
                    matched.append(record)
                    budgets.append(budget / 1e6)
                except ValueError as error:
                    record.update(status="unassociated", reason=str(error))
                    rejected.append(record)
                decisions.write(json.dumps(record, allow_nan=False) + "\n")
        with (output / "body-poses.csv").open("x", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["image", "exposure_monotonic_ns", "exposure_ros_ns", "estimated_px4_boot_ns",
                             "body_x_enu_m", "body_y_enu_m", "body_z_enu_m", "q_x", "q_y", "q_z", "q_w",
                             "estimated_alignment_budget_ms"])
            for row in matched:
                pose = row["body_pose"]
                writer.writerow([row["image"], row["exposure_monotonic_ns"], row["exposure_ros_ns"],
                                 row["estimated_px4_boot_ns"], *pose["position_enu_m"],
                                 *pose["orientation_body_flu_to_enu_xyzw"], row["estimated_alignment_budget_ms"]])
        total = len(matched) + len(rejected)
        fraction = len(matched) / total if total else 0
        summary.update(status="complete", passed=bool(matched) and fraction >= min_fraction,
                       associated=len(matched), unassociated=len(rejected), associated_fraction=fraction,
                       alignment_budget_ms=distribution(budgets),
                       gnss_associated=sum(r["gnss"] is not None for r in matched),
                       output_hashes={name: sha256_file(output / name) for name in ("associations.jsonl", "body-poses.csv")})
    except BaseException as error:
        summary.update(status="failed", error=str(error))
        raise
    finally:
        write_json(output / "report.json", summary)
    return summary
