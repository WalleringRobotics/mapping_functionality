"""Offline rosbag2/MCAP auditing and import. This module never acquires live data."""

import dataclasses
import json
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np

from .config import CaptureConfig
from .dataset import Session, safe_path, sha256_file, write_json
from .validate import distribution


CAMERAS = {name: f"/oak/{name}/image_raw" for name in ("rgb", "left", "right")}
IMU = "/oak/imu/data"
PX4_REQUIRED = ("/mavros/state", "/mavros/imu/data_raw", "/mavros/local_position/pose",
                "/mavros/timesync_status")


def reader(root):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    bag = root / "bag" if (root / "bag").is_dir() else root
    return AnyReader([bag], default_typestore=get_typestore(Stores.ROS2_HUMBLE))


def stamp(message):
    return int(message.header.stamp.sec) * 10**9 + int(message.header.stamp.nanosec)


def plain(value):
    if dataclasses.is_dataclass(value):
        return {f.name: plain(getattr(value, f.name)) for f in dataclasses.fields(value)
                if f.name != "__msgtype__"}
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    return value


def check_seal(root):
    path = root / "SHA256SUMS"
    if not path.is_file():
        raise ValueError("Recording has no SHA256SUMS seal; finalize it with record-rosbag.sh")
    checked = set()
    for line in path.read_text().splitlines():
        expected, relative = line.split("  ", 1)
        if sha256_file(safe_path(root, relative)) != expected:
            raise ValueError(f"Recording checksum mismatch: {relative}")
        checked.add(Path(relative).as_posix())
    required = {p.relative_to(root).as_posix() for p in (root / "bag").glob("*.mcap")}
    required |= {"bag/metadata.yaml", "oak-requested.yaml", "oak-parameters.yaml", "mcap.yaml", "topics.txt"}
    # Every present input is immutable, including factory calibration, phase/rate
    # logs and copied transport profiles. Backward-compatible with older sessions
    # that did not produce those optional provenance files.
    required |= {p.relative_to(root).as_posix() for p in root.rglob("*")
                 if p.is_file() and p.name not in {"state", "SHA256SUMS"}}
    if any((root / name).exists() for name in ("acquisition-start-ns.txt", "acquisition-end-ns.txt")):
        required |= {"acquisition-start-ns.txt", "acquisition-end-ns.txt"}
    if "/mavros/state" in (root / "topics.txt").read_text().splitlines():
        required.add("mavros-time.yaml")
    if not required <= checked:
        raise ValueError("Recording seal omits required files")
    return sorted(checked)


def camera_geometry(message):
    if message.distortion_model not in {"rational_polynomial", "plumb_bob", "equidistant"}:
        raise ValueError(f"Unsupported camera model: {message.distortion_model}")
    k = np.asarray(message.k).reshape(3, 3)
    if not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0:
        raise ValueError("Invalid CameraInfo intrinsics")
    return {"width": int(message.width), "height": int(message.height),
            "frame_id": message.header.frame_id,
            "camera": {"K": k.tolist(), "distortion": message.d.tolist(),
                       "model": ("CameraModel.Fisheye" if message.distortion_model == "equidistant"
                                 else "CameraModel.Perspective")}}


def image_array(message):
    channels = {"bgr8": 3, "rgb8": 3, "mono8": 1}.get(message.encoding)
    if channels is None:
        raise ValueError(f"Unsupported image encoding: {message.encoding}")
    if (min(message.width, message.height) <= 0 or message.step < message.width * channels
            or len(message.data) != message.height * message.step):
        raise ValueError("Invalid image shape/stride/data length")
    rows = np.asarray(message.data, dtype=np.uint8).reshape(message.height, message.step)
    array = rows[:, :message.width * channels]
    array = array.reshape(message.height, message.width, channels) if channels > 1 else array
    if message.encoding == "rgb8":
        array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
    return array


def requested_parameters(root):
    from ruamel.yaml import YAML
    value = YAML(typ="safe").load((root / "oak-requested.yaml").read_text())
    return value["/oak"]["ros__parameters"]


def connection_window(samples, start, end):
    """Evaluate connection during acquisition, retaining pre/post-roll evidence."""
    before = [connected for receipt, connected in samples if receipt < start]
    within = [connected for receipt, connected in samples if start <= receipt <= end]
    after = [connected for receipt, connected in samples if receipt > end]
    states = before[-1:] + within
    return {"connected_at_start": before[-1] if before else None,
            "disconnected_before_window": before.count(False),
            "disconnected_in_window": within.count(False),
            "disconnected_after_window": after.count(False),
            "connected_throughout_window": bool(states) and all(states)}


# BNO086 rate points; the driver rounds requests up. /oak/imu/data follows the
# sensor that is not interpolated (COPY emits at about the gyro rate).
BNO086_HZ = {"i_gyro_freq": (25, 33, 50, 100, 200, 400), "i_acc_freq": (15, 31, 62, 125, 250, 500)}
IMU_MESSAGE_RATE = {"LINEAR_INTERPOLATE_GYRO": "i_acc_freq"}


def imu_message_rate(imu):
    """Nominal /oak/imu/data message rate implied by the driver IMU parameters."""
    key = IMU_MESSAGE_RATE.get(imu.get("i_sync_method"), "i_gyro_freq")
    requested = imu[key]
    return next((rate for rate in BNO086_HZ[key] if rate >= requested), requested)


def sample_loss(stamps, start, end, rate_hz=None, values=None):
    """In-window sample count against the requested rate, header-stamp gaps and repeats.

    A gap is an interval over 1.5 nominal periods (1/rate, or the median interval when
    no rate is requested), so a single lost sample counts. missing_samples compares the count with rate x window;
    missing_in_gaps sums whole periods absent inside gaps. Values, when given, are
    per-message tuples whose consecutive repeats are counted (for example gyro samples).
    """
    if end < start or (rate_hz is not None and (not np.isfinite(rate_hz) or rate_hz <= 0)):
        raise ValueError("Invalid sample window or nominal rate")
    stamps = np.asarray(stamps, dtype=np.int64)
    inside = (stamps >= start) & (stamps <= end)
    window = stamps[inside]
    result = {"window_seconds": (end - start) / 1e9, "samples": int(window.size),
              "requested_hz": rate_hz, "expected_samples": None, "missing_samples": None,
              "loss_percent": None, "gaps": 0, "max_gap_ms": None, "missing_in_gaps": 0,
              "repeated_samples": None, "measured_hz": None,
              "hardware_sample_loss": None, "invalid_intervals": 0,
              "count_basis": "nominal rate times duration; boundary phase and clock drift are unknown"}
    if window.size >= 2:
        intervals = np.diff(window) / 1e6
        result["invalid_intervals"] = int((intervals <= 0).sum())
        positive = intervals[intervals > 0]
        period = 1000 / rate_hz if rate_hz else (float(np.median(positive)) if positive.size else None)
        if period:
            gaps = intervals[intervals > 1.5 * period]
            result.update(nominal_period_ms=period, gaps=int(gaps.size),
                          max_gap_ms=float(gaps.max()) if gaps.size else None,
                          missing_in_gaps=int(sum(round(gap / period) - 1 for gap in gaps)))
        if not result["invalid_intervals"]:
            result["measured_hz"] = (window.size - 1) * 1e9 / float(window[-1] - window[0])
    if rate_hz:
        expected = round(rate_hz * (end - start) / 1e9)
        result.update(expected_samples=expected, missing_samples=max(0, expected - int(window.size)))
    else:
        result.update(expected_samples=int(window.size) + result["missing_in_gaps"],
                      missing_samples=result["missing_in_gaps"])
    if result["expected_samples"]:
        result["loss_percent"] = 100 * result["missing_samples"] / result["expected_samples"]
    if values is not None:
        selected = np.asarray(values, dtype=float)[inside]
        result["repeated_samples"] = (int(np.all(selected[1:] == selected[:-1], axis=1).sum())
                                      if selected.size else 0)
    return result


def mavlink_window(packets, start, end):
    """Observe raw-link receipt load and modulo-256 gaps; not hardware loss proof."""
    selected = [row for row in packets if start <= row[0] <= end]
    previous, gaps, duplicates, reorder = {}, 0, 0, 0
    for _, system, component, sequence, _ in selected:
        key = (system, component)
        if key in previous:
            step = (sequence - previous[key]) % 256
            if step == 0:
                duplicates += 1
                continue
            if step > 127:
                reorder += 1
                continue
            gaps += step - 1
        previous[key] = sequence
    seconds = (end - start) / 1e9
    return {"messages": len(selected), "duration_seconds": seconds,
            "received_wire_bytes_per_second": sum(row[4] for row in selected) / seconds if seconds > 0 else None,
            "inferred_sequence_gaps": gaps, "duplicate_sequences": duplicates,
            "reordered_or_reset_sequences": reorder,
            "limitation": "Receipt-side estimate; ROS loss, source restart, routing and wrap ambiguity remain. "
                          "Wire load excludes outbound traffic; serial baud capacity is not inferred."}


def audit_bag(root):
    root = Path(root).resolve()
    report = {"schema_version": 1, "format": "rosbag2_mcap", "valid": False,
              "capture_ready": False, "coverage_complete": True, "survey_ready": False, "errors": [], "warnings": [],
              "topics": {}, "cameras": {}, "imu_source_sequence_gaps": None,
              "limitations": ["ROS Image/Imu headers do not expose hardware sequence counters.",
                              "Driver ROS timestamps and bag receipt times are distinct.",
                              "Physical exposure timing and camera-to-body calibration remain unqualified."]}
    try:
        report["sealed_files"] = check_seal(root)
        if (root / "state").read_text().strip() != "complete":
            raise ValueError("Recording did not finish cleanly")
        parameters = requested_parameters(root)
        session_path = root / "session.json"
        session = json.loads(session_path.read_text()) if session_path.exists() else {"kind": "survey"}
        report["kind"] = session["kind"]
        if report["kind"] not in {"survey", "calibration"}:
            raise ValueError("Unknown recording kind")
        px4_rate = session.get("px4_imu_requested_hz", 0)
        if px4_rate not in {0, 100}:
            raise ValueError("Invalid session PX4 rate request")
        if px4_rate:
            for name in ("px4-rate-request.json", "px4-rate-restore.json"):
                evidence = json.loads((root / name).read_text())
                report[name.removesuffix(".json").replace("-", "_")] = evidence
                expected_rate = px4_rate if name == "px4-rate-request.json" else 0
                rows = evidence.get("requests", [])
                if (not evidence.get("success") or evidence.get("command") != 511
                        or evidence.get("requested_hz") != expected_rate
                        or [r.get("message_id") for r in rows] != [105, 31]
                        or any(not r.get("success") or r.get("ack_result") != 0
                               or r.get("requested_hz") != expected_rate for r in rows)):
                    report["errors"].append(f"PX4 rate command unsuccessful: {name}")
        from ruamel.yaml import YAML
        actual = YAML(typ="safe").load((root / "oak-parameters.yaml").read_text())["/oak"]["ros__parameters"]
        report["driver_parameters"] = {name: actual[name] for name in (*CAMERAS, "imu")}
        for name, keys in [(name, ["i_fps", "i_resolution"]) for name in CAMERAS] + [
                ("imu", ["i_acc_freq", "i_gyro_freq", "i_sync_method"])]:
            for key in keys:
                if actual[name].get(key) != parameters[name][key]:
                    report["errors"].append(f"Requested/effective driver parameter mismatch: {name}.{key}")
        if actual["imu"].get("i_sync_method") == "COPY":
            report["limitations"].append(
                "COPY IMU messages carry accelerometer timestamps with the latest gyroscope sample "
                "copied in; gyroscope timing is quantised and may repeat (#18).")
        else:
            report["limitations"].append(
                f"IMU sync {actual['imu'].get('i_sync_method')}: messages follow one sensor's samples; "
                "the other sensor is interpolated onto them.")
        topics = (root / "topics.txt").read_text().splitlines()
        required = list(CAMERAS.values()) + [t.replace("image_raw", "camera_info")
                                           for t in CAMERAS.values()] + [IMU]
        if "/mavros/state" in topics:
            required += list(PX4_REQUIRED)
        if px4_rate:
            required += ["/mavros/imu/data", *PX4_REQUIRED]
        received, headers = defaultdict(list), defaultdict(list)
        observed, geometry = Counter(), {}
        imu_values, sync_rows, gnss = [], [], defaultdict(list)
        sync_receipts, mavlink_packets = [], []
        gyro, imu_topics = defaultdict(list), set()
        connected = []
        with reader(root) as bag:
            metadata_counts = Counter()
            for connection in bag.connections:
                metadata_counts[connection.topic] += connection.msgcount
            for connection, receipt, raw in bag.messages():
                topic = connection.topic
                message = bag.deserialize(raw, connection.msgtype)
                observed[topic] += 1
                received[topic].append(receipt)
                if hasattr(message, "header"):
                    headers[topic].append(stamp(message))
                    if connection.msgtype == "sensor_msgs/msg/Imu":
                        imu_topics.add(topic)
                        rate = message.angular_velocity
                        gyro[topic].append((rate.x, rate.y, rate.z))
                        vectors = (message.angular_velocity, message.linear_acceleration)
                        if not all(np.isfinite(getattr(v, axis)) for v in vectors for axis in "xyz"):
                            raise ValueError(f"Nonfinite IMU values: {topic}")
                if topic in CAMERAS.values():
                    image_array(message)
                    shape = (message.width, message.height, message.encoding, message.header.frame_id)
                    if topic in geometry and geometry[topic] != shape:
                        raise ValueError(f"Image geometry changed on {topic}")
                    geometry[topic] = shape
                if topic in [t.replace("image_raw", "camera_info") for t in CAMERAS.values()]:
                    info = camera_geometry(message)
                    if topic in report["cameras"] and report["cameras"][topic] != info:
                        raise ValueError(f"CameraInfo changed on {topic}")
                    report["cameras"][topic] = info
                if topic == IMU:
                    imu_values.append([getattr(vector, axis) for vector in
                                       (message.linear_acceleration, message.angular_velocity)
                                       for axis in "xyz"])
                if topic == "/mavros/state":
                    connected.append((receipt, bool(message.connected)))
                if topic == "/mavros/timesync_status":
                    sync_rows.append(plain(message))
                    sync_receipts.append(receipt)
                if topic == "/uas1/mavlink_source":
                    wire_bytes = int(message.len) + (12 if message.magic == 253 else 8)
                    if message.magic == 253 and message.incompat_flags & 1:
                        wire_bytes += 13
                    mavlink_packets.append((receipt, int(message.sysid), int(message.compid),
                                            int(message.seq), wire_bytes))
                if topic in {"/mavros/global_position/global", "/mavros/global_position/raw/fix"}:
                    gnss[topic].append(int(message.status.status))
            if dict(observed) != {k: v for k, v in metadata_counts.items() if v}:
                raise ValueError("Read message counts differ from rosbag metadata")
        for topic, times in received.items():
            source = headers[topic]
            intervals = list(np.diff(source) / 1e6) if len(source) > 1 else []
            item = {"messages": observed[topic], "received_hz": (
                (len(times) - 1) * 1e9 / (times[-1] - times[0]) if times[-1] > times[0] else None),
                "header_interval_ms": distribution(intervals),
                "first_header_stamp_ros_ns": source[0] if source else None,
                "last_header_stamp_ros_ns": source[-1] if source else None}
            report["topics"][topic] = item
            if topic in ("/mavros/imu/data_raw", "/mavros/imu/data") and px4_rate:
                item["requested_hz"] = px4_rate
            if topic in list(CAMERAS.values()) + [IMU]:
                if any(t <= 0 for t in source) or any(t <= 0 for t in intervals):
                    report["errors"].append(f"Nonmonotonic/invalid source timestamps: {topic}")
                stream = next((s for s, t in CAMERAS.items() if t == topic), None)
                expected = parameters[stream]["i_fps"] if stream else imu_message_rate(parameters["imu"])
                item["requested_hz"] = expected
                if len(source) < 2 or (source[-1] - source[0]) <= 0:
                    report["errors"].append(f"Insufficient source samples: {topic}")
                else:
                    item["header_hz"] = (len(source) - 1) * 1e9 / (source[-1] - source[0])
                    item["intervals_over_1_5_periods"] = int(sum(i > 1500 / expected for i in intervals))
                    if item["header_hz"] < .9 * expected:
                        report["errors"].append(f"Source rate below 90% of request: {topic}")
                    if stream and item["intervals_over_1_5_periods"]:
                        report["errors"].append(f"Image timestamp gaps: {topic}")
        for topic in required:
            if not observed[topic]:
                report["errors"].append(f"Required topic missing: {topic}")
        all_receipts = [t for times in received.values() for t in times]
        if all_receipts:
            start, end = min(all_receipts), max(all_receipts)
            report["receipt_duration_seconds"] = (end - start) / 1e9
            if (root / "acquisition-start-ns.txt").is_file():
                requested_start = int((root / "acquisition-start-ns.txt").read_text())
                requested_end = int((root / "acquisition-end-ns.txt").read_text())
                if requested_start >= requested_end:
                    raise ValueError("Invalid acquisition time window")
                if max(start, requested_start) >= min(end, requested_end):
                    raise ValueError("Acquisition window does not overlap the bag")
                start, end = requested_start, requested_end
                report["coverage_window"] = {"start_ns": start, "end_ns": end,
                    "duration_seconds": (end - start) / 1e9,
                    "source": "Acquisition host realtime; excludes pre/post-roll, not physical exposure timing"}
            for topic, times in received.items():
                report["topics"][topic].update(
                    first_receipt_delay_seconds=max(0, times[0] - start) / 1e9,
                    last_receipt_gap_seconds=max(0, end - times[-1]) / 1e9)
                if topic in CAMERAS.values():
                    expected = report["topics"][topic]["requested_hz"]
                    if max(times[0] - start, end - times[-1]) > 3e9 / expected:
                        report["coverage_complete"] = False
                        report["warnings"].append(f"Image coverage misses over three frame periods at a bag boundary: {topic}")
            for topic in [*CAMERAS.values(), IMU, *(PX4_REQUIRED if "/mavros/state" in topics else ())]:
                times = received[topic]
                if times and max(times[0] - start, end - times[-1], max(np.diff(times), default=0)) > 5e9:
                    report["errors"].append(f"Required stream absent/stalled for over 5 seconds: {topic}")
        for topic, (width, height, _, frame_id) in geometry.items():
            info = report["cameras"].get(topic.replace("image_raw", "camera_info"))
            if info is None or (info["width"], info["height"], info["frame_id"]) != (width, height, frame_id):
                report["errors"].append(f"Image/CameraInfo mismatch: {topic}")
        if not np.isfinite(imu_values).all():
            report["errors"].append("Nonfinite IMU values")
        if connected:
            report["px4_connection"] = connection_window(connected, start, end)
            if not report["px4_connection"]["connected_throughout_window"]:
                report["errors"].append("PX4 disconnected during acquisition window")
        report["gnss"] = {topic: {"samples": len(values), "valid_fixes": sum(v >= 0 for v in values)}
                          for topic, values in gnss.items()}
        report["missing_optional_topics"] = [t for t in topics if not observed[t] and t not in required]
        if report["missing_optional_topics"]:
            report["warnings"].append("Some requested optional topics have no messages")
        for topic in [*CAMERAS.values(), *sorted(imu_topics)]:
            source = headers.get(topic)
            if not source:
                continue
            window = (start, end) if "coverage_window" in report else (source[0], source[-1])
            loss = sample_loss(source, *window, report["topics"][topic].get("requested_hz"),
                               gyro[topic] if topic in imu_topics else None)
            report["topics"][topic]["in_window"] = loss
            if topic in imu_topics and loss["invalid_intervals"]:
                report["errors"].append(f"Nonmonotonic IMU stamps in acquisition window: {topic}")
            if topic in ("/mavros/imu/data_raw", "/mavros/imu/data") and px4_rate:
                if loss["measured_hz"] is None or loss["measured_hz"] < .95 * px4_rate:
                    report["errors"].append(f"PX4 IMU rate below 95% of request: {topic}")
            # Repeats alone are not flagged: a stationary, quantised gyro repeats by chance.
            if topic in imu_topics and loss["gaps"]:
                expected = (f"{loss['expected_samples']} expected at {loss['requested_hz']} Hz"
                            if loss["requested_hz"] else "rate unrequested")
                report["warnings"].append(
                    f"IMU sample loss estimate in acquisition window: {topic} {loss['samples']} samples, "
                    f"{expected} ({loss['loss_percent']:.2f}% missing), {loss['gaps']} gaps "
                    f"(max {loss['max_gap_ms'] or 0:.0f} ms, {loss['missing_in_gaps']} samples), "
                    f"{loss['repeated_samples']} repeated")
        if report["kind"] == "calibration":
            from .recording import CALIBRATION_PHASES
            phases = json.loads((root / "calibration-phases.json").read_text())
            report["calibration_phases"] = phases
            rows = phases.get("phases", [])
            if [p.get("phase") for p in rows] != [p[1] for p in CALIBRATION_PHASES]:
                report["errors"].append("Calibration guidance did not complete all phases")
            elif end - rows[-1]["started_utc_ns"] < (CALIBRATION_PHASES[-1][0] - .5) * 1e9:
                report["errors"].append("Calibration final still phase was cut short")
        if sync_rows:
            from .telemetry_config import TelemetryConfig
            from .timing import SyncMonitor
            from ruamel.yaml import YAML
            params = YAML(typ="safe").load((root / "mavros-time.yaml").read_text())
            values = next(iter(params.values()))["ros__parameters"]
            monitor = SyncMonitor(TelemetryConfig(), values)
            qualities = [monitor.observe(row) for row in sync_rows]
            report["timesync"] = {
                "samples": len(sync_rows), "qualified_samples": sum(
                    q["qualification"] == "qualified" for q in qualities),
                "maximum_good_streak": max(q["consecutive_good"] for q in qualities),
                "required_good": monitor.minimum,
                "rtt_ms": distribution([row["round_trip_time_ms"] for row in sync_rows]),
                "offset_residual_ms": distribution([q["offset_residual_ns"] / 1e6 for q in qualities])}
            if not report["timesync"]["qualified_samples"]:
                report["warnings"].append("PX4 timing did not satisfy the existing qualification gate")
            windows = {"acquisition": (start, end)}
            requests = report.get("px4_rate_request", {}).get("requests", [])
            if requests and all_receipts:
                windows["before_rate_request"] = (min(all_receipts), requests[0]["requested_utc_ns"])
            report["link_windows"] = {}
            for name, (window_start, window_end) in windows.items():
                indices = [i for i, t in enumerate(sync_receipts) if window_start <= t <= window_end]
                pose_times = [t for t in received["/mavros/local_position/pose"]
                              if window_start <= t <= window_end]
                report["link_windows"][name] = {
                    "mavlink_source": mavlink_window(mavlink_packets, window_start, window_end),
                    "pose_received_hz": ((len(pose_times) - 1) * 1e9 / (pose_times[-1] - pose_times[0])
                                         if len(pose_times) > 1 and pose_times[-1] > pose_times[0] else None),
                    "timesync_samples": len(indices),
                    "timesync_qualified_samples": sum(qualities[i]["qualification"] == "qualified" for i in indices),
                    "rtt_ms": distribution([sync_rows[i]["round_trip_time_ms"] for i in indices]),
                    "offset_residual_ms": distribution([qualities[i]["offset_residual_ns"] / 1e6 for i in indices])}
        report["valid"] = not report["errors"]
        report["capture_ready"] = report["valid"] and report["coverage_complete"]
    except Exception as error:
        report["errors"].append(f"{type(error).__name__}: {error}")
    return report


def import_bag(root, output):
    """Produce a derived image dataset for existing offline engines, retaining the MCAP source."""
    root, output = Path(root).resolve(), Path(output).resolve()
    if output.is_relative_to(root) or root.is_relative_to(output):
        raise ValueError("Import output must be separate from the immutable recording")
    audit = audit_bag(root)
    if not audit["valid"]:
        raise ValueError(f"Bag failed validation: {audit['errors']}")
    parameters = requested_parameters(root)
    rgb = parameters["rgb"]
    config = CaptureConfig(fps=rgb["i_fps"], rgb_format="png", imu="off",
                           exposure_us=rgb["r_exposure"], iso=rgb["r_iso"],
                           lens_position=rgb["r_focus"], white_balance_k=rgb["r_whitebalance"])
    calibration = {"source": "ROS CameraInfo", "cameras": audit["cameras"]}
    device = {"adapter": "official_depthai_ros_driver", "imu_enabled": False,
              "hardware_sequence_numbers": False, "sequence_origin": "bag_ordinal",
              "timestamp_domain": "ros_header", "stream_settings": {
                  name: {"fps": parameters[name]["i_fps"], "resolution": parameters[name]["i_resolution"]}
                  for name in CAMERAS}}
    session = Session(output, config, "rosbag2", device, calibration)
    session.manifest["rosbag_source"] = {
        "path": str(root), "seal_sha256": sha256_file(root / "SHA256SUMS"),
        "note": "Original IMU, PX4 CDR messages and receipt timestamps remain in MCAP."}
    session.manifest["time_semantics"] = {
        "device_ns": "Compatibility field containing ORIGINAL ROS HEADER time, not device uptime",
        "host_synced_ns": "Original ROS header time; not DepthAI steady time",
        "received_utc_ns": "rosbag2 receipt timestamp",
        "sequence": "Per-stream bag ordinal, NOT hardware sequence; source losses unobservable",
        "settings": "Requested driver parameters, NOT per-frame hardware settings readback"}
    status, reason = "complete", "offline MCAP import"
    try:
        with reader(root) as bag:
            selected = [c for c in bag.connections if c.topic in CAMERAS.values()]
            ordinals = Counter()
            for connection, receipt, raw in bag.messages(connections=selected):
                stream = next(name for name, topic in CAMERAS.items() if topic == connection.topic)
                message = bag.deserialize(raw, connection.msgtype)
                ordinals[stream] += 1
                setting = parameters[stream]
                camera = calibration["cameras"][connection.topic.replace("image_raw", "camera_info")]
                metadata = {"sequence": ordinals[stream], "sequence_origin": "bag_ordinal",
                            "device_ns": stamp(message), "host_synced_ns": stamp(message),
                            "source_stamp_ros_ns": stamp(message), "received_utc_ns": receipt,
                            "timestamp_domain": "ros_header", "frame_id": message.header.frame_id,
                            "camera": camera["camera"], "settings_source": "requested_ros_parameters",
                            "exposure_us": setting["r_exposure"], "iso": setting["r_iso"],
                            "lens_position": setting.get("r_focus"),
                            "white_balance_k": setting.get("r_whitebalance")}
                session.frame(stream, image_array(message), metadata)
        write_json(output / "bag-audit.json", audit)
    except BaseException as error:
        status, reason = "failed", str(error)
        raise
    finally:
        session.finish(status, reason)
    return session.manifest
