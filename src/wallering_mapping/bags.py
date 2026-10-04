"""Offline rosbag2/MCAP auditing and import. This module never acquires live data."""

import dataclasses
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


def audit_bag(root):
    root = Path(root).resolve()
    report = {"schema_version": 1, "format": "rosbag2_mcap", "valid": False,
              "capture_ready": False, "coverage_complete": True, "survey_ready": False, "errors": [], "warnings": [],
              "topics": {}, "cameras": {}, "imu_source_sequence_gaps": None,
              "limitations": ["ROS Image/Imu headers do not expose hardware sequence counters.",
                              "COPY IMU messages combine reports; separate sensor timestamps are absent.",
                              "Driver ROS timestamps and bag receipt times are distinct.",
                              "Physical exposure timing and camera-to-body calibration remain unqualified."]}
    try:
        report["sealed_files"] = check_seal(root)
        if (root / "state").read_text().strip() != "complete":
            raise ValueError("Recording did not finish cleanly")
        parameters = requested_parameters(root)
        from ruamel.yaml import YAML
        actual = YAML(typ="safe").load((root / "oak-parameters.yaml").read_text())["/oak"]["ros__parameters"]
        report["driver_parameters"] = {name: actual[name] for name in (*CAMERAS, "imu")}
        for name, keys in [(name, ["i_fps", "i_resolution"]) for name in CAMERAS] + [
                ("imu", ["i_acc_freq", "i_gyro_freq", "i_sync_method"])]:
            for key in keys:
                if actual[name].get(key) != parameters[name][key]:
                    report["errors"].append(f"Requested/effective driver parameter mismatch: {name}.{key}")
        topics = (root / "topics.txt").read_text().splitlines()
        required = list(CAMERAS.values()) + [t.replace("image_raw", "camera_info")
                                           for t in CAMERAS.values()] + [IMU]
        if "/mavros/state" in topics:
            required += list(PX4_REQUIRED)
        received, headers = defaultdict(list), defaultdict(list)
        observed, geometry = Counter(), {}
        imu_values, sync_rows, gnss = [], [], defaultdict(list)
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
                    connected.append(bool(message.connected))
                if topic == "/mavros/timesync_status":
                    sync_rows.append(plain(message))
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
            if topic in list(CAMERAS.values()) + [IMU]:
                if any(t <= 0 for t in source) or any(t <= 0 for t in intervals):
                    report["errors"].append(f"Nonmonotonic/invalid source timestamps: {topic}")
                stream = next((s for s, t in CAMERAS.items() if t == topic), None)
                expected = parameters[stream]["i_fps"] if stream else parameters["imu"]["i_gyro_freq"]
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
            for topic, times in received.items():
                report["topics"][topic].update(
                    first_receipt_delay_seconds=(times[0] - start) / 1e9,
                    last_receipt_gap_seconds=(end - times[-1]) / 1e9)
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
        if connected and not all(connected):
            report["errors"].append("PX4 disconnected during recording")
        report["gnss"] = {topic: {"samples": len(values), "valid_fixes": sum(v >= 0 for v in values)}
                          for topic, values in gnss.items()}
        report["missing_optional_topics"] = [t for t in topics if not observed[t] and t not in required]
        if report["missing_optional_topics"]:
            report["warnings"].append("Some requested optional topics have no messages")
        if report["topics"].get(IMU, {}).get("intervals_over_1_5_periods"):
            report["warnings"].append("IMU timestamps have irregular intervals; hardware sample loss is unknown")
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
              "timestamp_domain": "ros_header", "stream_settings": {}}
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
