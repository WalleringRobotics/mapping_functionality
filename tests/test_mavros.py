import json
from dataclasses import replace

import numpy as np
import pytest

from wallering_mapping.config import CaptureConfig
from wallering_mapping.dataset import Session, sha256_file, write_json
from wallering_mapping.mavros import TelemetryBuffer, json_fields
from wallering_mapping.telemetry_config import TelemetryConfig
from wallering_mapping.validate import validate


PARAMETERS = {"timesync_mode": "MAVLINK", "convergence_window": 1, "max_rtt_sample": 10}


def clock(mono, ros):
    return {"kind": "clock_bridge", "reference": "python_monotonic", "target": "ros_system",
            "reference_ns": mono, "target_ns": ros, "bracket_ns": 1000}


def header(ros):
    sec, nano = divmod(ros, 10**9)
    return {"header": {"stamp": {"sec": sec, "nanosec": nano}, "frame_id": "base_link"}}


def timesync(remote, offset=1700000000000000000, rtt=1):
    return {**header(offset + remote), "remote_timestamp_ns": remote,
            "estimated_offset_ns": offset, "observed_offset_ns": offset + 100,
            "round_trip_time_ms": rtt}


def test_callback_queue_fails_on_overload_and_required_stream_stall():
    buffer = TelemetryBuffer(TelemetryConfig(queue_records=16, min_sync_samples=2), PARAMETERS)
    for i in range(16):
        buffer.ingest("pose", header(i + 1), "geometry_msgs/msg/PoseStamped", b"cdr",
                      clock(i + 1, i + 1), i + 1)
    with pytest.raises(RuntimeError, match="overflow"):
        buffer.ingest("pose", header(17), "geometry_msgs/msg/PoseStamped", b"cdr", clock(17, 17), 17)
    empty = TelemetryBuffer(TelemetryConfig(), PARAMETERS)
    with pytest.raises(RuntimeError, match="stalled"):
        empty.check(empty.started_ns + 6 * 10**9)


def test_preserves_header_cdr_and_marks_nonfinite_values_without_ros_import():
    buffer = TelemetryBuffer(TelemetryConfig(min_sync_samples=2), PARAMETERS)
    fields = {**header(1234567890), "latitude": float("nan")}
    buffer.ingest("gnss", fields, "sensor_msgs/msg/NavSatFix", b"original bits",
                  clock(10000000, 1235000000), 1700000000000000000)
    row = buffer.drain(check=False)[0]
    assert row["source_stamp_ros_ns"] == 1234567890
    assert row["fields"]["latitude"] == {"nonfinite": "nan"}
    assert row["cdr_base64"] == "b3JpZ2luYWwgYml0cw=="
    assert json_fields(np.array([1, 2])) == [1, 2]
    json.dumps(row, allow_nan=False)


def test_disconnection_and_ros_time_jump_do_not_silently_start_new_epoch():
    config = TelemetryConfig(min_sync_samples=2)
    buffer = TelemetryBuffer(config, PARAMETERS)
    buffer.ingest("state", {**header(1), "connected": True}, "mavros_msgs/msg/State", b"x", clock(1, 1), 1)
    with pytest.raises(RuntimeError, match="disconnected"):
        buffer.ingest("state", {**header(2), "connected": False}, "mavros_msgs/msg/State", b"x", clock(2, 2), 2)
    buffer.clock(clock(10**9, 1700000000000000000))
    with pytest.raises(RuntimeError, match="clock jump"):
        buffer.clock(clock(2 * 10**9, 1700000002000000000))


def telemetry_fixture(root):
    """IO/clock fixture with known clocks; not PX4/OAK hardware validation."""
    config = TelemetryConfig(min_sync_samples=2)
    capture = CaptureConfig(streams=("rgb",), min_free_gib=0, imu="off")
    contract = {"schema_version": 1, "adapter": "mavros_ros2", "config": config.to_dict(),
                "parameters": PARAMETERS, "frames": "ENU/FLU", "altitude": "WGS84 ellipsoidal"}
    session = Session(root, capture, "telemetry-test", {"imu_enabled": False}, {}, contract)
    buffer = TelemetryBuffer(config, PARAMETERS)
    # Python monotonic starts at 10 s; PX4 boot starts at 1 s; ROS has a distinct epoch.
    offset = 1700000000000000000
    for index in range(20):
        remote = 10**9 + index * 10**8
        mono = 10 * 10**9 + index * 10**8
        ros = offset + remote
        buffer.clock(clock(mono, ros))
        buffer.ingest("timesync", timesync(remote, offset), "mavros_msgs/msg/TimesyncStatus", b"cdr",
                      clock(mono + 1000, ros + 1000), ros + 1000)
        for role in ("state", "imu_raw", "pose"):
            fields = {**header(ros), "connected": True} if role == "state" else header(ros)
            if role == "pose":
                fields["pose"] = {"position": {"x": index, "y": 2, "z": 3},
                                  "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}
            buffer.ingest(role, fields, "fixture/msg/Message", b"cdr", clock(mono + 2000, ros + 2000), ros + 2000)
        session.append("clock", {**clock(mono, mono + 5 * 10**9), "target": "depthai_steady"})
    session.telemetry_batch(buffer.drain(check=False))
    for index in (5, 10, 15):
        remote = 10**9 + index * 10**8
        mono = 10 * 10**9 + index * 10**8
        camera = {"K": [[50, 0, 16], [0, 50, 16], [0, 0, 1]], "distortion": [0] * 8,
                  "model": "CameraModel.Perspective"}
        session.frame("rgb", np.zeros((32, 32, 3), np.uint8), {
            "sequence": index // 5, "device_ns": remote, "host_synced_ns": mono + 5 * 10**9,
            "received_monotonic_ns": mono + 50000000, "received_utc_ns": offset + remote + 50000000,
            "lens_position": 130, "camera": camera})
    session.finish()
    return root, config


def test_session_seals_telemetry_and_audit_rejects_tampering(tmp_path):
    root, _ = telemetry_fixture(tmp_path / "capture")
    result = validate(root)
    assert result["valid"], result["errors"]
    assert result["telemetry"]["ever_qualified"]
    path = root / "telemetry.jsonl"
    path.write_text(path.read_text().replace('"qualified"', '"invented"', 1))
    result = validate(root)
    assert not result["valid"]
    assert "Journal checksum mismatch: telemetry" in result["errors"]
    # Even recomputing integrity hashes must not legitimize fabricated sync quality.
    manifest = json.loads((root / "manifest.json").read_text())
    manifest["journals_sha256"]["telemetry"] = sha256_file(path)
    write_json(root / "manifest.json", manifest)
    assert "Timesync qualification differs from recorded observations" in validate(root)["errors"]


def test_missing_sync_is_a_failed_capture_not_optional_success():
    config = replace(TelemetryConfig(), required=("timesync",), min_sync_samples=2)
    buffer = TelemetryBuffer(config, PARAMETERS)
    buffer.ingest("timesync", timesync(1), "mavros_msgs/msg/TimesyncStatus", b"cdr",
                  clock(buffer.started_ns, 1700000000000000001), 1700000000000000001)
    with pytest.raises(RuntimeError, match="timing-qualified"):
        buffer.finish_check()
