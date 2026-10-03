"""ROS telemetry and clock evidence audit without ROS runtime dependencies."""

import base64
from collections import Counter

from .dataset import jsonl
from .mavros import stamp_ns
from .telemetry_config import TelemetryConfig
from .timing import ClockTracker, SyncMonitor


def audit_telemetry(root, manifest):
    contract = manifest["telemetry"]
    if contract.get("schema_version") != 1 or contract.get("adapter") != "mavros_ros2":
        raise ValueError("Unsupported telemetry contract")
    values = dict(contract["config"])
    values["required"] = tuple(values["required"])
    config = TelemetryConfig(**values)
    monitor = SyncMonitor(config, contract["parameters"])
    tracker = ClockTracker(config.clock_jump_ms)
    errors, warnings = [], []
    counts = Counter()
    connected = False
    for row in jsonl(root / "telemetry.jsonl"):
        if row["record_type"] == "clock":
            tracker.observe(row)
            counts["telemetry_clock"] += 1
            continue
        if row["record_type"] != "message" or row["role"] not in config.topics:
            raise ValueError("Invalid telemetry record type/role")
        role = row["role"]
        key = "telemetry_" + role
        counts[key] += 1
        if row["topic"] != config.topics[role] or row["ordinal"] != counts[key]:
            errors.append(f"Telemetry topic/ordinal mismatch: {role}")
        receipt = row["receipt_clock"]
        if receipt["target"] != "ros_system" or receipt["reference"] != "python_monotonic":
            raise ValueError("Invalid ROS receipt clock domain")
        if row["received_monotonic_ns"] != receipt["reference_ns"] or receipt["bracket_ns"] < 0:
            raise ValueError("Invalid telemetry receipt-clock observation")
        if stamp_ns(row["fields"]) != row["source_stamp_ros_ns"]:
            errors.append(f"Original ROS header stamp differs from indexed timestamp: {role}")
        if not base64.b64decode(row["cdr_base64"], validate=True):
            errors.append(f"Missing serialized telemetry message: {role}")
        if role == "timesync":
            expected = monitor.observe(row["fields"])
            if row["sync_quality"] != expected:
                errors.append("Timesync qualification differs from recorded observations")
        if role == "state":
            if connected and not row["fields"]["connected"]:
                errors.append("MAVROS connection continuity was lost")
            connected |= row["fields"]["connected"]
    for key in {"telemetry_clock", *("telemetry_" + role for role in config.topics)}:
        if manifest["counts"].get(key, 0) != counts[key]:
            errors.append(f"Manifest telemetry count mismatch: {key}")
    for role in config.required:
        if not counts["telemetry_" + role]:
            errors.append(f"Missing required telemetry role: {role}")
    if not connected or not monitor.ever_qualified:
        errors.append("No connected, qualified PX4 timing interval")
    if not counts["telemetry_clock"]:
        errors.append("No ROS/monotonic clock bridge observations")
    warnings.append("PX4 timing is an estimated association budget; sensor delay/exposure synchronization remains unmeasured")
    return {"errors": errors, "warnings": warnings, "counts": dict(counts),
            "ever_qualified": monitor.ever_qualified,
            "frames": contract["frames"], "altitude": contract["altitude"]}
