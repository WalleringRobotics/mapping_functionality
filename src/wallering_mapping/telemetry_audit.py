"""ROS telemetry and clock evidence audit without ROS runtime dependencies."""

import base64
import math
import statistics
from collections import Counter

from .dataset import jsonl
from .mavros import stamp_ns
from .telemetry_config import TelemetryConfig
from .timing import ClockTracker, SyncMonitor


def summary_statistics(values):
    if not values:
        return {}
    return {"min": min(values), "median": statistics.median(values),
            "p95": statistics.quantiles(values, n=20, method="inclusive")[-1]
            if len(values) > 1 else values[0], "max": max(values)}


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
    arrivals = {}
    rtts, residuals, offsets = [], [], []
    qualified_samples = max_good = valid_fixes = 0
    connected = False
    for row in jsonl(root / "telemetry.jsonl"):
        if row["record_type"] == "clock":
            if row["target"] != "ros_system":
                raise ValueError("Invalid periodic ROS clock domain")
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
        received = row["received_monotonic_ns"]
        if role not in arrivals:
            arrivals[role] = {"first": received, "last": received, "max_gap_ns": 0}
        else:
            arrival = arrivals[role]
            arrival["max_gap_ns"] = max(arrival["max_gap_ns"], received - arrival["last"])
            arrival["last"] = received
        if stamp_ns(row["fields"]) != row["source_stamp_ros_ns"]:
            errors.append(f"Original ROS header stamp differs from indexed timestamp: {role}")
        if not base64.b64decode(row["cdr_base64"], validate=True):
            errors.append(f"Missing serialized telemetry message: {role}")
        if role == "timesync":
            expected = monitor.observe(row["fields"])
            rtts.append(row["fields"]["round_trip_time_ms"])
            residuals.append(expected["offset_residual_ns"] / 1e6)
            offsets.append(row["fields"]["estimated_offset_ns"])
            qualified_samples += expected["qualification"] == "qualified"
            max_good = max(max_good, expected["consecutive_good"])
            if row["sync_quality"] != expected:
                errors.append("Timesync qualification differs from recorded observations")
        if role == "gnss":
            fields = row["fields"]
            coordinates = [fields.get(key) for key in ("latitude", "longitude", "altitude")]
            valid_fixes += (fields.get("status", {}).get("status", -1) >= 0
                           and all(type(v) in (int, float) and math.isfinite(v) for v in coordinates)
                           and -90 <= coordinates[0] <= 90 and -180 <= coordinates[1] <= 180)
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
    rates = {}
    for role, arrival in arrivals.items():
        count = counts["telemetry_" + role]
        elapsed = (arrival["last"] - arrival["first"]) / 1e9
        rates[role] = {"samples": count, "first_to_last_seconds": elapsed,
                       "observed_hz": (count - 1) / elapsed if elapsed > 0 else None,
                       "max_receipt_gap_ms": arrival["max_gap_ns"] / 1e6}
    return {"errors": errors, "warnings": warnings, "counts": dict(counts),
            "ever_qualified": monitor.ever_qualified,
            "rates": rates, "rates_scope": "Entire saved telemetry interval, including camera warmup",
            "missing_topics": {role: topic for role, topic in config.topics.items()
                               if not counts["telemetry_" + role]},
            "gnss_valid_fix_samples": valid_fixes,
            "timing": {"samples": len(rtts), "qualified_samples": qualified_samples,
                       "gate_diagnostics": monitor.diagnostics(),
                       "max_consecutive_good": max_good, "required_consecutive_good": monitor.minimum,
                       "round_trip_time_ms": summary_statistics(rtts),
                       "offset_residual_ms": summary_statistics(residuals),
                       "estimated_offset_ns": {"first": offsets[0], "last": offsets[-1],
                                               "range": max(offsets) - min(offsets)} if offsets else {},
                       "note": "ROS epoch minus PX4 boot clock; offset is not UTC error"},
            "frames": contract["frames"], "altitude": contract["altitude"]}
