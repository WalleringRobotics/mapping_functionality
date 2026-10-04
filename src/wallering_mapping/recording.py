"""Recording guidance and bounded, auditable PX4 telemetry requests."""

import argparse
import json
import math
from pathlib import Path
import time


CALIBRATION_PHASES = ((10, "still_start", "Keep the rig still for 10 seconds."),) + tuple(
    (seconds, f"{axis}_{cycle}",
     f"Cycle {cycle}/3: one smooth {axis} sweep through +/-30 degrees, then return to centre.")
    for cycle in range(1, 4)
    for seconds, axis in ((6, "roll"), (7, "pitch"), (7, "yaw"))
) + (
    (30, "free", "Rotate smoothly about multiple axes for 30 seconds."),
    (10, "still_end", "Keep the rig still for the final 10 seconds."),
)
CALIBRATION_DURATION = sum(phase[0] for phase in CALIBRATION_PHASES)
PX4_IMU_MESSAGES = {105: "/mavros/imu/data_raw", 31: "/mavros/imu/data"}


def phase_record(index, started_ns):
    seconds, name, instruction = CALIBRATION_PHASES[index]
    return {"phase": name, "instruction": instruction, "duration_seconds": seconds,
            "started_utc_ns": started_ns,
            "planned_offset_seconds": sum(p[0] for p in CALIBRATION_PHASES[:index])}


def request_imu_rate(rate_hz, report_path, call):
    """Call an injected CommandLong transport, preserving partial failure evidence.

    Zero restores firmware default intervals, not an unknown previous override.
    Every request is flushed before sending so interrupted calls remain auditable.
    """
    if not math.isfinite(rate_hz) or not 0 <= rate_hz <= 200:
        raise ValueError("PX4 IMU rate must be between 0 and 200 Hz")
    report = {"schema_version": 1, "requested_hz": rate_hz, "command": 511,
              "restore_policy": "firmware_default_interval", "requests": [], "success": False}
    path = Path(report_path)
    with path.open("x") as out:
        def save():
            out.seek(0)
            json.dump(report, out, indent=2)
            out.write("\n")
            out.truncate()
            out.flush()
        save()
        for message_id, topic in PX4_IMU_MESSAGES.items():
            row = {"message_id": message_id, "topic": topic, "requested_hz": rate_hz,
                   "interval_us": round(1e6 / rate_hz) if rate_hz else 0,
                   "requested_utc_ns": time.time_ns(), "success": False}
            report["requests"].append(row)
            save()
            try:
                response = call(message_id, row["interval_us"])
                row.update(ack_result=int(response.result),
                           success=bool(response.success) and int(response.result) == 0)
                if not row["success"]:
                    row["error"] = "PX4 rejected the interval request"
            except Exception as error:
                row["error"] = f"{type(error).__name__}: {error}"
            row["finished_utc_ns"] = time.time_ns()
            save()
        report["success"] = all(row["success"] for row in report["requests"])
        save()
    return report


def ros_rate_request(rate_hz, report):
    import rclpy
    from mavros_msgs.srv import CommandLong
    rclpy.init()
    node = rclpy.create_node("mapping_imu_rate_request")
    client = node.create_client(CommandLong, "/mavros/cmd/command")
    try:
        def call(message_id, interval_us):
            if not client.wait_for_service(timeout_sec=5.):
                raise TimeoutError("MAVROS /mavros/cmd/command is unavailable")
            request = CommandLong.Request(command=511, broadcast=False, confirmation=0,
                                          param1=float(message_id), param2=float(interval_us))
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=8.)
            if not future.done():
                future.cancel()
                raise TimeoutError("PX4 interval acknowledgement timed out")
            return future.result()
        return request_imu_rate(rate_hz, report, call)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate", type=float, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    return 0 if ros_rate_request(args.rate, args.report)["success"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
