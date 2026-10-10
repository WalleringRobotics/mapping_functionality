"""Recording guidance and bounded, auditable PX4 telemetry requests."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import time


CALIBRATION_PHASES = ((10, "still_start", "Keep the rig still for 10 seconds."),) + tuple(
    (seconds, f"{axis}_{cycle}",
     f"Cycle {cycle}/3: about three smooth {axis} sweeps through +/-30 degrees "
     "(one full swing per 2 seconds), within comfortable hand motion.")
    for cycle in range(1, 4)
    for seconds, axis in ((6, "roll"), (7, "pitch"), (7, "yaw"))
) + (
    (30, "free", "Rotate smoothly about multiple axes for 30 seconds."),
    (10, "still_end", "Keep the rig still for the final 10 seconds."),
)
CALIBRATION_DURATION = sum(phase[0] for phase in CALIBRATION_PHASES)
PX4_IMU_MESSAGES = {105: "/mavros/imu/data_raw", 31: "/mavros/imu/data"}
SURVEY_FILES = ("survey-check.json", "survey.plan")
# STATUSTEXT for QGroundControl's message panel and a PX4 buzzer tune (QBASIC1_1).
ANNOUNCEMENTS = {"started": ("WR mapping: recording started", "MFT200L16O4CEG"),
                 "stopped": ("WR mapping: recording stopped", "MFT200L16O4GEC")}


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


def verify_survey(directory, profile=None):
    """Return the checked plan hash, refusing a failed, foreign or modified check."""
    directory = Path(directory)
    report = json.loads((directory / SURVEY_FILES[0]).read_text())
    if report.get("schema_version") != 1 or report.get("kind") != "survey_plan_check":
        raise ValueError("Not a survey-check report")
    if report.get("passed") is not True:
        raise ValueError("The survey plan did not pass survey-check")
    digest = hashlib.sha256((directory / SURVEY_FILES[1]).read_bytes()).hexdigest()
    if digest != report.get("plan", {}).get("sha256"):
        raise ValueError("survey.plan differs from the plan that was checked")
    if profile is not None:
        if report.get("platform", "px4_multirotor") != "px4_multirotor":
            raise ValueError("OAK/PX4 recorder cannot capture the proposed Plane/GigE profile")
        actual = hashlib.sha256(Path(profile).read_bytes()).hexdigest()
        if actual != report.get("inputs", {}).get("profile_sha256"):
            raise ValueError("Active capture profile differs from the checked profile; recheck the plan")
    if report.get("platform") == "ardupilot_plane":
        for name, key in (("mapping_handoff.json", "handoff_sha256"),
                          ("aircraft-limits.json", "aircraft_sha256")):
            actual = hashlib.sha256((directory / name).read_bytes()).hexdigest()
            if actual != report.get("inputs", {}).get(key):
                raise ValueError(f"Stale checked profile hash: {name}")
    return digest


def write_report(path, report):
    with Path(path).open("x") as out:
        json.dump(report, out, indent=2)
        out.write("\n")
    return report


def pull_mission(report_path, call):
    """Download the vehicle mission once so the recording holds the list PX4 will fly.

    A pull is a read-only MAVLink mission download. Failure is recorded, not raised:
    survey legs then stay unverified instead of the flight going unrecorded.
    """
    report = {"schema_version": 1, "service": "/mavros/mission/pull",
              "requested_utc_ns": time.time_ns(), "success": False}
    try:
        response = call()
        report.update(success=bool(response.success), wp_received=int(response.wp_received))
    except Exception as error:
        report["error"] = f"{type(error).__name__}: {error}"
    report["finished_utc_ns"] = time.time_ns()
    return write_report(report_path, report)


def announce(event, report_path, publish):
    """Tell the operator through QGroundControl and the vehicle buzzer; never raise."""
    text, tune = ANNOUNCEMENTS[event]
    report = {"schema_version": 1, "event": event, "text": text, "tune": tune,
              "requested_utc_ns": time.time_ns(), "statustext": False, "play_tune": False}
    try:
        report.update(publish(text, tune))
    except Exception as error:
        report["error"] = f"{type(error).__name__}: {error}"
    report["finished_utc_ns"] = time.time_ns()
    return write_report(report_path, report)


def ros_mission_pull(report):
    import rclpy
    from mavros_msgs.srv import WaypointPull
    rclpy.init()
    node = rclpy.create_node("mapping_mission_pull")
    client = node.create_client(WaypointPull, "/mavros/mission/pull")
    try:
        def call():
            if not client.wait_for_service(timeout_sec=5.):
                raise TimeoutError("MAVROS /mavros/mission/pull is unavailable")
            future = client.call_async(WaypointPull.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=30.)
            if not future.done():
                future.cancel()
                raise TimeoutError("Mission download timed out")
            return future.result()
        return pull_mission(report, call)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def ros_announce(event, report, wait_seconds=2.):
    import rclpy
    from mavros_msgs.msg import PlayTuneV2, StatusText
    rclpy.init()
    node = rclpy.create_node("mapping_announce")
    text_publisher = node.create_publisher(StatusText, "/mavros/statustext/send", 10)
    tune_publisher = node.create_publisher(PlayTuneV2, "/mavros/play_tune", 1)
    try:
        def publish(text, tune):
            deadline = time.monotonic() + wait_seconds
            while time.monotonic() < deadline and not (
                    text_publisher.get_subscription_count() and tune_publisher.get_subscription_count()):
                rclpy.spin_once(node, timeout_sec=.1)
            sent = {"statustext": text_publisher.get_subscription_count() > 0,
                    "play_tune": tune_publisher.get_subscription_count() > 0}
            if sent["statustext"]:
                message = StatusText(severity=StatusText.NOTICE, text=text)
                message.header.stamp = node.get_clock().now().to_msg()
                text_publisher.publish(message)
            if sent["play_tune"]:
                tune_publisher.publish(PlayTuneV2(format=PlayTuneV2.QBASIC1_1, tune=tune))
            rclpy.spin_once(node, timeout_sec=.3)
            return sent
        return announce(event, report, publish)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--rate", type=float)
    action.add_argument("--pull-mission", action="store_true")
    action.add_argument("--announce", choices=sorted(ANNOUNCEMENTS))
    action.add_argument("--verify-survey", type=Path)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    if args.verify_survey:
        print(verify_survey(args.verify_survey, args.profile))
        return 0
    if args.report is None:
        parser.error("--report is required")
    if args.pull_mission:
        ros_mission_pull(args.report)
        return 0
    if args.announce:
        ros_announce(args.announce, args.report)
        return 0
    return 0 if ros_rate_request(args.rate, args.report)["success"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
