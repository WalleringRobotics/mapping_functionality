#!/usr/bin/env python3
"""Exercise a generated Plan in a self-owned, network-isolated upstream PX4 SITL.

This is a simulator harness, never a vehicle connection tool. It only runs inside
a container with loopback networking and starts its own PX4 process. It retains
the downloaded mission, MAVLink observations and original PX4 logs. No camera or
ROS recorder acceptance is implied by a successful mission.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require_isolation():
    if not Path("/.dockerenv").exists():
        raise ValueError("SITL harness requires a dedicated Docker container")
    interfaces = {p.name for p in Path("/sys/class/net").iterdir()}
    if interfaces != {"lo"}:
        raise ValueError("SITL harness requires --network none (only loopback may exist)")
    if any(Path("/dev").glob("ttyACM*")) or any(Path("/dev").glob("ttyUSB*")):
        raise ValueError("Physical serial devices must not be mounted into the SITL container")


def mission_items(plan):
    """Convert the generator's simple items to MAVLink, preserving NaN yaw."""
    if (plan.get("fileType") != "Plan" or plan.get("version") != 1
            or plan.get("mission", {}).get("version") != 2
            or plan["mission"].get("firmwareType") != 12
            or plan["mission"].get("vehicleType") != 2):
        raise ValueError("Expected QGC Plan v1 / Mission v2 for a PX4 quadrotor")
    items = plan["mission"].get("items", [])
    if not items or len(items) > 200:
        raise ValueError("Expected one to 200 generated simple mission items")
    result = []
    for seq, item in enumerate(items):
        params = item.get("params", [])
        if (item.get("type") != "SimpleItem" or item.get("command") not in {16, 21, 22, 178}
                or item.get("doJumpId") != seq + 1 or item.get("autoContinue") is not True
                or len(params) != 7):
            raise ValueError(f"Unsupported or incorrectly numbered simple item {seq}")
        if any(v is not None and (type(v) not in (int, float) or not math.isfinite(v))
               for v in params):
            raise ValueError(f"Nonfinite or nonnumeric mission parameter at {seq}")
        navigation = item["command"] in {16, 21, 22}
        if item.get("frame") != (3 if navigation else 2):
            raise ValueError(f"Unexpected coordinate frame at {seq}")
        if navigation and (any(v is None for v in params[4:])
                           or not -80 <= params[4] <= 80 or not -180 <= params[5] <= 180
                           or not 0 <= params[6] <= 120):
            raise ValueError(f"Invalid navigation coordinate at {seq}")
        result.append({"seq": seq, "frame": item["frame"], "command": item["command"],
                       "autocontinue": 1, "params": params})
    return result


def comparable_download(message):
    integer = message.get_type() == "MISSION_ITEM_INT"
    frame = {5: 0, 6: 3, 11: 10}.get(message.frame, message.frame)
    return {"seq": message.seq, "frame": frame, "command": message.command,
            "autocontinue": message.autocontinue,
            "params": [None if math.isnan(v) else float(v) for v in
                       (message.param1, message.param2, message.param3, message.param4)]
                      + [message.x / 1e7 if integer and frame in {0, 3, 10} else message.x,
                         message.y / 1e7 if integer and frame in {0, 3, 10} else message.y,
                         message.z]}


def check_download(expected, actual):
    if len(expected) != len(actual):
        raise ValueError("Downloaded mission count differs from uploaded Plan")
    for before, after in zip(expected, actual):
        if any(before[k] != after[k] for k in ("seq", "frame", "command", "autocontinue")):
            raise ValueError(f"Downloaded mission item identity differs at {before['seq']}")
        for index, (a, b) in enumerate(zip(before["params"], after["params"])):
            tolerance = 1e-7 if index in {4, 5} and before["frame"] in {0, 3, 10} else 1e-4
            if (a is None) != (b is None) or (a is not None and abs(a - b) > tolerance):
                raise ValueError(f"Downloaded mission parameter differs at {before['seq']}")


class Link:
    def __init__(self, events):
        os.environ["MAVLINK20"] = "1"
        from pymavlink import mavutil

        self.m = mavutil.mavlink
        self.connection = mavutil.mavlink_connection("udpin:127.0.0.1:14550",
                                                    source_system=255, source_component=190)
        self.events = events
        self.last_heartbeat = 0
        self.latest = {}
        self.reached = set()

    def poll(self, timeout=.2):
        now = time.monotonic()
        if now - self.last_heartbeat >= 1:
            self.connection.mav.heartbeat_send(self.m.MAV_TYPE_GCS,
                self.m.MAV_AUTOPILOT_INVALID, 0, 0, self.m.MAV_STATE_ACTIVE)
            self.last_heartbeat = now
        message = self.connection.recv_match(blocking=True, timeout=timeout)
        if message is not None and message.get_type() != "BAD_DATA":
            self.latest[message.get_type()] = message
            if message.get_type() == "MISSION_ITEM_REACHED":
                self.reached.add(message.seq)
            row = message.to_dict()
            # MAVLink NaNs denote unspecified parameters; JSON stores them as null.
            row = {k: None if isinstance(v, float) and not math.isfinite(v) else v
                   for k, v in row.items()}
            self.events.write(json.dumps({"received_monotonic_s": now, "message": row},
                                         allow_nan=False) + "\n")
        return message

    def wait(self, types, timeout=10, predicate=lambda message: True):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.poll()
            if msg is not None and msg.get_type() in types and predicate(msg):
                return msg
        raise TimeoutError("Timed out waiting for " + ", ".join(types))

    def command(self, command, *params):
        self.connection.mav.command_long_send(1, 1, command, 0,
                                               *(list(params) + [0] * (7 - len(params))))
        ack = self.wait({"COMMAND_ACK"}, predicate=lambda m: m.command == command)
        if ack.result != self.m.MAV_RESULT_ACCEPTED:
            raise ValueError(f"PX4 rejected command {command}: ACK {ack.result}")

    def send_item(self, item, mission_type, integer=True):
        p = [float("nan") if value is None else value for value in item["params"]]
        frame = item["frame"]
        if integer:
            if frame in {0, 3, 10}:
                p[4:6] = [round(v * 1e7) for v in p[4:6]]
            else:
                p[4:6] = [round(v) for v in p[4:6]]
            frame = {0: 5, 3: 6, 10: 11}.get(frame, frame)
            send = self.connection.mav.mission_item_int_send
        else:
            send = self.connection.mav.mission_item_send
        send(1, 1, item["seq"], frame, item["command"], 0, item["autocontinue"],
             *p, mission_type=mission_type)

    def upload(self, items, mission_type=0):
        self.connection.mav.mission_count_send(1, 1, len(items), mission_type=mission_type)
        deadline = time.monotonic() + 60
        sent = set()
        while time.monotonic() < deadline:
            message = self.wait({"MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"},
                predicate=lambda m: getattr(m, "mission_type", 0) == mission_type)
            if message.get_type() == "MISSION_ACK":
                if message.type != self.m.MAV_MISSION_ACCEPTED or len(sent) != len(items):
                    raise ValueError(f"PX4 refused/incompletely accepted mission: {message}")
                return
            if not 0 <= message.seq < len(items):
                raise ValueError("PX4 requested a mission index outside the Plan")
            self.send_item(items[message.seq], mission_type,
                           integer=message.get_type() == "MISSION_REQUEST_INT")
            sent.add(message.seq)
        raise TimeoutError("Mission upload timed out")

    def download(self, mission_type=0):
        self.connection.mav.mission_request_list_send(1, 1, mission_type=mission_type)
        count = self.wait({"MISSION_COUNT"}, predicate=lambda m:
                          getattr(m, "mission_type", 0) == mission_type).count
        items = []
        for index in range(count):
            self.connection.mav.mission_request_int_send(1, 1, index, mission_type=mission_type)
            message = self.wait({"MISSION_ITEM_INT", "MISSION_ITEM"}, predicate=lambda m:
                m.seq == index and getattr(m, "mission_type", 0) == mission_type)
            items.append(comparable_download(message))
        self.connection.mav.mission_ack_send(1, 1, self.m.MAV_MISSION_ACCEPTED,
                                              mission_type=mission_type)
        return items


def run(args):
    require_isolation()
    build = args.px4_build.resolve()
    binary, romfs = build / "bin/px4", build / "etc"
    if not binary.is_file() or not (romfs / "init.d-posix/airframes/10040_sihsim_quadx").is_file():
        raise ValueError("Expected an upstream PX4 v1.17 SITL build with SIH quadrotor")
    plan = json.loads(args.plan.read_text())
    items = mission_items(plan)
    home = plan["mission"]["plannedHomePosition"]
    fence = plan["geoFence"]["circles"]
    if len(fence) != 1 or fence[0].get("inclusion") is not True:
        raise ValueError("Expected the generator's single inclusion-circle fence")
    circle = fence[0]["circle"]
    fence_items = [{"seq": 0, "frame": 0, "command": 5003, "autocontinue": 1,
                    "params": [circle["radius"], 0, 0, 0, *circle["center"], 0]}]
    output = args.output.resolve()
    if args.plan.resolve().is_relative_to(output) or build.is_relative_to(output):
        raise ValueError("Output must be separate from Plan and build inputs")
    output.mkdir(parents=True, exist_ok=False)
    rootfs = output / "px4-root"
    rootfs.mkdir()
    env = {k: v for k, v in os.environ.items() if not k.startswith("PX4_")}
    env.update(PX4_SIM_MODEL="sihsim_quadx", PX4_PARAM_SIH_LOC_LAT0=str(home[0]),
               PX4_PARAM_SIH_LOC_LON0=str(home[1]), PX4_PARAM_SIH_LOC_H0=str(home[2]),
               PX4_SIM_SPEED_FACTOR="1")
    command = [str(binary), "-d", "-w", str(rootfs),
               "-s", "etc/init.d-posix/rcS", str(romfs)]
    report = {"schema_version": 1, "kind": "real_px4_sih_sitl", "passed": False,
              "plan_sha256": sha256(args.plan), "px4_binary_sha256": sha256(binary),
              "command": command, "mode": args.mode, "network": "loopback-only container",
              "survey_ready": False, "qgc_runtime_validated": False,
              "recorder_solver_pipeline_validated": False,
              "limitations": ["Synthetic vehicle dynamics, not real-flight evidence",
                "No synthetic camera images or ROS recorder/solver run is included",
                "Fence is round-trip checked; enforcement/vertical limits are not qualified",
                "MAVLink mode switch is not an RC-stick override verification",
                "Waypoint completion does not measure yaw settling, hold timing or axis excitation"]}
    process = None
    link = None
    try:
        with (output / "px4.log").open("w") as console, (output / "mavlink.jsonl").open("w") as events:
            link = Link(events)
            process = subprocess.Popen(command, cwd=rootfs, env=env, stdout=console,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            hb = link.wait({"HEARTBEAT"}, timeout=45, predicate=lambda m:
                           m.autopilot == link.m.MAV_AUTOPILOT_PX4)
            if hb.get_srcSystem() != 1 or hb.type != link.m.MAV_TYPE_QUADROTOR:
                raise ValueError("Unexpected simulated PX4 system identity")
            report["heartbeat"] = hb.to_dict()
            link.command(link.m.MAV_CMD_REQUEST_MESSAGE, link.m.MAVLINK_MSG_ID_AUTOPILOT_VERSION)
            version = link.latest.get("AUTOPILOT_VERSION") or link.wait({"AUTOPILOT_VERSION"})
            report["autopilot_version"] = version.to_dict()
            # Give the upstream GPS/EKF preflight checks their normal convergence time.
            settle = time.monotonic() + 15
            while time.monotonic() < settle:
                link.poll()
            link.upload(fence_items, link.m.MAV_MISSION_TYPE_FENCE)
            downloaded_fence = link.download(link.m.MAV_MISSION_TYPE_FENCE)
            check_download(fence_items, downloaded_fence)
            (output / "downloaded-fence.json").write_text(json.dumps(downloaded_fence, indent=2) + "\n")
            link.upload(items)
            downloaded = link.download()
            check_download(items, downloaded)
            (output / "downloaded-mission.json").write_text(json.dumps(downloaded, indent=2) + "\n")
            report["mission_roundtrip_verified"] = True
            report["fence_roundtrip_verified"] = True
            link.command(link.m.MAV_CMD_DO_SET_MODE, 1, 4, 4)  # PX4 AUTO/MISSION
            link.command(link.m.MAV_CMD_COMPONENT_ARM_DISARM, 1)  # this isolated simulator only
            deadline = time.monotonic() + args.timeout
            took_off, abort_requested, hold_verified = False, False, False
            positions = []
            landed = False
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError("PX4 exited before mission completion")
                message = link.poll()
                if message is None:
                    continue
                if message.get_type() == "GLOBAL_POSITION_INT":
                    positions.append({"lat": message.lat / 1e7, "lon": message.lon / 1e7,
                                      "relative_alt_m": message.relative_alt / 1000,
                                      "horizontal_speed_m_s": math.hypot(message.vx, message.vy) / 100})
                    took_off |= message.relative_alt > 2000
                if args.mode == "abort" and took_off and not abort_requested:
                    link.command(link.m.MAV_CMD_DO_SET_MODE, 1, 4, 3)  # AUTO/LOITER
                    abort_requested = True
                if (abort_requested and not hold_verified and message.get_type() == "HEARTBEAT"
                        and (message.custom_mode >> 24) & 255 == 3):
                    hold_verified = True
                    report["hold_mode_observed"] = True
                    link.command(link.m.MAV_CMD_NAV_LAND)
                if (took_off and message.get_type() == "HEARTBEAT"
                        and not message.base_mode & link.m.MAV_MODE_FLAG_SAFETY_ARMED):
                    landed = True
                    break
            report.update(reached_sequences=sorted(link.reached), took_off=took_off,
                          landed_and_disarmed=landed, position_samples=len(positions))
            if positions:
                report["maximum_relative_alt_m"] = max(p["relative_alt_m"] for p in positions)
                report["maximum_horizontal_speed_m_s"] = max(p["horizontal_speed_m_s"] for p in positions)
            navigation = {i["seq"] for i in items if i["command"] in {16, 22}}
            missing = sorted(navigation - link.reached)
            report["unreached_navigation_sequences"] = missing
            report["passed"] = (took_off and landed and
                                  (hold_verified if args.mode == "abort" else not missing))
            if not report["passed"]:
                raise RuntimeError("SITL mission/abort did not satisfy completion checks")
    except Exception as error:
        report["error"] = f"{type(error).__name__}: {error}"
    finally:
        if link is not None:
            link.connection.close()
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        report["artifacts"] = {str(p.relative_to(output)): sha256(p)
                               for p in sorted(output.rglob("*")) if p.is_file() and not p.is_symlink()}
        (output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"passed": report["passed"], "output": str(output),
                      "error": report.get("error")}))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--px4-build", required=True, type=Path)
    parser.add_argument("--plan", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--mode", choices=("mission", "abort"), default="mission")
    parser.add_argument("--timeout", type=float, default=900)
    arguments = parser.parse_args()
    if not 60 <= arguments.timeout <= 1800:
        parser.error("--timeout must be between60 and1800 seconds")
    raise SystemExit(run(arguments))
