"""Small command-line surface with optional hardware dependencies."""

import argparse
import json
import subprocess
import sys
from pathlib import Path

from .config import CaptureConfig


def positive(value):
    number = float(value)
    if not 0 < number < float("inf"):
        raise argparse.ArgumentTypeError("must be finite and positive")
    return number


def nonnegative_integer(value):
    number = int(value)
    if number < 0:
        raise argparse.ArgumentTypeError("must be a nonnegative integer")
    return number


def parser():
    result = argparse.ArgumentParser(prog="wr-map")
    commands = result.add_subparsers(dest="command", required=True)
    inspect = commands.add_parser("inspect", help="Probe OAK sensors, IMU and USB link")
    inspect.add_argument("--device-id")
    record = commands.add_parser("capture", aliases=["record"], help="Record official ROS drivers with rosbag2/MCAP")
    record.add_argument("--config", type=Path, default=Path("configs/oakd-ros.yaml"))
    record.add_argument("--output", type=Path, required=True)
    record.add_argument("--device-id")
    record.add_argument("--duration", type=nonnegative_integer, default=60, help="Recording seconds; 0 until Ctrl+C")
    record.add_argument("--warmup", type=nonnegative_integer, default=5)
    record.add_argument("--require-mount", type=Path)
    ownership = record.add_mutually_exclusive_group()
    ownership.add_argument("--camera-only", action="store_true")
    ownership.add_argument("--start-mavros", action="store_true", help="Own MAVROS for this session; otherwise reuse it")
    record.add_argument("--fcu-url", default="/dev/ttyUSB0:921600")
    bag_import = commands.add_parser("bag-import", help="Offline lossless image import from a sealed ROS recording")
    bag_import.add_argument("session", type=Path)
    bag_import.add_argument("--output", type=Path, required=True)
    record = commands.add_parser("legacy-capture", help="Legacy direct-SDK diagnostic writer (not the recording stack)")
    record.add_argument("--config", type=Path, required=True)
    record.add_argument("--output", type=Path, required=True)
    record.add_argument("--device-id")
    record.add_argument("--telemetry-config", type=Path, help="Subscribe to the existing PX4/MAVROS connector")
    record.add_argument("--duration", type=positive, help="Seconds after warmup; otherwise until Ctrl+C")
    doctor = commands.add_parser("doctor", help="Check processing or legacy SDK dependencies/storage")
    doctor.add_argument("--mode", choices=["capture", "process"], required=True)
    doctor.add_argument("--config", type=Path, default=Path("configs/oakd-survey.json"))
    doctor.add_argument("--output-root", type=Path, required=True)
    doctor.add_argument("--probe-device", action="store_true")
    doctor.add_argument("--device-id")
    doctor.add_argument("--backend", choices=["building", "terrain"], default="building")
    hardware = commands.add_parser("hardware-check", help="Run legacy SDK or processing startup probes")
    hardware.add_argument("--mode", choices=["capture", "building", "terrain"], default="capture")
    hardware.add_argument("--config", type=Path, default=Path("configs/oakd-survey.json"))
    hardware.add_argument("--output-root", type=Path, required=True)
    hardware.add_argument("--require-jetson", action="store_true")
    hardware.add_argument("--require-mount", type=Path)
    hardware.add_argument("--telemetry-config", type=Path)
    hardware.add_argument("--gnss-profile", type=Path)
    hardware.add_argument("--device-id")
    hardware.add_argument("--serial-device", type=Path)
    hardware.add_argument("--camera-seconds", type=positive, default=5)
    hardware.add_argument("--telemetry-seconds", type=positive, default=75)
    hardware.add_argument("--min-memory-gib", type=positive, default=1)
    hardware.add_argument("--report", type=Path)
    serial_check = commands.add_parser("serial-check", help="Passively listen for MAVLink at selected baud rates")
    serial_check.add_argument("--device", type=Path, default=Path("/dev/ttyUSB0"))
    serial_check.add_argument("--baud", type=int, nargs="+", default=[115200], help="One or more baud rates, checked sequentially")
    serial_check.add_argument("--seconds", type=positive, default=20, help="Listen seconds per baud, at most 300")
    serial_check.add_argument("--report", type=Path)
    status = commands.add_parser("status", help="Read the recorder's current status")
    status.add_argument("session", type=Path)
    simulate = commands.add_parser("simulate", help="Create an IO fixture without hardware")
    simulate.add_argument("--output", type=Path, required=True)
    simulate.add_argument("--frames", type=int, default=12)
    validate = commands.add_parser("validate", help="Audit dataset integrity, gaps and timing")
    validate.add_argument("session", type=Path)
    validate.add_argument("--report", type=Path)
    validate.add_argument("--rig-calibration", type=Path, help="Compare bag IMU timing/mounting against this calibration")
    validate.add_argument("--calibration-warn-sigma", type=positive, default=3.0)
    validate.add_argument("--calibration-fail-sigma", type=positive, default=5.0)
    sync = commands.add_parser("sync", help="Audit exposure-to-PX4 timing and associate vehicle poses")
    sync.add_argument("session", type=Path)
    sync.add_argument("--output", type=Path, required=True)
    sync.add_argument("--stream", choices=["rgb", "left", "right"], default="rgb")
    sync.add_argument("--min-fraction", type=float, default=.9)
    sync.add_argument("--rig-calibration", type=Path, help="Compose diagnostic camera poses with explicit timing qualification limits")
    ntrip = commands.add_parser("ntrip", help="Forward verified fixed-base NTRIP v2 corrections through MAVROS")
    ntrip.add_argument("--config", type=Path, required=True)
    ntrip.add_argument("--output", type=Path, required=True)
    ntrip.add_argument("--duration", type=positive)
    images_accuracy = commands.add_parser("image-accuracy", help="Report corrected-GNSS camera uncertainty and qualified geolocation")
    images_accuracy.add_argument("session", type=Path)
    images_accuracy.add_argument("--alignment", type=Path, required=True)
    images_accuracy.add_argument("--profile", type=Path, required=True)
    images_accuracy.add_argument("--output", type=Path, required=True)
    images_accuracy.add_argument("--project", type=Path, help="Restrict geolocation to an existing selected image export")
    images_accuracy.add_argument("--rig-calibration", type=Path, help="Use lever arm and uncertainty from the same rig file as sync")
    map_accuracy = commands.add_parser("map-accuracy", help="Assess withheld checkpoints and reference-aware map accuracy")
    map_accuracy.add_argument("checkpoints", type=Path)
    map_accuracy.add_argument("--profile", type=Path, required=True)
    map_accuracy.add_argument("--output", type=Path, required=True)
    map_accuracy.add_argument("--image-accuracy", type=Path)
    map_accuracy.add_argument("--workflow", type=Path)
    map_accuracy.add_argument("--georeference", type=Path)
    georef = commands.add_parser("georeference", help="Align COLMAP and PLY products using qualified camera GNSS positions")
    georef.add_argument("project", type=Path)
    georef.add_argument("--model", type=Path, required=True)
    georef.add_argument("--image-accuracy", type=Path, required=True)
    georef.add_argument("--output", type=Path, required=True)
    georef.add_argument("--artifact", type=Path, action="append", default=[])
    georef.add_argument("--max-residual-m", type=positive, default=0.15)
    rig = commands.add_parser("calibration-check", help="Validate a rig calibration file and compose camera/antenna transforms")
    rig.add_argument("calibration", type=Path)
    rig.add_argument("--session", type=Path, help="Recording whose OAK factory calibration to use and match")
    rig.add_argument("--require-complete", action="store_true", help="Exit 2 while any link or uncertainty is unset")
    solve = commands.add_parser("calibrate-solve", help="Estimate OAK-to-PX4 clock offset and mounting rotation from a motion session")
    solve.add_argument("session", type=Path)
    solve.add_argument("--calibration", type=Path, required=True, help="Rig calibration to start from (not modified)")
    solve.add_argument("--output", type=Path, required=True, help="New directory for the updated calibration and report")
    solve.add_argument("--max-offset-ms", type=float, default=200.0)
    camera_solve = commands.add_parser("calibrate-camera", help="Estimate camera-to-IMU rotation and timing with a stereo cross-check")
    camera_solve.add_argument("session", type=Path)
    camera_solve.add_argument("--calibration", type=Path, required=True)
    camera_solve.add_argument("--output", type=Path, required=True)
    camera_solve.add_argument("--camera", choices=["left", "right"], default="left")
    camera_solve.add_argument("--max-offset-ms", type=positive, default=50.0)
    export = commands.add_parser("export", help="Export one camera for COLMAP or ODM")
    export.add_argument("session", type=Path)
    export.add_argument("--output", type=Path, required=True)
    export.add_argument("--stream", choices=["rgb", "left", "right"], default="rgb")
    export.add_argument("--interval", type=float, default=1.0)
    export.add_argument("--min-sharpness", type=float, default=0.0)
    export.add_argument("--allow-gaps", action="store_true")
    process = commands.add_parser("process", help="Mode 2: resumable offline photogrammetry")
    process.add_argument("session", type=Path)
    process.add_argument("--config", type=Path, required=True)
    process.add_argument("--output", type=Path, required=True)
    action = process.add_mutually_exclusive_group()
    action.add_argument("--execute", action="store_true")
    action.add_argument("--prepare-only", action="store_true")
    process.add_argument("--resume", action="store_true")
    process.add_argument("--gcp", type=Path)
    process.add_argument("--geo", type=Path)
    process.add_argument("--vertical-datum")
    sparse = commands.add_parser("reconstruct", help="Plan/run COLMAP sparse reconstruction")
    sparse.add_argument("project", type=Path)
    sparse.add_argument("--output", type=Path, required=True)
    sparse.add_argument("--matcher", choices=["exhaustive", "sequential"], default="exhaustive")
    sparse.add_argument("--cpu", action="store_true")
    sparse.add_argument("--execute", action="store_true")
    dense = commands.add_parser("dense", help="Plan/run MVS for an explicitly selected sparse model")
    dense.add_argument("project", type=Path)
    dense.add_argument("--model", type=Path, required=True)
    dense.add_argument("--output", type=Path, required=True)
    dense.add_argument("--max-image-size", type=int, default=2000)
    dense.add_argument("--execute", action="store_true")
    accuracy = commands.add_parser("accuracy", help="Report independent checkpoint residuals")
    accuracy.add_argument("checkpoints", type=Path)
    from .calibration_mission import add_arguments
    mission = commands.add_parser("calibrate-mission", help="Generate a bounded offline QGroundControl mission draft")
    add_arguments(mission)
    calibration = commands.add_parser("calibrate", help="Record, solve and verify rig calibration")
    modes = calibration.add_subparsers(dest="calibration_command", required=True)
    for name in ("record", "solve", "camera", "mission", "phases"):
        flat = f"calibrate-{name}"
        if flat in commands.choices:
            mode = modes.add_parser(name, parents=[commands.choices[flat]], add_help=False)
            mode.set_defaults(command=flat)
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if args.command == "calibrate-mission":
            from .calibration_mission import run
            result = run(args)
        elif args.command == "inspect":
            from .oak import inspect_device
            result = inspect_device(args.device_id)
        elif args.command in {"record", "capture"}:
            import os
            repo = Path(__file__).resolve().parents[2]
            launcher = repo / "deploy/run-ros.sh"
            if not launcher.is_file():
                raise ValueError("ROS capture requires this repository checkout (install with pip -e)")
            command = ["bash", str(launcher), "bash", str(repo / "deploy/record-rosbag.sh"),
                       "--output", str(args.output.resolve()), "--config", str(args.config.resolve()),
                       "--duration", str(args.duration), "--warmup", str(args.warmup),
                       "--fcu-url", args.fcu_url]
            for flag, value in (("--device-id", args.device_id), ("--require-mount", args.require_mount)):
                if value:
                    command.extend([flag, str(value)])
            if args.camera_only:
                command.append("--camera-only")
            if args.start_mavros:
                command.append("--start-mavros")
            os.execvp("bash", command)
        elif args.command == "legacy-capture":
            from .oak import record
            from .telemetry_config import TelemetryConfig
            telemetry = TelemetryConfig.read(args.telemetry_config) if args.telemetry_config else None
            result = record(args.output, CaptureConfig.read(args.config), args.duration, args.device_id, telemetry)
        elif args.command == "doctor":
            from .operations import doctor
            config = CaptureConfig.read(args.config) if args.mode == "capture" else None
            result = doctor(args.mode, args.output_root, config, args.probe_device,
                            args.device_id, args.backend)
            print(json.dumps(result, indent=2))
            return 0 if result["ready"] else 2
        elif args.command == "hardware-check":
            from .hardware import hardware_check
            from .telemetry_config import TelemetryConfig
            if args.mode != "capture" and (args.telemetry_config or args.gnss_profile or args.device_id):
                raise ValueError("Camera/telemetry options require --mode capture")
            result = hardware_check(
                args.mode, args.output_root,
                CaptureConfig.read(args.config) if args.mode == "capture" else None,
                require_jetson=args.require_jetson, required_mount=args.require_mount,
                telemetry_config=TelemetryConfig.read(args.telemetry_config) if args.telemetry_config else None,
                gnss_profile=args.gnss_profile, device_id=args.device_id,
                camera_seconds=args.camera_seconds, telemetry_seconds=args.telemetry_seconds,
                min_memory_gib=args.min_memory_gib, serial_device=args.serial_device)
            encoded = json.dumps(result, indent=2) + "\n"
            if args.report:
                # Never replace an earlier acceptance report or a deployment config.
                with args.report.open("x") as report:
                    report.write(encoded)
            print(encoded, end="")
            return 0 if result["ready"] else 2
        elif args.command == "serial-check":
            from .serial_diagnostics import check_serial
            if args.report:
                if args.report.exists():
                    raise FileExistsError(f"Report already exists: {args.report}")
                if not args.report.parent.is_dir():
                    raise FileNotFoundError(f"Report directory does not exist: {args.report.parent}")
            result = check_serial(args.device, args.baud, args.seconds)
            encoded = json.dumps(result, indent=2) + "\n"
            if args.report:
                with args.report.open("x") as report:
                    report.write(encoded)
            print(encoded, end="")
            return 0 if result["ready"] else 2
        elif args.command == "status":
            if (args.session / "state").is_file():
                result = {"session": str(args.session), "state": (args.session / "state").read_text().strip()}
            else:
                from .operations import status
                result = status(args.session)
        elif args.command == "bag-import":
            from .bags import import_bag
            result = import_bag(args.session, args.output)
        elif args.command == "simulate":
            from .simulate import simulate
            if args.frames < 1:
                raise ValueError("frames must be positive")
            result = simulate(args.output, args.frames)
        elif args.command == "validate":
            from .dataset import write_json
            from .validate import validate
            if (args.session / "state").is_file() or (args.session / "bag").is_dir():
                from .bags import audit_bag
                result = audit_bag(args.session)
                if args.rig_calibration:
                    from .calibration_drift import add_to_validation
                    add_to_validation(result, args.session, args.rig_calibration,
                                      warn_sigma=args.calibration_warn_sigma,
                                      fail_sigma=args.calibration_fail_sigma)
            else:
                if args.rig_calibration:
                    raise ValueError("--rig-calibration drift checking requires a ROS bag session")
                result = validate(args.session)
            if args.report:
                if args.report.resolve().is_relative_to(args.session.resolve()):
                    raise ValueError("Write reports outside the immutable source session")
                write_json(args.report, result)
            print(json.dumps(result, indent=2))
            return 0 if result["valid"] else 2
        elif args.command == "sync":
            from .association import associate
            result = associate(args.session, args.output, args.stream, args.min_fraction, args.rig_calibration)
            print(json.dumps(result, indent=2))
            return 0 if result["passed"] else 2
        elif args.command == "ntrip":
            from .ntrip import NtripConfig, bridge
            result = bridge(NtripConfig.read(args.config), args.output, args.duration)
        elif args.command == "image-accuracy":
            from .gnss_accuracy import image_accuracy
            result = image_accuracy(args.session, args.alignment, args.profile, args.output, args.project,
                                    args.rig_calibration)
            print(json.dumps(result, indent=2))
            return 0 if result["passed"] else 2
        elif args.command == "map-accuracy":
            from .map_accuracy import assess
            result = assess(args.checkpoints, args.profile, args.output, args.image_accuracy,
                            args.workflow, args.georeference)
            print(json.dumps(result, indent=2))
            return 0 if result["passed"] else 2
        elif args.command == "georeference":
            from .georeference import georeference
            result = georeference(args.project, args.model, args.image_accuracy, args.output,
                                  args.artifact, args.max_residual_m)
        elif args.command == "calibration-check":
            from .rig_calibration import RigCalibration, check
            result = check(RigCalibration.read(args.calibration), args.session)
            print(json.dumps(result, indent=2))
            return 2 if args.require_complete and not result["complete"] else 0
        elif args.command == "calibrate-solve":
            import shutil
            import tempfile
            from .calibration_imu import calibration_entries, solve_session
            from .dataset import write_json
            from .rig_calibration import RigCalibration, apply_entries, check
            if args.output.exists():
                raise ValueError("Output must be a new directory")
            if args.output.resolve().is_relative_to(args.session.resolve()):
                raise ValueError("Write calibration results outside the immutable source session")
            calibration = RigCalibration.read(args.calibration)
            check(calibration, args.session)  # device identity and factory data, before any work
            solved = solve_session(args.session, max_offset_ms=args.max_offset_ms)
            evidence = f"session {args.session.resolve().name}, SHA256SUMS {solved['seal_sha256']}"
            updated = apply_entries(calibration.data, calibration_entries(solved, calibration, evidence))
            solved["rotation"] = solved["rotation"].tolist()
            result = {"imu_to_imu": solved, "calibration": check(RigCalibration(updated), args.session),
                      "outputs": {"calibration": str(args.output / "rig-calibration.json")}}
            # Publish the directory only once both files are complete.
            args.output.parent.mkdir(parents=True, exist_ok=True)
            staging = Path(tempfile.mkdtemp(prefix=".calibrate-", dir=args.output.parent))
            try:
                write_json(staging / "rig-calibration.json", updated)
                write_json(staging / "report.json", result)
                staging.rename(args.output)
            except BaseException:
                shutil.rmtree(staging, ignore_errors=True)
                raise
        elif args.command == "calibrate-camera":
            from .calibration_workflow import solve_camera
            result = solve_camera(args.session, args.calibration, args.output,
                                  camera=args.camera, max_offset_ms=args.max_offset_ms)
        elif args.command == "export":
            from .export import export
            result = export(args.session, args.output, args.stream, args.interval,
                            args.min_sharpness, args.allow_gaps)
        elif args.command == "process":
            from .process import process
            from .process_config import ProcessConfig
            result = process(args.session, args.output, ProcessConfig.read(args.config),
                             args.execute, args.prepare_only, args.resume,
                             args.gcp, args.geo, args.vertical_datum)
        elif args.command == "reconstruct":
            from .reconstruct import execute, sparse_plan
            result = sparse_plan(args.project, args.output, args.matcher, args.cpu)
            if args.execute:
                result = execute(result)
        elif args.command == "dense":
            from .reconstruct import dense_plan, execute
            result = dense_plan(args.project, args.model, args.output, args.max_image_size)
            if args.execute:
                result = execute(result)
        elif args.command == "accuracy":
            from .accuracy import checkpoints
            result = checkpoints(args.checkpoints)
        print(json.dumps(result, indent=2))
        return 0
    except (OSError, ValueError, RuntimeError, ImportError, TypeError, KeyError,
            subprocess.CalledProcessError) as error:
        print(f"wr-map: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
