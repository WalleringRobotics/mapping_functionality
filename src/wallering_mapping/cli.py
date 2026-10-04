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


def parser():
    result = argparse.ArgumentParser(prog="wr-map")
    commands = result.add_subparsers(dest="command", required=True)
    inspect = commands.add_parser("inspect", help="Probe OAK sensors, IMU and USB link")
    inspect.add_argument("--device-id")
    record = commands.add_parser("capture", aliases=["record"], help="Mode 1: onboard recording")
    record.add_argument("--config", type=Path, required=True)
    record.add_argument("--output", type=Path, required=True)
    record.add_argument("--device-id")
    record.add_argument(
        "--telemetry-config", type=Path, help="Subscribe to the existing PX4/MAVROS connector"
    )
    record.add_argument(
        "--duration", type=positive, help="Seconds after warmup; otherwise until Ctrl+C"
    )
    doctor = commands.add_parser("doctor", help="Check dependencies and output storage")
    doctor.add_argument("--mode", choices=["capture", "process"], required=True)
    doctor.add_argument("--config", type=Path, default=Path("configs/oakd-survey.json"))
    doctor.add_argument("--output-root", type=Path, required=True)
    doctor.add_argument("--probe-device", action="store_true")
    doctor.add_argument("--device-id")
    doctor.add_argument("--backend", choices=["building", "terrain"], default="building")
    status = commands.add_parser("status", help="Read the recorder's current status")
    status.add_argument("session", type=Path)
    simulate = commands.add_parser("simulate", help="Create an IO fixture without hardware")
    simulate.add_argument("--output", type=Path, required=True)
    simulate.add_argument("--frames", type=int, default=12)
    validate = commands.add_parser("validate", help="Audit dataset integrity, gaps and timing")
    validate.add_argument("session", type=Path)
    validate.add_argument("--report", type=Path)
    sync = commands.add_parser(
        "sync", help="Audit exposure-to-PX4 timing and associate vehicle poses"
    )
    sync.add_argument("session", type=Path)
    sync.add_argument("--output", type=Path, required=True)
    sync.add_argument("--stream", choices=["rgb", "left", "right"], default="rgb")
    sync.add_argument("--min-fraction", type=float, default=0.9)
    ntrip = commands.add_parser(
        "ntrip", help="Forward verified fixed-base NTRIP v2 corrections through MAVROS"
    )
    ntrip.add_argument("--config", type=Path, required=True)
    ntrip.add_argument("--output", type=Path, required=True)
    ntrip.add_argument("--duration", type=positive)
    images_accuracy = commands.add_parser(
        "image-accuracy", help="Report corrected-GNSS camera uncertainty and qualified geolocation"
    )
    images_accuracy.add_argument("session", type=Path)
    images_accuracy.add_argument("--alignment", type=Path, required=True)
    images_accuracy.add_argument("--profile", type=Path, required=True)
    images_accuracy.add_argument("--output", type=Path, required=True)
    images_accuracy.add_argument(
        "--project", type=Path, help="Restrict geolocation to an existing selected image export"
    )
    map_accuracy = commands.add_parser(
        "map-accuracy", help="Assess withheld checkpoints and reference-aware map accuracy"
    )
    map_accuracy.add_argument("checkpoints", type=Path)
    map_accuracy.add_argument("--profile", type=Path, required=True)
    map_accuracy.add_argument("--output", type=Path, required=True)
    map_accuracy.add_argument("--image-accuracy", type=Path)
    map_accuracy.add_argument("--workflow", type=Path)
    map_accuracy.add_argument("--georeference", type=Path)
    georef = commands.add_parser(
        "georeference", help="Align COLMAP and PLY products using qualified camera GNSS positions"
    )
    georef.add_argument("project", type=Path)
    georef.add_argument("--model", type=Path, required=True)
    georef.add_argument("--image-accuracy", type=Path, required=True)
    georef.add_argument("--output", type=Path, required=True)
    georef.add_argument("--artifact", type=Path, action="append", default=[])
    georef.add_argument("--max-residual-m", type=positive, default=0.15)
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
    dense = commands.add_parser(
        "dense", help="Plan/run MVS for an explicitly selected sparse model"
    )
    dense.add_argument("project", type=Path)
    dense.add_argument("--model", type=Path, required=True)
    dense.add_argument("--output", type=Path, required=True)
    dense.add_argument("--max-image-size", type=int, default=2000)
    dense.add_argument("--execute", action="store_true")
    accuracy = commands.add_parser("accuracy", help="Report independent checkpoint residuals")
    accuracy.add_argument("checkpoints", type=Path)
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if args.command == "inspect":
            from .oak import inspect_device

            result = inspect_device(args.device_id)
        elif args.command in {"record", "capture"}:
            from .oak import record
            from .telemetry_config import TelemetryConfig

            telemetry = (
                TelemetryConfig.read(args.telemetry_config) if args.telemetry_config else None
            )
            result = record(
                args.output,
                CaptureConfig.read(args.config),
                args.duration,
                args.device_id,
                telemetry,
            )
        elif args.command == "doctor":
            from .operations import doctor

            config = CaptureConfig.read(args.config) if args.mode == "capture" else None
            result = doctor(
                args.mode, args.output_root, config, args.probe_device, args.device_id, args.backend
            )
            print(json.dumps(result, indent=2))
            return 0 if result["ready"] else 2
        elif args.command == "status":
            from .operations import status

            result = status(args.session)
        elif args.command == "simulate":
            from .simulate import simulate

            if args.frames < 1:
                raise ValueError("frames must be positive")
            result = simulate(args.output, args.frames)
        elif args.command == "validate":
            from .dataset import write_json
            from .validate import validate

            result = validate(args.session)
            if args.report:
                if args.report.resolve().is_relative_to(args.session.resolve()):
                    raise ValueError("Write reports outside the immutable source session")
                write_json(args.report, result)
            print(json.dumps(result, indent=2))
            return 0 if result["valid"] else 2
        elif args.command == "sync":
            from .association import associate

            result = associate(args.session, args.output, args.stream, args.min_fraction)
            print(json.dumps(result, indent=2))
            return 0 if result["passed"] else 2
        elif args.command == "ntrip":
            from .ntrip import NtripConfig, bridge

            result = bridge(NtripConfig.read(args.config), args.output, args.duration)
        elif args.command == "image-accuracy":
            from .gnss_accuracy import image_accuracy

            result = image_accuracy(
                args.session, args.alignment, args.profile, args.output, args.project
            )
            print(json.dumps(result, indent=2))
            return 0 if result["passed"] else 2
        elif args.command == "map-accuracy":
            from .map_accuracy import assess

            result = assess(
                args.checkpoints,
                args.profile,
                args.output,
                args.image_accuracy,
                args.workflow,
                args.georeference,
            )
            print(json.dumps(result, indent=2))
            return 0 if result["passed"] else 2
        elif args.command == "georeference":
            from .georeference import georeference

            result = georeference(
                args.project,
                args.model,
                args.image_accuracy,
                args.output,
                args.artifact,
                args.max_residual_m,
            )
        elif args.command == "export":
            from .export import export

            result = export(
                args.session,
                args.output,
                args.stream,
                args.interval,
                args.min_sharpness,
                args.allow_gaps,
            )
        elif args.command == "process":
            from .process import process
            from .process_config import ProcessConfig

            result = process(
                args.session,
                args.output,
                ProcessConfig.read(args.config),
                args.execute,
                args.prepare_only,
                args.resume,
                args.gcp,
                args.geo,
                args.vertical_datum,
            )
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
    except (
        OSError,
        ValueError,
        RuntimeError,
        ImportError,
        TypeError,
        KeyError,
        subprocess.CalledProcessError,
    ) as error:
        print(f"wr-map: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
