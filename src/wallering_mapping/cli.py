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
    record = commands.add_parser("record", help="Capture a new immutable session")
    record.add_argument("--config", type=Path, required=True)
    record.add_argument("--output", type=Path, required=True)
    record.add_argument("--device-id")
    record.add_argument("--duration", type=positive, help="Seconds after warmup; otherwise until Ctrl+C")
    simulate = commands.add_parser("simulate", help="Create an IO fixture without hardware")
    simulate.add_argument("--output", type=Path, required=True)
    simulate.add_argument("--frames", type=int, default=12)
    validate = commands.add_parser("validate", help="Audit dataset integrity, gaps and timing")
    validate.add_argument("session", type=Path)
    validate.add_argument("--report", type=Path)
    export = commands.add_parser("export", help="Export one camera for COLMAP or ODM")
    export.add_argument("session", type=Path)
    export.add_argument("--output", type=Path, required=True)
    export.add_argument("--stream", choices=["rgb", "left", "right"], default="rgb")
    export.add_argument("--interval", type=float, default=1.0)
    export.add_argument("--min-sharpness", type=float, default=0.0)
    export.add_argument("--allow-gaps", action="store_true")
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
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if args.command == "inspect":
            from .oak import inspect_device
            result = inspect_device(args.device_id)
        elif args.command == "record":
            from .oak import record
            result = record(args.output, CaptureConfig.read(args.config), args.duration, args.device_id)
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
        elif args.command == "export":
            from .export import export
            result = export(args.session, args.output, args.stream, args.interval,
                            args.min_sharpness, args.allow_gaps)
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
