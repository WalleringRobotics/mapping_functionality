"""Small command-line surface with optional hardware dependencies."""

import argparse
import json
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
        print(json.dumps(result, indent=2))
        return 0
    except (OSError, ValueError, RuntimeError, ImportError, TypeError) as error:
        print(f"wr-map: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())

