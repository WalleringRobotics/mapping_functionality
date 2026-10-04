#!/usr/bin/env python3
"""Commission BNO firmware separately from startup checks and capture."""

import argparse
import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path

from wallering_mapping.oak import depthai, device_details


def save(path, value):
    with path.open("x") as stream:
        json.dump(value, stream, indent=2)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def check_target(details, device_id, from_version, to_version):
    if details["id"] != device_id:
        raise ValueError("Connected device does not match the requested device ID")
    if details["imu_type"] not in {"BNO085", "BNO086"}:
        raise ValueError("This procedure only supports BNO085/BNO086 IMUs")
    if details["imu_firmware_embedded"] != to_version:
        raise ValueError("Requested target differs from the pinned SDK's bundled firmware")
    if details["imu_firmware"] == to_version:
        return False
    if details["imu_firmware"] != from_version:
        raise ValueError("Installed firmware differs from the reviewed starting version")
    if tuple(map(int, to_version.split("."))) <= tuple(map(int, from_version.split("."))):
        raise ValueError("Only firmware upgrades are supported")
    return True


def update(args):
    dai = depthai()
    args.output.mkdir(parents=True, exist_ok=False)
    with (args.output / "events.jsonl").open("x", buffering=1) as journal:
        def event(**fields):
            row = {"time": datetime.now(timezone.utc).isoformat(), **fields}
            journal.write(json.dumps(row) + "\n")
            journal.flush()
            os.fsync(journal.fileno())
            print(json.dumps(row), flush=True)

        try:
            with dai.Device(args.device_id) as device:
                before = device_details(device, dai)
                save(args.output / "before.json", before)
                needed = check_target(before, args.device_id, args.from_version, args.to_version)
                calibration = device.readCalibration().eepromToJson()
                save(args.output / "calibration-before.json", calibration)
                event(status="preflight_passed", before=before, update_needed=needed)
                if not args.apply:
                    event(status="inspection_only", hint="Use --apply to flash the reviewed target")
                    return
                if needed:
                    # No pipeline/IMU node runs during the SDK's asynchronous update.
                    event(status="starting_update", target=args.to_version,
                          instruction="Keep USB/power connected; do not interrupt this process")
                    if not device.startIMUFirmwareUpdate(False):
                        raise RuntimeError("SDK refused to start the IMU firmware update")
                    while True:
                        finished, percentage = device.getIMUFirmwareUpdateStatus()
                        event(status="flashing", finished=finished, percentage=percentage)
                        if finished:
                            if percentage != 100:
                                raise RuntimeError(f"IMU firmware update failed at {percentage}%")
                            break
                        time.sleep(1)
                else:
                    event(status="already_current")
            # A fresh connection checks persistent firmware and calibration readback.
            time.sleep(3)
            with dai.Device(args.device_id) as device:
                after = device_details(device, dai)
                save(args.output / "after.json", after)
                calibration_after = device.readCalibration().eepromToJson()
                save(args.output / "calibration-after.json", calibration_after)
                if after["id"] != args.device_id or after["imu_firmware"] != args.to_version:
                    raise RuntimeError("Firmware readback after reconnect does not match the target")
                if calibration_after != calibration:
                    raise RuntimeError("Calibration readback changed; inspect the saved backups")
                event(status="verified", after=after, calibration_unchanged=True)
        except Exception as exc:
            event(status="failed", error=str(exc))
            raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device-id", required=True)
    parser.add_argument("--from-version", required=True)
    parser.add_argument("--to-version", required=True)
    parser.add_argument("--output", type=Path, required=True, help="New private evidence directory")
    parser.add_argument("--apply", action="store_true", help="Flash the SDK's bundled IMU firmware")
    update(parser.parse_args())


if __name__ == "__main__":
    main()
