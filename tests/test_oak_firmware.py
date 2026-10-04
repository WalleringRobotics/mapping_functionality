import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest


spec = importlib.util.spec_from_file_location(
    "update_oak_imu", Path(__file__).resolve().parents[1] / "deploy/update-oak-imu.py")
firmware = importlib.util.module_from_spec(spec)
spec.loader.exec_module(firmware)


def details(**overrides):
    return {"id": "camera", "imu_type": "BNO086", "imu_firmware": "3.2.13",
            "imu_firmware_embedded": "3.9.9", **overrides}


@pytest.mark.parametrize("overrides", [
    {"id": "other-camera"}, {"imu_type": "BMI270"},
    {"imu_firmware_embedded": "4.0.0"}, {"imu_firmware": "3.9.7"},
])
def test_unreviewed_target_is_rejected(overrides):
    with pytest.raises(ValueError):
        firmware.check_target(details(**overrides), "camera", "3.2.13", "3.9.9")


def test_upgrade_guard_rejects_downgrade_and_skips_current_firmware():
    assert firmware.check_target(details(), "camera", "3.2.13", "3.9.9")
    assert not firmware.check_target(
        details(imu_firmware="3.9.9"), "camera", "3.2.13", "3.9.9")
    with pytest.raises(ValueError, match="Only firmware upgrades"):
        firmware.check_target(details(imu_firmware="4.0.0"), "camera", "4.0.0", "3.9.9")


@pytest.mark.parametrize("apply,started,progress,error", [
    (False, True, 100, None),
    (True, False, 0, "refused"),
    (True, True, 65, "failed at"),
    (True, True, 100, "readback"),
])
def test_update_requires_opt_in_and_never_claims_unverified_success(
        tmp_path, monkeypatch, apply, started, progress, error):
    calls = []

    class Device:
        def __init__(self, device_id):
            assert device_id == "camera"

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def readCalibration(self):
            return SimpleNamespace(eepromToJson=lambda: {"calibration": "preserved"})

        def startIMUFirmwareUpdate(self, force):
            assert force is False
            assert (tmp_path / "report/calibration-before.json").is_file()
            calls.append("flash")
            return started

        def getIMUFirmwareUpdateStatus(self):
            return True, progress

    monkeypatch.setattr(firmware, "depthai", lambda: SimpleNamespace(Device=Device))
    monkeypatch.setattr(firmware, "device_details", lambda *_: details())
    monkeypatch.setattr(firmware.time, "sleep", lambda _: None)
    args = SimpleNamespace(device_id="camera", from_version="3.2.13", to_version="3.9.9",
                           output=tmp_path / "report", apply=apply)
    if error:
        with pytest.raises(RuntimeError, match=error):
            firmware.update(args)
    else:
        firmware.update(args)
    events = [json.loads(line) for line in (args.output / "events.jsonl").read_text().splitlines()]
    assert not any(row["status"] == "verified" for row in events)
    assert events[-1]["status"] == ("failed" if error else "inspection_only")
    assert calls == (["flash"] if apply else [])
    with pytest.raises(FileExistsError):
        firmware.update(args)
