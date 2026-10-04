import json
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from wallering_mapping import hardware
from wallering_mapping.cli import main
from wallering_mapping.config import CaptureConfig
from wallering_mapping.hardware_probe import dependencies, profile_ready, telemetry, telemetry_ready
from wallering_mapping.oak import check_imu_firmware
from wallering_mapping.odm import check_image_platform
from wallering_mapping.telemetry_config import TelemetryConfig


def test_storage_requires_existing_root_and_mounted_destination(tmp_path, monkeypatch):
    with pytest.raises(FileNotFoundError):
        hardware.storage(tmp_path / "absent", 0)
    assert not (tmp_path / "absent").exists()
    with pytest.raises(ValueError, match="mount is absent"):
        hardware.storage(tmp_path, 0, tmp_path)
    outside = tmp_path / "outside"
    outside.mkdir()
    mount = tmp_path / "mount"
    mount.mkdir()
    (mount / "escape").symlink_to(outside, target_is_directory=True)
    monkeypatch.setattr(hardware.os.path, "ismount", lambda p: p == mount)
    with pytest.raises(ValueError, match="required mount"):
        hardware.storage(mount / "escape", 0, mount)
    result = hardware.storage(mount, 0, mount)
    assert result["free_bytes"] > 0 and list(mount.iterdir()) == [mount / "escape"]


def test_storage_enforces_disk_reserve(tmp_path):
    with pytest.raises(ValueError, match="reserve"):
        hardware.storage(tmp_path, 10**9)


def test_native_probe_failure_cannot_produce_ready_or_leak_temporary_images(tmp_path, monkeypatch):
    monkeypatch.setattr(hardware, "memory", lambda _: {})

    def probe(kind, payload, timeout):
        if kind == "camera":
            Path(payload["root"]).mkdir()
            raise subprocess.TimeoutExpired("camera", timeout)
        return {}
    monkeypatch.setattr(hardware, "probe", probe)
    result = hardware.hardware_check("capture", tmp_path, CaptureConfig(min_free_gib=0))
    assert not result["ready"]
    failure = next(c for c in result["checks"] if c["name"] == "oak_capture")
    assert failure["status"] == "fail" and "timed out" in failure["detail"]
    assert list(tmp_path.iterdir()) == []


def test_missing_dependencies_skip_hardware_and_fail_readiness(tmp_path, monkeypatch):
    seen = []

    def probe(kind, *args):
        seen.append(kind)
        raise RuntimeError("wrong DepthAI version")
    monkeypatch.setattr(hardware, "probe", probe)
    monkeypatch.setattr(hardware, "memory", lambda _: {})
    result = hardware.hardware_check("capture", tmp_path, CaptureConfig(min_free_gib=0))
    assert not result["ready"] and seen == ["dependencies"]


@pytest.mark.parametrize("stdout", ["native SDK crashed", 'WR_HARDWARE_RESULT={"ok": false, "detail": "bad USB"}'])
def test_probe_rejects_missing_or_failed_results(monkeypatch, stdout):
    monkeypatch.setattr(hardware, "command", lambda *args: stdout)
    with pytest.raises(RuntimeError):
        hardware.probe("camera", {}, 1)


def test_command_deadline_terminates_child():
    with pytest.raises(subprocess.TimeoutExpired):
        hardware.command([sys.executable, "-c", "import time; time.sleep(10)"], .1)


def test_failed_cli_report_is_saved_with_nonzero_exit_and_never_overwritten(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(hardware, "hardware_check", lambda *args, **kwargs: {"ready": False})
    report = tmp_path / "report.json"
    args = ["hardware-check", "--mode", "building", "--output-root", str(tmp_path), "--report", str(report)]
    assert main(args) == 2
    assert json.loads(report.read_text()) == {"ready": False}
    assert json.loads(capsys.readouterr().out) == {"ready": False}
    report.write_text("old evidence")
    assert main(args) == 2
    assert report.read_text() == "old evidence"


def test_current_state_and_current_sync_are_required():
    config = TelemetryConfig()
    now = 100_000_000_000
    latest = {role: {"received_monotonic_ns": now} for role in config.required}
    latest["state"]["fields"] = {"connected": True}
    latest["timesync"]["sync_quality"] = {"qualification": "qualified"}
    assert telemetry_ready(latest, config, now)
    latest["state"]["fields"]["connected"] = False
    assert not telemetry_ready(latest, config, now)
    latest["state"]["fields"]["connected"] = True
    assert not telemetry_ready(latest, config, now + 600_000_000)
    latest["timesync"]["sync_quality"]["qualification"] = "warming_or_degraded"
    assert not telemetry_ready(latest, config, now)


def test_placeholder_accuracy_profile_cannot_pass_commissioning():
    with pytest.raises(ValueError, match="Incomplete GNSS commissioning"):
        profile_ready("configs/gnss-accuracy.template.json")


def test_overlay_cannot_shadow_pinned_binary_package(monkeypatch):
    monkeypatch.setattr("importlib.metadata.version", lambda _: "3.10.0")
    monkeypatch.setattr("importlib.import_module", lambda _: SimpleNamespace(__version__="2.31.0"))
    with pytest.raises(ValueError, match="imported 2.31.0"):
        dependencies({"depthai": "3.10.0"})


@pytest.mark.parametrize("corrupt", [False, True])
def test_rtk_startup_requires_crc_valid_observation_frames(monkeypatch, corrupt):
    from test_rtcm import frame
    from wallering_mapping.telemetry_config import rtk_topics
    import time
    config = TelemetryConfig(topics={**TelemetryConfig().topics, **rtk_topics()},
                             required=("state", "timesync", "gps_raw", "rtcm"))
    now = time.monotonic_ns()
    packet = frame(bytes([0x43, 0x20, 42]) + bytes(10))
    if corrupt:
        packet = packet[:-1] + bytes([packet[-1] ^ 1])
    rows = [
        {"role": "state", "fields": {"connected": True}, "received_monotonic_ns": now},
        {"role": "timesync", "sync_quality": {"qualification": "qualified"}, "received_monotonic_ns": now},
        {"role": "gps_raw", "fields": {"fix_type": 6, "h_acc": 10, "v_acc": 20}, "received_monotonic_ns": now},
        {"role": "rtcm", "fields": {"data": list(packet)}, "received_monotonic_ns": now},
    ]
    closed = []
    source = SimpleNamespace(start=lambda: None, close=lambda: closed.append(True),
                             buffer=SimpleNamespace(drain=lambda: rows, parameters={}, summary=lambda: {}))
    monkeypatch.setattr("wallering_mapping.mavros.MavrosSubscriber", lambda _: source)
    if corrupt:
        with pytest.raises(ValueError, match="CRC"):
            telemetry({"config": config.to_dict(), "seconds": 1, "gnss_profile": None})
    else:
        assert telemetry({"config": config.to_dict(), "seconds": 1, "gnss_profile": None})["rtcm_frames"] == 1
    assert closed


@pytest.mark.parametrize("installed,embedded,ok", [("3.2.13", "3.9.9", False), ("3.9.9", "3.9.9", True)])
def test_bno_firmware_gate_never_flashes(installed, embedded, ok):
    device = SimpleNamespace(getConnectedIMU=lambda: "BNO086",
                             getIMUFirmwareVersion=lambda: installed,
                             getEmbeddedIMUFirmwareVersion=lambda: embedded)
    if ok:
        check_imu_firmware(device)
    else:
        with pytest.raises(RuntimeError, match="3.2.13.*3.9.9"):
            check_imu_firmware(device)


def test_bmi_imu_does_not_use_bno_firmware_gate():
    check_imu_firmware(SimpleNamespace(getConnectedIMU=lambda: "BMI270"))


def test_incompatible_docker_architecture_is_rejected():
    with pytest.raises(ValueError, match="does not match"):
        check_image_platform({"Os": "linux", "Architecture": "amd64"}, "aarch64")
    check_image_platform({"Os": "linux", "Architecture": "arm64"}, "aarch64")
    check_image_platform({"Os": "linux", "Architecture": "amd64"}, "x86_64")


def test_power_gated_thermal_zone_does_not_hide_active_hot_zone(tmp_path, monkeypatch):
    for index, name, temperature in ((0, "cpu-thermal", "50000"), (1, "cv0-thermal", "0")):
        zone = tmp_path / f"thermal_zone{index}"
        zone.mkdir()
        (zone / "type").write_text(name)
        (zone / "temp").write_text(temperature)
    real_read = hardware.os.read
    def read(fd, size):
        if os.readlink(f"/proc/self/fd/{fd}").endswith("thermal_zone1/temp"):
            raise BlockingIOError("power gated")
        return real_read(fd, size)
    monkeypatch.setattr(hardware.os, "read", read)
    readings = hardware.thermals(tmp_path)
    assert readings[0]["celsius"] == 50 and "unavailable" in readings[1]
    cpu = tmp_path / "thermal_zone0"
    (cpu / "trip_point_0_type").write_text("critical")
    (cpu / "trip_point_0_temp").write_text("45000")
    with pytest.raises(ValueError, match="reached"):
        hardware.thermals(tmp_path)


def test_service_launcher_blocks_recording_on_failed_startup(tmp_path):
    fake = tmp_path / "python"
    log = tmp_path / "calls"
    fake.write_text('#!/bin/bash\nprintf "%s\\n" "$@" >> "$WR_TEST_LOG"\nexit 2\n')
    fake.chmod(0o755)
    result = subprocess.run(["bash", "deploy/record-session.sh"], env={
        **os.environ, "WR_MAPPING_PYTHON": str(fake), "WR_MAPPING_ROOT": str(tmp_path),
        "WR_MAPPING_MOUNT": str(tmp_path), "WR_TEST_LOG": str(log),
        "WR_MAPPING_TELEMETRY_CONFIG": "configs/mavros-survey.json"}, capture_output=True)
    assert result.returncode == 2
    calls = log.read_text().splitlines()
    assert "hardware-check" in calls and "record" not in calls
    assert "--telemetry-config" in calls and "--require-mount" in calls
