import json
import os
import threading

import pytest

serial = pytest.importorskip("serial")
common = pytest.importorskip("pymavlink.dialects.v20.common")

from wallering_mapping.cli import main  # noqa: E402
from wallering_mapping.serial_diagnostics import check_serial  # noqa: E402


@pytest.mark.parametrize("packet,finding", [
    (b"", "no_bytes"),
    (b"not mavlink\n", "bytes_without_valid_mavlink"),
    ("heartbeat", "heartbeat_received"),
    ("ping", "mavlink_without_heartbeat"),
])
def test_real_serial_read_and_parser_without_transmission(packet, finding, monkeypatch):
    if packet == "heartbeat":
        encoder = common.MAVLink(None, srcSystem=1, srcComponent=1)
        packet = encoder.heartbeat_encode(2, 12, 0, 0, 4).pack(encoder)
    elif packet == "ping":
        encoder = common.MAVLink(None, srcSystem=1, srcComponent=1)
        packet = encoder.ping_encode(1, 1, 1, 1).pack(encoder)
    master, slave = os.openpty()
    path = os.ttyname(slave)
    stop = threading.Event()
    opened = threading.Event()
    real_serial = serial.Serial

    def open_port(*args, **kwargs):
        port = real_serial(*args, **kwargs)
        opened.set()
        return port
    monkeypatch.setattr(serial, "Serial", open_port)

    def transmit():
        # Wait for raw mode to avoid pre-open echo on a slow test runner.
        if not opened.wait(timeout=2):
            return
        while not stop.is_set():
            if packet:
                os.write(master, packet)
            stop.wait(.01)
    thread = threading.Thread(target=transmit)
    thread.start()
    try:
        report = check_serial(path, [115200], .3)
        result = report["results"][0]
        assert result["finding"] == finding
        assert report["ready"] == (finding == "heartbeat_received")
        assert report["receive_only"]
        # Bytes written by the probe would arrive at the PTY's master side.
        os.set_blocking(master, False)
        with pytest.raises(BlockingIOError):
            os.read(master, 4096)
    finally:
        stop.set()
        thread.join(timeout=1)
        os.close(master)
        os.close(slave)


@pytest.mark.parametrize("bauds,seconds", [([], 1), ([115200, 115200], 1), ([0], 1),
                                          ([True], 1), ([115200], float("inf")),
                                          ([115200], 301), ([115200], 0)])
def test_invalid_scans_rejected_before_device_access(bauds, seconds):
    with pytest.raises(ValueError):
        check_serial("/does/not/exist", bauds, seconds)


def test_multiple_rates_and_silent_report_exit_status(tmp_path, capsys):
    master, slave = os.openpty()
    report = tmp_path / "silent.json"
    try:
        assert main(["serial-check", "--device", os.ttyname(slave), "--baud", "115200", "57600",
                     "--seconds", ".01", "--report", str(report)]) == 2
        data = json.loads(report.read_text())
        assert [r["baud"] for r in data["results"]] == [115200, 57600]
        assert all(r["finding"] == "no_bytes" for r in data["results"])
        assert json.loads(capsys.readouterr().out) == data
    finally:
        os.close(master)
        os.close(slave)


def test_existing_report_prevents_device_access(tmp_path, monkeypatch):
    report = tmp_path / "old.json"
    report.write_text("previous evidence")
    def unexpected(*args):
        pytest.fail("Must not open a device when the report already exists")
    monkeypatch.setattr("wallering_mapping.serial_diagnostics.check_serial", unexpected)
    assert main(["serial-check", "--report", str(report)]) == 2
    assert report.read_text() == "previous evidence"


def test_missing_report_directory_prevents_device_access(tmp_path, monkeypatch):
    def unexpected(*args):
        pytest.fail("Must not open a device when the report destination is invalid")
    monkeypatch.setattr("wallering_mapping.serial_diagnostics.check_serial", unexpected)
    assert main(["serial-check", "--report", str(tmp_path / "missing" / "report.json")]) == 2
