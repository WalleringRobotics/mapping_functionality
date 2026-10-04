import os
import select
import struct
import threading
import time

import pytest

pytest.importorskip("serial")
common = pytest.importorskip("pymavlink.dialects.v20.common")

from wallering_mapping.px4_link_diagnostics import PARAMETERS, audit_link, main  # noqa: E402


@pytest.mark.parametrize("px4", [True, False])
def test_real_serial_exchange_measures_px4_and_never_sends_configuration(px4):
    master, slave = os.openpty()
    stop = threading.Event()
    received = []
    errors = []
    expected = dict(zip(PARAMETERS, (102, 2, 921600, 0, 0, 2)))

    class Writer:
        def write(self, packet):
            return os.write(master, packet)

    def peer():
        mav = common.MAVLink(Writer(), srcSystem=1, srcComponent=1)
        mav.robust_parsing = True
        try:
            while not stop.is_set():
                if not select.select([master], [], [], .02)[0]:
                    continue
                for message in mav.parse_buffer(os.read(master, 4096)) or []:
                    kind = message.get_type()
                    received.append(kind)
                    if kind == "HEARTBEAT":
                        mav.heartbeat_send(2, common.MAV_AUTOPILOT_PX4 if px4 else
                                           common.MAV_AUTOPILOT_INVALID, 0, 0, 3)
                        mav.raw_imu_send(1, 0, 0, 0, 0, 0, 0, 0, 0, 0)
                        mav.attitude_send(1, 0, 0, 0, 0, 0, 0)
                        mav.gps_raw_int_send(1, 3, 0, 0, 0, 100, 100, 0, 0, 10)
                    elif kind == "PARAM_REQUEST_READ":
                        value = struct.unpack("<f", struct.pack("<i", expected[message.param_id]))[0]
                        mav.param_value_send(message.param_id.encode(), value,
                                             common.MAV_PARAM_TYPE_INT32, len(PARAMETERS), 0)
                    elif kind == "TIMESYNC" and message.tc1 == 0:
                        mav.timesync_send(time.monotonic_ns() - 20_000_000, message.ts1)
        except Exception as error:
            errors.append(error)

    worker = threading.Thread(target=peer)
    worker.start()
    try:
        report = audit_link(os.ttyname(slave), 921600, "off", .5)
        assert report["error"] is None
        assert report["complete"] is px4
        assert report["bidirectional_protocol_evidence"] is px4
        assert set(received) <= {"HEARTBEAT", "PARAM_REQUEST_READ", "TIMESYNC"}
        assert report["host_writes"]["HEARTBEAT"] >= 1
        if px4:
            assert report["parameters"] == expected
            assert all(report["telemetry_present"].values())
            assert report["timesync"]["rtt_ns"]["count"] >= 2
            assert abs(report["timesync"]["local_monotonic_minus_px4_ns"]["median"] - 20_000_000) < 50_000_000
            assert report["px4_stream_rates_hz"]["RAW_IMU"] > 0
        else:
            assert report["target"] is None
            assert report["parameters"] == {}
            assert set(received) == {"HEARTBEAT"}
    finally:
        stop.set()
        worker.join(timeout=2)
        os.close(master)
        os.close(slave)
    assert not errors


def test_silence_is_not_bidirectional_evidence():
    master, slave = os.openpty()
    try:
        report = audit_link(os.ttyname(slave), 921600, "off", .05)
        assert report["received_bytes"] == 0
        assert report["host_writes"]["HEARTBEAT"] == 1
        assert not report["bidirectional_protocol_evidence"]
        assert not report["complete"]
        assert report["timesync"]["rtt_ns"]["median"] is None
        assert report["missing_parameters"] == list(PARAMETERS)
    finally:
        os.close(master)
        os.close(slave)


def test_report_collision_prevents_port_access(tmp_path):
    report = tmp_path / "existing.json"
    report.write_text("previous evidence")
    with pytest.raises(FileExistsError):
        main(["--device", "/does/not/exist", "--baud", "921600", "--flow-control", "off",
              "--report", str(report)])
    assert report.read_text() == "previous evidence"


@pytest.mark.parametrize("baud,flow,seconds", [(0, "off", 1), (921600, "auto", 1),
                                              (921600, "off", float("nan"))])
def test_invalid_options_rejected(baud, flow, seconds):
    with pytest.raises(ValueError):
        audit_link("/does/not/exist", baud, flow, seconds)
