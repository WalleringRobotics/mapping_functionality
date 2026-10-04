import json
from types import SimpleNamespace

import pytest

from wallering_mapping.cli import parser
from wallering_mapping.recording import (
    CALIBRATION_DURATION, CALIBRATION_PHASES, phase_record, request_imu_rate,
)


def test_calibration_prompts_cover_fixed_duration_and_are_marked_as_hints():
    rows = [phase_record(i, 100 + i) for i in range(len(CALIBRATION_PHASES))]
    assert CALIBRATION_DURATION == 110
    assert [row["planned_offset_seconds"] for row in rows] == [0, 10, 16, 23, 30, 36, 43, 50, 56, 63, 70, 100]
    assert rows[-1]["phase"] == "still_end"
    args = parser().parse_args(["calibrate-record", "--output", "unused"])
    assert (args.duration, args.px4_imu_rate, args.warmup) == (110, 100, 60)


def test_rate_requests_log_acknowledgements_and_restore_default(tmp_path):
    calls = []
    def call(message, interval):
        calls.append((message, interval))
        return SimpleNamespace(success=True, result=0)
    path = tmp_path / "request.json"
    report = request_imu_rate(100, path, call)
    assert report["success"]
    assert json.loads(path.read_text()) == report
    assert calls == [(105, 10000), (31, 10000)]
    with pytest.raises(FileExistsError):
        request_imu_rate(100, path, call)
    request_imu_rate(0, tmp_path / "restore.json", call)
    assert calls[-2:] == [(105, 0), (31, 0)]


def test_partial_request_failure_preserves_every_attempt_for_cleanup(tmp_path):
    path = tmp_path / "request.json"
    def call(message, interval):
        # The pending attempt already exists before the FCU is contacted.
        pending = json.loads(path.read_text())["requests"][-1]
        assert pending["message_id"] == message
        assert not pending["success"]
        if message == 105:
            raise TimeoutError("no ACK")
        return SimpleNamespace(success=False, result=3)
    result = request_imu_rate(100, path, call)
    assert not result["success"]
    assert len(result["requests"]) == 2
    assert "TimeoutError" in result["requests"][0]["error"]
    assert result["requests"][1]["ack_result"] == 3


@pytest.mark.parametrize("rate", [float("nan"), float("inf"), -1, 201])
def test_invalid_rate_is_rejected_before_calling_fcu(tmp_path, rate):
    with pytest.raises(ValueError):
        request_imu_rate(rate, tmp_path / "invalid.json", lambda *_: pytest.fail("called FCU"))
