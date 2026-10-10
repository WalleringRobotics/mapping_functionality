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


def checked_plan(tmp_path, **plan):
    from pathlib import Path
    from qgc_plans import survey_plan, write_plan
    from wallering_mapping.survey_plan import check_survey, write_check
    repo = Path(__file__).resolve().parents[1]
    tmp_path.mkdir(parents=True, exist_ok=True)
    path = write_plan(tmp_path / "field.plan", survey_plan(**plan))
    report = check_survey(path, repo / "configs/qgc-oak-rgb-12mp.json", repo / "configs/oakd-ros.yaml", 15)
    return write_check(path, tmp_path / "checked", report)


def test_only_a_passed_unmodified_survey_check_can_be_bound(tmp_path):
    from wallering_mapping.recording import verify_survey
    from wallering_mapping.dataset import sha256_file
    good = checked_plan(tmp_path)
    assert verify_survey(good) == sha256_file(good / "survey.plan")
    (good / "survey.plan").write_text((good / "survey.plan").read_text() + " ")
    with pytest.raises(ValueError, match="differs"):
        verify_survey(good)
    failed = checked_plan(tmp_path / "high", height=130)
    with pytest.raises(ValueError, match="did not pass"):
        verify_survey(failed)


def test_mission_download_records_success_and_failure_without_raising(tmp_path):
    from wallering_mapping.recording import pull_mission
    report = pull_mission(tmp_path / "pull.json", lambda: SimpleNamespace(success=True, wp_received=41))
    assert report["success"] and report["wp_received"] == 41
    assert json.loads((tmp_path / "pull.json").read_text()) == report
    def timeout():
        raise TimeoutError("no MAVROS")
    failed = pull_mission(tmp_path / "failed.json", timeout)
    assert not failed["success"] and "TimeoutError" in failed["error"]
    with pytest.raises(FileExistsError):
        pull_mission(tmp_path / "pull.json", timeout)


def test_announcements_name_the_event_and_never_raise(tmp_path):
    from wallering_mapping.recording import ANNOUNCEMENTS, announce
    sent = []
    report = announce("started", tmp_path / "start.json",
                      lambda text, tune: sent.append((text, tune)) or {"statustext": True, "play_tune": True})
    assert sent == [ANNOUNCEMENTS["started"]] and report["statustext"] and report["play_tune"]
    assert len(report["text"]) <= 50  # MAVLink STATUSTEXT payload
    def broken(*_):
        raise RuntimeError("no rclpy")
    failed = announce("stopped", tmp_path / "stop.json", broken)
    assert not failed["statustext"] and "RuntimeError" in failed["error"]


def test_capture_passes_survey_and_announce_flags():
    args = parser().parse_args(["capture", "--output", "x", "--survey-check", "checked", "--announce"])
    assert str(args.survey_check) == "checked" and args.announce


@pytest.mark.parametrize("extra,message", [
    (["--camera-only", "--survey-check", "CHECKED"], "needs PX4 telemetry"),
    (["--survey-check", "FAILED"], "passed wr-map survey-check"),
    (["--camera-only", "--announce"], "remove --camera-only"),
])
def test_recorder_refuses_survey_and_announce_misuse_before_ros(tmp_path, extra, message):
    import subprocess
    from pathlib import Path
    repo = Path(__file__).resolve().parents[1]
    good = checked_plan(tmp_path / "good")
    failed = checked_plan(tmp_path / "failed", height=130)
    extra = [str(good) if value == "CHECKED" else str(failed) if value == "FAILED" else value
             for value in extra]
    result = subprocess.run(["bash", str(repo / "deploy/record-rosbag.sh"), "--output",
                             str(tmp_path / "session"), *extra], capture_output=True, text=True)
    assert result.returncode == 2 and message in result.stderr
    assert not (tmp_path / "session").exists()
