import json
from pathlib import Path

import pytest

from wallering_mapping import cli


@pytest.mark.parametrize("mode", ["solve", "camera"])
def test_nested_solve_commands_keep_flat_options(mode):
    args = cli.parser().parse_args(["calibrate", mode, "recording", "--calibration", "rig.json",
                                   "--output", "result", "--max-offset-ms", "30"])
    assert args.command == f"calibrate-{mode}"
    assert args.session == Path("recording")
    assert args.calibration == Path("rig.json")
    assert args.max_offset_ms == 30


@pytest.mark.parametrize("nested", [False, True])
def test_mission_cli_writes_unqualified_plan_and_matching_phase_map(tmp_path, capsys, nested):
    command = ["calibrate", "mission"] if nested else ["calibrate-mission"]
    output = tmp_path / "calibration.plan"
    result = cli.main(command + ["--home", "52,13", "--home-amsl-m", "42", "--alt", "20",
                                "--speed", "2", "--radius", "15", "--site-radius", "30",
                                "--margin", "10", "--max-alt", "30", "--output", str(output)])
    assert result == 0
    report = json.loads(capsys.readouterr().out)
    assert not report["flight_ready"]
    phases = json.loads(Path(report["phases"]).read_text())
    assert phases["plan_sha256"] == report["plan_sha256"]
    assert json.loads(output.read_text())["fileType"] == "Plan"


def test_nested_flight_phase_cli_cannot_claim_unverified_sequence(tmp_path, capsys):
    from test_calibration_flight import fixture
    root, plan, phases, _ = fixture(tmp_path)
    output = tmp_path / "phases-output"
    assert cli.main(["calibrate", "flight-phases", str(root), "--plan", str(plan),
                     "--phase-map", str(phases), "--output", str(output)]) == 2
    report = json.loads(capsys.readouterr().out)
    assert not report["sequence_qualified"]
    assert not report["survey_ready"]
    assert (output / "flight-phases.json").is_file()


def test_sync_bag_cli_preserves_diagnostic_qualification(tmp_path, capsys):
    from test_bag_association import fixture
    root = fixture(tmp_path / "capture")
    output = tmp_path / "poses"
    assert cli.main(["sync-bag", str(root), "--output", str(output)]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["associated"] == 3
    assert not report["survey_ready"]
    assert not report["timestamp_bridge"]["applied"]
    assert (output / "body-poses.csv").is_file()


def test_nested_record_uses_calibration_profile_and_rate_request():
    args = cli.parser().parse_args(["calibrate", "record", "--output", "capture"])
    assert args.command == "calibrate-record"
    assert args.config.name == "oakd-ros-calibration.yaml"
    assert args.px4_imu_rate == 100
