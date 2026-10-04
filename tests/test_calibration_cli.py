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
