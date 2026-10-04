import json

import numpy as np
import pytest

from wallering_mapping import calibration_camera as cc, calibration_workflow as workflow, cli
from wallering_mapping.rig_calibration import RigCalibration
from test_calibration_imu import hand_measured


@pytest.fixture
def camera_workflow(tmp_path, monkeypatch):
    session = tmp_path / "session"
    session.mkdir()
    (session / "TESTDEVICE_calibration.json").write_text(json.dumps({"cameraData": []}))
    calibration = tmp_path / "rig.json"
    calibration.write_text(json.dumps(hand_measured()))
    result = {"qualified": True, "seal_sha256": "a" * 64,
              "entries": cc.calibration_entries(np.eye(3), [.001] * 3, 7300000, 100000, "fixture")}
    monkeypatch.setattr(workflow, "solve_camera_imu", lambda *a, **k: result)
    return session, calibration, result


def test_camera_cli_writes_new_calibration_and_report(tmp_path, camera_workflow, capsys):
    session, calibration, _ = camera_workflow
    output = tmp_path / "solved"
    original = calibration.read_bytes()
    arguments = ["calibrate-camera", str(session), "--calibration", str(calibration), "--output", str(output)]
    assert cli.main(arguments) == 0
    rig = RigCalibration.read(output / "rig-calibration.json")
    assert rig.offsets[("oak_camera_exposure", "oak_imu")]["offset_ns"] == 7300000
    assert json.loads((output / "report.json").read_text())["camera_to_imu"]["qualified"]
    assert calibration.read_bytes() == original
    assert cli.main(arguments) == 2
    assert "new directory" in capsys.readouterr().err


def test_camera_workflow_refuses_unqualified_result(tmp_path, camera_workflow):
    session, calibration, result = camera_workflow
    result["qualified"] = False
    with pytest.raises(ValueError, match="stereo cross-check"):
        workflow.solve_camera(session, calibration, tmp_path / "solved")
    assert not (tmp_path / "solved").exists()


def test_camera_workflow_does_not_write_into_session(camera_workflow):
    session, calibration, _ = camera_workflow
    with pytest.raises(ValueError, match="immutable source"):
        workflow.solve_camera(session, calibration, session / "solved")


def test_camera_workflow_refuses_mounting_disagreement(tmp_path, camera_workflow, monkeypatch):
    session, calibration, _ = camera_workflow
    monkeypatch.setattr(workflow, "check", lambda *a: {"direct_vs_imu_chain": {"consistent_3_sigma": False}})
    with pytest.raises(ValueError, match="manual camera mounting"):
        workflow.solve_camera(session, calibration, tmp_path / "solved")
    assert not (tmp_path / "solved").exists()


def test_failed_publication_leaves_no_partial_output(tmp_path, camera_workflow, monkeypatch):
    session, calibration, _ = camera_workflow
    def fail(*a, **k):
        raise OSError("simulated disk failure")
    monkeypatch.setattr(workflow, "write_json", fail)
    with pytest.raises(OSError, match="disk failure"):
        workflow.solve_camera(session, calibration, tmp_path / "solved")
    assert sorted(p.name for p in tmp_path.iterdir()) == ["rig.json", "session"]
