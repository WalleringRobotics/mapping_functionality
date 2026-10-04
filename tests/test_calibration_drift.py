import copy
import json

import numpy as np
import pytest

from wallering_mapping import calibration_drift as drift
from wallering_mapping.calibration_imu import calibration_entries, solve_gyros
from wallering_mapping.rig_calibration import RigCalibration, apply_entries, rotation_from_rpy_deg
from test_calibration_imu import hand_measured, recorded_session, synthetic


@pytest.fixture(scope="module")
def reference():
    oak, px4, _ = synthetic(seconds=40)
    result = solve_gyros(oak, px4)
    data = apply_entries(hand_measured(), calibration_entries(result))
    return result, RigCalibration(data)


@pytest.mark.parametrize("sigma,status", [(1., "pass"), (3.5, "warn"), (6., "fail")])
def test_offset_drift_thresholds(reference, sigma, status):
    result, rig = reference
    estimated = copy.deepcopy(result)
    estimated["offset_ns"] += round(sigma * np.sqrt(2) * result["offset_sigma_ns"])
    report = drift.compare(estimated, rig)
    assert report["status"] == status
    assert report["difference"]["offset_sigmas"] == pytest.approx(sigma, abs=1e-4)


def test_rotation_drift_and_unknown_translation(reference):
    result, rig = reference
    assert rig.transforms[("base_link", "oak_imu_frame")].covariance is None
    estimated = copy.deepcopy(result)
    estimated["rotation"] = rotation_from_rpy_deg([2, 0, 0]) @ result["rotation"]
    report = drift.compare(estimated, rig)
    assert report["status"] == "fail"
    assert report["difference"]["rotation_angle_rad"] == pytest.approx(np.deg2rad(2))


def test_missing_reference_is_unqualified(reference):
    result, _ = reference
    report = drift.compare(result, RigCalibration(hand_measured()))
    assert report["status"] == "unqualified"


def test_custom_thresholds_and_validation(reference):
    result, rig = reference
    estimated = copy.deepcopy(result)
    estimated["offset_ns"] += round(4 * np.sqrt(2) * result["offset_sigma_ns"])
    assert drift.compare(estimated, rig, warn_sigma=5, fail_sigma=8)["status"] == "pass"
    for limits in [(0, 5), (5, 3), (3, float("nan"))]:
        with pytest.raises(ValueError, match="thresholds"):
            drift.compare(result, rig, warn_sigma=limits[0], fail_sigma=limits[1])


def test_sealed_session_passes_own_result_without_mutation(tmp_path, reference):
    _, rig = reference
    oak, px4, _ = synthetic(seconds=40)
    session = recorded_session(tmp_path / "session", oak, px4)
    calibration = tmp_path / "rig.json"
    calibration.write_text(json.dumps(rig.data))
    before = calibration.read_bytes()
    report = drift.check_session(session, calibration)
    assert report["status"] == "pass"
    assert len(report["evidence"]["seal_sha256"]) == 64
    assert calibration.read_bytes() == before


def test_static_session_reports_insufficient_motion(tmp_path, reference):
    _, rig = reference
    oak, px4, _ = synthetic(seconds=10, axes=())
    session = recorded_session(tmp_path / "session", oak, px4)
    report = drift.check_session(session, rig)
    assert report["status"] == "insufficient_motion"


def test_corrupted_session_fails_instead_of_motion_warning(tmp_path, reference):
    _, rig = reference
    oak, px4, _ = synthetic(seconds=10, axes=())
    session = recorded_session(tmp_path / "session", oak, px4)
    (session / "topics.txt").write_text("altered")
    report = drift.check_session(session, rig)
    assert report["status"] == "fail" and "checksum" in report["reason"]


def test_validation_failure_does_not_change_calibration(tmp_path, reference, monkeypatch):
    _, rig = reference
    monkeypatch.setattr(drift, "check_session", lambda *a, **k: {
        "status": "fail", "recommendation": "Recalibrate"})
    report = {"valid": True, "survey_ready": True, "warnings": [], "errors": []}
    drift.add_to_validation(report, tmp_path, rig)
    assert not report["valid"] and not report["survey_ready"]
    assert report["errors"] == ["Calibration drift check failed: Recalibrate"]


def test_validate_cli_includes_drift_and_failure_exit(tmp_path, reference, monkeypatch, capsys):
    from wallering_mapping import bags, cli
    _, rig = reference
    session = tmp_path / "session"
    session.mkdir()
    (session / "state").write_text("complete")
    monkeypatch.setattr(bags, "audit_bag", lambda path: {
        "valid": True, "survey_ready": True, "errors": [], "warnings": []})
    def checked(*args, **kwargs):
        assert kwargs == {"warn_sigma": 2., "fail_sigma": 4.}
        return {"status": "fail", "recommendation": "Recalibrate"}
    monkeypatch.setattr(drift, "check_session", checked)
    report = tmp_path / "report.json"
    status = cli.main(["validate", str(session), "--rig-calibration", str(tmp_path / "rig.json"),
                       "--calibration-warn-sigma", "2", "--calibration-fail-sigma", "4", "--report", str(report)])
    assert status == 2
    assert json.loads(report.read_text())["calibration_check"]["status"] == "fail"
    capsys.readouterr()
