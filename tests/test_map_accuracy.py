import copy

import numpy as np
import pytest

from wallering_mapping.dataset import write_json
from wallering_mapping.map_accuracy import MAP_PROFILE, assess


def inputs(tmp_path, n=3):
    profile = copy.deepcopy(MAP_PROFILE)
    profile.update(
        crs="EPSG:32633",
        vertical_datum="WGS84 ellipsoidal height",
        reference_survey_evidence="synthetic withheld survey",
        independence_evidence="synthetic no fitted checks",
        map_product_evidence="synthetic existing metric model",
        minimum_checkpoints=3,
        shared_base_covariance_map_xyz_m2=np.diag([0.0001, 0.0001, 0.0004]).tolist(),
    )
    path = tmp_path / "profile.json"
    write_json(path, profile)
    checks = tmp_path / "checks.csv"
    header = "id,role,surface_class,reference_x_m,reference_y_m,reference_z_m,model_x_m,model_y_m,model_z_m,reference_sigma_x_m,reference_sigma_y_m,reference_sigma_z_m\n"
    lines = []
    for index in range(n):
        x, y = index % 2, index // 2
        lines.append(f"{index},check,hard,{x},{y},0,{x + 0.003},{y + 0.004},.002,.001,.001,.002")
    checks.write_text(header + "\n".join(lines) + "\n")
    return checks, path


def test_known_residual_bias_reference_uncertainty_and_shared_base_breakdown(tmp_path):
    checks, profile = inputs(tmp_path)
    report = assess(checks, profile, tmp_path / "report")
    measured = report["measured_checkpoint_comparison"]
    assert measured["rmse_horizontal_m"] == pytest.approx(0.005)
    assert measured["rmse_vertical_m"] == pytest.approx(0.002)
    np.testing.assert_allclose(measured["bias_xyz_m"], [0.003, 0.004, 0.002])
    np.testing.assert_allclose(measured["centred_rmse_axes_m"], [0, 0, 0], atol=1e-15)
    assert report["indicative_reference_aware_rmse_envelope"]["horizontal_m"] == pytest.approx(
        np.sqrt(0.005**2 + 2 * 0.001**2 + 2 * 0.01**2)
    )
    assert report["passed"] and "Not assessed" in report["standards_compliance"]
    assert (tmp_path / "report/report.html").exists()


def test_more_checkpoints_never_divide_shared_base_uncertainty(tmp_path):
    a = tmp_path / "a"
    b = tmp_path / "b"
    a.mkdir()
    b.mkdir()
    ac, ap = inputs(a, 3)
    bc, bp = inputs(b, 30)
    ra = assess(ac, ap, a / "report")
    rb = assess(bc, bp, b / "report")
    assert ra["indicative_reference_aware_rmse_envelope"]["horizontal_m"] == pytest.approx(
        rb["indicative_reference_aware_rmse_envelope"]["horizontal_m"]
    )
    assert rb["sample_size"]["actual"] == 30


def test_controls_are_not_checks_and_no_fit_on_held_out_points(tmp_path):
    checks, profile = inputs(tmp_path)
    import json

    p = json.loads(profile.read_text())
    p["control_ids"] = ["1"]
    write_json(profile, p)
    with pytest.raises(ValueError, match="fitted control"):
        assess(checks, profile, tmp_path / "report")
    assert not (tmp_path / "report").exists()


def test_small_sample_cannot_pass_30_point_profile(tmp_path):
    checks, profile = inputs(tmp_path)
    import json

    p = json.loads(profile.read_text())
    p["minimum_checkpoints"] = 30
    write_json(profile, p)
    report = assess(checks, profile, tmp_path / "report")
    assert not report["passed"] and not report["sample_size"]["sufficient"]
    assert report["observed_target_passed"]
    assert "Empirical sample percentiles" in report["percentile_meaning"]


def test_external_checks_do_not_add_common_base_twice(tmp_path):
    checks, profile = inputs(tmp_path)
    import json

    p = json.loads(profile.read_text())
    p["shared_base_with_map"] = False
    write_json(profile, p)
    with pytest.raises(ValueError, match="external checkpoints"):
        assess(checks, profile, tmp_path / "invalid")
    p["shared_base_covariance_map_xyz_m2"] = None
    write_json(profile, p)
    report = assess(checks, profile, tmp_path / "report")
    assert report["indicative_reference_aware_rmse_envelope"]["horizontal_m"] == pytest.approx(
        np.sqrt(0.005**2 + 2 * 0.001**2)
    )
