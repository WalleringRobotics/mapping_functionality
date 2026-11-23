import json
from pathlib import Path

import pytest

from wallering_mapping.survey_design import read_aircraft

REPO = Path(__file__).resolve().parents[1]
TEMPLATE = REPO / "configs/px4-multirotor.template.json"
AIRCRAFT = {"schema": "wallering.px4-multirotor/v1", "schema_version": 1, "aircraft_id": "hexa-test",
            "source": "synthetic test values", "vehicle_type": 13, "max_survey_speed_m_s": 5.0,
            "accel_m_s2": 3.0, "acceptance_radius_m": 2.0, "climb_m_s": 3.0, "descent_m_s": 1.0,
            "return_height_m": 60.0, "yaw_rate_deg_s": 60.0, "yaw_accel_deg_s2": 20.0}


def write_aircraft(tmp_path, **changes):
    path = tmp_path / "aircraft.json"
    path.write_text(json.dumps({**AIRCRAFT, **changes}))
    return path


def test_aircraft_template_is_refused_until_filled_in():
    with pytest.raises(ValueError, match="aircraft_id"):
        read_aircraft(TEMPLATE)
    assert set(json.loads(TEMPLATE.read_text())) == set(AIRCRAFT)


def test_aircraft_definition_is_read_and_bounded(tmp_path):
    assert read_aircraft(write_aircraft(tmp_path))["accel_m_s2"] == 3.0
    for changes, message in (({"vehicle_type": 1}, "multirotor"),
                             ({"accel_m_s2": None}, "MPC_ACC_HOR"),
                             ({"max_survey_speed_m_s": 40}, "max_survey_speed_m_s"),
                             ({"acceptance_radius_m": True}, "finite"),
                             ({"schema": "other"}, "schema")):
        with pytest.raises(ValueError, match=message):
            read_aircraft(write_aircraft(tmp_path, **changes))


SPEC = json.loads((REPO / "configs/qgc-oak-rgb-12mp.json").read_text())["qgc_custom_camera"]
PROFILE = {"fps": 2.0, "exposure_us": 1000.0}


def test_worked_example_at_30_m_on_level_ground():
    from wallering_mapping.survey_design import design
    plan = design(SPEC, PROFILE, AIRCRAFT, height_m=30, relief_m=0)
    assert plan["gsd_cm"] == pytest.approx(0.969, abs=0.001)
    assert (plan["footprint_across_m"], plan["footprint_along_m"]) == pytest.approx((39.31, 29.46), abs=0.01)
    assert plan["line_spacing_m"] == pytest.approx(11.79, abs=0.01)
    assert plan["speed_limits_m_s"]["forward_overlap"] == pytest.approx(11.79, abs=0.01)
    assert plan["speed_limits_m_s"]["blur"] == pytest.approx(4.846, abs=0.001)
    assert (plan["binding_speed_limit"], plan["speed_m_s"]) == ("blur", 4.8)
    assert plan["front_overlap_worst"] == pytest.approx(91.9, abs=0.1)
    assert plan["blur_px_worst"] <= 0.5
    assert plan["acceleration_distance_m"] == pytest.approx(3.84)
    assert plan["turnaround_m"] == 7.9  # 3.84 m to reach speed + 2 m acceptance radius + 2 m margin
    assert (plan["lines_needed"], plan["frames_needed"]) == (3, 5)
    assert plan["sensitivities"]["speed_at_half_exposure_m_s"] == pytest.approx(5.0)  # aircraft limit
    assert plan["sensitivities"]["rolling_shutter_skew_px_per_ms"] == pytest.approx(0.495, abs=0.001)


def test_relief_tightens_spacing_and_speed_over_high_ground():
    from wallering_mapping.survey_design import design
    plan = design(SPEC, PROFILE, AIRCRAFT, height_m=30, relief_m=5)
    assert plan["line_spacing_m"] == pytest.approx(9.83, abs=0.01)
    assert plan["speed_m_s"] == 4.0 and plan["blur_px_worst"] <= 0.5
    # QGroundControl recomputes spacing from the saved overlap at the nominal height.
    assert plan["saved_side_overlap"] == pytest.approx(75.0, abs=0.01)
    assert plan["line_spacing_m"] == pytest.approx(
        plan["footprint_across_m"] * (1 - plan["saved_side_overlap"] / 100))
    assert plan["line_spacing_m"] <= plan["footprint_across_low_m"] * 0.3 + 1e-9


def test_target_gsd_is_held_over_the_lowest_ground_and_height_is_bounded():
    from wallering_mapping.survey_design import design
    plan = design(SPEC, PROFILE, AIRCRAFT, gsd_cm=1.0, relief_m=2)
    assert plan["height_m"] == pytest.approx(28.9) and plan["gsd_cm_worst"] <= 1.0
    for arguments, message in (({"gsd_cm": 0.2, "relief_m": 0}, "outside"),
                               ({"height_m": 12, "relief_m": 5}, "highest ground"),
                               ({"height_m": 30, "gsd_cm": 1, "relief_m": 0}, "exactly one"),
                               ({"height_m": 30, "relief_m": -1}, "relief"),
                               ({"height_m": 30, "relief_m": 0, "target": "forest"}, "Target type")):
        with pytest.raises(ValueError, match=message):
            design(SPEC, PROFILE, AIRCRAFT, **arguments)


def test_forward_overlap_binds_with_a_short_exposure_and_sets_the_along_margin():
    from wallering_mapping.survey_design import design
    plan = design(SPEC, {"fps": 2.0, "exposure_us": 200.0}, {**AIRCRAFT, "max_survey_speed_m_s": 15},
                  height_m=30, relief_m=0)
    assert (plan["binding_speed_limit"], plan["speed_m_s"]) == ("forward_overlap", 11.7)
    assert plan["front_overlap_worst"] >= 80
    assert plan["along_margin_m"] > plan["footprint_along_m"] / 2
    building = design(SPEC, PROFILE, AIRCRAFT, height_m=30, relief_m=0, target="building")
    assert building["crosshatch"] and building["requested"]["side_overlap"] == 75
