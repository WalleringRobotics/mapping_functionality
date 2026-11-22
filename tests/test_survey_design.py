import json
from pathlib import Path

import pytest

from wallering_mapping.survey_design import read_aircraft

REPO = Path(__file__).resolve().parents[1]
TEMPLATE = REPO / "configs/px4-multirotor.template.json"
AIRCRAFT = {"schema": "wallering.px4-multirotor/v1", "schema_version": 1, "aircraft_id": "hexa-test",
            "source": "synthetic test values", "vehicle_type": 13, "max_survey_speed_m_s": 5.0,
            "accel_m_s2": 3.0, "acceptance_radius_m": 2.0, "climb_m_s": 3.0, "descent_m_s": 1.0,
            "return_height_m": 60.0}


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
