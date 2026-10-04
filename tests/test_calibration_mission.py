import argparse
import hashlib
import json

import pytest
from pyproj import Geod

from wallering_mapping.calibration_mission import add_arguments, generate_mission, run


def mission(tmp_path, **overrides):
    inputs = dict(home=(52, 13), home_amsl_m=42, alt=20, speed=2, radius=15,
                  site_radius=30, margin=10, max_alt=30, output=tmp_path / "calibration.plan")
    inputs.update(overrides)
    return generate_mission(**inputs)


def test_plan_is_deterministic_and_all_navigation_stays_within_site(tmp_path):
    result = mission(tmp_path)
    repeated = mission(tmp_path / "second")
    assert result["plan_sha256"] == repeated["plan_sha256"]
    plan = json.loads((tmp_path / "calibration.plan").read_text())
    phase_map = json.loads((tmp_path / "calibration.plan.phases.json").read_text())
    assert plan["mission"]["plannedHomePosition"] == [52, 13, 42]
    assert plan["fileType"] == "Plan" and plan["version"] == 1
    assert (plan["mission"]["firmwareType"], plan["mission"]["vehicleType"]) == (12, 2)
    items = plan["mission"]["items"]
    assert items[0]["command"] == 22 and items[-1]["command"] == 21
    assert items[1]["command"] == 178 and items[1]["params"][:3] == [1, 2, -1]
    geod = Geod(ellps="WGS84")
    for seq, (item, phase) in enumerate(zip(items, phase_map["mission_items"], strict=True)):
        assert item["doJumpId"] == phase["do_jump_id"] == seq + 1
        assert phase["mission_seq"] == seq
        assert item["type"] == "SimpleItem" and len(item["params"]) == 7
        if item["command"] == 178:
            continue
        lat, lon, height = item["params"][4:]
        assert item["frame"] == 3 and item["AltitudeMode"] == 1
        assert 0 <= height <= 30 and item["Altitude"] == height
        assert geod.inv(13, 52, lon, lat)[2] < 20
    assert {phase["phase"] for phase in phase_map["mission_items"]} == {
        "takeoff", "speed", "yaw", "figure_eight", "translation", "altitude", "return", "land"}
    assert set(phase_map["acceptance"].values()) == {"pending"}
    assert not phase_map["flight_ready"] and not phase_map["sequence_mapping_verified"]
    assert phase_map["plan_sha256"] == hashlib.sha256((tmp_path / "calibration.plan").read_bytes()).hexdigest()


@pytest.mark.parametrize("overrides,reason", [
    ({"home": (float("nan"), 13)}, "finite"),
    ({"home": (90, 13)}, "latitude"),
    ({"home": (52,)}, "home"),
    ({"speed": 6}, "speed"),
    ({"radius": 25}, "clear site"),
    ({"margin": 5, "speed": 2}, "three seconds"),
    ({"alt": 29}, "ceiling"),
    ({"altitude_step": 0}, "altitude_step"),
])
def test_unsafe_or_invalid_parameters_do_not_create_outputs(tmp_path, overrides, reason):
    with pytest.raises(ValueError, match=reason):
        mission(tmp_path, **overrides)
    assert not list(tmp_path.iterdir())


def test_no_replacement_even_when_only_phase_map_exists(tmp_path):
    sidecar = tmp_path / "calibration.plan.phases.json"
    sidecar.write_text("operator evidence")
    with pytest.raises(FileExistsError):
        mission(tmp_path)
    assert sidecar.read_text() == "operator evidence"
    assert not (tmp_path / "calibration.plan").exists()


def test_parser_requires_site_inputs_and_generation_never_claims_flight_acceptance(tmp_path):
    parser = argparse.ArgumentParser()
    add_arguments(parser)
    args = parser.parse_args(["--home", "52,13", "--home-amsl-m", "42", "--alt", "20",
                              "--speed", "2", "--radius", "15", "--site-radius", "30",
                              "--margin", "10", "--max-alt", "30", "--output", str(tmp_path / "a.plan")])
    assert run(args)["flight_ready"] is False
    with pytest.raises(SystemExit):
        parser.parse_args(["--home", "52,13"])
