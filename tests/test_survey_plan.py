import json
from pathlib import Path

import pytest
from ruamel.yaml import YAML

from qgc_plans import HOME, simple, survey_plan, to_latlon, write_plan
from wallering_mapping.cli import main
from wallering_mapping.survey_plan import DEFAULT_LIMITS, check_survey, estimate_minutes, load_plan, write_check
from wallering_mapping.recording import verify_survey

REPO = Path(__file__).resolve().parents[1]
CAMERA = REPO / "configs/qgc-oak-rgb-12mp.json"
PROFILE = REPO / "configs/oakd-ros.yaml"
MAVSDK = Path(__file__).parent / "fixtures/qgc/mavsdk-survey.plan"


def checked(tmp_path, budget=15, profile=PROFILE, **plan):
    path = write_plan(tmp_path / "field.plan", survey_plan(**plan))
    return check_survey(path, CAMERA, profile, budget)


def test_real_qgroundcontrol_survey_from_mavsdk_is_labelled_by_transect():
    loaded = load_plan(MAVSDK)
    roles = [item["role"] for item in loaded["items"]]
    assert roles == ["simple", "turnaround", "camera_command", "leg_entry", "camera_command",
                     "leg_exit", "turnaround", "turnaround", "leg_entry", "camera_command",
                     "leg_exit", "turnaround", "camera_command"]
    assert [(leg["entry_seq"], leg["exit_seq"]) for leg in loaded["legs"]] == [(3, 5), (8, 10)]
    assert all(9 < leg["length_m"] < 40 for leg in loaded["legs"])
    assert not loaded["errors"]


def test_manual_camera_and_missing_fence_are_refused_for_the_mavsdk_sample():
    report = check_survey(MAVSDK, CAMERA, PROFILE, 15)
    assert not report["passed"]
    text = " ".join(report["errors"])
    assert "Custom Camera" in text and "geofence" in text and "explicit flight speed" in text


def test_qgroundcontrol_515_survey_passes_and_maps_every_px4_item(tmp_path):
    report = checked(tmp_path)
    assert report["passed"], report["errors"]
    survey = report["surveys"][0]
    assert survey["gsd_cm"] == pytest.approx(40 * 6.29 * 100 / (4056 * 4.8))
    assert survey["frontal_overlap_at_frame_rate"] > 80
    assert survey["blur_px"] < 0.5
    assert survey["legs"] == len(report["legs"]) >= 2
    assert survey["area_ha"] == pytest.approx(1.08, rel=1e-3)
    seqs = [row["mission_seq"] for row in report["mission_items"]]
    assert seqs == list(range(len(seqs)))
    roles = {row["role"] for row in report["mission_items"]}
    assert roles == {"simple", "turnaround", "leg_entry", "leg_exit", "camera_command"}
    for leg in report["legs"]:
        entry, exit_ = (report["mission_items"][leg[key]] for key in ("entry_seq", "exit_seq"))
        assert entry["role"] == "leg_entry" and exit_["role"] == "leg_exit"
        assert entry["transect"] == exit_["transect"] == leg["transect"]
    assert any("trigger" in warning for warning in report["warnings"])
    assert not report["flight_ready"]
    assert report["flight"]["estimated_minutes"] < 15


@pytest.mark.parametrize("plan,reason", [
    ({"height": 130}, "outside"),
    ({"height": 15, "speed": 10}, "forward overlap"),
    ({"speed": 12}, "motion blur"),
    ({"fence": False}, "geofence"),
    ({"camera": {"FocalLength": 4.5}}, "FocalLength"),
    ({"camera": {"Landscape": False}}, "Landscape"),
    ({"hover": True}, "hover-and-capture"),
    ({"distance_mode": 2}, "mean sea level"),
    ({"vehicle_type": 1}, "multirotor"),
    ({"firmware": 3}, "PX4"),
    ({"speed_item": False}, "explicit flight speed"),
    ({"width": 900, "depth": 900}, "flight time"),
])
def test_each_capture_and_safety_limit_refuses_the_plan(tmp_path, plan, reason):
    report = checked(tmp_path, **plan)
    assert not report["passed"]
    assert any(reason in error for error in report["errors"]), report["errors"]


def test_waypoints_outside_the_fence_and_tampered_spacing_are_refused(tmp_path):
    plan = survey_plan()
    fence = plan["geoFence"]["polygons"][0]["polygon"]
    plan["geoFence"]["polygons"][0]["polygon"] = [[lat, lon] for lat, lon in fence[:2]] + [
        [fence[0][0] + 0.0005, fence[1][1]], [fence[0][0] + 0.0005, fence[0][1]]]
    calc = plan["mission"]["items"][2]["TransectStyleComplexItem"]["CameraCalc"]
    calc["AdjustedFootprintSide"] *= 1.5
    report = check_survey(write_plan(tmp_path / "field.plan", plan), CAMERA, PROFILE, 15)
    text = " ".join(report["errors"])
    assert "outside the inclusion geofence" in text and "line spacing" in text


def test_other_qgroundcontrol_patterns_are_refused(tmp_path):
    plan = survey_plan()
    plan["mission"]["items"][2]["complexItemType"] = "CorridorScan"
    report = check_survey(write_plan(tmp_path / "field.plan", plan), CAMERA, PROFILE, 15)
    assert any("only Survey" in error for error in report["errors"])


def test_terrain_follow_points_are_leg_interior_and_turnaround_free_plans_warn(tmp_path):
    report = checked(tmp_path, distance_mode=3, terrain_points=True, turnaround=0)
    assert report["passed"], report["errors"]
    interior = [row for row in report["mission_items"] if row["role"] == "leg_interior"]
    assert len(interior) == len(report["legs"])
    assert any("no turnaround" in warning for warning in report["warnings"])


def test_terrain_metadata_cannot_lower_the_commanded_speed_for_blur_check(tmp_path):
    plan = survey_plan(distance_mode=3, speed=12)
    plan['mission']['items'][2]['TransectStyleComplexItem']['TerrainFlightSpeed'] = 1
    report = check_survey(write_plan(tmp_path / 'terrain-speed.plan', plan), CAMERA, PROFILE, 15)
    assert not report['passed']
    assert report['surveys'][0]['speed_m_s'] == 12
    assert any('motion blur' in error for error in report['errors'])


def test_items_out_of_transect_order_are_refused(tmp_path):
    plan = survey_plan()
    items = plan["mission"]["items"][2]["TransectStyleComplexItem"]["Items"]
    items[1]["params"][4:6], items[3]["params"][4:6] = items[3]["params"][4:6], items[1]["params"][4:6]
    report = check_survey(write_plan(tmp_path / "field.plan", plan), CAMERA, PROFILE, 15)
    assert any("does not follow the saved transects" in error for error in report["errors"])


def test_profile_must_fix_exposure_and_match_the_camera_resolution(tmp_path):
    yaml = YAML()
    profile = yaml.load(PROFILE.read_text())
    profile["/oak"]["ros__parameters"]["rgb"]["i_resolution"] = "4K"
    changed = tmp_path / "profile.yaml"
    with changed.open("w") as out:
        yaml.dump(profile, out)
    report = checked(tmp_path, profile=changed)
    assert any("resolution 4K" in error for error in report["errors"])
    profile["/oak"]["ros__parameters"]["rgb"]["r_set_man_exposure"] = False
    with changed.open("w") as out:
        yaml.dump(profile, out)
    with pytest.raises(ValueError, match="automatic exposure"):
        checked(tmp_path, profile=changed)


def test_checked_plan_is_copied_exactly_and_never_replaced(tmp_path):
    path = write_plan(tmp_path / "field.plan", survey_plan())
    report = check_survey(path, CAMERA, PROFILE, 15)
    output = write_check(path, tmp_path / "checked", report)
    assert (output / "survey.plan").read_bytes() == path.read_bytes()
    saved = json.loads((output / "survey-check.json").read_text())
    assert saved["plan"]["sha256"] == report["plan"]["sha256"]
    with pytest.raises(FileExistsError):
        write_check(path, tmp_path / "checked", report)
    assert verify_survey(output, PROFILE) == report["plan"]["sha256"]
    changed = tmp_path / "changed.yaml"
    changed.write_bytes(PROFILE.read_bytes() + b"\n# different active profile\n")
    with pytest.raises(ValueError, match="Active capture profile"):
        verify_survey(output, changed)
    path.write_text(path.read_text() + "\n")
    with pytest.raises(ValueError, match="Plan changed"):
        write_check(path, tmp_path / "changed-plan", report)


def test_cli_exit_status_follows_the_check(tmp_path, capsys):
    good = write_plan(tmp_path / "good.plan", survey_plan())
    bad = write_plan(tmp_path / "bad.plan", survey_plan(height=130))
    common = ["--camera", str(CAMERA), "--profile", str(PROFILE), "--flight-time-budget-min", "15"]
    assert main(["survey-check", str(good), *common, "--output", str(tmp_path / "ok")]) == 0
    assert main(["survey-check", str(bad), *common, "--output", str(tmp_path / "no")]) == 2
    assert json.loads((tmp_path / "no/survey-check.json").read_text())["passed"] is False
    capsys.readouterr()


def test_time_estimate_uses_each_active_speed_and_preserves_unchanged_speed(tmp_path):
    plan = survey_plan()
    plan['mission']['items'] = [
        simple(0, 178, 2, [1, 10, -1, 0, 0, 0, 0]),
        simple(1, 16, 3, [0, 0, 0, None, *to_latlon(100, 0), 0]),
        simple(2, 178, 2, [1, 1, -1, 0, 0, 0, 0]),
        simple(3, 16, 3, [0, 0, 0, None, *to_latlon(200, 0), 0]),
        simple(4, 178, 2, [1, -1, -1, 0, 0, 0, 0]),
        simple(5, 20, 2, [0, 0, 0, 0, 0, 0, 0]),
    ]
    loaded = load_plan(write_plan(tmp_path / 'speeds.plan', plan))
    errors = []
    minutes, path = estimate_minutes(HOME, loaded['items'], DEFAULT_LIMITS, errors, [])
    assert not errors
    assert path == pytest.approx(400)
    assert minutes * 60 == pytest.approx(10 + 100 + 200 + 3 * 2)


def test_slow_second_survey_exceeds_a_budget_that_fast_plan_meets(tmp_path):
    import copy
    plan = survey_plan()
    plan['mission']['items'].insert(-1, copy.deepcopy(plan['mission']['items'][2]))
    path = write_plan(tmp_path / 'two-surveys.plan', plan)
    fast = check_survey(path, CAMERA, PROFILE, 60)
    assert fast['passed'], fast['errors']
    slow = copy.deepcopy(plan['mission']['items'][1])
    slow['params'][1] = 1
    plan['mission']['items'].insert(3, slow)
    write_plan(path, plan)
    report = check_survey(path, CAMERA, PROFILE, fast['flight']['estimated_minutes'] + .5)
    assert not report['passed']
    assert any('Estimated flight time' in error for error in report['errors'])


@pytest.mark.parametrize('params', [[0, 2, -1, 0], [1, -2, -1, 0], [1, 2, -1, 1]])
def test_time_estimate_refuses_unmodelled_speed_commands(tmp_path, params):
    plan = survey_plan()
    plan['mission']['items'][1]['params'][:4] = params
    report = check_survey(write_plan(tmp_path / 'speed.plan', plan), CAMERA, PROFILE, 15)
    assert not report['passed']
    assert any('absolute groundspeed' in error for error in report['errors'])


@pytest.mark.parametrize('command', [177, 17, 18, 93, 400])
def test_time_estimate_refuses_unmodelled_execution_commands(tmp_path, command):
    plan = survey_plan()
    plan['mission']['items'].insert(-1, simple(99, command, 2, [2, 10, 0, 0, 0, 0, 0]))
    report = check_survey(write_plan(tmp_path / 'control.plan', plan), CAMERA, PROFILE, 15)
    assert not report['passed']
    assert any(f'unsupported mission command {command}' in error for error in report['errors'])


def test_manual_continuation_cannot_have_a_bounded_flight_time(tmp_path):
    plan = survey_plan()
    plan['mission']['items'][0]['autoContinue'] = False
    report = check_survey(write_plan(tmp_path / 'manual.plan', plan), CAMERA, PROFILE, 15)
    assert not report['passed']
    assert any('manual continuation' in error for error in report['errors'])
