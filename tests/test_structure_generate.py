import json
import math
from pathlib import Path
from xml.etree import ElementTree

import pytest

from qgc_plans import HOME, survey_plan, to_latlon
from test_survey_design import AIRCRAFT
from wallering_mapping.cli import main
from wallering_mapping.structure_check import check_structure, convex_hull, structure_faces
from wallering_mapping.survey_plan import local_xy

pytest.importorskip("shapely")
from wallering_mapping.structure_generate import plan_structure, write_bundle  # noqa: E402
from wallering_mapping.survey_generate import SurveyRefused  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
CAMERA = REPO / "configs/qgc-oak-rgb-12mp.json"
MOUNT = REPO / "configs/oak-forward-mount.json"
PROFILE = REPO / "configs/oakd-ros.yaml"
SHAPES = {"rectangle": [(40, 30), (60, 30), (60, 42), (40, 42)],
          "rotated": [(40, 30), (58, 39), (52, 51), (34, 42)],
          "triangle": [(40, 30), (70, 34), (48, 58)],
          "hexagon": [(40, 30), (52, 26), (64, 32), (66, 46), (54, 52), (42, 46)],
          "l_shape": [(40, 30), (64, 30), (64, 40), (52, 40), (52, 54), (40, 54)]}


def outline(shape):
    return [tuple(to_latlon(*xy)) for xy in SHAPES[shape]]


def aircraft_file(tmp_path, **changes):
    path = tmp_path / "aircraft.json"
    path.write_text(json.dumps({**AIRCRAFT, **changes}))
    return path


def generate(tmp_path, shape="rectangle", budget=30, wall=6, base=0, aircraft=None, mount=MOUNT, **request):
    if "gsd_cm" not in request:
        request.setdefault("standoff_m", 10)
    return plan_structure(outline=outline(shape), home=HOME, structure_height_m=wall, base_m=base, camera=CAMERA,
                          mount=mount, profile=PROFILE, aircraft=aircraft_file(tmp_path, **(aircraft or {})),
                          flight_time_budget_min=budget, grid_m=1.0, **request)


def recheck(tmp_path, plan, shape="rectangle", wall=6, base=0, budget=30, mount=MOUNT, **options):
    path = tmp_path / "changed.plan"
    path.write_text(json.dumps(plan))
    return check_structure(path, outline(shape), structure_height_m=wall, base_m=base, camera_path=CAMERA,
                           mount_path=mount, profile_path=PROFILE, aircraft_path=aircraft_file(tmp_path),
                           flight_time_budget_min=budget, grid_m=1.0, **options)[0]


def waypoints(plan):
    return [item for item in plan["mission"]["items"] if item["command"] == 16]


@pytest.mark.parametrize("shape", SHAPES)
@pytest.mark.parametrize("request_", [{}, {"standoff_m": 20}, {"gsd_cm": 0.8}, {"base": 3, "wall": 12}])
def test_every_generated_plan_passes_the_independent_check(tmp_path, shape, request_):
    bundle = generate(tmp_path, shape, **request_)
    check, review = bundle["check"], bundle["review"]
    pointing = check["pointing"]
    assert check["passed"] and check["errors"] == [] and check["flight_ready"] is False
    assert pointing["failing_samples"] == 0 and pointing["min_lines"] >= pointing["lines_needed"] == 2
    assert pointing["blur_px"] <= 0.5 + 1e-6
    assert pointing["largest_yaw_step_deg"] <= 30.5
    assert pointing["coarsest_gsd_cm"] <= review["design"]["gsd_cm_coarsest"] * 1.01
    assert check["clearance"]["closest_m"] >= 0.9 * review["design"]["standoff_m"]
    assert check["geofence"]["outside"] == []
    assert ("concave" in " ".join(check["warnings"])) is (shape == "l_shape")
    assert review["flight_ready"] is False and review["yaw"]["yaw_rate_while_imaging_deg_s"] == 0.0


def test_plan_is_plain_waypoints_that_all_carry_a_yaw_and_is_repeatable(tmp_path):
    bundle = generate(tmp_path)
    plan = json.loads(bundle["plan"])
    commands = [item["command"] for item in plan["mission"]["items"]]
    assert all(item["type"] == "SimpleItem" for item in plan["mission"]["items"])
    assert commands[:2] == [178, 22] and commands[-1] == 20 and set(commands[2:-1]) == {16}
    assert plan["mission"]["items"][0]["params"][:4] == [1, 1.4, -1, 0]
    assert plan["mission"]["firmwareType"] == 12 and plan["mission"]["vehicleType"] == 13
    assert len({item["doJumpId"] for item in plan["mission"]["items"]}) == len(commands)
    for item in waypoints(plan):
        assert isinstance(item["params"][3], float) and 0 <= item["params"][3] < 360
        assert item["frame"] == 3 and item["AltitudeMode"] == 1 and item["Altitude"] == item["params"][6]
    assert generate(tmp_path)["plan"] == bundle["plan"]
    assert generate(tmp_path, standoff_m=12)["plan"] != bundle["plan"]


@pytest.mark.parametrize("shape", ["rectangle", "rotated", "hexagon"])
def test_passes_are_flown_sideways_with_the_nose_square_to_the_wall_at_the_stand_off(tmp_path, shape):
    bundle = generate(tmp_path, shape)
    review = bundle["review"]
    by_seq = {seq: item for seq, item in enumerate(json.loads(bundle["plan"])["mission"]["items"])}
    _, faces, _ = structure_faces(HOME, outline(shape))
    assert len(review["faces"]) == len(faces) and review["totals"]["passes"] == 3 * len(faces)
    for face, row in zip(faces, review["faces"]):
        for flown in row["passes"]:
            entry, exit_ = by_seq[flown["entry_seq"]], by_seq[flown["exit_seq"]]
            assert entry["params"][3] == exit_["params"][3] == pytest.approx(face["yaw_deg"], abs=0.01)
            assert entry["params"][6] == exit_["params"][6] == pytest.approx(flown["height_m"], abs=0.01)
            nose = (math.sin(math.radians(entry["params"][3])), math.cos(math.radians(entry["params"][3])))
            start, end = (local_xy(HOME, item["params"][4:6]) for item in (entry, exit_))
            travel = (end[0] - start[0], end[1] - start[1])
            # The nose points into the wall and the aircraft moves at right angles to it.
            assert nose[0] * face["normal"][0] + nose[1] * face["normal"][1] == pytest.approx(-1, abs=1e-6)
            assert abs(nose[0] * travel[0] + nose[1] * travel[1]) < 0.01
            for point in (start, end):
                offset = ((point[0] - face["start"][0]) * face["normal"][0]
                          + (point[1] - face["start"][1]) * face["normal"][1])
                assert offset == pytest.approx(10.0, abs=0.01)
            assert math.hypot(*travel) >= face["length_m"]


def test_corner_stops_step_the_yaw_from_one_face_to_the_next_and_hold_still(tmp_path):
    bundle = generate(tmp_path)
    stops = [item for item in waypoints(json.loads(bundle["plan"])) if item["params"][0] == 3.0]
    assert len(stops) == bundle["review"]["totals"]["corner_stops"] == 4 * 2 * 3
    assert sorted({item["params"][3] for item in stops}) == [30.0, 60.0, 120.0, 150.0, 210.0, 240.0, 300.0, 330.0]
    assert bundle["review"]["yaw"]["headings_deg"] == [float(a) for a in range(0, 360, 30)]
    corner = convex_hull(SHAPES["rectangle"])
    for item in stops:
        x, y = local_xy(HOME, item["params"][4:6])
        nearest = min(corner, key=lambda c: math.hypot(c[0] - x, c[1] - y))
        assert math.hypot(nearest[0] - x, nearest[1] - y) == pytest.approx(10.0, abs=0.01)
        bearing = math.degrees(math.atan2(nearest[0] - x, nearest[1] - y)) % 360
        assert bearing == pytest.approx(item["params"][3], abs=0.01)
    wide = generate(tmp_path, requirements={"max_yaw_step_deg": 45})
    assert wide["review"]["totals"]["corner_stops"] == 4 * 1 * 3


def test_flight_is_simulated_from_take_off_to_landing_with_a_still_yaw_while_frames_count(tmp_path):
    bundle = generate(tmp_path)
    keys, mission = bundle["check"]["timeline"]["keys"], bundle["mission"]
    times = [key[0] for key in keys]
    assert times == sorted(times) and keys[0][1:4] == [0.0, 0.0, 0.0] and keys[-1][1:4] == [0.0, 0.0, 0.0]
    assert times[-1] / 60 == pytest.approx(bundle["check"]["flight"]["estimated_minutes"], abs=1e-3)
    steps = mission["steps"]
    assert steps[0]["kind"] == "takeoff" and [s["kind"] for s in steps[-3:]] == ["climb_out", "return", "land"]
    assert all(a["t1"] == b["t0"] for a, b in zip(steps, steps[1:])) and steps[-1]["t1"] == times[-1]
    assert sum(step["kind"] == "pass" for step in steps) == 12
    assert sum(window[2] == "pass" for window in mission["windows"]) == 12
    for start, end, _ in mission["windows"]:
        inside = [key for key in keys if start - 1e-6 <= key[0] <= end + 1e-6]
        assert len({key[4] for key in inside}) == 1
    assert max(key[3] for key in keys) == pytest.approx(11.9, abs=0.01)


def changed(bundle, change):
    plan = json.loads(bundle["plan"])
    change(plan, waypoints(plan))
    return plan


def test_check_refuses_a_waypoint_without_a_yaw_and_a_pattern_item(tmp_path):
    bundle = generate(tmp_path)

    def strip(plan, points):
        points[5]["params"][3] = None
    report = recheck(tmp_path, changed(bundle, strip))
    assert not report["passed"] and any("carries no yaw" in error for error in report["errors"])

    survey = survey_plan()
    report = recheck(tmp_path, survey)
    assert any("pattern cannot carry a yaw" in error for error in report["errors"])


def test_check_refuses_a_plan_that_points_the_camera_the_wrong_way(tmp_path):
    bundle = generate(tmp_path)

    def turn(plan, points):
        for item in points:
            item["params"][3] = (item["params"][3] + 90) % 360
    report = recheck(tmp_path, changed(bundle, turn))
    assert not report["passed"] and report["pointing"]["min_lines"] == 0
    assert any("wall samples are seen sharply from fewer" in error for error in report["errors"])


def test_check_refuses_a_missing_pass_too_fast_a_speed_and_a_wrong_pitch(tmp_path):
    bundle = generate(tmp_path)
    top = bundle["review"]["faces"][0]["passes"][-1]["height_m"]
    assert top == 11.88

    def lower(plan, points):
        for item in points:
            if item["params"][6] == top:
                item["params"][6] = item["Altitude"] = 8.44
    report = recheck(tmp_path, changed(bundle, lower))
    # Flying the middle pass twice does not replace the top one: one line is one view.
    assert not report["passed"] and report["pointing"]["failing_samples"] > 0
    assert report["pointing"]["min_lines"] == 1

    def faster(plan, points):
        plan["mission"]["items"][0]["params"][1] = 3.0
    report = recheck(tmp_path, changed(bundle, faster))
    assert report["pointing"]["min_lines"] == 0 and report["pointing"]["blur_px_on_uncounted_legs"] > 0.5

    steep = tmp_path / "mount.json"
    steep.write_text(json.dumps({**json.loads(MOUNT.read_text()), "pitch_down_deg": 48.0}))
    report = recheck(tmp_path, json.loads(bundle["plan"]), mount=steep)
    assert not report["passed"] and report["pointing"]["failing_samples"] > 0
    assert not recheck(tmp_path, json.loads(bundle["plan"]), target_gsd_cm=0.4)["passed"]


def test_check_refuses_a_path_too_close_an_unsettled_yaw_and_unlinked_faces(tmp_path):
    bundle = generate(tmp_path)

    def closer(plan, points):
        for item in points[1:5]:
            item["params"][4] += 7 / 111_195  # seven metres north, towards the south wall
    report = recheck(tmp_path, changed(bundle, closer))
    assert any("comes within 3.0 m of the structure" in error for error in report["errors"])

    def no_hold(plan, points):
        points[0]["params"][0] = 0
    report = recheck(tmp_path, changed(bundle, no_hold))
    assert any("starts a new yaw without a 2.0 s hold" in error for error in report["errors"])

    def no_corners(plan, points):
        plan["mission"]["items"] = [item for item in plan["mission"]["items"]
                                    if item["command"] != 16 or item["params"][0] != 3.0]
    report = recheck(tmp_path, changed(bundle, no_corners))
    assert report["pointing"]["largest_yaw_step_deg"] == 90.0
    assert any("yaw steps 90.0 degrees" in error for error in report["errors"])


def test_check_refuses_low_flight_missing_fence_and_missing_speed(tmp_path):
    bundle = generate(tmp_path)

    def low(plan, points):
        points[3]["params"][6] = points[3]["Altitude"] = 3.0
    assert any("minimum is 5.0 m" in error for error in recheck(tmp_path, changed(bundle, low))["errors"])

    def no_fence(plan, points):
        plan["geoFence"]["polygons"] = []
    assert any("no inclusion geofence" in error for error in recheck(tmp_path, changed(bundle, no_fence))["errors"])

    def no_speed(plan, points):
        plan["mission"]["items"] = plan["mission"]["items"][1:]
    assert any("exactly one absolute groundspeed" in error
               for error in recheck(tmp_path, changed(bundle, no_speed))["errors"])

    def no_return(plan, points):
        plan["mission"]["items"] = plan["mission"]["items"][:-1]
    assert any("end with one Return" in error for error in recheck(tmp_path, changed(bundle, no_return))["errors"])


def test_over_budget_is_refused_with_a_stand_off_that_fits(tmp_path):
    with pytest.raises(SurveyRefused) as refusal:
        generate(tmp_path, budget=9)
    assert any("exceeds the 9.0 min budget" in error for error in refusal.value.errors)
    further = refusal.value.alternatives["further_stand_off"]
    assert further["standoff_m"] > 10 and further["estimated_minutes"] <= 9
    assert generate(tmp_path, budget=9, standoff_m=further["standoff_m"])["check"]["passed"]
    with pytest.raises(SurveyRefused, match="foot of the wall"):
        generate(tmp_path, standoff_m=6)
    with pytest.raises(SurveyRefused, match="height limit"):
        generate(tmp_path, wall=100, standoff_m=60)


def test_bundle_holds_the_plan_its_check_and_a_self_contained_mission_view(tmp_path):
    bundle = generate(tmp_path)
    output = write_bundle(bundle, tmp_path / "out")
    assert sorted(path.name for path in output.iterdir()) == sorted(bundle["review"]["files"])
    page = (output / "mission.html").read_text()
    assert "__MISSION_DATA__" not in page and bundle["review"]["plan_sha256"] in page
    embedded = json.loads(page.split('<script id="mission" type="application/json">')[1].split("</script>")[0])
    assert embedded["passed"] is True and len(embedded["faces"]) == 4 and len(embedded["steps"]) > 30
    assert all(len(face["lines"]) == face["rows"] * face["columns"] for face in embedded["faces"])
    assert "<script src" not in page and "<link" not in page  # nothing to install or fetch but map tiles
    layers = {feature["properties"]["layer"] for feature in
              json.loads((output / "review.geojson").read_text())["features"]}
    assert layers == {"outline", "hull", "home", "fence", "path", "passes", "stops"}
    names = [element.text for element in ElementTree.parse(output / "review.kml").iter()
             if element.tag.endswith("name")]
    assert {"structure", "path", "fence", "take-off"} <= set(names)
    with pytest.raises(FileExistsError):
        write_bundle(bundle, tmp_path / "out")


def test_cli_plans_then_rechecks_and_refuses_a_changed_plan(tmp_path, capsys):
    ring = [[lon, lat] for lat, lon in outline("rectangle")]
    shape = tmp_path / "barn.geojson"
    shape.write_text(json.dumps({"type": "Feature", "properties": {},
                                 "geometry": {"type": "Polygon", "coordinates": [ring + ring[:1]]}}))
    common = ["--outline", str(shape), "--structure-height-m", "6", "--base-m", "0", "--aircraft",
              str(aircraft_file(tmp_path)), "--camera", str(CAMERA), "--mount", str(MOUNT), "--profile",
              str(PROFILE), "--flight-time-budget-min", "20"]
    out = tmp_path / "plan"
    assert main(["structure-plan", *common, "--home", f"{HOME[0]},{HOME[1]}", "--home-amsl-m", str(HOME[2]),
                 "--standoff-m", "10", "--output", str(out)]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["passed"] and result["flight_ready"] is False and result["speed_m_s"] == 1.4
    assert result["pass_heights_m"] == [5.0, 8.44, 11.88] and Path(result["mission_view"]).is_file()
    assert main(["structure-plan", *common, "--home", f"{HOME[0]},{HOME[1]}", "--home-amsl-m", str(HOME[2]),
                 "--standoff-m", "6", "--output", str(tmp_path / "close")]) == 2
    assert "foot of the wall" in capsys.readouterr().out and not (tmp_path / "close").exists()

    assert main(["structure-check", str(out / "structure.plan"), *common, "--output", str(tmp_path / "ok")]) == 0
    assert json.loads(capsys.readouterr().out)["plan"]["sha256"] == result["plan_sha256"]
    plan = json.loads((out / "structure.plan").read_text())
    for item in waypoints(plan):
        item["params"][3] = 0.0
    edited = tmp_path / "edited.plan"
    edited.write_text(json.dumps(plan))
    assert main(["structure-check", str(edited), *common, "--output", str(tmp_path / "bad")]) == 2
    assert json.loads(capsys.readouterr().out)["passed"] is False
