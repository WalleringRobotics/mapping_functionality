import json
from pathlib import Path
from xml.etree import ElementTree

import pytest

from qgc_plans import HOME, survey_plan, to_latlon
from test_survey_design import AIRCRAFT
from wallering_mapping.cli import main
from wallering_mapping.recording import verify_survey
from wallering_mapping.survey_coverage import check_coverage
from wallering_mapping.survey_plan import CAMERA_COMMANDS, check_survey

pytest.importorskip("shapely")
from wallering_mapping.survey_generate import SurveyRefused, plan_survey, write_bundle  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
CAMERA = REPO / "configs/qgc-oak-rgb-12mp.json"
PROFILE = REPO / "configs/oakd-ros.yaml"
SPEC = json.loads(CAMERA.read_text())["qgc_custom_camera"]
RELATIVE_HEIGHT = "height is relative to take-off"
SHAPES = {"rectangle": [(20, 30), (140, 30), (140, 120), (20, 120)],
          "rotated": [(20, 30), (120, 80), (90, 150), (-10, 100)],
          "concave": [(20, 30), (160, 30), (160, 80), (90, 80), (90, 160), (20, 160)],
          "triangle": [(20, 30), (200, 40), (60, 140)]}


def boundary(shape):
    return [tuple(to_latlon(*xy)) for xy in SHAPES[shape]]


def generate(tmp_path, shape="rectangle", budget=30, aircraft=None, profile=PROFILE, **request):
    path = tmp_path / "aircraft.json"
    path.write_text(json.dumps({**AIRCRAFT, **(aircraft or {})}))
    request.setdefault("relief_m", 0)
    if "gsd_cm" not in request:
        request.setdefault("height_m", 30)
    return plan_survey(boundary=boundary(shape), home=HOME, camera=CAMERA, profile=profile, aircraft=path,
                       flight_time_budget_min=budget, **request)


@pytest.mark.parametrize("shape", SHAPES)
@pytest.mark.parametrize("request_", [{}, {"relief_m": 3}, {"target": "building"}, {"gsd_cm": 1.5, "angle": 30}])
def test_every_generated_plan_passes_the_checker_and_the_coverage_test(tmp_path, shape, request_):
    bundle = generate(tmp_path, shape, budget=40, **request_)
    check, coverage = bundle["check"], bundle["coverage"]
    assert check["passed"] and coverage["passed"] and coverage["failing_samples"] == 0
    assert len(check["warnings"]) == 1 and RELATIVE_HEIGHT in check["warnings"][0]
    assert coverage["min_lines"] >= coverage["lines_needed"]
    assert check["geofence"]["outside"] == [] and check["flight_ready"] is False
    plan = json.loads(bundle["plan"])
    survey = plan["mission"]["items"][2]
    assert not any(item["command"] in CAMERA_COMMANDS for item in survey["TransectStyleComplexItem"]["Items"])
    assert [item["command"] for item in plan["mission"]["items"] if item["type"] == "SimpleItem"] == [178, 22, 20]
    assert survey["TransectStyleComplexItem"]["Refly90Degrees"] is (request_.get("target") == "building")
    assert bundle["review"]["flight_ready"] is False


def test_plan_has_the_fields_qgroundcontrol_saves_and_is_repeatable(tmp_path):
    bundle = generate(tmp_path)
    plan, reference = json.loads(bundle["plan"]), survey_plan()
    survey, saved = plan["mission"]["items"][2], reference["mission"]["items"][2]
    for ours, theirs in ((plan, reference), (plan["mission"], reference["mission"]), (survey, saved),
                         (survey["TransectStyleComplexItem"], saved["TransectStyleComplexItem"]),
                         (survey["TransectStyleComplexItem"]["CameraCalc"],
                          saved["TransectStyleComplexItem"]["CameraCalc"]),
                         (survey["TransectStyleComplexItem"]["Items"][0],
                          saved["TransectStyleComplexItem"]["Items"][0]),
                         (plan["geoFence"]["polygons"][0], reference["geoFence"]["polygons"][0])):
        assert set(ours) == set(theirs)
    assert (plan["mission"]["firmwareType"], plan["mission"]["vehicleType"]) == (12, 13)
    assert bundle["plan"] == generate(tmp_path)["plan"]
    assert bundle["plan"] != generate(tmp_path, height_m=31)["plan"]
    jump_ids = [item["doJumpId"] for item in plan["mission"]["items"] if "doJumpId" in item] + [
        item["doJumpId"] for item in survey["TransectStyleComplexItem"]["Items"]]
    assert len(set(jump_ids)) == len(jump_ids)


def test_dropping_an_edge_line_passes_survey_check_but_fails_coverage(tmp_path):
    bundle = generate(tmp_path)
    plan = json.loads(bundle["plan"])
    transects = plan["mission"]["items"][2]["TransectStyleComplexItem"]
    transects["Items"], transects["VisualTransectPoints"] = transects["Items"][4:], transects["VisualTransectPoints"][4:]
    path = tmp_path / "short.plan"
    path.write_text(json.dumps(plan))
    assert check_survey(path, CAMERA, PROFILE, 30)["passed"]
    coverage, _ = check_coverage(path, boundary("rectangle"), SPEC, {"fps": 2.0, "exposure_us": 1000.0},
                                 relief_m=0, acceptance_radius_m=2.0, front=80, side=70)
    assert not coverage["passed"] and coverage["min_lines"] == 2


def test_relief_adds_lines_and_slows_the_flight(tmp_path):
    level, hilly = generate(tmp_path), generate(tmp_path, relief_m=5)
    assert hilly["review"]["layout"]["legs"] > level["review"]["layout"]["legs"]
    assert hilly["review"]["design"]["speed_m_s"] < level["review"]["design"]["speed_m_s"]
    assert any("Relief is stated as 0" in warning for warning in level["review"]["warnings"])
    assert not any("Relief is stated as 0" in warning for warning in hilly["review"]["warnings"])


def test_over_budget_request_is_refused_with_the_nearest_plans_that_fit(tmp_path):
    with pytest.raises(SurveyRefused) as refusal:
        generate(tmp_path, "concave", budget=9)
    assert "exceeds the 9" in refusal.value.errors[0]
    coarser = refusal.value.alternatives["coarser_gsd_for_this_area"]
    smaller = refusal.value.alternatives["largest_area_at_this_gsd"]
    assert coarser["height_m"] > 30 and coarser["estimated_minutes"] <= 9
    assert 0 < smaller["fraction_of_field"] < 1 and smaller["estimated_minutes"] <= 9
    assert generate(tmp_path, "concave", budget=9, height_m=coarser["height_m"])["check"]["passed"]


def test_aircraft_limits_reach_the_plan_and_the_review(tmp_path):
    slow = generate(tmp_path, aircraft={"max_survey_speed_m_s": 3.0, "climb_m_s": 1.5})
    assert slow["review"]["design"]["binding_speed_limit"] == "aircraft"
    assert slow["review"]["design"]["speed_m_s"] == 3.0
    warnings = " ".join(slow["review"]["warnings"])
    assert "climb rate 1.5" in warnings and "Return climbs to 60.0 m" in warnings
    wide = generate(tmp_path, aircraft={"acceptance_radius_m": 6.0})
    assert wide["review"]["design"]["turnaround_m"] > generate(tmp_path)["review"]["design"]["turnaround_m"] + 3.9
    with pytest.raises(SurveyRefused, match="RTL_RETURN_ALT"):
        generate(tmp_path, aircraft={"return_height_m": 150.0})


def test_survey_plan_command_writes_a_bundle_the_recorder_accepts(tmp_path, capsys):
    ring = boundary("concave")
    coordinates = " ".join(f"{lon},{lat},0" for lat, lon in ring + ring[:1])
    (tmp_path / "field.kml").write_text(
        '<kml xmlns="http://www.opengis.net/kml/2.2"><Placemark><Polygon><outerBoundaryIs><LinearRing>'
        f"<coordinates>{coordinates}</coordinates></LinearRing></outerBoundaryIs></Polygon></Placemark></kml>")
    (tmp_path / "aircraft.json").write_text(json.dumps(AIRCRAFT))
    arguments = ["survey-plan", "--boundary", str(tmp_path / "field.kml"), "--home", f"{HOME[0]},{HOME[1]}",
                 "--home-amsl-m", str(HOME[2]), "--aircraft", str(tmp_path / "aircraft.json"),
                 "--camera", str(CAMERA), "--profile", str(PROFILE), "--gsd-cm", "1.0", "--relief-m", "2",
                 "--flight-time-budget-min", "15", "--output", str(tmp_path / "out")]
    assert main(arguments) == 0
    summary = json.loads(capsys.readouterr().out)
    output = tmp_path / "out"
    assert summary["passed"] and summary["height_m"] == 28.9 and summary["flight_ready"] is False
    assert sorted(p.name for p in output.iterdir()) == [
        "coverage.png", "review.geojson", "review.json", "review.kml", "survey-check.json", "survey.plan"]
    assert verify_survey(output, PROFILE) == summary["plan_sha256"]
    review = json.loads((output / "review.json").read_text())
    assert review["totals"]["estimated_minutes"] <= 15 and review["totals"]["max_distance_from_home_m"] > 150
    assert len(review["legs"]) == review["layout"]["legs"] and all(leg["frames"] > 0 for leg in review["legs"])
    assert {leg["heading_deg"] for leg in review["legs"]} == {90.0, 270.0}
    layers = {f["properties"]["layer"] for f in json.loads((output / "review.geojson").read_text())["features"]}
    assert layers == {"boundary", "survey_polygon", "home", "fence", "path", "turnarounds", "legs",
                      "frame_centres", "sample_footprints"}
    kml = ElementTree.fromstring((output / "review.kml").read_text())
    assert any(element.tag.endswith("GroundOverlay") for element in kml.iter())
    import cv2
    image = cv2.imread(str(output / "coverage.png"), cv2.IMREAD_UNCHANGED)
    grid = review["checks"]["coverage"]["grid"]
    assert image.shape == (grid["rows"], grid["columns"], 4)
    assert (image[..., 3] == 0).any() and (image[..., 1] > 100).any() and not (image[..., 2] > 200).any()

    assert main(arguments) == 2  # the output directory is never replaced
    assert "already exists" in capsys.readouterr().err
    arguments[-1] = str(tmp_path / "refused")
    arguments[arguments.index("--flight-time-budget-min") + 1] = "9"
    assert main(arguments) == 2
    refused = json.loads(capsys.readouterr().out)
    assert not refused["passed"] and refused["alternatives"] and not (tmp_path / "refused").exists()


def test_bundle_is_not_written_over_an_existing_directory(tmp_path):
    bundle = generate(tmp_path)
    write_bundle(bundle, tmp_path / "plan")
    with pytest.raises(FileExistsError):
        write_bundle(bundle, tmp_path / "plan")
