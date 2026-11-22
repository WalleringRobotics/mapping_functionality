import json
from pathlib import Path

from qgc_plans import HOME, survey_plan, to_latlon, write_plan
from wallering_mapping.survey_coverage import check_coverage

REPO = Path(__file__).resolve().parents[1]
SPEC = json.loads((REPO / "configs/qgc-oak-rgb-12mp.json").read_text())["qgc_custom_camera"]
PROFILE = {"fps": 2.0, "exposure_us": 1000.0}
# qgc_plans.survey_plan flies east-west lines over a rectangle whose south-west corner is 30 m
# north of home, as QGroundControl clips them: the first line lies on the southern edge.
# At 40 m the footprint is 52.4 m across and 39.3 m along; 70 % side overlap gives 15.7 m lines.


def field(west, south, east, north):
    return [tuple(to_latlon(x, y)) for x, y in ((west, south), (east, south), (east, north), (west, north))]


def coverage(tmp_path, boundary, relief=0.0, radius=2.0, front=80, side=70, **plan):
    path = write_plan(tmp_path / "field.plan", survey_plan(**plan))
    return check_coverage(path, boundary, SPEC, PROFILE, relief_m=relief, acceptance_radius_m=radius,
                          front=front, side=side)


def test_lines_clipped_to_the_field_leave_its_edges_short_of_views(tmp_path):
    report, raster = coverage(tmp_path, field(0, 30, 120, 120))
    assert not report["passed"] and report["min_lines"] == 2
    assert (report["lines_needed"], report["frames_needed"]) == (3, 5)
    assert "fewer than 3 lines" in report["errors"][0]
    assert raster.shape == (90, 120) and raster[45, 60] >= 3
    assert raster[-1, 60] == 2  # the southern edge row sees the two lines north of it only
    assert report["grid"]["south_west"][0] < report["grid"]["north_east"][0]


def test_a_field_inside_the_flown_lines_passes(tmp_path):
    report, raster = coverage(tmp_path, field(5, 42, 115, 108))
    assert report["passed"] and report["failing_samples"] == 0 and report["min_lines"] >= 3
    assert report["frame_spacing_m"] == 2.0 and (raster >= 3).all()


def test_spacing_wider_than_the_requested_overlap_fails_in_the_interior(tmp_path):
    # The plan is drawn for 40 % side overlap (31.4 m lines) but 70 % is required.
    wide = check_coverage(write_plan(tmp_path / "wide.plan", survey_plan(side=40)), field(5, 60, 115, 100),
                          SPEC, PROFILE, relief_m=0, acceptance_radius_m=2.0, front=80, side=70)[0]
    assert not wide["passed"] and wide["min_lines"] < 3


def test_relief_speed_and_leg_ends_reduce_the_views(tmp_path):
    inside = field(5, 42, 115, 108)
    # Over ground 15 m higher the footprint is 32.8 m across: lines 15.7 m apart give two views.
    assert not coverage(tmp_path, inside, relief=15.0)[0]["passed"]
    # At 8 m/s frames are 4 m apart; 39.3 m along track still holds five, but not eleven.
    assert coverage(tmp_path, inside, speed=8.0)[0]["passed"]
    assert not coverage(tmp_path, inside, speed=8.0, front=91)[0]["passed"]
    # Frames stop an acceptance radius before the exit: a wide radius starves the leg ends.
    ends = field(0, 42, 120, 108)
    assert coverage(tmp_path, ends, speed=8.0, radius=0.0, front=75)[0]["passed"]
    assert not coverage(tmp_path, ends, speed=8.0, radius=10.0, front=75)[0]["passed"]


def test_a_plan_without_speed_or_legs_is_reported_not_raised(tmp_path):
    report, _ = coverage(tmp_path, field(5, 42, 115, 108), speed_item=False)
    assert not report["passed"] and "speed" in " ".join(report["errors"])
    assert HOME[0] < report["grid"]["north_east"][0]
