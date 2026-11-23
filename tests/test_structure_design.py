import json
import math
from pathlib import Path

import pytest

from test_survey_design import AIRCRAFT, PROFILE, SPEC
from wallering_mapping.structure_check import optics, read_mount, requirements_from
from wallering_mapping.structure_design import band_angles, design, wall_scale

REPO = Path(__file__).resolve().parents[1]
MOUNT_PATH = REPO / "configs/oak-forward-mount.json"
MOUNT = json.loads(MOUNT_PATH.read_text())


def plan(**request):
    request.setdefault("structure_height_m", 6)
    if "gsd_cm" not in request:
        request.setdefault("standoff_m", 10)
    return design(SPEC, PROFILE, AIRCRAFT, MOUNT, **request)


def test_mount_states_a_forward_camera_with_a_sourced_pitch(tmp_path):
    mount = read_mount(MOUNT_PATH)
    assert mount["pitch_down_deg"] == 30.0 and mount["pitch_tolerance_deg"] == 3.0
    for changes, message in (({"direction": "nadir"}, "forward"), ({"image_width": "vertical"}, "forward"),
                             ({"pitch_source": " "}, "pitch_source"), ({"pitch_down_deg": 90}, "pitch"),
                             ({"pitch_tolerance_deg": None}, "finite"), ({"schema": "other"}, "schema")):
        path = tmp_path / "mount.json"
        path.write_text(json.dumps({**MOUNT, **changes}))
        with pytest.raises(ValueError, match=message):
            read_mount(path)


def test_usable_band_runs_from_the_top_of_the_image_to_the_steepest_usable_ray():
    lens = optics(SPEC)
    top, bottom = band_angles(MOUNT, lens, requirements_from())
    half_view = math.degrees(math.atan(lens[2] / lens[0]))
    assert top == pytest.approx(30 + 3 - half_view) and 6.5 < top < 7.2
    assert bottom == 45.0
    # A shallower limit than the image reaches is not a limit: the image's own edge is.
    assert band_angles({**MOUNT, "pitch_down_deg": 10.0}, lens, requirements_from())[1] == pytest.approx(
        10 - 3 + half_view)
    with pytest.raises(ValueError, match="usable band"):
        band_angles({**MOUNT, "pitch_down_deg": 60.0}, lens, requirements_from({"max_depression_deg": 40}))


def test_worked_example_ten_metres_from_a_six_metre_wall():
    chosen = plan()
    # Square-on at the top of the image a pixel covers stand-off / 3440: 0.29 cm at 10 m.
    assert chosen["gsd_cm_finest_along_wall"] == pytest.approx(0.29, abs=0.005)
    assert chosen["gsd_cm_coarsest"] == pytest.approx(0.62, abs=0.01)
    assert chosen["band_below_aircraft_m"] == pytest.approx([1.30, 9.90], abs=0.01)
    assert chosen["pass_heights_m"] == pytest.approx([5.0, 8.44, 11.88], abs=0.01)
    assert chosen["safe_height_m"] == 11.9 and chosen["lines_needed"] == 2 and chosen["frames_needed"] == 5
    assert chosen["binding_speed_limit"] == "blur" and chosen["speed_m_s"] == 1.4
    assert chosen["blur_px"] <= 0.5 and chosen["front_overlap_achieved"] > 90
    assert chosen["stop_hold_s"] == 3.0 and chosen["settle_hold_s"] == 2.0


def test_gsd_request_sets_the_stand_off_and_is_met_at_the_worst_pitch():
    chosen = plan(gsd_cm=1.0)
    assert chosen["gsd_cm_coarsest"] <= 1.0 and chosen["standoff_m"] == pytest.approx(16.1, abs=0.11)
    coarsest, _ = wall_scale(MOUNT, optics(SPEC), requirements_from())
    assert chosen["standoff_m"] * coarsest * 100 <= 1.0 < (chosen["standoff_m"] + 0.1) * coarsest * 100
    with pytest.raises(ValueError, match="exactly one"):
        design(SPEC, PROFILE, AIRCRAFT, MOUNT, structure_height_m=6)
    with pytest.raises(ValueError, match="exactly one"):
        plan(gsd_cm=1.0, standoff_m=10)


def test_too_close_to_see_the_foot_of_the_wall_is_refused_with_the_distance_that_works():
    with pytest.raises(ValueError, match=r"foot of the wall.*stand off at least 8\.0 m"):
        plan(standoff_m=6)
    assert plan(standoff_m=8)["pass_heights_m"][0] == 5.0
    assert plan(standoff_m=6, requirements={"min_height_m": 3})["pass_heights_m"][0] == 3.0
    with pytest.raises(ValueError, match="Stand-off"):
        plan(standoff_m=4)
    with pytest.raises(ValueError, match="Stand-off"):
        plan(standoff_m=8, requirements={"min_clearance_m": 9})


@pytest.mark.parametrize("standoff", [8, 10, 13, 20, 35])
@pytest.mark.parametrize("wall", [3, 6, 15, 30])
def test_every_wall_height_has_the_required_passes_and_each_face_ends_at_its_far_corner(standoff, wall):
    chosen = plan(standoff_m=standoff, structure_height_m=wall)
    heights, (above, below) = chosen["pass_heights_m"], chosen["band_below_aircraft_m"]
    assert len(heights) % 2 == 1 and heights == sorted(heights) and heights[0] >= 5.0
    for step in range(0, wall * 10 + 1):
        point = step / 10
        assert sum(point + above - 1e-6 <= z <= point + below + 1e-6 for z in heights) >= chosen["lines_needed"]
    assert chosen["safe_height_m"] >= max(heights[-1], wall + 5)


def test_speed_is_the_lowest_limit_and_reports_which():
    assert plan(standoff_m=30)["binding_speed_limit"] == "blur"
    slow = design(SPEC, PROFILE, {**AIRCRAFT, "max_survey_speed_m_s": 1.0}, MOUNT, structure_height_m=6,
                  standoff_m=10)
    assert slow["binding_speed_limit"] == "aircraft" and slow["speed_m_s"] == 1.0
    short = design(SPEC, {"fps": 2.0, "exposure_us": 100.0}, AIRCRAFT, MOUNT, structure_height_m=6, standoff_m=10)
    assert short["binding_speed_limit"] == "forward_overlap" and short["front_overlap_achieved"] >= 80
    with pytest.raises(ValueError, match="blur limit allows only"):
        design(SPEC, {"fps": 2.0, "exposure_us": 8000.0}, AIRCRAFT, MOUNT, structure_height_m=6, standoff_m=10)
