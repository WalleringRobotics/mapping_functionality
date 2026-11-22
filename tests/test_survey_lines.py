import json
import math

import pytest

from qgc_plans import HOME, survey_plan, to_latlon
from wallering_mapping.survey_plan import local_xy

pytest.importorskip("shapely")
from wallering_mapping.survey_lines import (  # noqa: E402
    fence_polygon, field_polygon, lay_lines, plan_lines, read_boundary, route)
from wallering_mapping import survey_lines  # noqa: E402

SQUARE = [(0, 0), (100, 0), (100, 60), (0, 60)]
DESIGN = {"line_spacing_m": 10.0, "turnaround_m": 8.0, "speed_m_s": 4.0, "footprint_across_low_m": 30.0,
          "across_extent_m": 5.0, "along_margin_m": 3.0, "crosshatch": False}


def latlon(shape):
    return [tuple(to_latlon(*xy)) for xy in shape]


def kml(rings):
    body = "".join(f"<{tag}><LinearRing><coordinates>{' '.join(f'{lon},{lat},0' for lat, lon in ring)}"
                   f"</coordinates></LinearRing></{tag}>" for tag, ring in rings)
    return ('<?xml version="1.0"?><kml xmlns="http://www.opengis.net/kml/2.2"><Document><Placemark>'
            f"<Polygon>{body}</Polygon></Placemark></Document></kml>")


def test_boundary_is_read_from_kml_geojson_and_a_plan(tmp_path):
    ring = latlon(SQUARE)
    (tmp_path / "field.kml").write_text(kml([("outerBoundaryIs", ring + ring[:1])]))
    lonlat = [[lon, lat] for lat, lon in ring + ring[:1]]
    (tmp_path / "field.geojson").write_text(json.dumps({"type": "FeatureCollection", "features": [
        {"type": "Feature", "properties": {}, "geometry": {"type": "Polygon", "coordinates": [lonlat]}}]}))
    (tmp_path / "field.plan").write_text(json.dumps(survey_plan(width=100, depth=60)))
    for name in ("field.kml", "field.geojson"):
        assert read_boundary(tmp_path / name) == pytest.approx(ring)
    assert len(read_boundary(tmp_path / "field.plan")) == 4
    origin = local_xy(HOME, read_boundary(tmp_path / "field.kml")[2])
    assert origin == pytest.approx((100, 60), abs=1e-6)


def test_unsupported_boundaries_are_refused(tmp_path):
    ring = latlon(SQUARE)
    cases = {
        "holes.kml": (kml([("outerBoundaryIs", ring), ("innerBoundaryIs", ring)]), "holes"),
        "entity.kml": ('<!DOCTYPE kml [<!ENTITY a "b">]>' + kml([("outerBoundaryIs", ring)]), "refused"),
        "empty.kml": ("<kml><Document/></kml>", "exactly one"),
        "two.geojson": (json.dumps({"type": "MultiPolygon", "coordinates": [[[[0, 0]]], [[[0, 0]]]]}),
                        "exactly one"),
        "holes.geojson": (json.dumps({"type": "Polygon", "coordinates": [[[0, 0]], [[0, 0]]]}), "holes"),
        "line.geojson": (json.dumps({"type": "Polygon", "coordinates": [[[12.9, 48.4], [12.91, 48.4]]]}),
                         "three vertices"),
        "field.txt": ("", "must be a"),
    }
    for name, (text, message) in cases.items():
        (tmp_path / name).write_text(text)
        with pytest.raises(ValueError, match=message):
            read_boundary(tmp_path / name)
    for shape, message in (([(0, 0), (100, 100), (100, 0), (0, 100)], "crosses itself"),
                           ([(0, 0), (5, 0), (5, 5)], "100 square metres"),
                           ([(6000, 0), (6100, 0), (6100, 100)], "take-off point")):
        with pytest.raises(ValueError, match=message):
            field_polygon(HOME, latlon(shape))


def test_lines_are_evenly_spaced_and_reach_past_every_edge():
    field = field_polygon(HOME, latlon(SQUARE))
    polygon, lines = lay_lines(field, 90, 10.0, 15.0, 5.0, 3.0)  # legs run east-west
    positions = [line[0] for line in lines]
    assert all(b - a == pytest.approx(10.0) for a, b in zip(positions, positions[1:]))
    # Across is to the right of an eastbound leg, i.e. south: the field spans -60..0.
    assert positions[0] <= -60 - 5 + 10 and positions[-1] >= 0 + 5 - 10
    assert positions[0] <= -65 or positions[-1] >= 5 or len(lines) == 8
    assert all((start, end) == pytest.approx((-3.0, 103.0), abs=0.01) for _, start, end in lines)
    assert min(y for _, y in polygon) < -1 and max(y for _, y in polygon) > 61


def test_a_line_is_flown_wherever_its_footprint_touches_a_slanted_edge():
    triangle = field_polygon(HOME, latlon([(0, 0), (100, 0), (0, 100)]))
    _, lines = lay_lines(triangle, 0, 10.0, 15.0, 5.0, 3.0)  # legs run north-south, across is east
    assert len(lines) >= 10
    for position, start, end in lines:
        # The field ends at 100 - east, but the footprint reaches 15 m further west than the line.
        assert end == pytest.approx(100 - max(0.0, position - 15) + 3, abs=0.05)
        assert start == pytest.approx(-3, abs=0.01)


def test_route_alternates_direction_and_starts_nearest_the_take_off_point():
    lines = [(0.0, 0.0, 50.0), (10.0, 0.0, 50.0), (20.0, 5.0, 40.0)]
    transects = route(lines, 0, 8.0, (25.0, 60.0))
    assert transects[0][:2] == [pytest.approx((20.0, 48.0)), pytest.approx((20.0, 40.0))]
    assert transects[0][2:] == [pytest.approx((20.0, 5.0)), pytest.approx((20.0, -3.0))]
    assert transects[1][1] == pytest.approx((10.0, 0.0)) and transects[2][1] == pytest.approx((0.0, 50.0))
    for turn_in, entry, exit_, turn_out in transects:
        assert math.dist(turn_in, entry) == pytest.approx(8.0) == math.dist(exit_, turn_out)
        assert math.dist(turn_in, turn_out) == pytest.approx(math.dist(entry, exit_) + 16.0)


def test_angle_follows_the_long_axis_unless_given_and_layout_is_repeatable():
    long_field = field_polygon(HOME, latlon([(10, 10), (310, 10), (310, 70), (10, 70)]))
    layout = plan_lines(long_field, (0.0, 0.0), DESIGN)
    assert layout["angle"] == 90.0 and layout["angle_selected"] == "shortest flight"
    assert layout == plan_lines(long_field, (0.0, 0.0), DESIGN)
    given = plan_lines(long_field, (0.0, 0.0), DESIGN, angle=0)
    assert given["angle"] == 0.0 and len(given["transects"]) > 3 * len(layout["transects"])
    assert given["cost_m"] > layout["cost_m"]
    crossed = plan_lines(long_field, (0.0, 0.0), {**DESIGN, "crosshatch": True}, angle=90)
    assert len(crossed["transects"]) == len(layout["transects"]) + len(given["transects"])


def test_fence_contains_every_point_and_the_straight_paths_between_them():
    from shapely.geometry import LineString, Polygon
    points = [(0, 0), (50, 80), (120, 10), (60, 30), (-20, 40)]
    fence = Polygon(fence_polygon(points, 20.0))
    assert all(fence.contains(LineString([a, b]).buffer(19.9)) for a in points for b in points if a != b)
    assert survey_lines.MAX_SITE_M == 5000
