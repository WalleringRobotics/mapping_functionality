import copy
import json

import pytest

from qgc_plans import survey_plan, write_plan, to_latlon
from test_capabilities import handoff
from wallering_mapping.capabilities import camera_definition
from wallering_mapping.dataset import sha256_file
from wallering_mapping.recording import verify_survey
from wallering_mapping.survey_flight import compare_mission, verify_numbering
from wallering_mapping.survey_plane import check_plane, load_aircraft, segment_safe, write_plane_check
from wallering_mapping.survey_plan import load_plan


def inputs(tmp_path):
    data = handoff()
    data["capture"].update(available_storage_bytes=400_000_000_000,
                           sustained_write_bytes_per_s=100_000_000)
    data["resources"].update(storage_capacity_pass=True, storage_throughput_pass=True)
    path = tmp_path / "handoff.json"
    path.write_text(json.dumps(data))
    limits = {"schema": "wallering.plane-limits/v1", "schema_version": 1,
              "handoff_sha256": sha256_file(path), "aircraft_id": "synthetic-low-stall-wing",
              "pack_id": "fixture-pack", "payload_id": "fixture-camera", "calibration_id": "nominal-only",
              "calibration_sha256": "c" * 64,
              "source_files": {"fixture-design.json": "a" * 64}, "endurance_basis": "estimated",
              "endurance_min": 20, "reserve_fraction": .25, "stall_speed_m_s": 4.5,
              "stall_margin": 1.1, "min_airspeed_m_s": 6, "max_airspeed_m_s": 16,
              "bank_limit_deg": 45, "min_turn_radius_m": 5, "wind_bound_m_s": 1,
              "climb_m_s": 2, "descent_m_s": 2, "launch_land_allowance_s": 60, "max_agl_m": 120,
              "terrain": {"model": "level_at_home", "height_uncertainty_m": 1,
                          "evidence": "synthetic level-plane fixture"}}
    aircraft = tmp_path / "aircraft.json"
    aircraft.write_text(json.dumps(limits))
    plan = survey_plan(height=100, speed=7, width=200, depth=90, turnaround=30,
                       firmware=3, vehicle_type=1, end=21, camera=camera_definition(data)["qgc_custom_camera"])
    plan["mission"]["items"][1]["params"][0] = 0  # Plane absolute airspeed
    plan["geoFence"]["polygons"][0]["polygon"] = [to_latlon(x, y) for x, y in
        [(-400, -400), (600, -400), (600, 600), (-400, 600)]]
    return write_plan(tmp_path / "wing.plan", plan), path, aircraft


def change(path, edit):
    data = json.loads(path.read_text())
    edit(data)
    path.write_text(json.dumps(data))


def test_plane_opt_in_preserves_px4_refusal_and_home_based_numbering(tmp_path):
    plan, handoff_path, aircraft = inputs(tmp_path)
    assert load_plan(plan)["errors"]  # PX4/OAK default unchanged
    loaded = load_plan(plan, platform="ardupilot_plane")
    assert not loaded["errors"]
    assert loaded["items"][0]["role"] == "home"
    assert loaded["items"][1]["command"] == 22
    assert loaded["legs"][0]["entry_seq"] == 4
    report = check_plane(plan, handoff_path, aircraft)
    assert report["passed"], report["errors"]
    assert not report["flight_ready"] and not report["survey_ready"]
    assert report["flight"]["budget_minutes"] == 15
    assert report["flight"]["max_groundspeed_m_s"] == 8


@pytest.mark.parametrize("field,value,reason", [
    ("wind_bound_m_s", 11, "incompatible"),
    ("min_turn_radius_m", 40, "turn radius"),
    ("endurance_min", 2, "endurance"),
    ("stall_speed_m_s", 8, "safe aircraft"),
])
def test_wind_turn_stall_and_reserve_refusals(tmp_path, field, value, reason):
    plan, handoff_path, aircraft = inputs(tmp_path)
    change(aircraft, lambda d: d.update({field: value}))
    report = check_plane(plan, handoff_path, aircraft)
    assert not report["passed"]
    assert any(reason in e for e in report["errors"]), report["errors"]


@pytest.mark.parametrize("edit,reason", [
    (lambda d: d.update(wind_bound_m_s=None), "wind"),
    (lambda d: d.update(endurance_basis="design_target"), "design target"),
    (lambda d: d.update(terrain=None), "terrain"),
    (lambda d: d.update(handoff_sha256="b" * 64), "stale"),
])
def test_missing_or_stale_flight_assumptions_are_refused(tmp_path, edit, reason):
    _, handoff_path, aircraft = inputs(tmp_path)
    change(aircraft, edit)
    with pytest.raises(ValueError, match=reason):
        load_aircraft(aircraft, sha256_file(handoff_path))


@pytest.mark.parametrize("edit,reason", [
    (lambda p: p["mission"].update(vehicleType=2), "ArduPilot Plane"),
    (lambda p: p["mission"]["items"][0].update(frame=10), "altitude frame"),
    (lambda p: p["mission"]["items"][-1].update(command=20), "unsupported"),
    (lambda p: p["mission"]["items"][1]["params"].__setitem__(0, 1), "airspeed"),
])
def test_unsupported_plan_items_and_altitude_frames_fail(tmp_path, edit, reason):
    plan, handoff_path, aircraft = inputs(tmp_path)
    change(plan, edit)
    report = check_plane(plan, handoff_path, aircraft)
    assert not report["passed"] and reason.lower() in " ".join(report["errors"]).lower()


def test_thin_exclusion_and_concave_fence_crossing_between_valid_endpoints():
    outer = (True, "polygon", [(-10, -10), (10, -10), (10, 10), (-10, 10)])
    thin = (False, "circle", ((.1234, 0), .00001))
    assert not segment_safe([outer, thin], (-9, 0), (9, 0))
    assert segment_safe([outer, thin], (-9, 1), (9, 1))
    notch = (True, "polygon", [(-5, -5), (5, -5), (5, 5), (1, 5), (1, -1), (-1, -1), (-1, 5), (-5, 5)])
    assert not segment_safe([notch], (-4, 2), (4, 2))


def test_plan_and_pack_hashes_are_preserved_and_stale_changes_refused(tmp_path):
    plan, handoff_path, aircraft = inputs(tmp_path)
    report = check_plane(plan, handoff_path, aircraft)
    checked = tmp_path / "checked"
    write_plane_check(plan, handoff_path, aircraft, checked, report)
    assert verify_survey(checked) == sha256_file(plan)
    with pytest.raises(ValueError, match="OAK/PX4"):
        verify_survey(checked, profile=aircraft)
    change(checked / "aircraft-limits.json", lambda d: d.update(pack_id="other-pack"))
    with pytest.raises(ValueError, match="Stale"):
        verify_survey(checked)


def test_vehicle_home_item_is_required_for_plane_numbering(tmp_path):
    plan, _, _ = inputs(tmp_path)
    items = load_plan(plan, platform="ardupilot_plane")["items"]
    waypoints = [{"command": i["command"], "frame": i["frame"], "x_lat": i.get("lat", 0),
                  "auto_continue": i["auto_continue"],
                  "y_long": i.get("lon", 0), "z_alt": i.get("alt", 0),
                  **{f"param{n}": value for n, value in enumerate(i["params"][:4], 1)}} for i in items]
    assert not compare_mission(items, waypoints)
    for key, value in (("x_lat", float("nan")), ("z_alt", float("inf")), ("auto_continue", False)):
        invalid = copy.deepcopy(waypoints)
        invalid[4][key] = value
        assert compare_mission(items, invalid)
    assert compare_mission(items, waypoints[1:])
    lists = [{"receipt_ns": 1, "waypoints": waypoints}]
    assert verify_numbering(items, lists, [{"receipt_timestamp_ns": 2}])[0]
    changed = copy.deepcopy(waypoints)
    changed[4]["x_lat"] += .001
    assert not verify_numbering(items, [*lists, {"receipt_ns": 3, "waypoints": changed}],
                                [{"receipt_timestamp_ns": 2}])[0]


@pytest.mark.parametrize("restart", [False, True])
def test_plane_endpoint_association_uses_real_mcap_decoding_and_refuses_restart(tmp_path, restart):
    # Synthetic mission telemetry in an OAK IO fixture, not Plane/camera acceptance.
    from test_survey_flight import recording
    from wallering_mapping.survey_flight import extract_survey_legs
    plan, handoff_path, aircraft = inputs(tmp_path)
    report = check_plane(plan, handoff_path, aircraft)
    checked = tmp_path / "checked"
    write_plane_check(plan, handoff_path, aircraft, checked, report)
    events = None
    if restart:
        events = [(i, .1+i*.01) for i in range(len(report["mission_items"]))]
        events.append((1, events[-1][1]+.1))
    root, _, _ = recording(tmp_path, checked, events=events)
    result = extract_survey_legs(root, tmp_path / "legs")
    assert result["passed"] is not restart
    assert result["profile_hashes"]["handoff_sha256"] == sha256_file(handoff_path)
    assert all(row["qualified"] is not restart for row in result["legs"])
    if restart:
        assert any("went backwards" in e for e in result["errors"])


def test_self_intersecting_geofence_is_not_given_an_implicit_interpretation(tmp_path):
    plan, handoff_path, aircraft = inputs(tmp_path)
    change(plan, lambda p: p["geoFence"]["polygons"][0].update(polygon=[
        to_latlon(x, y) for x, y in [(-300, -300), (600, 500), (-300, 500), (500, -300)]]))
    with pytest.raises(ValueError, match="Self-intersecting"):
        check_plane(plan, handoff_path, aircraft)
