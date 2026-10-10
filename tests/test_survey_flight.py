"""Survey legs from real MCAP/CDR decoding of MAVROS mission topics."""
import json
import shutil
from pathlib import Path

import pytest
from rosbags.typesys import get_types_from_msg

from qgc_plans import survey_plan, write_plan
from test_bags import fixture_bag, seal
from wallering_mapping.bags import import_bag
from wallering_mapping.dataset import jsonl, sha256_file
from wallering_mapping.survey_flight import REACHED, WAYPOINTS, extract_survey_legs
from wallering_mapping.survey_plan import check_survey, load_plan, write_check

REPO = Path(__file__).resolve().parents[1]
T0 = 1_790_000_000_000_000_000
MSGS = {
    "mavros_msgs/msg/Waypoint": "uint8 frame\nuint16 command\nbool is_current\nbool autocontinue\n"
        "float32 param1\nfloat32 param2\nfloat32 param3\nfloat32 param4\n"
        "float64 x_lat\nfloat64 y_long\nfloat64 z_alt\n",
    "mavros_msgs/msg/WaypointList": "uint16 current_seq\nmavros_msgs/Waypoint[] waypoints\n",
    "mavros_msgs/msg/WaypointReached": "std_msgs/Header header\nuint16 wp_seq\n",
}


def checked(tmp_path):
    plan = write_plan(tmp_path / "field.plan", survey_plan())
    report = check_survey(plan, REPO / "configs/qgc-oak-rgb-12mp.json", REPO / "configs/oakd-ros.yaml", 15)
    assert report["passed"], report["errors"]
    return write_check(plan, tmp_path / "checked", report)


def timeline(items, legs):
    """Reached times (seconds): leg 0 spans 0.4-1.1 s, so it holds the 0.5 s and 1.0 s frames."""
    entry, exit_ = legs[0]["entry_seq"], legs[0]["exit_seq"]
    times, later = {}, 1.2
    for item in items:
        seq = item["mission_seq"]
        if seq < entry:
            times[seq] = .1 + seq * .01
        elif seq == entry:
            times[seq] = .4
        elif seq < exit_:
            times[seq] = .45 + (seq - entry) * .01
        elif seq == exit_:
            times[seq] = 1.1
        else:
            times[seq], later = later, later + .005
    return [times[item["mission_seq"]] for item in items]


def recording(tmp_path, checked_dir, *, events=None, lists=None, bind=True):
    report = json.loads((checked_dir / "survey-check.json").read_text())
    loaded = load_plan(checked_dir / "survey.plan", platform=report.get("platform", "px4_multirotor"))
    items, legs = loaded["items"], loaded["legs"]
    if events is None:
        events = list(zip(range(len(items)), timeline(items, legs)))
    if lists is None:
        lists = [(.05, items)]

    def extra(writer, store, header):
        for name, text in MSGS.items():
            store.register(get_types_from_msg(text, name))
        t = store.types
        reached = writer.add_connection(REACHED, "mavros_msgs/msg/WaypointReached", typestore=store)
        mission = writer.add_connection(WAYPOINTS, "mavros_msgs/msg/WaypointList", typestore=store)
        rows = [(seconds, "list", content) for seconds, content in lists] + [
            (seconds, "reached", seq) for seq, seconds in events]
        for seconds, kind, value in sorted(rows, key=lambda row: row[0]):
            ns = T0 + round(seconds * 1e9)
            if kind == "list":
                waypoints = [t["mavros_msgs/msg/Waypoint"](
                    item["frame"], item["command"], False, True,
                    *[float("nan") if value is None else float(value) for value in item["params"][:4]],
                    item.get("lat", 0.), item.get("lon", 0.), item.get("alt") or 0.) for item in value]
                message = t["mavros_msgs/msg/WaypointList"](0, waypoints)
                writer.write(mission, ns, store.serialize_cdr(message, message.__msgtype__))
            else:
                message = t["mavros_msgs/msg/WaypointReached"](header(ns - 2_000_000, "mission"), value)
                writer.write(reached, ns, store.serialize_cdr(message, message.__msgtype__))

    root = fixture_bag(tmp_path / "flight", extra=extra)
    if bind:
        for name in ("survey-check.json", "survey.plan"):
            shutil.copyfile(checked_dir / name, root / name)
        for name in ("mapping_handoff.json", "aircraft-limits.json", "camera.json"):
            if (checked_dir / name).is_file():
                shutil.copyfile(checked_dir / name, root / name)
        digest = sha256_file(root / "survey.plan")
        (root / "session.json").write_text(json.dumps({"schema_version": 1, "kind": "survey",
                                                       "survey_plan_sha256": digest}))
        seal(root)
    return root, items, legs


def test_qualified_legs_select_only_frames_flown_on_a_leg(tmp_path):
    checked_dir = checked(tmp_path)
    root, items, legs = recording(tmp_path, checked_dir)
    result = extract_survey_legs(root, tmp_path / "legs")
    assert result["passed"] and result["numbering_verified"], result["errors"]
    first = result["legs"][0]
    assert first["qualified"] and first["entry_seq"] == legs[0]["entry_seq"]
    assert first["header_start_ros_ns"] == T0 + 400_000_000 - 2_000_000
    assert first["header_end_ros_ns"] == T0 + 1_100_000_000 - 2_000_000
    assert all(row["qualified"] for row in result["legs"])
    assert result["source_seal_sha256"] == sha256_file(root / "SHA256SUMS")
    manifest = import_bag(root, tmp_path / "derived", tmp_path / "legs/survey-legs.json")
    rows = list(jsonl(tmp_path / "derived/frames.jsonl"))
    assert sorted({row["source_stamp_ros_ns"] - T0 for row in rows}) == [500_000_000, 1_000_000_000]
    assert {row["stream"] for row in rows} == {"rgb", "left", "right"}
    selection = manifest["survey_selection"]
    assert selection["imported"] == {"rgb": 2, "left": 2, "right": 2}
    assert selection["skipped"] == {"rgb": 2, "left": 2, "right": 2}
    assert selection["plan_sha256"] == result["plan_sha256"]
    assert [row["sequence"] for row in rows if row["stream"] == "rgb"] == [2, 3]


def test_vehicle_mission_differing_from_the_plan_disqualifies_every_leg(tmp_path):
    checked_dir = checked(tmp_path)
    items = load_plan(checked_dir / "survey.plan")["items"]
    moved = [dict(item) for item in items]
    moved[3]["lat"] += 1e-5
    root, _, _ = recording(tmp_path, checked_dir, lists=[(.05, moved)])
    result = extract_survey_legs(root, tmp_path / "legs")
    assert not result["passed"] and not result["numbering_verified"]
    assert any("position differs" in error for error in result["errors"])
    assert not any(row["qualified"] for row in result["legs"])
    with pytest.raises(ValueError, match="no qualified legs"):
        import_bag(root, tmp_path / "derived", tmp_path / "legs/survey-legs.json")


def test_a_stale_list_before_upload_is_superseded_by_the_list_in_force(tmp_path):
    checked_dir = checked(tmp_path)
    items = load_plan(checked_dir / "survey.plan")["items"]
    root, _, _ = recording(tmp_path, checked_dir, lists=[(.01, items[:3]), (.05, items)])
    result = extract_survey_legs(root, tmp_path / "legs")
    assert result["passed"], result["errors"]
    assert len(result["vehicle_missions"]) == 1


def test_missing_mission_list_leaves_legs_diagnostic(tmp_path):
    root, _, _ = recording(tmp_path, checked(tmp_path), lists=[])
    result = extract_survey_legs(root, tmp_path / "legs")
    assert not result["passed"] and not result["numbering_verified"]
    assert any("unverified" in warning for warning in result["warnings"])
    assert all("header_start_ros_ns" in row and not row["qualified"] for row in result["legs"])


def test_mission_download_after_progress_cannot_prove_which_plan_was_flown(tmp_path):
    checked_dir = checked(tmp_path)
    items = load_plan(checked_dir / "survey.plan")["items"]
    root, _, _ = recording(tmp_path, checked_dir, lists=[(1.9, items)])
    result = extract_survey_legs(root, tmp_path / "legs")
    assert not result["passed"]
    assert any("before the first" in e for e in result["errors"])
    assert not any(row["qualified"] for row in result["legs"])


def test_changed_vehicle_speed_or_camera_trigger_parameters_disqualify_legs(tmp_path):
    import copy
    checked_dir = checked(tmp_path)
    items = copy.deepcopy(load_plan(checked_dir / "survey.plan")["items"])
    items[1]["params"][1] *= 2
    root, _, _ = recording(tmp_path, checked_dir, lists=[(.05, items)])
    result = extract_survey_legs(root, tmp_path / "legs")
    assert not result["passed"]
    assert any("param2 differs" in e for e in result["errors"])


def test_restarted_mission_disqualifies_every_leg(tmp_path):
    checked_dir = checked(tmp_path)
    items = load_plan(checked_dir / "survey.plan")["items"]
    events = list(zip(range(len(items)), [.1 + i * .02 for i in range(len(items))]))
    events.append((0, events[-1][1] + .05))
    root, _, _ = recording(tmp_path, checked_dir, events=events)
    result = extract_survey_legs(root, tmp_path / "legs")
    assert any("went backwards" in error for error in result["errors"])
    assert not any(row["qualified"] for row in result["legs"])


def test_an_unfinished_survey_keeps_its_completed_legs(tmp_path):
    checked_dir = checked(tmp_path)
    loaded = load_plan(checked_dir / "survey.plan")
    times = timeline(loaded["items"], loaded["legs"])
    last_exit = loaded["legs"][-1]["exit_seq"]
    events = [(seq, seconds) for seq, seconds in zip(range(len(times)), times) if seq < last_exit]
    root, _, _ = recording(tmp_path, checked_dir, events=events)
    result = extract_survey_legs(root, tmp_path / "legs")
    assert not result["passed"]
    assert any("without both endpoints" in error for error in result["errors"])
    assert result["legs"][0]["qualified"] and not result["legs"][-1]["qualified"]


def test_plan_binding_and_report_ownership_are_enforced(tmp_path):
    checked_dir = checked(tmp_path)
    root, _, _ = recording(tmp_path, checked_dir, bind=False)
    with pytest.raises(ValueError, match="pass --check"):
        extract_survey_legs(root, tmp_path / "legs")
    result = extract_survey_legs(root, tmp_path / "legs", checked_dir)
    assert result["passed"] and result["plan_source"] == str(checked_dir)
    other = fixture_bag(tmp_path / "other")
    with pytest.raises(ValueError, match="different recording"):
        import_bag(other, tmp_path / "derived", tmp_path / "legs/survey-legs.json")
    (checked_dir / "survey.plan").write_text("{}")
    with pytest.raises(ValueError, match="differs"):
        extract_survey_legs(root, tmp_path / "legs-2", checked_dir)
    assert not (tmp_path / "legs-2").exists()
