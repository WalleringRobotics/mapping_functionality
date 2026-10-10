"""Survey-leg windows from a sealed recording; offline only.

PX4 reports each mission item it reaches. With the checked QGroundControl plan
bound into the session, a leg runs from the reached event of its entry waypoint to
that of its exit waypoint. The PX4 numbering is verified against the mission list
MAVROS downloaded and the recorder stored, not assumed.
"""

import json
import math
from pathlib import Path

from .bags import check_seal, reader, stamp
from .dataset import sha256_file, write_json
from .survey_plan import FRAME_CLASSES, POSITIONAL_COMMANDS, load_plan

REACHED = "/mavros/mission/reached"
WAYPOINTS = "/mavros/mission/waypoints"
POSITION_TOLERANCE_DEG = 2e-7  # MISSION_ITEM_INT stores 1e-7 degrees
ALTITUDE_TOLERANCE_M = .05     # float32 transport


def add_arguments(parser):
    parser.add_argument("session", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--check", type=Path,
                        help="survey-check directory, if the plan was not bound at recording time")


def run(args):
    return extract_survey_legs(args.session, args.output, args.check)


def compare_mission(items, waypoints):
    """Return differences between the plan's PX4 items and one downloaded mission list."""
    if len(waypoints) != len(items):
        return [f"vehicle holds {len(waypoints)} items, plan has {len(items)}"]
    differences = []
    for item, waypoint in zip(items, waypoints):
        seq = item["mission_seq"]
        if int(waypoint["command"]) != item["command"]:
            differences.append(f"item {seq} command {waypoint['command']} != {item['command']}")
            continue
        if FRAME_CLASSES.get(int(waypoint["frame"])) != FRAME_CLASSES.get(item["frame"]):
            differences.append(f"item {seq} altitude frame differs")
        if waypoint.get("auto_continue") is not item.get("auto_continue"):
            differences.append(f"item {seq} auto-continue differs")
        for index, expected in enumerate(item["params"][:4], 1):
            actual = waypoint.get(f"param{index}")
            if expected is None:
                if actual is not None and not math.isnan(actual):
                    differences.append(f"item {seq} param{index} differs")
            elif actual is None or not math.isfinite(actual):
                differences.append(f"item {seq} param{index} missing/nonfinite")
            else:
                delta = actual - expected
                if index == 4 and item["command"] in POSITIONAL_COMMANDS:
                    delta = (delta + 180) % 360 - 180
                if abs(delta) > max(1e-5, abs(expected) * 1e-6):
                    differences.append(f"item {seq} param{index} differs")
        if item["command"] in POSITIONAL_COMMANDS and "lat" in item:
            if not all(type(waypoint.get(k)) in (int, float) and math.isfinite(waypoint[k])
                       for k in ("x_lat", "y_long", "z_alt")):
                differences.append(f"item {seq} nonfinite navigation coordinate")
                continue
            if (abs(waypoint["x_lat"] - item["lat"]) > POSITION_TOLERANCE_DEG
                    or abs(waypoint["y_long"] - item["lon"]) > POSITION_TOLERANCE_DEG):
                differences.append(f"item {seq} position differs")
            if item["alt"] is not None and abs(waypoint["z_alt"] - item["alt"]) > ALTITUDE_TOLERANCE_M:
                differences.append(f"item {seq} altitude differs")
    return differences


def bound_check(root, check):
    """The plan bound at recording time, or an explicit check directory that agrees with it."""
    session = json.loads((root / "session.json").read_text()) if (root / "session.json").is_file() else {}
    bound = root / "survey-check.json"
    if bound.is_file():
        directory = root
        if check is not None and sha256_file(Path(check) / "survey.plan") != sha256_file(root / "survey.plan"):
            raise ValueError("--check names a different plan from the one bound to this recording")
    elif check is not None:
        directory = Path(check).resolve()
    else:
        raise ValueError("No survey plan is bound to this recording; pass --check")
    report = json.loads((directory / "survey-check.json").read_text())
    from .recording import verify_survey
    verify_survey(directory)
    plan_hash = sha256_file(directory / "survey.plan")
    if report.get("kind") != "survey_plan_check" or report.get("plan", {}).get("sha256") != plan_hash:
        raise ValueError("survey.plan differs from the plan that was checked")
    if report.get("passed") is not True:
        raise ValueError("The survey plan did not pass survey-check")
    if session.get("survey_plan_sha256") not in (None, plan_hash):
        raise ValueError("Session metadata names a different survey plan")
    return directory, report, plan_hash


def read_mission_topics(root):
    lists, events = [], []
    with reader(root) as bag:
        for connection, receipt, raw in bag.messages(connections=[
                c for c in bag.connections if c.topic in (REACHED, WAYPOINTS)]):
            message = bag.deserialize(raw, connection.msgtype)
            if connection.topic == WAYPOINTS:
                lists.append({"receipt_ns": int(receipt), "current_seq": int(message.current_seq),
                              "waypoints": [{"frame": int(w.frame), "command": int(w.command),
                                             "auto_continue": bool(w.autocontinue),
                                             "x_lat": float(w.x_lat), "y_long": float(w.y_long),
                                             **{f"param{i}": float(getattr(w, f"param{i}"))
                                                for i in range(1, 5)},
                                             "z_alt": float(w.z_alt)} for w in message.waypoints]})
            else:
                events.append({"ordinal": len(events), "mission_seq": int(message.wp_seq),
                               "receipt_timestamp_ns": int(receipt), "header_stamp_ros_ns": stamp(message)})
    return lists, events


def verify_numbering(items, lists, events):
    """Use the list in force when the mission first progressed, and every later list."""
    if not lists:
        return False, [], ["No vehicle mission list was recorded; leg numbering is unverified"]
    start = events[0]["receipt_timestamp_ns"] if events else math.inf
    before = [index for index, row in enumerate(lists) if row["receipt_ns"] <= start]
    if events and not before:
        return False, [], ["No vehicle mission list was recorded before the first reached event"]
    first = before[-1] if before else 0
    errors, compared = [], []
    seen = set()
    for row in lists[first:]:
        key = json.dumps(row["waypoints"], sort_keys=True)
        if key in seen:
            continue
        seen.add(key)
        differences = compare_mission(items, row["waypoints"])
        compared.append({"receipt_ns": row["receipt_ns"], "items": len(row["waypoints"]),
                         "differences": differences[:20]})
        if differences:
            errors.append("Vehicle mission differs from the checked plan: " + "; ".join(differences[:5]))
    return not errors, compared, errors


def extract_survey_legs(root, output, check=None):
    root, output = Path(root).resolve(), Path(output).resolve()
    if output.is_relative_to(root) or root.is_relative_to(output):
        raise ValueError("Survey-leg output must be separate from the immutable recording")
    sealed = check_seal(root)
    if (root / "state").read_text().strip() != "complete":
        raise ValueError("Recording did not finish cleanly")
    source_seal = sha256_file(root / "SHA256SUMS")
    directory, report, plan_hash = bound_check(root, check)
    loaded = load_plan(directory / "survey.plan", report["inputs"]["limits"]["match_tolerance_m"],
                       platform=report.get("platform", "px4_multirotor"))
    if loaded["errors"]:
        raise ValueError("Bound plan is unsupported: " + "; ".join(loaded["errors"]))
    items, legs = loaded["items"], loaded["legs"]
    lists, events = read_mission_topics(root)
    verified, compared, errors = verify_numbering(items, lists, events)
    warnings = []
    if not lists:
        warnings.append(errors.pop())
    for index, event in enumerate(events):
        if not 0 <= event["mission_seq"] < len(items):
            errors.append(f"Unknown mission index {event['mission_seq']} at event {index}")
        if index and event["mission_seq"] < events[index - 1]["mission_seq"]:
            errors.append(f"Mission index went backwards at event {index}; restarts and resumes "
                          "are not supported")
        if index and any(event[key] <= events[index - 1][key]
                         for key in ("header_stamp_ros_ns", "receipt_timestamp_ns")):
            errors.append(f"Nonmonotonic event clock at event {index}")
    if not events:
        errors.append("No mission reached events were recorded")
    by_seq = {}
    for event in events:
        by_seq.setdefault(event["mission_seq"], event)
    rows = []
    for leg in legs:
        entry, exit_ = by_seq.get(leg["entry_seq"]), by_seq.get(leg["exit_seq"])
        row = {**leg, "entry_event": None if entry is None else entry["ordinal"],
               "exit_event": None if exit_ is None else exit_["ordinal"], "qualified": False}
        if entry is None or exit_ is None:
            row["reason"] = "entry or exit was not reached"
        elif exit_["header_stamp_ros_ns"] <= entry["header_stamp_ros_ns"]:
            row["reason"] = "exit precedes entry"
        else:
            row.update(header_start_ros_ns=entry["header_stamp_ros_ns"],
                       header_end_ros_ns=exit_["header_stamp_ros_ns"],
                       receipt_start_ns=entry["receipt_timestamp_ns"],
                       receipt_end_ns=exit_["receipt_timestamp_ns"],
                       qualified=verified and not errors)
        rows.append(row)
    missing = [f"{row['survey']}/{row['transect']}" for row in rows if "header_start_ros_ns" not in row]
    if missing:
        errors.append("Legs without both endpoints reached: " + ", ".join(missing))
    passed = verified and not errors
    result = {"schema_version": 1, "kind": "survey_legs", "status": "complete", "passed": passed,
              "numbering_verified": verified, "source_seal_sha256": source_seal, "sealed_files": sealed,
              "plan_sha256": plan_hash, "plan_source": "session" if directory == root else str(directory),
              "profile_hashes": {key: value for key, value in report["inputs"].items()
                                 if key.endswith("_sha256")},
              "errors": errors, "warnings": warnings, "vehicle_missions": compared,
              "events": events, "legs": rows, "survey_ready": False,
              "limitations": [
                  "Reached events mark arrival within PX4's acceptance radius, not exact leg ends",
                  "Header times are MAVROS receipt of MISSION_ITEM_REACHED, not exposure times",
                  "A qualified leg window selects images; it does not qualify geolocation"]}
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / "survey-legs.json", result)
    return result


def leg_windows(path, source_seal):
    """Qualified header-time windows from a survey-legs report bound to this recording."""
    report = json.loads(Path(path).read_text())
    if report.get("kind") != "survey_legs" or report.get("schema_version") != 1:
        raise ValueError("Not a survey-legs report")
    if report.get("source_seal_sha256") != source_seal:
        raise ValueError("Survey-legs report belongs to a different recording")
    windows = [(row["header_start_ros_ns"], row["header_end_ros_ns"])
               for row in report["legs"] if row.get("qualified")]
    if not windows:
        raise ValueError("Survey-legs report has no qualified legs")
    return windows, report
