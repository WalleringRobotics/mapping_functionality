"""Extract mission progress evidence offline; endpoints do not establish motion onset."""

import json
from pathlib import Path

from .bags import check_seal, reader, stamp
from .dataset import safe_path, sha256_file, write_json

TOPIC = "/mavros/mission/reached"
MESSAGE_TYPE = "mavros_msgs/msg/WaypointReached"
NAVIGATION = {16, 21, 22}


def add_arguments(parser):
    parser.add_argument("session", type=Path)
    parser.add_argument("--plan", required=True, type=Path)
    parser.add_argument("--phase-map", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--verified-numbering", action="store_true",
                        help="Operator confirms downloaded PX4 indices match the reviewed plan")
    parser.add_argument("--numbering-evidence", type=Path,
                        help="JSON review binding plan, phase-map and downloaded-mission hashes")


def run(args):
    return extract_flight_phases(args.session, args.plan, args.phase_map, args.output,
                                 args.verified_numbering, args.numbering_evidence)


def verified_map(plan_path, phase_path, verified_numbering, evidence_path):
    plan_hash, phase_hash = sha256_file(plan_path), sha256_file(phase_path)
    plan, phases = (json.loads(p.read_text()) for p in (plan_path, phase_path))
    if (plan.get("fileType") != "Plan" or phases.get("schema_version") != 1
            or phases.get("plan_sha256") != plan_hash):
        raise ValueError("Phase map does not match the supplied Plan SHA256/schema")
    items = plan["mission"]["items"]
    mapped = phases["mission_items"]
    if not items or len(items) != len(mapped):
        raise ValueError("Phase map must cover every plan item")
    for seq, (item, phase) in enumerate(zip(items, mapped, strict=True)):
        if (phase.get("mission_seq") != seq or phase.get("do_jump_id") != item["doJumpId"]
                or phase.get("command") != item["command"] or not phase.get("phase")
                or item.get("type") != "SimpleItem"):
            raise ValueError("Phase map item index/command does not match the plan")
    evidence = None
    if verified_numbering:
        if evidence_path is None:
            raise ValueError("Verified numbering requires --numbering-evidence")
        evidence_path = Path(evidence_path).resolve()
        review = json.loads(evidence_path.read_text())
        if (review.get("schema_version") != 1 or review.get("plan_sha256") != plan_hash
                or review.get("phase_map_sha256") != phase_hash
                or review.get("zero_based_px4_items_match") is not True
                or not str(review.get("operator") or "").strip()):
            raise ValueError("Numbering review must bind both hashes and operator confirmation")
        downloaded = review.get("downloaded_mission", {})
        download_path = safe_path(evidence_path.parent, downloaded.get("path", ""))
        if not download_path.is_file() or sha256_file(download_path) != downloaded.get("sha256"):
            raise ValueError("Downloaded mission evidence checksum mismatch")
        evidence = {"review_sha256": sha256_file(evidence_path), "operator": review["operator"],
                    "downloaded_mission_sha256": downloaded["sha256"]}
    elif evidence_path is not None:
        raise ValueError("Numbering evidence requires explicit --verified-numbering confirmation")
    return mapped, plan_hash, phase_hash, evidence


def extract_flight_phases(root, plan, phase_map, output, verified_numbering=False,
                          numbering_evidence=None):
    root, plan, phase_map, output = (Path(p).resolve() for p in (root, plan, phase_map, output))
    if output.is_relative_to(root) or root.is_relative_to(output):
        raise ValueError("Flight phase output must be separate from the immutable recording")
    if any(p.is_relative_to(output) for p in (plan, phase_map,
            *([Path(numbering_evidence).resolve()] if numbering_evidence else []))):
        raise ValueError("Flight phase output must be separate from mission inputs")
    mapped, plan_hash, phase_hash, evidence = verified_map(
        plan, phase_map, verified_numbering, numbering_evidence)
    sealed = check_seal(root)
    if (root / "state").read_text().strip() != "complete":
        raise ValueError("Recording did not finish cleanly")
    errors, events = [], []
    with reader(root) as bag:
        selected = [c for c in bag.connections if c.topic == TOPIC]
        if any(c.msgtype != MESSAGE_TYPE for c in selected):
            raise ValueError("Mission reached topic has an unexpected message type")
        count = sum(c.msgcount for c in selected)
        for connection, receipt, raw in (bag.messages(connections=selected) if selected else []):
            message = bag.deserialize(raw, connection.msgtype)
            seq, source_ns = int(message.wp_seq), stamp(message)
            item = mapped[seq] if 0 <= seq < len(mapped) else None
            events.append({"ordinal": len(events), "mission_seq": seq,
                           "receipt_timestamp_ns": int(receipt), "header_stamp_ros_ns": source_ns,
                           "header_frame_id": message.header.frame_id,
                           "phase_endpoint": None if item is None else item["phase"],
                           "command": None if item is None else item["command"]})
    if len(events) != count:
        errors.append("Read event count differs from bag metadata")
    if not events:
        errors.append("No mission reached events recorded")
    for index, event in enumerate(events):
        if event["command"] is None:
            errors.append(f"Unknown mission index at event {index}")
        if event["header_stamp_ros_ns"] <= 0 or event["receipt_timestamp_ns"] <= 0:
            errors.append(f"Invalid event timestamp at event {index}")
        if index:
            previous = events[index - 1]
            if event["mission_seq"] <= previous["mission_seq"]:
                errors.append(f"Repeated/reordered mission index or restart at event {index}")
            if any(event[key] <= previous[key] for key in
                   ("header_stamp_ros_ns", "receipt_timestamp_ns")):
                errors.append(f"Nonmonotonic event clock at event {index}")
    observed = {event["mission_seq"] for event in events}
    missing = [item["mission_seq"] for item in mapped
               if item["command"] in NAVIGATION and item["mission_seq"] not in observed]
    if missing:
        errors.append("Missing navigation endpoint(s): " + ", ".join(map(str, missing)))
    qualified = evidence is not None and not errors
    windows = []
    for previous, event in zip(events, events[1:]):
        between = mapped[previous["mission_seq"] + 1:event["mission_seq"]]
        contiguous = (previous["command"] is not None
                      and event["mission_seq"] > previous["mission_seq"]
                      and event["receipt_timestamp_ns"] > previous["receipt_timestamp_ns"]
                      and event["header_stamp_ros_ns"] > previous["header_stamp_ros_ns"]
                      and not any(item["command"] in NAVIGATION for item in between))
        if event["command"] not in NAVIGATION or not contiguous:
            continue
        windows.append({"phase": "flight", "destination_phase_hint": event["phase_endpoint"],
                        "from_event_ordinal": previous["ordinal"], "to_event_ordinal": event["ordinal"],
                        "receipt_start_ns": previous["receipt_timestamp_ns"],
                        "receipt_end_ns": event["receipt_timestamp_ns"],
                        "header_start_ros_ns": previous["header_stamp_ros_ns"],
                        "header_end_ros_ns": event["header_stamp_ros_ns"],
                        "sequence_qualified": qualified, "solver_window_qualified": False,
                        "meaning": "Between reached endpoints; transit, settling and holds may overlap"})
    report = {"schema_version": 1, "kind": "flight_phase_evidence", "status": "complete",
              "passed": qualified, "sequence_qualified": qualified, "survey_ready": False,
              "source_seal_sha256": sha256_file(root / "SHA256SUMS"), "sealed_files": sealed,
              "plan_sha256": plan_hash, "phase_map_sha256": phase_hash,
              "numbering_evidence": evidence, "errors": errors,
              "warnings": [] if evidence else ["Mission numbering unverified: diagnostic labels only"],
              "events": events, "candidate_windows": windows,
              "limitations": ["Reached endpoints are not phase onset or guaranteed hold completion",
                              "Header and receipt times are preserved, not camera exposure time",
                              "Mission labels do not establish per-axis excitation or observability",
                              "No solver window is qualified without state/clock/abort evidence"]}
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / "flight-phases.json", report)
    return report
