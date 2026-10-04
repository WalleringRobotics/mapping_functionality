"""Diagnostic camera/body association on sealed ROS recordings, with explicit clocks."""

import csv
import json
import math
from pathlib import Path

from .association import STREAM_CAMERAS, camera_pose, pose_at
from .bags import CAMERAS, check_seal, plain, reader, stamp
from .dataset import safe_path, sha256_file, write_json
from .rig_calibration import LEFT, RigCalibration, check, factory_camera_links

POSE_TOPIC = "/mavros/local_position/pose"
NO_BRIDGE = "No explicit image-header to pose-header bridge; timestamps are unshifted diagnostics"


def add_arguments(parser):
    parser.add_argument("session", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--stream", choices=list(CAMERAS), default="left")
    parser.add_argument("--rig-calibration", type=Path)
    parser.add_argument("--timestamp-bridge", type=Path)
    parser.add_argument("--max-pose-gap-ms", type=float, default=100)
    parser.add_argument("--min-fraction", type=float, default=.9)


def run(args):
    return associate_bag(args.session, args.output, args.stream, args.rig_calibration,
                         args.timestamp_bridge, args.max_pose_gap_ms, args.min_fraction)


def load_bridge(path, image_topic, seal_hash, rig_path, rig):
    """Ordered, evidenced links only: never infer identity from similar clock names."""
    if path is None:
        return {"applied": False, "offset_ns": 0, "sigma_sum_ns": None, "links": [],
                "blocking": [NO_BRIDGE]}
    path = Path(path).resolve()
    bridge = json.loads(path.read_text())
    if not isinstance(bridge, dict):
        raise ValueError("Timestamp bridge must be a JSON object")
    rig_hash = sha256_file(rig_path) if rig_path else None
    if (bridge.get("schema_version") != 1 or bridge.get("source_seal_sha256") != seal_hash
            or bridge.get("rig_calibration_sha256") != rig_hash):
        raise ValueError("Timestamp bridge schema or source/rig hash mismatch")
    links = bridge.get("links", [])
    if not isinstance(links, list) or not links or len(links) > 8:
        raise ValueError("Timestamp bridge requires one to eight explicit ordered links")
    domain, seen = f"ros_header:{image_topic}", set()
    offset_sum, sigma_sum, resolved = 0, 0, []
    for link in links:
        if not isinstance(link, dict):
            raise ValueError("Timestamp bridge links must be JSON objects")
        target = link.get("target")
        if (link.get("source") != domain or not isinstance(target, str) or not target
                or target == domain or target in seen):
            raise ValueError("Timestamp bridge domains must be ordered, adjacent and acyclic")
        seen.add(domain)
        entry = {"source": domain, "target": target}
        if "rig_offset" in link:
            key = link["rig_offset"]
            if (rig is None or not isinstance(key, list) or len(key) != 2
                    or not all(isinstance(value, str) for value in key)
                    or key != [domain, target] or tuple(key) not in rig.offsets
                    or "offset_ns" in link or "sigma_ns" in link):
                raise ValueError("Timestamp bridge rig offset must match an existing exact clock pair")
            offset = rig.offsets[tuple(key)]
            entry.update(offset_ns=offset["offset_ns"], sigma_ns=offset["sigma_ns"],
                         rig_offset=key, calibration_source=offset["source"])
        else:
            evidence = link.get("evidence", {})
            if (not isinstance(evidence, dict) or not isinstance(evidence.get("path"), str)
                    or not evidence["path"]):
                raise ValueError("Measured timestamp bridge link requires hashed evidence")
            evidence_path = safe_path(path.parent, evidence["path"])
            if not evidence_path.is_file() or sha256_file(evidence_path) != evidence.get("sha256"):
                raise ValueError("Timestamp bridge evidence checksum mismatch")
            entry.update(offset_ns=link.get("offset_ns"), sigma_ns=link.get("sigma_ns"),
                         evidence=evidence)
        if (type(entry["offset_ns"]) is not int or abs(entry["offset_ns"]) > 10**9
                or type(entry["sigma_ns"]) is not int or not 0 <= entry["sigma_ns"] <= 10**9):
            raise ValueError("Bridge offsets/sigmas must be known integer ns within one second")
        offset_sum += entry["offset_ns"]
        sigma_sum += entry["sigma_ns"]
        resolved.append(entry)
        domain = target
    if domain != f"ros_header:{POSE_TOPIC}" or abs(offset_sum) > 10**9:
        raise ValueError("Bridge must end at the exact pose header domain with net offset within one second")
    return {"applied": True, "sha256": sha256_file(path), "offset_ns": offset_sum,
            "sigma_sum_ns": sigma_sum, "links": resolved, "blocking": [],
            "semantics": "target timestamp = source timestamp + offset; sigma sum is not a latency bound"}


def rig_geometry(path, root, stream):
    if path is None:
        return None, None, {"blocking": ["No rig calibration: camera transform unavailable"]}
    rig = RigCalibration.read(path)
    report = {"sha256": sha256_file(path), **check(rig)}
    report["blocking"] = list(report["blocking"])
    factory = None
    files = sorted(root.glob("*_calibration.json"))
    if len(files) > 1:
        raise ValueError("Ambiguous OAK factory calibration identity")
    if files:
        device = files[0].name.removesuffix("_calibration.json")
        if rig.oak_device_id not in (None, device):
            raise ValueError("Rig and recorded OAK device identities differ")
        report.update(recorded_oak_device_id=device, factory_sha256=sha256_file(files[0]))
        factory = factory_camera_links(json.loads(files[0].read_text()))
    else:
        report["blocking"].append("No recorded OAK factory identity")
    if rig.oak_device_id is None:
        report["blocking"].append("Rig OAK device identity unset")
    camera = STREAM_CAMERAS[stream]
    transform = None
    if camera != LEFT and (factory is None or camera not in factory):
        report["blocking"].append(f"Missing factory transform for {camera}")
    else:
        transform = rig.camera_to_body(camera, factory)
    report["body_to_camera"] = None if transform is None else transform.report()
    if transform is not None and transform.covariance is None:
        report["blocking"].append(f"Camera transform uncertainty unknown: {camera}")
    return rig, transform, report


def associate_bag(root, output, stream="left", rig_calibration=None, timestamp_bridge=None,
                  max_pose_gap_ms=100, min_fraction=.9):
    root, output = Path(root).resolve(), Path(output).resolve()
    rig_calibration = Path(rig_calibration).resolve() if rig_calibration else None
    if output.is_relative_to(root) or root.is_relative_to(output):
        raise ValueError("Association output must be separate from the immutable recording")
    if any(Path(p).resolve().is_relative_to(output) for p in (rig_calibration, timestamp_bridge) if p):
        raise ValueError("Association output must be separate from calibration inputs")
    if (stream not in CAMERAS or not math.isfinite(max_pose_gap_ms) or not 0 < max_pose_gap_ms <= 1000
            or not math.isfinite(min_fraction) or not 0 <= min_fraction <= 1):
        raise ValueError("Invalid stream, pose gap (0,1000] ms or minimum fraction [0,1]")
    sealed = check_seal(root)
    if (root / "state").read_text().strip() != "complete":
        raise ValueError("Recording did not finish cleanly")
    seal_hash = sha256_file(root / "SHA256SUMS")
    rig, transform, rig_report = rig_geometry(rig_calibration, root, stream)
    bridge = load_bridge(timestamp_bridge, CAMERAS[stream], seal_hash, rig_calibration, rig)
    images, poses, counts = [], [], {}
    with reader(root) as bag:
        selected = [c for c in bag.connections if c.topic in {CAMERAS[stream], POSE_TOPIC}]
        for connection in selected:
            expected = "geometry_msgs/msg/PoseStamped" if connection.topic == POSE_TOPIC else "sensor_msgs/msg/Image"
            if connection.msgtype != expected:
                raise ValueError(f"Unexpected message type on {connection.topic}")
            counts[connection.topic] = counts.get(connection.topic, 0) + connection.msgcount
        for connection, receipt, raw in (bag.messages(connections=selected) if selected else []):
            message = bag.deserialize(raw, connection.msgtype)
            source = stamp(message)
            rows = poses if connection.topic == POSE_TOPIC else images
            if source <= 0 or (rows and source <= rows[-1]["source_stamp_ros_ns"]):
                raise ValueError(f"Invalid/nonmonotonic header stamps: {connection.topic}")
            if int(receipt) <= 0 or (rows and int(receipt) <= rows[-1]["receipt_timestamp_ns"]):
                raise ValueError(f"Invalid/nonmonotonic receipt stamps: {connection.topic}")
            frame = message.header.frame_id
            if not frame or (rows and frame != rows[-1]["frame_id"]):
                raise ValueError(f"Missing/changed frame: {connection.topic}")
            row = {"ordinal": len(rows), "source_stamp_ros_ns": source,
                   "receipt_timestamp_ns": int(receipt), "frame_id": frame}
            if connection.topic == POSE_TOPIC:
                row["fields"] = plain(message)
            rows.append(row)
    if counts != {CAMERAS[stream]: len(images), POSE_TOPIC: len(poses)} or not images or not poses:
        raise ValueError("Image/pose topics are missing or counts disagree with bag metadata")
    times = [row["source_stamp_ros_ns"] for row in poses]
    # Validate every pose, including those outside the camera interval.
    for row in poses:
        pose_at([row], [row["source_stamp_ros_ns"]], row["source_stamp_ros_ns"], 1)
    blocking = list(rig_report["blocking"]) + bridge["blocking"] + [
        "Diagnostic ROS association only: absolute exposure latency/rolling shutter unqualified",
        "Vehicle attitude uncertainty and pose body/frame semantics require independent evidence",
        "No GNSS antenna reference or absolute geolocation uncertainty is evaluated"]
    camera_frame_matches = images[0]["frame_id"] == STREAM_CAMERAS[stream]
    if transform is not None and not camera_frame_matches:
        blocking.append("Image header frame does not match rig camera frame; no alias is inferred")
        transform = None
    decisions = []
    for image in images:
        target = image["source_stamp_ros_ns"] + bridge["offset_ns"]
        record = {"image_id": f"{CAMERAS[stream]}#{image['ordinal']}",
                  "image_header_stamp_ros_ns": image["source_stamp_ros_ns"],
                  "image_receipt_timestamp_ns": image["receipt_timestamp_ns"],
                  "image_frame_id": image["frame_id"], "pose_lookup_stamp_ros_ns": target,
                  "camera_pose": None, "survey_ready": False}
        try:
            body = pose_at(poses, times, target, round(max_pose_gap_ms * 1e6))
            if body["after_source_ros_ns"] - body["before_source_ros_ns"] > max_pose_gap_ms * 1e6:
                raise ValueError("Pose sample gap exceeds configured limit")
            record.update(status="associated", body_pose=body,
                          pose_before_receipt_timestamp_ns=poses[body["before_ordinal"]]["receipt_timestamp_ns"],
                          pose_after_receipt_timestamp_ns=poses[body["after_ordinal"]]["receipt_timestamp_ns"],
                          camera_pose=None if transform is None else camera_pose(body, transform))
        except ValueError as error:
            record.update(status="unassociated", reason=str(error))
        decisions.append(record)
    matched = [row for row in decisions if row["status"] == "associated"]
    report = {"schema_version": 1, "kind": "diagnostic_ros_bag_association", "status": "complete",
              "source_seal_sha256": seal_hash, "sealed_files": sealed, "stream": stream,
              "image_topic": CAMERAS[stream], "pose_topic": POSE_TOPIC,
              "rig_calibration": rig_report, "timestamp_bridge": bridge,
              "associated": len(matched), "unassociated": len(images) - len(matched),
              "associated_fraction": len(matched) / len(images),
              "passed": bool(matched) and len(matched) / len(images) >= min_fraction,
              "survey_ready": False, "blocking": list(dict.fromkeys(blocking)),
              "pose_convention": "Assumed MAVROS ENU pose of body FLU; not independently verified",
              "timestamp_semantics": "Image/Pose ROS headers and bag receipts retained separately",
              "output_hashes": {}}
    output.mkdir(parents=True, exist_ok=False)
    with (output / "associations.jsonl").open("x") as file:
        for row in decisions:
            file.write(json.dumps(row, allow_nan=False) + "\n")
    for name, key, orientation in (("body-poses.csv", "body_pose", "orientation_body_flu_to_enu_xyzw"),
                                    ("camera-poses.csv", "camera_pose", "orientation_camera_to_enu_xyzw")):
        with (output / name).open("x", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["image_id", "image_header_stamp_ros_ns", "image_receipt_timestamp_ns",
                             "pose_lookup_stamp_ros_ns", "x_enu_m", "y_enu_m", "z_enu_m",
                             "qx", "qy", "qz", "qw"])
            for row in matched:
                if row[key] is not None:
                    writer.writerow([row["image_id"], row["image_header_stamp_ros_ns"],
                                     row["image_receipt_timestamp_ns"], row["pose_lookup_stamp_ros_ns"],
                                     *row[key]["position_enu_m"], *row[key][orientation]])
    report["output_hashes"] = {name: sha256_file(output / name) for name in
                              ("associations.jsonl", "body-poses.csv", "camera-poses.csv")}
    write_json(output / "report.json", report)
    return report
