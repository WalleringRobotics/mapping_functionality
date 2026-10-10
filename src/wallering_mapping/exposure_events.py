"""Audit normalized camera/F9P evidence by explicit counters, never nearest time.

This is the offline adapter boundary, not a UBX decoder or live GigE driver.
Original RAWX/SFRBX/TIM-TM2 bytes remain separately hashed source evidence.
"""

from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import re

from .capabilities import number
from .dataset import jsonl, sha256_file, write_json

WEEK_NS = 604800 * 1_000_000_000


def integer(value, name, minimum=0):
    if type(value) is not int or value < minimum:
        raise ValueError(f"{name} must be an integer >= {minimum}")
    return value


def gps_ns(row):
    """Keep GPS epoch and full week; no implicit modulo-week or UTC/leap conversion."""
    if row.get("time_system") != "GPS" or row.get("week_is_full") is not True:
        raise ValueError("Events require explicit GPS time and a resolved full GNSS week")
    if row.get("time_valid") is not True:
        raise ValueError("GNSS event time is not valid")
    week = integer(row.get("week"), "GNSS week")
    tow = integer(row.get("tow_ns"), "GNSS tow_ns")
    if tow >= WEEK_NS:
        raise ValueError("GNSS tow_ns is outside its week")
    return week * WEEK_NS + tow


def _counter(row, bits):
    raw = integer(row.get("counter"), "counter")
    extended = integer(row.get("counter_extended"), "counter_extended")
    if raw >= 2**bits or extended % 2**bits != raw:
        raise ValueError("Raw and explicitly unwrapped counter disagree")
    if not isinstance(row.get("epoch"), str) or not row["epoch"]:
        raise ValueError("Every counter requires an explicit continuity epoch")
    return extended


def associate_events(frames, edges, timing):
    if (timing.get("schema") != "wallering.exposure-events/v1"
            or timing.get("status") != "measured" or timing.get("polarity") not in {"rising", "falling"}
            or not re.fullmatch(r"[0-9a-f]{64}", timing.get("alignment_evidence_sha256", ""))):
        raise ValueError("Measured edge/counter alignment with evidence hash and polarity is required")
    for name in ("frame_counter_bits", "event_counter_bits"):
        if type(timing.get(name)) is not int or not 1 <= timing[name] <= 64:
            raise ValueError(f"Invalid {name}")
    if type(timing.get("counter_offset")) is not int:
        raise ValueError("counter_offset must be an explicit integer")
    delay = timing.get("event_to_exposure_start_ns")
    if type(delay) is not int:
        raise ValueError("event_to_exposure_start_ns must be a measured signed integer")
    sigma = number(timing.get("event_to_exposure_sigma_ns"), "event_to_exposure_sigma_ns", zero=True)
    exposure_sigma = number(timing.get("exposure_duration_sigma_ns"), "exposure_duration_sigma_ns", zero=True)
    if not timing.get("camera_clock_domain") or not timing.get("epoch"):
        raise ValueError("Camera clock domain and continuity epoch are required")
    errors, matches, unmatched_frames, unmatched_edges = [], [], [], []
    frame_keys, edge_keys = defaultdict(list), defaultdict(list)
    previous_frame = previous_edge = None
    for index, row in enumerate(frames):
        try:
            count = _counter(row, timing["frame_counter_bits"])
            ticks = integer(row.get("timestamp_ticks"), "timestamp_ticks")
            number(row.get("exposure_ns"), "exposure_ns")
            if row["epoch"] != timing["epoch"] or row.get("clock_domain") != timing["camera_clock_domain"]:
                raise ValueError("Camera epoch/clock domain changed; start a new alignment")
            if previous_frame is not None and (count <= previous_frame[0] or ticks <= previous_frame[1]):
                raise ValueError("Camera counter or clock reset/repeated; continuity is ambiguous")
            if previous_frame is not None and count > previous_frame[0] + 1:
                errors.append(f"Frame {index}: native counter gap ({count - previous_frame[0] - 1})")
            previous_frame = count, ticks
            frame_keys[(row["epoch"], count + timing["counter_offset"])].append(index)
        except ValueError as error:
            errors.append(f"Frame {index}: {error}")
            unmatched_frames.append(index)
    for index, row in enumerate(edges):
        if row.get("polarity") != timing["polarity"]:
            unmatched_edges.append(index)
            continue
        try:
            count = _counter(row, timing["event_counter_bits"])
            stamp = gps_ns(row)
            number(row.get("sigma_ns"), "event sigma_ns", zero=True)
            if row["epoch"] != timing["epoch"]:
                raise ValueError("GNSS/event epoch changed; start a new alignment")
            if previous_edge is not None and (count <= previous_edge[0] or stamp <= previous_edge[1]):
                raise ValueError("Event counter or GPS clock reset/repeated; continuity is ambiguous")
            if previous_edge is not None and count > previous_edge[0] + 1:
                errors.append(f"Edge {index}: native counter gap ({count - previous_edge[0] - 1})")
            previous_edge = count, stamp
            edge_keys[(row["epoch"], count)].append(index)
        except ValueError as error:
            errors.append(f"Edge {index}: {error}")
            unmatched_edges.append(index)
    for key in sorted(frame_keys.keys() | edge_keys.keys()):
        fs, es = frame_keys[key], edge_keys[key]
        if len(fs) != 1 or len(es) != 1:
            unmatched_frames.extend(fs)
            unmatched_edges.extend(es)
            continue
        f, e = fs[0], es[0]
        exposure = frames[f]["exposure_ns"]
        matches.append({"frame_index": f, "edge_index": e, "epoch": key[0],
                        "event_counter_extended": key[1], "time_system": "GPS",
                        "exposure_midpoint_gps_ns": gps_ns(edges[e]) + delay + round(exposure / 2),
                        "event_sigma_ns": edges[e]["sigma_ns"], "edge_delay_sigma_ns": sigma,
                        "exposure_duration_sigma_ns": exposure_sigma,
                        "combined_sigma_ns": math.sqrt(sigma**2 + edges[e]["sigma_ns"]**2 + (exposure_sigma/2)**2),
                        "qualified": False})
    if not frames or not edges:
        errors.append("Both frames and event edges are required")
    if unmatched_frames:
        errors.append("Frames without one unambiguous event edge")
    if any(edges[i].get("polarity") == timing["polarity"] for i in unmatched_edges):
        errors.append("Selected event edges without one unambiguous camera frame")
    # Any reset/duplicate makes this continuity epoch ambiguous, including earlier matches.
    for row in matches:
        row["qualified"] = not errors
    return {"schema_version": 1, "kind": "exposure_event_audit", "passed": not errors,
            "errors": errors, "matches": matches, "unmatched_frames": sorted(set(unmatched_frames)),
            "unmatched_edges": sorted(set(unmatched_edges)), "survey_ready": False,
            "limitations": ["Host receipt timestamps are not exposure times",
                "Counter unwrapping and alignment must be independently evidenced by the adapters",
                "GPS epoch is retained; UTC conversion needs independently evidenced leap seconds",
                "This checks supplied normalized evidence; raw UBX decoding, PPK and geolocation remain separate"]}


def check_events(frame_path, edge_path, timing_path, output):
    paths = {"frames.jsonl": Path(frame_path), "edges.jsonl": Path(edge_path),
             "event-timing.json": Path(timing_path)}
    hashes = {name: sha256_file(path) for name, path in paths.items()}
    result = associate_events(list(jsonl(paths["frames.jsonl"])), list(jsonl(paths["edges.jsonl"])),
                              json.loads(paths["event-timing.json"].read_text()))
    result["inputs_sha256"] = hashes
    snapshots = {name: path.read_bytes() for name, path in paths.items()}
    if any(hashlib.sha256(data).hexdigest() != hashes[name] for name, data in snapshots.items()):
        raise ValueError("Evidence changed during event audit")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    for name, data in snapshots.items():
        (output / name).write_bytes(data)
    write_json(output / "event-check.json", result)
    return result
