import copy
import json

import pytest

from wallering_mapping.exposure_events import WEEK_NS, associate_events, check_events, gps_ns


def evidence():
    timing = {"schema": "wallering.exposure-events/v1", "status": "measured", "polarity": "rising",
              "alignment_evidence_sha256": "a" * 64, "frame_counter_bits": 16, "event_counter_bits": 16,
              "counter_offset": 10, "event_to_exposure_start_ns": -100, "event_to_exposure_sigma_ns": 30,
              "exposure_duration_sigma_ns": 20, "camera_clock_domain": "camera_ticks", "epoch": "boot-1"}
    frames = [{"counter": n, "counter_extended": n, "epoch": "boot-1", "timestamp_ticks": n*100,
               "clock_domain": "camera_ticks", "exposure_ns": 400_000,
               "host_received_ns": 100000000000 + n} for n in (1, 2, 3)]
    edges = [{"counter": n+10, "counter_extended": n+10, "epoch": "boot-1", "polarity": "rising",
              "time_system": "GPS", "week_is_full": True, "time_valid": True, "week": 2440,
              "tow_ns": 1_000_000_000 + n*1_000_000, "sigma_ns": 40} for n in (1, 2, 3)]
    return frames, edges, timing


def test_counter_alignment_and_midpoint_keep_gps_time_and_independent_uncertainties():
    frames, edges, timing = evidence()
    report = associate_events(frames, edges, timing)
    assert report["passed"] and len(report["matches"]) == 3
    row = report["matches"][0]
    assert row["exposure_midpoint_gps_ns"] == 2440 * WEEK_NS + 1_001_000_000 - 100 + 200_000
    assert row["combined_sigma_ns"] == pytest.approx((30**2 + 40**2 + 10**2)**.5)
    assert row["qualified"] and not report["survey_ready"]
    assert row["time_system"] == "GPS"
    frames[0]["host_received_ns"] += 999_000_000_000
    assert associate_events(frames, edges, timing) == report


@pytest.mark.parametrize("target,key,value", [
    ("frames", "timestamp_ticks", 0),
    ("frames", "counter_extended", 0),
    ("frames", "epoch", "boot-2"),
    ("edges", "tow_ns", 0),
    ("edges", "time_system", "UTC"),
    ("edges", "week_is_full", False),
    ("edges", "time_valid", False),
])
def test_resets_and_ambiguous_gnss_time_disqualify_the_entire_epoch(target, key, value):
    frames, edges, timing = evidence()
    (frames if target == "frames" else edges)[1][key] = value
    report = associate_events(frames, edges, timing)
    assert not report["passed"] and report["errors"]
    assert not any(m["qualified"] for m in report["matches"])


def test_missing_duplicate_and_opposite_polarity_edges_are_retained():
    frames, edges, timing = evidence()
    missing = associate_events(frames, edges[:1] + edges[2:], timing)
    assert not missing["passed"] and missing["unmatched_frames"] == [1]
    duplicate = associate_events(frames, [edges[0], copy.deepcopy(edges[0]), *edges[1:]], timing)
    assert not duplicate["passed"] and duplicate["unmatched_edges"] == [1]
    wrong = associate_events(frames, [{**e, "polarity": "falling"} for e in edges], timing)
    assert wrong["unmatched_edges"] == [0, 1, 2]
    assert wrong["unmatched_frames"] == [0, 1, 2]
    both_missing = associate_events(frames[::2], edges[::2], timing)
    assert not both_missing["passed"]
    assert any("counter gap" in e for e in both_missing["errors"])


def test_explicit_counter_wrap_and_gps_week_rollover_preserve_continuity():
    frames, edges, timing = evidence()
    timing["counter_offset"] = 0
    for n, (f, e) in enumerate(zip(frames, edges)):
        f.update(counter=(65535+n) % 65536, counter_extended=65535+n)
        e.update(counter=(65535+n) % 65536, counter_extended=65535+n,
                 week=2440 if n == 0 else 2441, tow_ns=WEEK_NS-100 if n == 0 else n*100)
    assert associate_events(frames, edges, timing)["passed"]
    frames[1]["counter_extended"] = 0  # raw rollover without an evidenced extended counter
    assert not associate_events(frames, edges, timing)["passed"]
    with pytest.raises(ValueError, match="outside"):
        gps_ns({**edges[0], "tow_ns": WEEK_NS})


def test_offline_audit_copies_exact_inputs_and_refuses_overwrite(tmp_path):
    frames, edges, timing = evidence()
    fp, ep, tp = (tmp_path/name for name in ("frames.jsonl", "edges.jsonl", "timing.json"))
    for path, rows in ((fp, frames), (ep, edges)):
        path.write_text("".join(json.dumps(row)+"\n" for row in rows))
    tp.write_text(json.dumps(timing))
    output = tmp_path / "audit"
    result = check_events(fp, ep, tp, output)
    assert result["passed"] and (output / "frames.jsonl").read_bytes() == fp.read_bytes()
    assert len(result["inputs_sha256"]) == 3
    with pytest.raises(FileExistsError):
        check_events(fp, ep, tp, output)
