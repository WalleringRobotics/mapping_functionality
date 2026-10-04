import json

import pytest
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

from test_bags import seal
from test_calibration_mission import mission
from wallering_mapping.calibration_flight import MESSAGE_TYPE, TOPIC, extract_flight_phases
from wallering_mapping.dataset import sha256_file


def fixture(tmp_path, change=None):
    generated = mission(tmp_path)
    plan, phases = map(lambda name: tmp_path / name,
                       ("calibration.plan", "calibration.plan.phases.json"))
    entries = json.loads(phases.read_text())["mission_items"]
    sequences = [item["mission_seq"] for item in entries if item["command"] != 178]
    if change:
        sequences = change(sequences)
    root = tmp_path / "capture"
    root.mkdir()
    for name in ("oak-requested.yaml", "oak-parameters.yaml", "mcap.yaml"):
        (root / name).write_text("fixture: true\n")
    (root / "topics.txt").write_text(TOPIC + "\n")
    (root / "state").write_text("complete\n")
    store = get_typestore(Stores.ROS2_HUMBLE)
    store.register(get_types_from_msg("std_msgs/Header header\nuint16 wp_seq\n", MESSAGE_TYPE))
    with Writer(root / "bag", version=9, storage_plugin=StoragePlugin.MCAP) as writer:
        connection = writer.add_connection(TOPIC, MESSAGE_TYPE, typestore=store)
        for ordinal, seq in enumerate(sequences):
            source = 1_790_000_000_000_000_123 + ordinal * 1_000_000_000
            header = store.types["std_msgs/msg/Header"](
                store.types["builtin_interfaces/msg/Time"](source // 10**9, source % 10**9), "mission")
            message = store.types[MESSAGE_TYPE](header, seq)
            writer.write(connection, source + 10_000_000, store.serialize_cdr(message, MESSAGE_TYPE))
    seal(root)
    # A synthetic operator review proves software hash/sequence handling, not a real SITL test.
    downloaded = tmp_path / "synthetic-download.json"
    downloaded.write_text(plan.read_text())
    evidence = tmp_path / "numbering-review.json"
    evidence.write_text(json.dumps({"schema_version": 1, "operator": "synthetic test only",
        "plan_sha256": generated["plan_sha256"], "phase_map_sha256": sha256_file(phases),
        "zero_based_px4_items_match": True,
        "downloaded_mission": {"path": downloaded.name, "sha256": sha256_file(downloaded)}}))
    return root, plan, phases, evidence


def test_original_event_clocks_and_sequence_mapping_survive_real_mcap_decode(tmp_path):
    root, plan, phases, evidence = fixture(tmp_path)
    result = extract_flight_phases(root, plan, phases, tmp_path / "out", True, evidence)
    assert result["passed"] and result["sequence_qualified"] and not result["survey_ready"]
    assert result["events"][0]["header_stamp_ros_ns"] == 1_790_000_000_000_000_123
    assert result["events"][0]["receipt_timestamp_ns"] == 1_790_000_000_010_000_123
    assert result["events"][0]["header_frame_id"] == "mission"
    assert result["candidate_windows"][0]["destination_phase_hint"] == "yaw"
    assert all(row["phase"] == "flight" and row["sequence_qualified"]
               and not row["solver_window_qualified"] for row in result["candidate_windows"])
    assert result["source_seal_sha256"] == sha256_file(root / "SHA256SUMS")
    with pytest.raises(FileExistsError):
        extract_flight_phases(root, plan, phases, tmp_path / "out", True, evidence)


def test_unverified_numbering_is_diagnostic_even_if_sidecar_claims_verified(tmp_path):
    root, plan, phases, _ = fixture(tmp_path)
    data = json.loads(phases.read_text())
    data["sequence_mapping_verified"] = True
    phases.write_text(json.dumps(data))
    result = extract_flight_phases(root, plan, phases, tmp_path / "out")
    assert not result["passed"] and result["warnings"]
    assert not any(row["sequence_qualified"] for row in result["candidate_windows"])


@pytest.mark.parametrize("change,reason", [
    (lambda rows: rows[:4] + rows[5:], "Missing navigation"),
    (lambda rows: rows[:4] + [rows[3]] + rows[4:], "Repeated/reordered"),
    (lambda rows: rows + [0], "Repeated/reordered"),
    (lambda rows: rows + [65535], "Unknown mission"),
    (lambda rows: [], "No mission"),
])
def test_gaps_replays_restarts_and_unknown_indices_never_qualify(tmp_path, change, reason):
    root, plan, phases, evidence = fixture(tmp_path, change)
    result = extract_flight_phases(root, plan, phases, tmp_path / "out", True, evidence)
    assert not result["passed"] and any(reason in error for error in result["errors"])
    assert not any(row["sequence_qualified"] for row in result["candidate_windows"])


def test_hashes_seal_and_explicit_confirmation_are_required(tmp_path):
    root, plan, phases, evidence = fixture(tmp_path)
    with pytest.raises(ValueError, match="numbering-evidence"):
        extract_flight_phases(root, plan, phases, tmp_path / "out", True)
    with pytest.raises(ValueError, match="explicit"):
        extract_flight_phases(root, plan, phases, tmp_path / "out", False, evidence)
    original = plan.read_bytes()
    plan.write_bytes(original + b"\n")
    with pytest.raises(ValueError, match="SHA256"):
        extract_flight_phases(root, plan, phases, tmp_path / "out", True, evidence)
    plan.write_bytes(original)
    (tmp_path / "synthetic-download.json").write_text("modified")
    with pytest.raises(ValueError, match="checksum"):
        extract_flight_phases(root, plan, phases, tmp_path / "out", True, evidence)
    (root / "topics.txt").write_text("modified")
    with pytest.raises(ValueError, match="checksum"):
        extract_flight_phases(root, plan, phases, tmp_path / "out")
    assert not (tmp_path / "out").exists()


def test_failed_recording_is_refused_without_output(tmp_path):
    root, plan, phases, _ = fixture(tmp_path)
    (root / "state").write_text("failed\n")
    with pytest.raises(ValueError, match="finish cleanly"):
        extract_flight_phases(root, plan, phases, tmp_path / "out")
    assert not (tmp_path / "out").exists()
