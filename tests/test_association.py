import json

import numpy as np
import pytest

from test_mavros import telemetry_fixture
from wallering_mapping.association import ClockMap, associate, pose_at, slerp_xyzw
from wallering_mapping.dataset import jsonl, sha256_file, write_json


def test_exposure_association_uses_device_mapping_not_receipt_and_does_not_apply_offset_twice(tmp_path):
    root, _ = telemetry_fixture(tmp_path / "capture")
    output = tmp_path / "association"
    result = associate(root, output)
    assert result["passed"] and result["associated"] == 3
    row = next(jsonl(output / "associations.jsonl"))
    assert row["exposure_monotonic_ns"] == 10500000000
    assert row["exposure_ros_ns"] == 1700000001500000000
    assert row["estimated_px4_boot_ns"] == 1500000000
    assert row["body_pose"]["position_enu_m"] == [5, 2, 3]
    assert row["estimated_alignment_budget_ms"] < 2
    assert row["gnss"] is None
    assert "camera" in result["camera_extrinsics"]
    with pytest.raises(FileExistsError):
        associate(root, output)


def test_pose_interpolation_with_antipodal_quaternions_and_no_extrapolation():
    a = {"ordinal": 1, "source_stamp_ros_ns": 1, "fields": {"header": {"frame_id": "map"},
         "pose": {"position": {"x": 0, "y": 0, "z": 0},
                  "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}}}
    b = json.loads(json.dumps(a))
    b["ordinal"], b["source_stamp_ros_ns"] = 2, 3
    b["fields"]["pose"]["position"]["x"] = 2
    b["fields"]["pose"]["orientation"]["w"] = -1
    result = pose_at([a, b], [100, 300], 200, 150)
    assert result["position_enu_m"] == [1, 0, 0]
    np.testing.assert_allclose(result["orientation_body_flu_to_enu_xyzw"], [0, 0, 0, 1])
    with pytest.raises(ValueError, match="extrapolation"):
        pose_at([a, b], [100, 300], 400, 150)
    with pytest.raises(ValueError, match="bracket"):
        pose_at([a, b], [100, 300], 200, 50)
    with pytest.raises(ValueError, match="quaternion"):
        slerp_xyzw([0, 0, 0, 0], [0, 0, 0, 1], .5)


def test_large_epoch_preserves_nanoseconds_and_stale_clock_is_rejected():
    epoch = 1700000000000000000
    samples = [{"target_ns": epoch + i * 1000, "reference_ns": 10000 + i * 1000, "bracket_ns": 2}
               for i in range(2)]
    mapped, _, _ = ClockMap(samples).to_monotonic(epoch + 501, 1000)
    assert mapped == 10501
    with pytest.raises(ValueError, match="Stale clock"):
        ClockMap(samples).to_monotonic(epoch + 10000, 1000)


def test_missing_fresh_clock_evidence_rejects_associations_without_fabrication(tmp_path):
    root, _ = telemetry_fixture(tmp_path / "capture")
    # Keep a valid clock journal with only its earliest sample; later frames have no bridge.
    clock = root / "clock.jsonl"
    clock.write_text(clock.read_text().splitlines()[0] + "\n")
    manifest = json.loads((root / "manifest.json").read_text())
    manifest["journals_sha256"]["clock"] = sha256_file(clock)
    write_json(root / "manifest.json", manifest)
    result = associate(root, tmp_path / "association")
    assert not result["passed"] and result["associated"] == 0 and result["unassociated"] == 3
    assert all(row["reason"] == "Stale clock bridge" for row in jsonl(tmp_path / "association/associations.jsonl"))


def test_timing_budget_gate_separate_from_nearby_pose(tmp_path):
    root, _ = telemetry_fixture(tmp_path / "capture")
    manifest = json.loads((root / "manifest.json").read_text())
    manifest["telemetry"]["config"]["sdk_sync_budget_ms"] = 10
    write_json(root / "manifest.json", manifest)
    result = associate(root, tmp_path / "association")
    assert not result["passed"] and result["associated"] == 0
    assert all("alignment budget" in row["reason"] for row in jsonl(tmp_path / "association/associations.jsonl"))
