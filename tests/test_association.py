import json
from pathlib import Path

import numpy as np
import pytest

from test_mavros import telemetry_fixture
from wallering_mapping.association import ClockMap, associate, camera_pose, pose_at, slerp_xyzw
from wallering_mapping.cli import main
from wallering_mapping.dataset import jsonl, sha256_file, write_json
from wallering_mapping.rig_calibration import (ANTENNA, BODY, CAMERA_SOCKETS, LEFT, Transform,
                                               rotation_from_rpy_deg, rotation_from_xyzw)

RGB = CAMERA_SOCKETS[0]
TEMPLATE = Path(__file__).resolve().parents[1] / "configs/rig-calibration.template.json"


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
    assert row["estimated_alignment_budget_ms"] < 3
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


def add_factory(root):
    """OAK factory extrinsics in the session: RGB coincides with left, right 7.5 cm along image-right."""
    def extrinsics(target, translation_cm):
        return {"toCameraSocket": target, "rotationMatrix": np.eye(3).tolist(),
                "translation": dict(zip("xyz", translation_cm))}
    write_json(root / "calibration.json", {"cameraData": [
        [0, {"extrinsics": extrinsics(1, [0, 0, 0])}], [1, {"extrinsics": extrinsics(2, [-7.5, 0, 0])}],
        [2, {"extrinsics": extrinsics(-1, [0, 0, 0])}]]})
    manifest = json.loads((root / "manifest.json").read_text())
    manifest["calibration_sha256"] = sha256_file(root / "calibration.json")
    write_json(root / "manifest.json", manifest)


def rig_file(path, camera_rpy=(0, 0, 0), camera_m=(0, 0, 0), camera_sigma=(0, 0), antenna_m=(0, 0, 0),
             antenna_sigma=0, offset_ns=0, sigma_ns=0, unset=()):
    data = json.loads(TEMPLATE.read_text())
    _, _, direct, antenna = data["transforms"]
    direct.update(source="manual_measurement", rotation_rpy_deg=list(camera_rpy), translation_m=list(camera_m),
                  rotation_sigma_rad=[camera_sigma[0]] * 3, translation_sigma_m=[camera_sigma[1]] * 3)
    antenna.update(source="manual_measurement", translation_m=list(antenna_m),
                   translation_sigma_m=[antenna_sigma] * 3)
    for entry in data["time_offsets"]:
        entry.update(source="manual_measurement", offset_ns=0, sigma_ns=0)
    data["time_offsets"][0].update(offset_ns=offset_ns, sigma_ns=sigma_ns)
    for entry in data["transforms"] + data["time_offsets"]:
        if (entry.get("parent", entry.get("clock")), entry.get("child", entry.get("reference"))) in unset:
            entry["source"] = "unset"
    write_json(path, data)
    return path


def test_identity_rig_calibration_reproduces_body_association(tmp_path):
    root, _ = telemetry_fixture(tmp_path / "capture")
    add_factory(root)
    plain = associate(root, tmp_path / "plain")
    calibrated = associate(root, tmp_path / "rig", rig_calibration=rig_file(tmp_path / "rig.json"))
    assert (tmp_path / "plain/body-poses.csv").read_bytes() == (tmp_path / "rig/body-poses.csv").read_bytes()
    assert calibrated["alignment_budget_ms"] == plain["alignment_budget_ms"]
    extra = {"sdk_exposure_monotonic_ns", "rig_time_offset_ns", "rig_time_offset_sigma_ns", "camera_pose"}
    for before, after in zip(jsonl(tmp_path / "plain/associations.jsonl"),
                             jsonl(tmp_path / "rig/associations.jsonl"), strict=True):
        assert {k: v for k, v in after.items() if k not in extra} == before
        assert after["camera_pose"]["position_enu_m"] == before["body_pose"]["position_enu_m"]
        assert after["camera_pose"]["orientation_camera_to_enu_xyzw"] == before["body_pose"][
            "orientation_body_flu_to_enu_xyzw"]
        assert after["camera_pose"]["camera_frame"] == RGB
    rig = calibrated["rig_calibration"]
    assert rig["survey_ready"] and rig["blocking"] == [] and rig["camera_path"] == "direct"
    assert rig["sha256"] == sha256_file(tmp_path / "rig.json")
    assert "Not supplied" not in calibrated["camera_extrinsics"]
    assert "camera-poses.csv" in calibrated["output_hashes"]
    assert "rig_calibration" not in plain and "camera-poses.csv" not in plain["output_hashes"]


def test_known_offset_rotation_and_lever_give_hand_computed_camera_pose(tmp_path, capsys):
    root, _ = telemetry_fixture(tmp_path / "capture")
    add_factory(root)
    plain = associate(root, tmp_path / "plain")
    # Forward-looking camera 0.2 m ahead of and 0.1 m below the body origin; OAK stamps 50 ms early.
    path = rig_file(tmp_path / "rig.json", camera_rpy=(-90, 0, -90), camera_m=(0.2, 0, -0.1),
                    camera_sigma=(0.01, 0.005), antenna_sigma=0.01, offset_ns=50_000_000, sigma_ns=1_000_000)
    assert main(["sync", str(root), "--output", str(tmp_path / "rig"), "--rig-calibration", str(path)]) == 0
    capsys.readouterr()
    row = next(jsonl(tmp_path / "rig/associations.jsonl"))
    before = next(jsonl(tmp_path / "plain/associations.jsonl"))
    assert row["sdk_exposure_monotonic_ns"] == 10_500_000_000
    assert row["exposure_monotonic_ns"] == 10_550_000_000
    assert row["exposure_ros_ns"] == 1700000001550000000
    assert row["body_pose"]["position_enu_m"] == pytest.approx([5.5, 2, 3])
    assert row["estimated_alignment_budget_ms"] == pytest.approx(before["estimated_alignment_budget_ms"] + 1)
    camera = row["camera_pose"]
    np.testing.assert_allclose(camera["position_enu_m"], [5.7, 2, 2.9], atol=1e-12)
    rotation = rotation_from_xyzw(camera["orientation_camera_to_enu_xyzw"])
    np.testing.assert_allclose(rotation @ [0, 0, 1], [1, 0, 0], atol=1e-12)  # lens along body forward
    np.testing.assert_allclose(rotation @ [1, 0, 0], [0, -1, 0], atol=1e-12)  # image right is body right
    report = json.loads((tmp_path / "rig/report.json").read_text())
    assert report["rig_calibration"]["time_offset"]["offset_ns"] == 50_000_000
    assert report["rig_calibration"]["body_to_camera"]["translation_sigma_m"] == pytest.approx([0.005] * 3)
    assert report["rig_calibration"]["by_source"]["manual_measurement"] == [
        "base_link->gnss_antenna_arp", "base_link->oak_left_camera_optical_frame",
        "oak_camera_exposure->oak_imu", "oak_ros_stamp->px4_ros_stamp"]
    assert plain["associated"] == report["associated"] == 3


def test_camera_pose_composes_body_attitude_with_mounting():
    yaw = {"position_enu_m": [1, 2, 3], "orientation_body_flu_to_enu_xyzw": [0, 0, 2**-0.5, 2**-0.5]}
    mount = Transform(BODY, LEFT, rotation_from_rpy_deg([180, 0, -90]), [0.2, 0.1, -0.05])
    result = camera_pose(yaw, mount)
    # Body forward points north: 0.2 m forward is +y, 0.1 m left is -x.
    np.testing.assert_allclose(result["position_enu_m"], [0.9, 2.2, 2.95], atol=1e-12)
    rotation = rotation_from_xyzw(result["orientation_camera_to_enu_xyzw"])
    np.testing.assert_allclose(rotation @ [0, 0, 1], [0, 0, -1], atol=1e-12)  # nadir camera
    np.testing.assert_allclose(rotation @ [0, -1, 0], [0, 1, 0], atol=1e-12)  # image top toward vehicle front


def test_missing_links_block_survey_readiness_with_precise_reasons(tmp_path):
    root, _ = telemetry_fixture(tmp_path / "capture")
    add_factory(root)
    path = rig_file(tmp_path / "rig.json", unset=[(BODY, LEFT), (BODY, ANTENNA)])
    result = associate(root, tmp_path / "rig", rig_calibration=path)
    rig = result["rig_calibration"]
    assert result["passed"] and not rig["survey_ready"]
    assert rig["blocking"] == [f"{BODY}->{LEFT} (directly or via oak_imu_frame)", f"{BODY}->{ANTENNA}"]
    assert rig["by_source"]["unset"] == [f"{BODY}->{ANTENNA}", f"{BODY}->oak_imu_frame", f"{BODY}->{LEFT}",
                                         f"oak_imu_frame->{LEFT}"]
    assert result["camera_extrinsics"].startswith(f"Not supplied: rig calibration leaves {BODY}->{RGB} unset")
    assert "camera-poses.csv" not in result["output_hashes"]
    assert all(row["camera_pose"] is None for row in jsonl(tmp_path / "rig/associations.jsonl"))
    # Camera link set but the OAK->PX4 offset sigma unknown: pose is composed, survey stays blocked.
    data = json.loads(rig_file(tmp_path / "partial.json").read_text())
    data["time_offsets"][0]["sigma_ns"] = None
    write_json(tmp_path / "partial.json", data)
    result = associate(root, tmp_path / "partial", rig_calibration=tmp_path / "partial.json")
    assert result["rig_calibration"]["blocking"] == ["time offset oak_ros_stamp->px4_ros_stamp"]
    assert "camera-poses.csv" in result["output_hashes"]
    assert "Rig calibration" in result["camera_extrinsics"]


def test_rig_calibration_for_rgb_needs_session_factory_extrinsics(tmp_path):
    root, _ = telemetry_fixture(tmp_path / "capture")
    with pytest.raises(ValueError, match="factory extrinsics"):
        associate(root, tmp_path / "rig", rig_calibration=rig_file(tmp_path / "rig.json"))
