import json

import numpy as np
import pytest
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore

from test_association import rig_file
from test_bags import seal
from wallering_mapping.association import STREAM_CAMERAS
from wallering_mapping.bag_association import NO_BRIDGE, POSE_TOPIC, associate_bag
from wallering_mapping.bags import CAMERAS
from wallering_mapping.dataset import jsonl, sha256_file

EPOCH = 1_790_000_000_000_000_000


def fixture(root, *, stream="left", bad_quaternion=False, changed_frame=False, gap=False,
            image_frame=None):
    root.mkdir()
    for name in ("oak-requested.yaml", "oak-parameters.yaml", "mcap.yaml"):
        (root / name).write_text("fixture: true\n")
    (root / "topics.txt").write_text(CAMERAS[stream] + "\n" + POSE_TOPIC + "\n")
    (root / "state").write_text("complete\n")
    store = get_typestore(Stores.ROS2_HUMBLE)
    t = store.types

    def header(ns, frame):
        return t["std_msgs/msg/Header"](t["builtin_interfaces/msg/Time"](ns // 10**9, ns % 10**9), frame)

    with Writer(root / "bag", version=9, storage_plugin=StoragePlugin.MCAP) as writer:
        image = writer.add_connection(CAMERAS[stream], "sensor_msgs/msg/Image", typestore=store)
        pose = writer.add_connection(POSE_TOPIC, "geometry_msgs/msg/PoseStamped", typestore=store)
        for index in range(4):
            if gap and index == 1:
                continue
            ns = EPOCH + index * 100_000_000
            frame = "odom" if changed_frame and index == 2 else "map"
            orientation = t["geometry_msgs/msg/Quaternion"](0., 0., 0., 0. if bad_quaternion else 1.)
            message = t["geometry_msgs/msg/PoseStamped"](header(ns, frame),
                t["geometry_msgs/msg/Pose"](t["geometry_msgs/msg/Point"](index * 10., 2., 3.), orientation))
            writer.write(pose, ns + 1_000_000, store.serialize_cdr(message, message.__msgtype__))
        for index in range(3):
            ns = EPOCH + 50_000_000 + index * 100_000_000
            message = t["sensor_msgs/msg/Image"](header(ns, image_frame or STREAM_CAMERAS[stream]),
                         1, 1, "mono8", 0, 1, np.array([0], dtype=np.uint8))
            writer.write(image, ns + 10_000_000, store.serialize_cdr(message, message.__msgtype__))
    seal(root)
    return root


def bridge_file(path, root, calibration, *, offset=0):
    data = json.loads(calibration.read_text())
    data["time_offsets"][1]["offset_ns"] = -20_000_000
    data["time_offsets"][1]["sigma_ns"] = 2_000_000
    calibration.write_text(json.dumps(data))
    evidence = path.parent / "synthetic-bridge-observations.txt"
    evidence.write_text("Synthetic bridge relation, not physical timing acceptance.\n")

    def measured(source, target, value=0):
        return {"source": source, "target": target, "offset_ns": value, "sigma_ns": 10,
                "evidence": {"path": evidence.name, "sha256": sha256_file(evidence)}}

    links = [measured(f"ros_header:{CAMERAS['left']}", "oak_camera_exposure", offset),
             {"source": "oak_camera_exposure", "target": "oak_imu",
              "rig_offset": ["oak_camera_exposure", "oak_imu"]},
             measured("oak_imu", "oak_ros_stamp"),
             {"source": "oak_ros_stamp", "target": "px4_ros_stamp",
              "rig_offset": ["oak_ros_stamp", "px4_ros_stamp"]},
             measured("px4_ros_stamp", f"ros_header:{POSE_TOPIC}")]
    path.write_text(json.dumps({"schema_version": 1, "source_seal_sha256": sha256_file(root / "SHA256SUMS"),
        "rig_calibration_sha256": sha256_file(calibration), "links": links}))
    return path


def test_unbridged_ros_headers_remain_diagnostic_and_spatial_identity_is_exact(tmp_path):
    root = fixture(tmp_path / "capture")
    calibration = rig_file(tmp_path / "rig.json", offset_ns=50_000_000)
    report = associate_bag(root, tmp_path / "out", rig_calibration=calibration)
    assert report["passed"] and not report["survey_ready"] and NO_BRIDGE in report["blocking"]
    assert not report["timestamp_bridge"]["applied"]
    row = next(jsonl(tmp_path / "out/associations.jsonl"))
    assert row["image_header_stamp_ros_ns"] == row["pose_lookup_stamp_ros_ns"] == EPOCH + 50_000_000
    assert row["image_receipt_timestamp_ns"] == EPOCH + 60_000_000
    assert row["pose_before_receipt_timestamp_ns"] == EPOCH + 1_000_000
    assert row["pose_after_receipt_timestamp_ns"] == EPOCH + 101_000_000
    assert row["body_pose"]["position_enu_m"] == row["camera_pose"]["position_enu_m"] == [5, 2, 3]
    assert report["output_hashes"]["camera-poses.csv"] == sha256_file(tmp_path / "out/camera-poses.csv")
    with pytest.raises(FileExistsError):
        associate_bag(root, tmp_path / "out", rig_calibration=calibration)


def test_explicit_ordered_rig_offsets_compose_with_measured_bridges(tmp_path):
    root = fixture(tmp_path / "capture")
    calibration = rig_file(tmp_path / "rig.json", camera_m=(.2, 0, -.1),
                           camera_rpy=(-90, 0, -90), camera_sigma=(.01, .005),
                           offset_ns=50_000_000, sigma_ns=1_000_000)
    bridge = bridge_file(tmp_path / "bridge.json", root, calibration)
    report = associate_bag(root, tmp_path / "out", rig_calibration=calibration, timestamp_bridge=bridge)
    assert report["timestamp_bridge"]["offset_ns"] == 30_000_000
    assert report["timestamp_bridge"]["sigma_sum_ns"] == 3_000_030
    assert report["passed"] and not report["survey_ready"]
    row = next(jsonl(tmp_path / "out/associations.jsonl"))
    assert row["pose_lookup_stamp_ros_ns"] == EPOCH + 80_000_000
    np.testing.assert_allclose(row["camera_pose"]["position_enu_m"], [8.2, 2, 2.9])
    assert report["rig_calibration"]["body_to_camera"]["translation_sigma_m"] == [.005] * 3
    assert row["image_header_stamp_ros_ns"] == EPOCH + 50_000_000


@pytest.mark.parametrize("mutation,reason", [
    (lambda d: d["links"][2].update(source="assumed_same_clock"), "domains"),
    (lambda d: d["links"][-1].update(target="other_pose_header"), "exact pose"),
    (lambda d: d.update(source_seal_sha256="wrong"), "hash mismatch"),
    (lambda d: d["links"][0].update(sigma_ns=None), "known integer"),
    (lambda d: d["links"][1].update(rig_offset=["oak_ros_stamp", "px4_ros_stamp"]), "exact clock"),
    (lambda d: d["links"][0]["evidence"].update(sha256="wrong"), "checksum"),
])
def test_invalid_explicit_bridge_is_refused_not_silently_ignored(tmp_path, mutation, reason):
    root = fixture(tmp_path / "capture")
    calibration = rig_file(tmp_path / "rig.json")
    bridge = bridge_file(tmp_path / "bridge.json", root, calibration)
    data = json.loads(bridge.read_text())
    mutation(data)
    bridge.write_text(json.dumps(data))
    with pytest.raises(ValueError, match=reason):
        associate_bag(root, tmp_path / "out", rig_calibration=calibration, timestamp_bridge=bridge)
    assert not (tmp_path / "out").exists()


def test_factory_covariance_unknown_does_not_become_zero(tmp_path):
    root = fixture(tmp_path / "capture", stream="right")
    calibration = rig_file(tmp_path / "rig.json")
    factory = {"cameraData": [[2, {"extrinsics": {"toCameraSocket": 1,
                   "rotationMatrix": np.eye(3).tolist(), "translation": {"x": 7.5, "y": 0, "z": 0}}}]]}
    (root / "synthetic-device_calibration.json").write_text(json.dumps(factory))
    seal(root)
    report = associate_bag(root, tmp_path / "out", stream="right", rig_calibration=calibration)
    assert report["rig_calibration"]["body_to_camera"]["translation_sigma_m"] is None
    assert any("uncertainty unknown" in item for item in report["blocking"])
    row = next(jsonl(tmp_path / "out/associations.jsonl"))
    np.testing.assert_allclose(row["camera_pose"]["position_enu_m"], [5.075, 2, 3])


@pytest.mark.parametrize("setting,reason", [("bad_quaternion", "quaternion"), ("changed_frame", "frame")])
def test_bad_pose_or_changed_frame_is_refused(tmp_path, setting, reason):
    root = fixture(tmp_path / "capture", **{setting: True})
    with pytest.raises(ValueError, match=reason):
        associate_bag(root, tmp_path / "out")
    assert not (tmp_path / "out").exists()


def test_pose_gap_rejects_only_unsupported_associations_and_seal_tamper_refused(tmp_path):
    root = fixture(tmp_path / "capture", gap=True)
    report = associate_bag(root, tmp_path / "out")
    assert not report["passed"] and report["associated"] == 1 and report["unassociated"] == 2
    (root / "topics.txt").write_text("modified")
    with pytest.raises(ValueError, match="checksum"):
        associate_bag(root, tmp_path / "other")


def test_output_cannot_overlap_inputs(tmp_path):
    root = fixture(tmp_path / "capture")
    with pytest.raises(ValueError, match="immutable"):
        associate_bag(root, root / "derived")
    with pytest.raises(ValueError, match="immutable"):
        associate_bag(root, tmp_path)


def test_frame_alias_is_not_inferred_for_camera_composition(tmp_path):
    root = fixture(tmp_path / "capture", image_frame="left_camera_unspecified")
    calibration = rig_file(tmp_path / "rig.json", camera_m=(.2, 0, 0))
    report = associate_bag(root, tmp_path / "out", rig_calibration=calibration)
    assert report["passed"] and any("no alias" in item for item in report["blocking"])
    assert all(row["camera_pose"] is None for row in jsonl(tmp_path / "out/associations.jsonl"))
