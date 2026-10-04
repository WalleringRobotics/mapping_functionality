import copy
import json
from pathlib import Path

import numpy as np
import pytest

from wallering_mapping import cli
from wallering_mapping.rig_calibration import (ANTENNA, BODY, LEFT, OAK_IMU, RigCalibration,
                                               Transform, check, factory_camera_links,
                                               rotation_from_rpy_deg, rotation_from_xyzw,
                                               xyzw_from_rotation)

TEMPLATE = Path(__file__).resolve().parents[1] / "configs/rig-calibration.template.json"


def filled():
    data = json.loads(TEMPLATE.read_text())
    imu, camera, _direct, antenna = data["transforms"]
    imu.update(source="manual_measurement", rotation_rpy_deg=[0, 0, 90], translation_m=[0.10, 0.0, 0.05],
               rotation_sigma_rad=[0.02, 0.02, 0.02], translation_sigma_m=[0.005, 0.005, 0.005])
    # OAK IMU x forward, camera optical z forward: a fixed axis permutation.
    camera.update(source="cad", rotation_rpy_deg=None, rotation_xyzw=[-0.5, 0.5, -0.5, 0.5],
                  translation_m=[0.0, -0.02, 0.0], rotation_sigma_rad=[0.01, 0.01, 0.01],
                  translation_sigma_m=[0.002, 0.002, 0.002])
    antenna.update(source="manual_measurement", translation_m=[-0.20, 0.0, 0.30],
                   translation_sigma_m=[0.01, 0.01, 0.01])
    data["time_offsets"][0].update(source="manual_measurement", offset_ns=-2_000_000, sigma_ns=5_000_000)
    data["time_offsets"][1].update(source="manual_measurement", offset_ns=0, sigma_ns=1_000_000)
    data["hardware"]["oak_device_id"] = "TESTDEVICE"
    return data


def oak_factory():
    def extrinsics(target, rotation, translation_cm):
        return {"toCameraSocket": target, "rotationMatrix": rotation,
                "translation": dict(zip("xyz", translation_cm))}
    return {"boardName": "TESTBOARD", "imuExtrinsics": {"toCameraSocket": -1},
            "cameraData": [[2, {"extrinsics": extrinsics(0, np.eye(3).tolist(), [3.75, 0, 0])}],
                           [1, {"extrinsics": extrinsics(2, np.eye(3).tolist(), [-7.5, 0, 0])}],
                           [0, {"extrinsics": extrinsics(-1, [], [0, 0, 0])}]]}


def random_rotation(rng):
    q = rng.normal(size=4)
    return rotation_from_xyzw(q / np.linalg.norm(q))


def exp_so3(v):
    angle = np.linalg.norm(v)
    if angle < 1e-12:
        return np.eye(3)
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]]) / angle
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * k @ k


def test_template_is_valid_and_reports_every_missing_link():
    result = check(RigCalibration.read(TEMPLATE))
    assert result["complete"] is False
    assert len(result["unset"]) == 6
    assert result["cameras"][LEFT] is None
    assert result["camera_path"] is None


def test_rpy_matches_quaternion_and_round_trips():
    rng = np.random.default_rng(1)
    assert np.allclose(rotation_from_rpy_deg([0, 0, 90]), rotation_from_xyzw([0, 0, 2**-0.5, 2**-0.5]))
    assert np.allclose(rotation_from_rpy_deg([90, 0, 0]) @ [0, 1, 0], [0, 0, 1])
    for _ in range(20):
        rotation = random_rotation(rng)
        assert np.allclose(rotation_from_xyzw(xyzw_from_rotation(rotation)), rotation)


def test_composition_matches_hand_computation():
    calibration = RigCalibration(filled())
    body_camera = calibration.camera_to_body()
    # Camera optical axis (z) points along the OAK IMU x axis, which is body +y after a 90 deg yaw.
    assert np.allclose(body_camera.rotation @ [0, 0, 1], [0, 1, 0])
    # Camera origin: IMU origin plus the yawed IMU-frame offset [0, -0.02, 0] -> body [+0.02, 0, 0].
    assert np.allclose(body_camera.translation, [0.12, 0.0, 0.05])
    lever = calibration.antenna_to_camera()
    assert np.allclose(lever["antenna_to_camera_flu_m"], [0.32, 0.0, -0.25])
    assert lever["sources"] == ["cad", "manual_measurement"]


@pytest.mark.parametrize("operation", ["compose", "inverse"])
def test_first_order_covariance_matches_monte_carlo(operation):
    rng = np.random.default_rng(7)
    a = Transform("a", "b", random_rotation(rng), [0.3, -0.2, 0.5],
                  np.diag([0.01, 0.02, 0.015, 0.004, 0.006, 0.005]) ** 2)
    b = Transform("b", "c", random_rotation(rng), [0.1, 0.4, -0.2],
                  np.diag([0.012, 0.008, 0.01, 0.003, 0.002, 0.004]) ** 2)
    nominal = a.compose(b) if operation == "compose" else a.inverse()
    samples = []
    for _ in range(20000):
        da, db = (rng.multivariate_normal(np.zeros(6), t.covariance) for t in (a, b))
        pa = Transform("a", "b", exp_so3(da[:3]) @ a.rotation, a.translation + da[3:])
        pb = Transform("b", "c", exp_so3(db[:3]) @ b.rotation, b.translation + db[3:])
        result = pa.compose(pb) if operation == "compose" else pa.inverse()
        # Recover the parent-frame rotation perturbation from R_true = Exp(d) R.
        delta = result.rotation @ nominal.rotation.T
        angle = np.arccos(np.clip((np.trace(delta) - 1) / 2, -1, 1))
        axis = np.array([delta[2, 1] - delta[1, 2], delta[0, 2] - delta[2, 0], delta[1, 0] - delta[0, 1]])
        rotvec = np.zeros(3) if angle < 1e-12 else axis * angle / (2 * np.sin(angle))
        samples.append(np.concatenate([rotvec, result.translation - nominal.translation]))
    empirical = np.cov(np.array(samples).T)
    # Each sample-covariance element has standard error about sqrt(S_ii S_jj / N).
    diagonal = np.diag(nominal.covariance)
    standard_error = np.sqrt(np.outer(diagonal, diagonal) / len(samples))
    assert (np.abs(empirical - nominal.covariance) <= 5 * standard_error).all()


def test_factory_extrinsics_place_right_and_rgb_cameras():
    links = factory_camera_links(oak_factory())
    right, rgb = links["oak_right_camera_optical_frame"], links["oak_rgb_camera_optical_frame"]
    assert np.allclose(right.translation, [0.075, 0, 0])
    assert np.allclose(rgb.translation, [0.0375, 0, 0])
    assert right.covariance is None


def test_check_with_session_uses_factory_cameras_and_matches_device(tmp_path):
    (tmp_path / "TESTDEVICE_calibration.json").write_text(json.dumps(oak_factory()))
    result = check(RigCalibration(filled()), tmp_path)
    assert result["complete"] is True
    assert result["session"]["factory_imu_extrinsics"] is False
    rgb = result["cameras"]["oak_rgb_camera_optical_frame"]
    # RGB sits 3.75 cm along camera x (right), which this mounting maps to body +x.
    assert np.allclose(rgb["body_to_camera"]["translation_m"], [0.1575, 0, 0.05])
    assert rgb["body_to_camera"]["rotation_sigma_rad"] is None
    other = filled()
    other["hardware"]["oak_device_id"] = "OTHER"
    with pytest.raises(ValueError, match="Calibration is for OAK OTHER"):
        check(RigCalibration(other), tmp_path)


@pytest.mark.parametrize("mutate, message", [
    (lambda d: d["transforms"][0].update(rotation_xyzw=[0, 0, 0, 1]), "exactly one of"),
    (lambda d: d["transforms"][0].update(translation_m=[100, 0, 0]), "translation_m"),
    (lambda d: d["transforms"][0].update(rotation_sigma_rad=[0.5, 0, 0]), "small-angle"),
    (lambda d: d["transforms"][1].update(rotation_rpy_deg=None, rotation_xyzw=[0, 0, 0, 2]), "unit quaternion"),
    (lambda d: d["transforms"][0].update(source="guess"), "source must be"),
    (lambda d: d["transforms"][0].update(child="oak_rgb_camera_optical_frame"), "Unsupported transform"),
    (lambda d: d["transforms"].append(copy.deepcopy(d["transforms"][0])), "Duplicate"),
    (lambda d: d["transforms"].pop(), "must list every link"),
    (lambda d: d["time_offsets"][0].update(offset_ns=0.5), "integer"),
    (lambda d: d["time_offsets"][0].update(source="factory"), "source must be"),
    (lambda d: d.update(schema_version=2), "schema_version"),
])
def test_invalid_calibrations_are_rejected(mutate, message):
    data = filled()
    mutate(data)
    with pytest.raises(ValueError, match=message):
        RigCalibration(data)


def test_unknown_sigma_keeps_calibration_incomplete():
    data = filled()
    data["transforms"][3]["translation_sigma_m"] = None
    result = check(RigCalibration(data))
    assert result["complete"] is False
    assert result["unknown_uncertainty"] == [f"{BODY}->{ANTENNA}"]
    assert result["blocking"] == [f"{BODY}->{ANTENNA} uncertainty"]
    assert result["cameras"][LEFT]["antenna_lever"]["lever_covariance_flu_m2"] is None
    assert (BODY, OAK_IMU) in RigCalibration(data).transforms


def test_cli_reports_and_requires_completeness(tmp_path, capsys):
    path = tmp_path / "rig.json"
    path.write_text(json.dumps(filled()))
    assert cli.main(["calibration-check", str(path), "--require-complete"]) == 0
    assert json.loads(capsys.readouterr().out)["complete"] is True
    assert cli.main(["calibration-check", str(TEMPLATE)]) == 0
    capsys.readouterr()
    assert cli.main(["calibration-check", str(TEMPLATE), "--require-complete"]) == 2


def hand_measured_only():
    data = filled()
    imu, camera, direct, _antenna = data["transforms"]
    for entry in (imu, camera):
        entry.update(source="unset")
    # Camera looking straight down, lens 12 cm ahead of the body origin.
    direct.update(source="manual_measurement", rotation_rpy_deg=[180, 0, -90],
                  translation_m=[0.12, 0.0, -0.05], rotation_sigma_rad=[0.05, 0.05, 0.05],
                  translation_sigma_m=[0.01, 0.01, 0.01])
    return data


def test_hand_measured_camera_link_alone_is_complete():
    result = check(RigCalibration(hand_measured_only()))
    assert result["complete"] is True
    assert result["camera_path"] == "direct"
    transform = result["cameras"][LEFT]["body_to_camera"]
    # Optical axis (camera z) points down in body FLU.
    assert np.allclose(np.array(transform["rotation_matrix"]) @ [0, 0, 1], [0, 0, -1])
    assert result["direct_vs_imu_chain"] is None


def test_imu_chain_takes_precedence_and_is_cross_checked():
    data = filled()
    data["transforms"][2].update(source="manual_measurement", rotation_rpy_deg=None,
                                 rotation_xyzw=xyzw_from_rotation(
                                     RigCalibration(filled()).imu_chain().rotation),
                                 translation_m=[0.13, 0.0, 0.05], rotation_sigma_rad=[0.05] * 3,
                                 translation_sigma_m=[0.01] * 3)
    result = check(RigCalibration(data))
    assert result["camera_path"] == "imu_chain"
    assert np.allclose(result["cameras"][LEFT]["body_to_camera"]["translation_m"], [0.12, 0, 0.05])
    agreement = result["direct_vs_imu_chain"]
    assert agreement["rotation_difference_rad"] < 1e-6
    assert agreement["translation_difference_m"] == pytest.approx(0.01)
    assert agreement["consistent_3_sigma"] is True
    data["transforms"][2]["translation_m"] = [0.40, 0.0, 0.05]
    assert check(RigCalibration(data))["direct_vs_imu_chain"]["consistent_3_sigma"] is False


def test_missing_time_offset_blocks_completeness():
    data = hand_measured_only()
    data["time_offsets"][0].update(source="unset")
    result = check(RigCalibration(data))
    assert result["complete"] is False
    assert result["blocking"] == ["time offset oak_ros_stamp->px4_ros_stamp"]
