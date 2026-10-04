import math

import cv2
import numpy as np
import pytest

from wallering_mapping import calibration_camera as cc
from wallering_mapping.rig_calibration import RigCalibration, rotation_from_xyzw

from test_rig_calibration import TEMPLATE

K = np.array([[855.0, 0, 644.0], [0, 855.5, 406.0], [0, 0, 1]])
DIST = np.array([-0.05, 0.01, 0.0005, -0.0003, 0.0, 0.02, 0.0, 0.0])
SIZE = (1280, 800)
# IMU x forward, camera optical z forward, tilted by a few degrees.
R_IMU_LEFT = rotation_from_xyzw([-0.5, 0.5, -0.5, 0.5]) @ cc.exp_so3(np.radians([2.0, -3.0, 1.5]))
# p_right = R p_left (stereo rectification-sized rotation), right lens 7.5 cm to image-right.
R_LEFT_TO_RIGHT = cc.exp_so3(np.radians([0.3, -0.4, 1.2]))
RIGHT_IN_LEFT = np.array([0.075, 0.001, 0.0005])
OFFSET_S = 0.0073
BIAS = np.array([0.012, -0.018, 0.009])
STRIDE = 3


def orientation(t, motion):
    """IMU orientation in the world: smooth multi-axis rotation, still at both ends."""
    t = np.asarray(t, float)
    envelope = np.clip(np.minimum(t - 1.5, 26.5 - t) / 2, 0, 1) ** 2
    amplitude = {"full": [0.45, 0.4, 0.35], "single": [0, 0, 0.6], "none": [0, 0, 0]}[motion]
    angles = np.stack([a * np.sin(2 * np.pi * f * t + p) + 0.15 * a * np.sin(2 * np.pi * 2.3 * f * t)
                       for a, f, p in zip(amplitude, [0.41, 0.57, 0.73], [0.3, 1.1, 2.0])], -1)
    return cc.exp_so3(angles * envelope[:, None])


def position(t, translate=True):
    t = np.asarray(t, float)
    if not translate:
        return np.zeros((len(t), 3))
    return 0.15 * np.stack([np.sin(2 * np.pi * 0.3 * t), np.sin(2 * np.pi * 0.47 * t + 1),
                            0.5 * np.sin(2 * np.pi * 0.61 * t + 2)], -1)


def synthetic(motion="full", translate=True, seconds=28.0, fps=20.0, outliers=0.1, seed=1):
    rng = np.random.default_rng(seed)
    # Gyro at 100 Hz with jitter, bias and white noise; true rate by central differences.
    imu_t = np.arange(0, seconds, 0.01) + rng.normal(0, 2e-4, int(round(seconds * 100)))
    imu_t.sort()
    h = 1e-5
    rate = cc.log_so3(np.swapaxes(orientation(imu_t - h, motion), -1, -2) @ orientation(imu_t + h, motion)) / (2 * h)
    gyro = rate + BIAS + rng.normal(0, 0.004, rate.shape)
    # Scene: points in a shell around the rig, so the camera always sees some.
    direction = rng.normal(size=(6000, 3))
    points = direction / np.linalg.norm(direction, axis=1)[:, None] * rng.uniform(3, 12, (6000, 1))
    exposure = np.arange(0.2, seconds - 0.2, 1 / fps)
    stamps = np.round((exposure - OFFSET_S) * 1e9).astype(np.int64)
    world_imu = orientation(exposure, motion)
    centre = position(exposure, translate)
    tracks = {}
    for camera, to_left, offset in [("left", np.eye(3), np.zeros(3)),
                                    ("right", R_LEFT_TO_RIGHT, RIGHT_IN_LEFT)]:
        # p_left = R_left_right p_right with R_left_right = R_LEFT_TO_RIGHT^T.
        world_camera = world_imu @ R_IMU_LEFT @ to_left.T
        origin = centre + np.einsum("nij,j->ni", world_imu @ R_IMU_LEFT, offset + [0.01, -0.02, 0.005])
        pixels, visible = [], []
        for rotation, c in zip(world_camera, origin):
            local = (points - c) @ rotation
            projected, _ = cv2.projectPoints(local, np.zeros(3), np.zeros(3), K, DIST)
            projected = projected.reshape(-1, 2)
            visible.append((local[:, 2] > 1) & (projected >= 0).all(1) & (projected < SIZE).all(1))
            pixels.append(projected)
        pairs = []
        for i in range(len(exposure) - STRIDE):
            j = i + STRIDE
            both = np.flatnonzero(visible[i] & visible[j])[:200]
            a = pixels[i][both] + rng.normal(0, 0.4, (len(both), 2))
            b = pixels[j][both] + rng.normal(0, 0.4, (len(both), 2))
            bad = rng.random(len(both)) < outliers
            b[bad] = rng.uniform([0, 0], SIZE, (bad.sum(), 2))
            if len(both) >= 40:
                pairs.append((i, j, a.astype(np.float32), b.astype(np.float32)))
        # A few grossly wrong pairs (tracker failure) on top of point outliers.
        for n in rng.choice(len(pairs), 5, replace=False):
            i, j, a, _ = pairs[n]
            pairs[n] = (i, j, a, a + rng.normal(0, 30, (1, 2)).astype(np.float32))
        tracks[camera] = pairs
    imu_ns = np.round(imu_t * 1e9).astype(np.int64) + 1_700_000_000_000_000_000
    return stamps + 1_700_000_000_000_000_000, tracks, imu_ns, gyro


def angle_between(a, b):
    return math.degrees(np.linalg.norm(cc.log_so3(a.T @ b)))


@pytest.fixture(scope="module")
def solved():
    stamps, tracks, imu_ns, gyro = synthetic()
    return {camera: cc.solve_tracks(stamps, tracks[camera], K, DIST, imu_ns, gyro, bootstrap=100)
            for camera in ("left", "right")}


def test_recovers_rotation_and_offset_with_noise_bias_and_outliers(solved):
    left = solved["left"]
    assert angle_between(left["rotation"], R_IMU_LEFT) < 0.3
    assert abs(left["offset_s"] - OFFSET_S) < 1e-3
    diagnostics = left["diagnostics"]
    assert np.allclose(diagnostics["gyro_bias_rad_s"], BIAS, atol=2e-3)
    assert diagnostics["pairs_used"] > 400 and diagnostics["pairs_rejected_residual"] >= 3
    assert diagnostics["excitation"]["weakest_rate_rad_s"] > 0.15
    # The reported sigma is honest: the true error lies within 3 sigma.
    error = cc.log_so3(R_IMU_LEFT @ left["rotation"].T)
    assert (np.abs(error) < 3 * left["rotation_sigma_rad"]).all()
    assert abs(left["offset_s"] - OFFSET_S) * 1e9 < 3 * left["offset_sigma_ns"]


def test_right_camera_cross_check_agrees(solved):
    right = solved["right"]
    assert angle_between(right["rotation"], R_IMU_LEFT @ R_LEFT_TO_RIGHT.T) < 0.3
    check = cc.cross_check(solved["left"], right, R_LEFT_TO_RIGHT)
    assert check["consistent_3_sigma"] and check["rotation_difference_rad"] < math.radians(0.3)
    assert abs(check["offset_difference_ns"]) < 1_000_000
    wrong = cc.cross_check(solved["left"], right, np.eye(3) @ cc.exp_so3([0, 0.05, 0]))
    assert not wrong["consistent_3_sigma"]


def test_entries_fit_the_rig_calibration_file(solved):
    import json
    left = solved["left"]
    entries = cc.calibration_entries(left["rotation"], left["rotation_sigma_rad"],
                                     round(left["offset_s"] * 1e9), left["offset_sigma_ns"],
                                     "synthetic", "2026-10-04")
    data = json.loads(TEMPLATE.read_text())
    data["transforms"][1] = entries[0]
    data["time_offsets"][1] = entries[1]
    rig = RigCalibration(data)
    transform = rig.transforms[("oak_imu_frame", "oak_left_camera_optical_frame")]
    assert angle_between(transform.rotation, R_IMU_LEFT) < 0.3
    assert transform.covariance is not None and transform.sources == ("estimated",)
    assert rig.offsets[("oak_camera_exposure", "oak_imu")]["offset_ns"] == entries[1]["offset_ns"]


@pytest.mark.parametrize("motion,translate", [("single", True), ("none", True), ("none", False)])
def test_refuses_unexcited_motion(motion, translate):
    stamps, tracks, imu_ns, gyro = synthetic(motion, translate, seconds=8.0)
    with pytest.raises(ValueError, match="Insufficient rotation"):
        cc.solve_tracks(stamps, tracks["left"], K, DIST, imu_ns, gyro)


def test_refuses_too_few_frames():
    stamps, tracks, imu_ns, gyro = synthetic(seconds=8.0)
    pairs = tracks["left"][::10]
    with pytest.raises(ValueError, match="Too few tracked frame pairs"):
        cc.solve_tracks(stamps, pairs, K, DIST, imu_ns, gyro)


def test_tracks_rendered_images_and_refuses_textureless():
    rng = np.random.default_rng(3)
    k = np.array([[300.0, 0, 160], [0, 300, 120], [0, 0, 1]])
    texture = cv2.GaussianBlur(rng.integers(0, 255, (1200, 1200)).astype(np.uint8), (0, 0), 2)
    centre = np.array([[1, 0, -440], [0, 1, -480], [0, 0, 1.0]])
    rotations = [cc.exp_so3([0.01 * i, -0.008 * i, 0.012 * i]) for i in range(6)]
    # Pure rotation renders as the homography K R^T K^-1 of a plane at infinity.
    frames = [(i * 50_000_000, cv2.warpPerspective(texture, k @ r.T @ np.linalg.inv(k) @ centre, (320, 240)))
              for i, r in enumerate(rotations)]
    stamps, pairs, counts = cc.track_pairs(frames, stride=2)
    assert counts["tracked_pairs"] == 4 and [p[:2] for p in pairs] == [(0, 2), (1, 3), (2, 4), (3, 5)]
    measured, models = cc.camera_rotations(pairs, k, np.zeros(5))
    for (i, j, _, _), vector in zip(pairs, measured):
        assert angle_between(cc.exp_so3(vector), rotations[i].T @ rotations[j]) < 0.1
    assert models["rotation_only"] == 4
    blank = [(i * 50_000_000, np.full((240, 320), 128, np.uint8)) for i in range(6)]
    with pytest.raises(ValueError, match="Textureless"):
        cc.track_pairs(blank)
