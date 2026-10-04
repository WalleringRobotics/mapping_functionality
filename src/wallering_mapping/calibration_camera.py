"""Targetless camera-to-OAK-IMU calibration: rotation and exposure-to-IMU time offset.

Solves ``oak_imu_frame -> oak_left_camera_optical_frame`` (``p_imu = R p_camera``) and
the ``oak_camera_exposure -> oak_imu`` offset (add it to camera stamps to express them
on the IMU clock) from hand-held motion in front of any textured scene:

1. track corners between consecutive global-shutter mono frames (pyramidal LK);
2. inter-frame camera rotation from the essential matrix (RANSAC on undistorted
   normalised points), or from a rotation-only fit when parallax is negligible;
3. integrate bias-corrected gyro over the same intervals shifted by a candidate offset;
4. rotation-only hand-eye: ``r_imu = R r_camera`` on rotation vectors (Kabsch), scanning
   the offset for the minimum residual with sub-sample refinement;
5. moving-block bootstrap over frame pairs for 1-sigma rotation and offset.

Translation is not observable from rotations. The core takes arrays (stamps, gyro,
tracked points) so it can be tested synthetically; the session reader is a thin layer.
"""

import datetime
import json
import math
from pathlib import Path

import cv2
import numpy as np

from . import bags
from .dataset import sha256_file
from .rig_calibration import CAMERA_SOCKETS, LEFT, OAK_IMU, factory_camera_links, xyzw_from_rotation

SOCKETS = {"left": 1, "right": 2}
# The IMU sits inside the OAK housing, within a few centimetres of either lens.
TRANSLATION_SIGMA_M = 0.03
# Floors for effects the bootstrap cannot see (intrinsics error, IMU sampling/filtering).
ROTATION_SIGMA_FLOOR_RAD = 5e-4
OFFSET_SIGMA_FLOOR_NS = 100_000
SOLVER = "camera-imu-hand-eye-v2"
LIMITATIONS = ["Relative exposure-to-IMU offset only; absolute exposure timing needs an optical event (#14).",
               "Global-shutter mono cameras only; RGB rolling-shutter readout is not modelled.",
               "Factory intrinsics are trusted, not re-estimated.",
               "Translation is not observable from rotation; a housing bound is reported instead."]


def exp_so3(v):
    v = np.asarray(v, float)
    angle = np.linalg.norm(v, axis=-1)[..., None, None]
    k = np.zeros(v.shape[:-1] + (3, 3))
    k[..., 0, 1], k[..., 0, 2], k[..., 1, 2] = -v[..., 2], v[..., 1], -v[..., 0]
    k = k - np.swapaxes(k, -1, -2)
    small = angle < 1e-8
    safe = np.where(small, 1.0, angle)
    a = np.where(small, 1.0, np.sin(safe) / safe)
    b = np.where(small, 0.5, (1 - np.cos(safe)) / safe ** 2)
    return np.eye(3) + a * k + b * k @ k


def log_so3(rotation):
    m = np.asarray(rotation, float)
    cos = np.clip((np.trace(m, axis1=-2, axis2=-1) - 1) / 2, -1.0, 1.0)
    angle = np.arccos(cos)
    vee = np.stack([m[..., 2, 1] - m[..., 1, 2], m[..., 0, 2] - m[..., 2, 0],
                    m[..., 1, 0] - m[..., 0, 1]], axis=-1)
    sin = np.sin(angle)
    scale = np.where(angle < 1e-6, 0.5 + angle ** 2 / 12, angle / (2 * np.where(sin == 0, 1, sin)))
    result = vee * scale[..., None]
    # The skew part vanishes at pi; Rodrigues recovers the axis from the diagonal.
    flat_m, flat_result = m.reshape(-1, 3, 3), result.reshape(-1, 3)
    for index in np.flatnonzero(np.abs(angle.reshape(-1) - np.pi) < 1e-5):
        flat_result[index] = cv2.Rodrigues(flat_m[index])[0].ravel()
    return result


def fit_rotation(source, target, weights=None):
    """Kabsch: rotation(s) R minimising sum w |target - R source|^2; batched over leading axes."""
    w = np.ones(source.shape[-2]) if weights is None else weights
    m = np.einsum("...n,...ni,...nj->...ij", w, target, source)
    return rotation_from_moment(m), m


def rotation_from_moment(m):
    u, _, vt = np.linalg.svd(m)
    d = np.sign(np.linalg.det(u @ vt))
    u = u.copy()
    u[..., :, 2] *= d[..., None]
    return u @ vt


class GyroPath:
    """Orientation of the IMU from integrated angular rate (constant rate per sample interval)."""

    def __init__(self, stamps_s, gyro):
        self.t = np.asarray(stamps_s, float)
        rates = np.asarray(gyro, float)
        self.rate = (rates[1:] + rates[:-1]) / 2
        steps = exp_so3(self.rate * np.diff(self.t)[:, None])
        self.cumulative = np.empty((len(self.t), 3, 3))
        self.cumulative[0] = np.eye(3)
        for i, step in enumerate(steps):
            self.cumulative[i + 1] = self.cumulative[i] @ step

    def orientation(self, t):
        i = np.clip(np.searchsorted(self.t, t, "right") - 1, 0, len(self.t) - 2)
        return self.cumulative[i] @ exp_so3(self.rate[i] * (t - self.t[i])[..., None])

    def relative(self, start, end):
        """Rotation vector of the IMU at ``end`` relative to ``start``, in the IMU frame."""
        return log_so3(np.swapaxes(self.orientation(start), -1, -2) @ self.orientation(end))


def still_bias(stamps_s, gyro, window_s=0.5, max_std=0.02, max_rate=0.05):
    """Gyro bias from still windows (low spread, low rate), or None without any."""
    edges = np.arange(stamps_s[0], stamps_s[-1], window_s)
    index = np.searchsorted(stamps_s, edges)
    means = []
    for start, end in zip(index[:-1], index[1:]):
        chunk = gyro[start:end]
        if len(chunk) >= 10 and chunk.std(axis=0).max() < max_std and np.linalg.norm(chunk.mean(0)) < max_rate:
            means.append(chunk.mean(axis=0))
    return (np.median(means, axis=0), len(means) * window_s) if means else (None, 0.0)


def excitation(rates):
    """Per-axis and weakest-direction RMS of angular rate samples (rad/s)."""
    # A constant rate can be absorbed by bias/mounting and is not an independent
    # excitation direction. In particular, a circular gyro trace plus a constant
    # third component makes a clock phase shift indistinguishable from mounting.
    centred = rates - rates.mean(axis=0)
    moment = centred.T @ centred / len(rates)
    return {"rms_rate_rad_s": np.sqrt(np.diag(moment)).tolist(),
            "weakest_rate_rad_s": float(math.sqrt(max(np.linalg.eigvalsh(moment)[0], 0.0)))}


def require_excitation(rates, min_rate):
    if len(rates) < 10:
        raise ValueError("Camera and IMU recordings do not overlap")
    result = excitation(rates)
    if result["weakest_rate_rad_s"] < min_rate:
        raise ValueError(
            f"Insufficient rotation: weakest-axis RMS rate {result['weakest_rate_rad_s']:.3f} rad/s "
            f"< {min_rate} rad/s (per IMU axis {np.round(result['rms_rate_rad_s'], 3).tolist()}); "
            "rotate the camera about all three axes")
    return result


def undistort(points, k, dist):
    criteria = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 40, 1e-10)
    return cv2.undistortPointsIter(np.asarray(points, np.float64).reshape(-1, 1, 2), k, dist,
                                   None, None, criteria).reshape(-1, 2)


def lucas_kanade(previous, image, points, fb_threshold):
    """Forward-backward checked pyramidal LK; returns moved points and a keep mask."""
    start = points.reshape(-1, 1, 2).astype(np.float32)
    moved, status, _ = cv2.calcOpticalFlowPyrLK(previous, image, start, None, winSize=(21, 21), maxLevel=4)
    back, status_back, _ = cv2.calcOpticalFlowPyrLK(image, previous, moved, None, winSize=(21, 21), maxLevel=4)
    keep = (status.ravel() == 1) & (status_back.ravel() == 1)
    keep &= np.linalg.norm((back - start).reshape(-1, 2), axis=1) < fb_threshold
    return moved.reshape(-1, 2), keep


def track_pairs(images, *, stride=3, max_features=300, min_features=40, fb_threshold=0.5):
    """Track corners from every frame of ``(stamp_ns, image)`` over ``stride`` frames.

    Returns frame stamps, pairs ``(i, j, points_i, points_j)`` in pixels with j = i + stride,
    and counts. A stride of a few frames gives more rotation and parallax per pair than
    consecutive frames. Holds only the previous frame, so a bag can be streamed.
    """
    stamps, pairs, textureless, previous, active, lost = [], [], 0, None, [], 0
    for index, (stamp, image) in enumerate(images):
        if stamps and stamp <= stamps[-1]:
            raise ValueError("Camera stamps must increase")
        if active:
            moved, keep = lucas_kanade(previous, image, np.concatenate([a[2] for a in active]), fb_threshold)
            survivors, offset = [], 0
            for start_index, start, current in active:
                part = slice(offset, offset + len(current))
                offset += len(current)
                start, current = start[keep[part]], moved[part][keep[part]]
                if len(current) < min_features:
                    lost += 1
                elif index - start_index == stride:
                    pairs.append((start_index, index, start, current))
                else:
                    survivors.append((start_index, start, current))
            active = survivors
        corners = cv2.goodFeaturesToTrack(image, max_features, 0.01, 12)
        if corners is None or len(corners) < min_features:
            textureless += 1
        else:
            active.append((index, corners.reshape(-1, 2), corners.reshape(-1, 2)))
        stamps.append(stamp)
        previous = image
    if stamps and textureless > len(stamps) / 2:
        raise ValueError(f"Textureless scene: {textureless}/{len(stamps)} frames have fewer than "
                         f"{min_features} corners; record in front of a textured, lit scene")
    return np.asarray(stamps, np.int64), pairs, {"frames": len(stamps), "textureless_frames": textureless,
                                                 "tracks_lost": lost, "tracked_pairs": len(pairs)}


def camera_rotations(pairs, k, dist, *, pixel_threshold=1.0, min_inliers=30):
    """Rotation vector of frame j relative to frame i (in i's camera frame) per pair, NaN if rejected."""
    k = np.asarray(k, float)
    dist = np.asarray(dist, float)
    threshold = pixel_threshold / k[0, 0]
    result = np.full((len(pairs), 3), np.nan)
    counts = {"few_inliers": 0, "rotation_only": 0, "essential": 0}
    for index, (_, _, points_a, points_b) in enumerate(pairs):
        if len(points_a) < min_inliers:
            counts["few_inliers"] += 1
            continue
        a, b = undistort(points_a, k, dist), undistort(points_b, k, dist)
        # Plain RANSAC keeps a noisy minimal-sample model; USAC with local optimisation,
        # re-run on its own inliers, gets ~0.03 deg at 0.4 px noise and 10 % outliers.
        e, mask = cv2.findEssentialMat(a, b, np.eye(3), cv2.USAC_ACCURATE, 0.999, threshold)
        if e is None or e.shape[0] < 3 or mask is None or mask.sum() < min_inliers:
            counts["few_inliers"] += 1
            continue
        inliers = mask.ravel() > 0
        refined, _ = cv2.findEssentialMat(a[inliers], b[inliers], np.eye(3), cv2.USAC_ACCURATE, 0.999, threshold)
        if refined is not None and refined.shape[0] >= 3:
            e = refined
        ray_a = np.column_stack([a, np.ones(len(a))])
        ray_b = np.column_stack([b, np.ones(len(b))])
        ray_a /= np.linalg.norm(ray_a, axis=1)[:, None]
        ray_b /= np.linalg.norm(ray_b, axis=1)[:, None]
        # Rotation-only fit x_j = R x_i: exact for pure rotation (where E degenerates), and
        # close enough otherwise to pick the right one of E's twisted-pair rotations.
        keep = inliers
        for _ in range(2):
            r_only, _ = fit_rotation(ray_a[keep], ray_b[keep])
            error = np.arccos(np.clip(np.sum(ray_b * (ray_a @ r_only.T), axis=1), -1, 1))
            keep = inliers & (error < 3 * threshold)
            if keep.sum() < min_inliers:
                keep = inliers
                break
        if keep.sum() >= 0.8 * inliers.sum() and np.median(error[keep]) < threshold:
            rotation = r_only
            counts["rotation_only"] += 1
        else:
            first, second, _ = cv2.decomposeEssentialMat(e[:3])
            rotation = min([first, second], key=lambda r: np.linalg.norm(log_so3(r @ r_only.T)))
            counts["essential"] += 1
        # x_j = R x_i + t: frame j's orientation in frame i is R^T.
        result[index] = log_so3(rotation.T)
    return result, counts


def parabola_minimum(grid, cost):
    i = int(np.clip(np.argmin(cost), 1, len(grid) - 2))
    left, middle, right = cost[i - 1], cost[i], cost[i + 1]
    curvature = left - 2 * middle + right
    shift = 0.5 * (left - right) / curvature if curvature > 0 else 0.0
    return grid[i] + float(np.clip(shift, -1, 1)) * (grid[1] - grid[0])


def scan(path, start, end, camera, offsets, weights):
    imu = path.relative(start[None] + offsets[:, None], end[None] + offsets[:, None])
    rotation, _ = fit_rotation(camera[None].repeat(len(offsets), 0), imu, weights[None].repeat(len(offsets), 0))
    residual = np.linalg.norm(imu - camera @ np.swapaxes(rotation, -1, -2), axis=-1)
    return imu, rotation, residual


def solve_rotation_offset(start_s, end_s, camera, imu_s, gyro, *, max_offset_s=0.05,
                          min_pairs=100, min_rate=0.15, bootstrap=200, block=20, seed=0,
                          max_gap_s=0.05, max_residual_rad=0.02):
    """Hand-eye rotation IMU->camera and camera->IMU clock offset from frame-pair rotations.

    ``start_s``/``end_s`` are camera stamps of each pair, ``camera`` the camera rotation
    vectors (NaN = rejected), ``imu_s``/``gyro`` raw IMU stamps and rates, all on one
    time origin in seconds.
    """
    if not all(math.isfinite(v) and v > 0 for v in
               (max_offset_s, min_rate, max_gap_s, max_residual_rad)) or max_offset_s < 0.002:
        raise ValueError("Solver limits must be positive and finite; max_offset_s must be at least 0.002")
    if min_pairs < 10 or bootstrap < 2 or block < 1:
        raise ValueError("Require at least 10 pairs, two bootstrap samples and a positive block size")
    start_s, end_s, camera = np.asarray(start_s, float), np.asarray(end_s, float), np.asarray(camera, float)
    if start_s.ndim != 1 or end_s.shape != start_s.shape or camera.shape != (len(start_s), 3):
        raise ValueError("Frame pair arrays must have matching shapes")
    if not np.isfinite(start_s).all() or not np.isfinite(end_s).all() or np.any(end_s <= start_s):
        raise ValueError("Frame intervals must be finite and increasing")
    imu_s, gyro = np.asarray(imu_s, float), np.asarray(gyro, float)
    if gyro.shape != (len(imu_s), 3) or not np.isfinite(gyro).all() or not np.isfinite(imu_s).all():
        raise ValueError("IMU stamps and matching Nx3 gyro rates must be finite")
    if len(imu_s) < 10 or np.any(np.diff(imu_s) <= 0):
        raise ValueError("IMU stamps must be increasing with at least 10 samples")
    initial_bias, still_s = still_bias(imu_s, gyro)
    bias = np.zeros(3) if initial_bias is None else initial_bias
    inside = (start_s - max_offset_s >= imu_s[0]) & (end_s + max_offset_s <= imu_s[-1])
    if not inside.any():
        raise ValueError("Camera and IMU recordings do not overlap")
    span = (imu_s >= start_s[inside].min()) & (imu_s <= end_s[inside].max())
    if span.sum() < 10:
        raise ValueError("Camera and IMU recordings do not overlap")
    require_excitation(gyro[span] - bias, min_rate)
    # Reject any pair whose integration interval could cross a sample gap, including
    # the whole offset scan. Never invent rotations through missing IMU samples.
    gap_prefix = np.r_[0, np.cumsum(np.diff(imu_s) > max_gap_s)]
    first = np.clip(np.searchsorted(imu_s, start_s - max_offset_s, side="right") - 1, 0, len(imu_s) - 1)
    last = np.clip(np.searchsorted(imu_s, end_s + max_offset_s), 0, len(imu_s) - 1)
    gap_free = gap_prefix[last] == gap_prefix[first]
    valid = inside & gap_free & np.isfinite(camera).all(axis=1)
    if valid.sum() < min_pairs:
        raise ValueError(f"Too few tracked frame pairs: {int(valid.sum())} < {min_pairs}; "
                         "record longer, at a higher frame rate, or with more texture")
    start, end, cam = start_s[valid], end_s[valid], camera[valid]
    step = 1e-3
    coarse = np.arange(-max_offset_s, max_offset_s + step / 2, step)
    weights = np.ones(len(cam))
    # Scan with a robust (median) cost to find outlier pairs, re-estimate the bias jointly
    # (r_imu ~ R r_camera + bias error * dt for small rotations), then scan again.
    for _ in range(2):
        path = GyroPath(imu_s, gyro - bias)
        imu, rotation, residual = scan(path, start, end, cam, coarse, weights)
        best = int(np.argmin(np.median(residual, axis=1) if weights.all() else
                             (weights * residual ** 2).sum(1)))
        scale = 1.4826 * np.median(np.abs(residual[best] - np.median(residual[best])))
        weights = (residual[best] < max(np.median(residual[best]) + 5 * scale, 1e-4)).astype(float)
        error = imu[best] - cam @ rotation[best].T
        dt = (end - start)[:, None]
        for _ in range(2):
            bias = bias + (weights[:, None] * dt * error).sum(0) / (weights * dt[:, 0] ** 2).sum()
            path = GyroPath(imu_s, gyro - bias)
            imu = path.relative(start + coarse[best], end + coarse[best])
            rotation_best, _ = fit_rotation(cam, imu, weights)
            error = imu - cam @ rotation_best.T
    _, _, residual = scan(path, start, end, cam, coarse, weights)
    cost = (weights * residual ** 2).sum(1)
    best = int(np.argmin(cost))
    if abs(coarse[best]) >= max_offset_s - step / 2:
        raise ValueError(f"Time offset at the scan limit ({coarse[best] * 1e3:.0f} ms); "
                         "increase max_offset_ms or check the clocks")
    fine = np.arange(max(-max_offset_s, coarse[best] - 3e-3),
                     min(max_offset_s, coarse[best] + 3e-3) + 5e-5, 1e-4)
    fine_imu, _, fine_residual = scan(path, start, end, cam, fine, weights)
    fine_cost = (weights * fine_residual ** 2).sum(1)
    if np.argmin(fine_cost) in (0, len(fine) - 1):
        raise ValueError("Time offset at the scan limit; increase max_offset_ms or check the clocks")
    offset = parabola_minimum(fine, fine_cost)
    final_imu = path.relative(start + offset, end + offset)
    rotation, _ = fit_rotation(cam, final_imu, weights)
    final_residual = np.linalg.norm(final_imu - cam @ rotation.T, axis=1)
    used = weights > 0
    if used.sum() < min_pairs:
        raise ValueError(f"Too few tracked frame pairs after residual rejection: {int(used.sum())} < {min_pairs}")
    require_excitation(final_imu[used] / (end - start)[used, None], min_rate)
    require_excitation(cam[used] / (end - start)[used, None], min_rate)
    residual_rms = float(np.sqrt(np.mean(final_residual[used] ** 2)))
    if residual_rms > max_residual_rad:
        raise ValueError(f"Camera/gyro residual {residual_rms:.4f} rad exceeds {max_residual_rad} rad")

    # Moving-block bootstrap over the used pairs (in time order), re-solving on the fine grid.
    rng = np.random.default_rng(seed)
    pairs_used = np.flatnonzero(used)
    count = len(pairs_used)
    length = min(block, count)
    draws = rng.integers(0, count - length + 1, (bootstrap, math.ceil(count / length)))
    resample = np.zeros((bootstrap, len(cam)))
    for b, starts in enumerate(draws):
        picks = (starts[:, None] + np.arange(length)).ravel()[:count]
        np.add.at(resample[b], pairs_used[picks], 1.0)
    norms = (fine_imu ** 2).sum(-1) + (cam ** 2).sum(-1)[None]
    outer = np.einsum("gni,nj->gnij", fine_imu, cam).reshape(len(fine), len(cam), 9)
    moments = np.swapaxes(resample[None] @ outer, 0, 1).reshape(bootstrap, len(fine), 3, 3)
    rotations = rotation_from_moment(moments)
    costs = resample @ norms.T - 2 * np.einsum("bgij,bgij->bg", rotations, moments)
    offsets = np.array([parabola_minimum(fine, c) for c in costs])
    at_edge = float(np.mean((np.argmin(costs, 1) == 0) | (np.argmin(costs, 1) == len(fine) - 1)))
    nearest = rotations[np.arange(bootstrap), np.argmin(costs, 1)]
    deviation = log_so3(nearest @ rotation.T)
    rotation_sigma = np.maximum(deviation.std(axis=0), ROTATION_SIGMA_FLOOR_RAD)
    offset_sigma_ns = max(int(math.ceil(offsets.std() * 1e9)), OFFSET_SIGMA_FLOOR_NS)
    if rotation_sigma.max() > 0.05:
        raise ValueError(f"Rotation unconstrained: bootstrap sigma {np.round(rotation_sigma, 3).tolist()} rad")
    warnings = []
    if at_edge > 0.05:
        offset_sigma_ns = max(offset_sigma_ns, 3_000_000)
        warnings.append("Time offset weakly constrained: vary the rotation speed more sharply")
    rates = final_imu[used] / (end - start)[used, None]
    return {"rotation": rotation, "offset_s": offset, "rotation_sigma_rad": rotation_sigma,
            "offset_sigma_ns": offset_sigma_ns,
            "diagnostics": {
                "pairs_valid": int(valid.sum()), "pairs_used": int(used.sum()),
                "pairs_rejected_residual": int((~used).sum()),
                "pairs_outside_imu": int((~inside).sum()),
                "pairs_crossing_imu_gaps": int((inside & ~gap_free).sum()),
                "gyro_bias_rad_s": bias.tolist(),
                "gyro_bias_still_rad_s": None if initial_bias is None else initial_bias.tolist(),
                "still_duration_s": still_s,
                "excitation": excitation(rates),
                "residual_rms_rad": residual_rms,
                "residual_rms_at_zero_offset_rad": float(np.sqrt(
                    (weights * np.linalg.norm(path.relative(start, end) - cam @ rotation.T, axis=1) ** 2).sum()
                    / weights.sum())),
                "bootstrap": {"samples": bootstrap, "block_pairs": length,
                              "rotation_sigma_rad": deviation.std(axis=0).tolist(),
                              "offset_sigma_ns": int(math.ceil(offsets.std() * 1e9)),
                              "offset_at_grid_edge_fraction": at_edge},
                "warnings": warnings}}


def solve_tracks(frame_stamps_ns, pairs, k, dist, imu_stamps_ns, gyro, *, max_offset_ms=50.0,
                 pixel_threshold=1.0, **options):
    """Array-level solver: frame stamps, tracked pixel pairs ``(i, j, points_i, points_j)``
    as from ``track_pairs``, factory intrinsics, and raw gyro stamps/rates (rad/s)."""
    frame_stamps_ns = np.asarray(frame_stamps_ns, np.int64)
    imu_stamps_ns = np.asarray(imu_stamps_ns, np.int64)
    if len(frame_stamps_ns) < 2 or np.any(np.diff(frame_stamps_ns) <= 0):
        raise ValueError("Camera stamps must be increasing with at least two frames")
    index = np.array([(i, j) for i, j, _, _ in pairs], np.int64).reshape(-1, 2)
    if len(index) and (index.min() < 0 or index.max() >= len(frame_stamps_ns) or (index[:, 1] <= index[:, 0]).any()):
        raise ValueError("Pairs must reference frames i < j")
    if len(imu_stamps_ns) < 10 or np.any(np.diff(imu_stamps_ns) <= 0):
        raise ValueError("IMU stamps must be increasing with at least 10 samples")
    if np.asarray(gyro).shape != (len(imu_stamps_ns), 3) or not np.isfinite(gyro).all():
        raise ValueError("IMU gyro rates must be finite matching Nx3 samples")
    origin = int(imu_stamps_ns[0])
    frames = (frame_stamps_ns - origin) / 1e9
    imu = (imu_stamps_ns - origin) / 1e9
    gyro = np.asarray(gyro, float)
    # Refuse unexcited input from the gyro alone, before the costly image geometry.
    bias, _ = still_bias(imu, gyro)
    span = (imu >= frames[0]) & (imu <= frames[-1])
    if span.sum() < 10:
        raise ValueError("Camera and IMU recordings do not overlap")
    require_excitation(gyro[span] - (0 if bias is None else bias), options.get("min_rate", 0.15))
    camera, counts = camera_rotations(pairs, k, dist, pixel_threshold=pixel_threshold)
    result = solve_rotation_offset(frames[index[:, 0]], frames[index[:, 1]], camera, imu, gyro,
                                   max_offset_s=max_offset_ms / 1e3, **options)
    result["diagnostics"]["frames"] = len(frame_stamps_ns)
    result["diagnostics"]["pairs_tracked"] = len(pairs)
    result["diagnostics"]["camera_rotation_models"] = counts
    return result


def cross_check(primary, secondary, secondary_to_primary):
    """Compare IMU->camera results of two cameras, mapping ``secondary`` through the rotation
    ``secondary_to_primary`` (p_secondary = R p_primary)."""
    mapped = secondary["rotation"] @ secondary_to_primary
    angle = float(np.linalg.norm(log_so3(primary["rotation"].T @ mapped)))
    rotation_sigma = float(np.linalg.norm(np.hypot(primary["rotation_sigma_rad"],
                                                   secondary["rotation_sigma_rad"])))
    offset_difference = int(round((secondary["offset_s"] - primary["offset_s"]) * 1e9))
    offset_sigma = math.hypot(primary["offset_sigma_ns"], secondary["offset_sigma_ns"])
    return {"rotation_difference_rad": angle, "rotation_sigma_rad": rotation_sigma,
            "offset_difference_ns": offset_difference, "offset_sigma_ns": int(math.ceil(offset_sigma)),
            "consistent_3_sigma": angle <= 3 * rotation_sigma and abs(offset_difference) <= 3 * offset_sigma}


def calibration_entries(rotation, rotation_sigma, offset_ns, sigma_ns, evidence, date=None):
    """Rig calibration file entries (template format) for an IMU->left-camera result."""
    date = date or datetime.date.today().isoformat()
    return [{"parent": OAK_IMU, "child": LEFT, "source": "estimated",
             "rotation_xyzw": xyzw_from_rotation(rotation), "translation_m": [0.0, 0.0, 0.0],
             "rotation_sigma_rad": [float(s) for s in rotation_sigma],
             "translation_sigma_m": [TRANSLATION_SIGMA_M] * 3, "evidence": evidence, "date": date},
            {"clock": "oak_camera_exposure", "reference": "oak_imu", "source": "estimated",
             "offset_ns": int(offset_ns), "sigma_ns": int(sigma_ns), "evidence": evidence, "date": date}]


def session_intrinsics(oak, socket, width, height):
    data = dict(oak["cameraData"])[socket]
    k = np.asarray(data["intrinsicMatrix"], float)
    if width != data["width"] or height != data["height"]:
        scale = width / data["width"]
        if abs(height / data["height"] - scale) > 1e-6:
            raise ValueError(f"Image {width}x{height} does not match calibration aspect")
        k[:2] *= scale
    return k, np.asarray(data["distortionCoeff"], float)


def read_gyro(bag):
    connections = [c for c in bag.connections if c.topic == bags.IMU]
    if not connections:
        raise ValueError(f"Session has no {bags.IMU}")
    stamps, rates, frames = [], [], set()
    for connection, _, raw in bag.messages(connections=connections):
        message = bag.deserialize(raw, connection.msgtype)
        stamps.append(bags.stamp(message))
        rates.append([message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z])
        frames.add(message.header.frame_id)
    if frames != {OAK_IMU}:
        raise ValueError(f"{bags.IMU} frames {sorted(frames)!r}, expected only {OAK_IMU!r}")
    stamps, rates = np.asarray(stamps, np.int64), np.asarray(rates, float)
    if len(stamps) < 10 or np.any(np.diff(stamps) <= 0):
        raise ValueError("IMU stamps must be increasing with at least 10 samples")
    if not np.isfinite(rates).all():
        raise ValueError("IMU gyro rates must be finite")
    return stamps, rates, sorted(frames)


def solve_session_camera(bag, oak, camera, imu_stamps, gyro, *, min_fps, max_offset_ms, stride, **options):
    topic = bags.CAMERAS[camera]
    connections = [c for c in bag.connections if c.topic == topic]
    if not connections:
        raise ValueError(f"Session has no {topic}")
    duration = (bag.end_time - bag.start_time) / 1e9
    fps = sum(c.msgcount for c in connections) / duration if duration > 0 else 0.0
    if fps < min_fps:
        raise ValueError(f"{topic} runs at {fps:.1f} fps < {min_fps}; use the calibration recording mode")
    geometry = {}

    def frames():
        for connection, _, raw in bag.messages(connections=connections):
            message = bag.deserialize(raw, connection.msgtype)
            image = bags.image_array(message)
            if image.ndim != 2:
                raise ValueError(f"{topic} must be mono8 for camera-IMU calibration")
            expected_frame = CAMERA_SOCKETS[SOCKETS[camera]]
            if message.header.frame_id != expected_frame:
                raise ValueError(f"{topic} must use {expected_frame}, got {message.header.frame_id!r}")
            if geometry.setdefault("shape", image.shape) != image.shape:
                raise ValueError(f"Image geometry changed on {topic}")
            yield bags.stamp(message), image

    stamps, pairs, tracking = track_pairs(frames(), stride=stride)
    if len(stamps) < 2:
        raise ValueError(f"Too few frames on {topic}")
    k, dist = session_intrinsics(oak, SOCKETS[camera], geometry["shape"][1], geometry["shape"][0])
    result = solve_tracks(stamps, pairs, k, dist, imu_stamps, gyro, max_offset_ms=max_offset_ms, **options)
    result["diagnostics"]["tracking"] = tracking
    result["diagnostics"]["fps"] = fps
    return result


def solve_camera_imu(session: Path, *, camera="left", max_offset_ms=50.0, cross_check_camera=True,
                     min_fps=10.0, min_rate=0.15, stride=3, bootstrap=200, date=None) -> dict:
    """Solve oak_imu_frame -> oak_left_camera_optical_frame and the exposure-to-IMU offset.

    ``camera`` selects the primary stream; a right-camera result is mapped to the left
    optical frame through the OAK factory extrinsics. The other mono camera is solved
    independently as a cross-check when ``cross_check_camera`` is set; its refusal or
    disagreement prevents calibrated entries. Disabling it returns diagnostics only. ``stride`` is the frame separation of each tracked pair.
    """
    session = Path(session)
    if camera not in SOCKETS:
        raise ValueError(f"camera must be one of {sorted(SOCKETS)}")
    # Only a cleanly finished, sealed recording is evidence for a calibration entry.
    state = session / "state"
    if not state.is_file() or state.read_text().strip() != "complete":
        raise ValueError("Recording did not finish cleanly (state is not complete)")
    sealed = bags.check_seal(session)
    seal_sha256 = sha256_file(session / "SHA256SUMS")
    files = sorted(session.glob("*_calibration.json"))
    if len(files) != 1:
        raise ValueError("Session must contain exactly one OAK *_calibration.json")
    if files[0].name not in sealed:
        raise ValueError("Recording seal omits the OAK factory calibration")
    oak = json.loads(files[0].read_text())
    # factory[...] is left -> camera: p_left = R p_camera, so p_camera = R^T p_left.
    factory = factory_camera_links(oak)
    to_left = {"left": np.eye(3), "right": factory[CAMERA_SOCKETS[2]].rotation.T}
    options = {"min_rate": min_rate, "bootstrap": bootstrap, "stride": stride,
               "min_fps": min_fps, "max_offset_ms": max_offset_ms}
    with bags.reader(session) as bag:
        imu_stamps, gyro, imu_frames = read_gyro(bag)
        # Refuse an unexcited session before decoding any image.
        imu = (imu_stamps - imu_stamps[0]) / 1e9
        bias, _ = still_bias(imu, gyro)
        require_excitation(gyro - (0 if bias is None else bias), min_rate)
        primary = solve_session_camera(bag, oak, camera, imu_stamps, gyro, **options)
        other = {"left": "right", "right": "left"}[camera]
        secondary, check = None, None
        if cross_check_camera:
            try:
                secondary = solve_session_camera(bag, oak, other, imu_stamps, gyro, **options)
            except ValueError as error:
                raise ValueError(f"{other} camera cross-check refused: {error}") from error
    rotation = primary["rotation"] @ to_left[camera]
    if secondary is not None:
        check = cross_check(primary, secondary, to_left[other] @ to_left[camera].T)
        check["camera"] = other
        if not check["consistent_3_sigma"]:
            raise ValueError(f"{other} camera cross-check disagrees beyond 3 sigma: "
                             f"{check['rotation_difference_rad']:.4f} rad, "
                             f"{check['offset_difference_ns'] / 1e6:.3f} ms")
    offset_ns = int(round(primary["offset_s"] * 1e9))
    diagnostics = primary["diagnostics"]
    evidence = (f"{SOLVER} on {session.name} (SHA256SUMS sha256 {seal_sha256}; "
                f"{camera} camera, {files[0].name}): "
                f"{diagnostics['pairs_used']} frame pairs, residual "
                f"{math.degrees(diagnostics['residual_rms_rad']):.3f} deg RMS, weakest-axis rate "
                f"{diagnostics['excitation']['weakest_rate_rad_s']:.2f} rad/s")
    if check is not None and "error" not in check:
        evidence += (f"; {other} camera agrees within {math.degrees(check['rotation_difference_rad']):.3f} deg"
                     f" and {check['offset_difference_ns'] / 1e6:.2f} ms"
                     f" ({'consistent' if check['consistent_3_sigma'] else 'INCONSISTENT'} at 3 sigma)")
    entries = calibration_entries(rotation, primary["rotation_sigma_rad"], offset_ns,
                                  primary["offset_sigma_ns"], evidence, date) if check is not None else []
    return {"schema_version": 1, "qualified": check is not None, "solver": SOLVER, "session": str(session), "seal_sha256": seal_sha256, "camera": camera,
            "oak_device_id": files[0].name.removesuffix("_calibration.json"),
            "imu_frame_ids": imu_frames, "entries": entries,
            "rotation_matrix": rotation.tolist(), "offset_ns": offset_ns,
            "diagnostics": diagnostics, "cross_check": check,
            "limitations": LIMITATIONS + [
                f"translation_sigma_m {TRANSLATION_SIGMA_M} m is a bound for an IMU inside the OAK housing."]}
