"""OAK IMU to PX4 body: clock offset and mounting rotation from gyro agreement.

Both IMUs are on one rigid body, so their angular rates differ only by the mounting
rotation, constant biases and the clock offset: ``w_px4(s) = R w_oak(s - offset) + c``.
For each candidate offset the OAK rates are interpolated onto PX4 stamps and ``R`` is
solved in closed form (Kabsch on bias-centred rates); the offset minimising the
residual wins. Gyros cannot observe translation, and different sensor filtering
appears as offset: the result is OAK-versus-PX4 *timestamps*, not exposure timing.
"""

import datetime
import math
from pathlib import Path

import numpy as np

from .rig_calibration import BODY, LEFT, OAK_IMU, xyzw_from_rotation

OAK_TOPIC = "/oak/imu/data"
PX4_TOPIC = "/mavros/imu/data_raw"
SOLVER = "imu-imu-kabsch-v1"


def read_gyros(session):
    from .bags import reader, stamp
    rows = {OAK_TOPIC: [], PX4_TOPIC: []}
    frames = {}
    with reader(Path(session)) as bag:
        connections = [c for c in bag.connections if c.topic in rows]
        for connection, _, raw in bag.messages(connections=connections):
            message = bag.deserialize(raw, connection.msgtype)
            frames[connection.topic] = message.header.frame_id
            w = message.angular_velocity
            rows[connection.topic].append((stamp(message), w.x, w.y, w.z))
    for topic, frame in ((OAK_TOPIC, OAK_IMU), (PX4_TOPIC, BODY)):
        if not rows[topic]:
            raise ValueError(f"Session has no {topic} messages")
        if frames[topic] != frame:
            raise ValueError(f"{topic} is in frame {frames[topic]!r}, expected {frame!r}")
    return tuple(series(rows[topic]) for topic in (OAK_TOPIC, PX4_TOPIC))


def series(rows):
    rows = sorted(rows)
    times = np.array([row[0] for row in rows], dtype=np.int64)
    if (np.diff(times) <= 0).any():
        raise ValueError("Gyro timestamps must be strictly increasing")
    return times, np.array([row[1:] for row in rows], float)


def kabsch(source, target):
    """Rotation R minimising |target - R source| over centred rows; also the reflection fit."""
    covariance = (target - target.mean(0)).T @ (source - source.mean(0))
    u, _, vt = np.linalg.svd(covariance)
    proper = np.diag([1, 1, np.sign(np.linalg.det(u @ vt))])
    improper = np.diag([1, 1, -np.sign(np.linalg.det(u @ vt))])
    return u @ proper @ vt, u @ improper @ vt


def residual_rms(source, target, rotation):
    error = (target - target.mean(0)) - (source - source.mean(0)) @ rotation.T
    return float(math.sqrt(np.mean(np.sum(error * error, axis=1))))


class Aligner:
    """Pairs PX4 samples with OAK rates interpolated at ``px4_time - offset``."""

    def __init__(self, oak, px4, max_gap_ns):
        self.oak_t, self.oak_w = oak
        self.px4_t, self.px4_w = px4
        self.max_gap_ns = max_gap_ns

    def pairs(self, offset_ns, window=None):
        query = self.px4_t - offset_ns
        index = np.searchsorted(self.oak_t, query)
        valid = (index > 0) & (index < len(self.oak_t))
        if window is not None:
            valid &= (self.px4_t >= window[0]) & (self.px4_t < window[1])
        index = np.where(valid, index, 1)
        left, right = self.oak_t[index - 1], self.oak_t[index]
        bridged = (right - left) <= self.max_gap_ns
        # OAK gaps are skipped, not interpolated across; report how much they cost.
        self.skipped = int(np.sum(valid & ~bridged))
        valid &= bridged
        weight = ((query - left) / np.maximum(right - left, 1))[:, None]
        oak = self.oak_w[index - 1] + weight * (self.oak_w[index] - self.oak_w[index - 1])
        return oak[valid], self.px4_w[valid]

    def cost(self, offset_ns, window=None):
        oak, px4 = self.pairs(offset_ns, window)
        if len(oak) < 50:
            return math.inf, None, len(oak)
        rotation, _ = kabsch(oak, px4)
        return residual_rms(oak, px4, rotation), rotation, len(oak)


def excitation(rates):
    """Per-axis RMS of bias-centred body rates and the weakest principal direction."""
    centred = rates - rates.mean(0)
    eigenvalues = np.linalg.eigvalsh(np.cov(centred.T))
    return {"rms_rad_s": np.sqrt(np.mean(centred ** 2, axis=0)).tolist(),
            "weakest_principal_rms_rad_s": float(math.sqrt(max(eigenvalues[0], 0)))}


def solve_window(aligner, max_offset_ns, window=None, coarse_step_ns=2_000_000):
    offsets = np.arange(-max_offset_ns, max_offset_ns + 1, coarse_step_ns)
    costs = [aligner.cost(int(o), window)[0] for o in offsets]
    best = int(offsets[int(np.argmin(costs))])
    if not math.isfinite(min(costs)):
        raise ValueError("Too few overlapping OAK/PX4 gyro samples")
    if abs(best) >= max_offset_ns:
        raise ValueError("Best clock offset is at the search limit; widen max_offset_ms")
    # Refine on a 0.1 ms grid, then a parabola through the minimum.
    fine = np.arange(best - coarse_step_ns, best + coarse_step_ns + 1, 100_000)
    fine_costs = np.array([aligner.cost(int(o), window)[0] for o in fine])
    i = int(np.argmin(fine_costs))
    offset = float(fine[i])
    if 0 < i < len(fine) - 1:
        a, b, c = fine_costs[i - 1:i + 2]
        curvature = a - 2 * b + c
        if curvature > 0:
            offset += 0.5 * (a - c) / curvature * 100_000
    cost, rotation, count = aligner.cost(round(offset), window)
    oak, px4 = aligner.pairs(round(offset), window)
    _, improper = kabsch(oak, px4)
    return {"offset_ns": round(offset), "rotation": rotation, "residual_rad_s": cost, "pairs": count,
            "skipped_for_oak_gaps": aligner.skipped,
            "reflection_residual_rad_s": residual_rms(oak, px4, improper), "excitation": excitation(px4)}


def rotation_vector(rotation):
    angle = math.acos(max(-1.0, min(1.0, (np.trace(rotation) - 1) / 2)))
    axis = np.array([rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0],
                     rotation[1, 0] - rotation[0, 1]])
    return np.zeros(3) if angle < 1e-12 else axis * angle / (2 * math.sin(angle))


def solve_gyros(oak, px4, *, max_offset_ms=200.0, segments=4, holdout_fraction=0.25,
                min_excitation_rad_s=0.3, max_gap_ms=50.0, offset_floor_ns=100_000,
                rotation_floor_rad=0.002):
    """Solve offset and rotation from (times_ns, rates) arrays; see module docstring."""
    aligner = Aligner(oak, px4, int(max_gap_ms * 1e6))
    max_offset_ns = int(max_offset_ms * 1e6)
    full = solve_window(aligner, max_offset_ns)
    weakest = full["excitation"]["weakest_principal_rms_rad_s"]
    if weakest < min_excitation_rad_s:
        raise ValueError(f"Insufficient rotation about every axis: weakest principal rate RMS "
                         f"{weakest:.3f} rad/s < {min_excitation_rad_s} rad/s")
    if full["reflection_residual_rad_s"] < 0.5 * full["residual_rad_s"]:
        raise ValueError("A reflection fits far better than a rotation: check the IMU axis/handedness conventions")

    start, end = int(px4[0][0]), int(px4[0][-1])
    split = start + int((end - start) * (1 - holdout_fraction))
    edges = np.linspace(start, split, segments + 1).astype(np.int64)
    estimates = []
    for a, b in zip(edges, edges[1:]):
        try:
            result = solve_window(aligner, max_offset_ns, (a, b))
        except ValueError:
            continue
        if result["excitation"]["weakest_principal_rms_rad_s"] >= min_excitation_rad_s:
            estimates.append(result)
    if len(estimates) < 2:
        raise ValueError("Fewer than two well-excited segments; record longer multi-axis motion")
    offsets = np.array([e["offset_ns"] for e in estimates], float)
    vectors = np.array([rotation_vector(e["rotation"] @ full["rotation"].T) for e in estimates])
    # Standard error of the segment mean, with floors for what segments cannot show.
    count = len(estimates)
    offset_sigma = max(offsets.std(ddof=1) / math.sqrt(count), offset_floor_ns)
    rotation_sigma = np.maximum(vectors.std(axis=0, ddof=1) / math.sqrt(count), rotation_floor_rad)

    holdout = solve_window(aligner, max_offset_ns, (split, end + 1))
    holdout_offset_error = holdout["offset_ns"] - full["offset_ns"]
    holdout_rotation_error = float(np.linalg.norm(rotation_vector(holdout["rotation"] @ full["rotation"].T)))
    consistent = (abs(holdout_offset_error) <= 3 * max(offsets.std(ddof=1), offset_floor_ns)
                  and holdout_rotation_error <= 3 * max(float(np.linalg.norm(vectors.std(axis=0, ddof=1))),
                                                         rotation_floor_rad))
    return {"offset_ns": full["offset_ns"], "offset_sigma_ns": round(offset_sigma),
            "rotation": full["rotation"], "rotation_sigma_rad": rotation_sigma.tolist(),
            "residual_rad_s": full["residual_rad_s"],
            "reflection_residual_rad_s": full["reflection_residual_rad_s"],
            "pairs": full["pairs"], "skipped_for_oak_gaps": full["skipped_for_oak_gaps"],
            "excitation": full["excitation"],
            "segments": [{"offset_ns": e["offset_ns"], "residual_rad_s": e["residual_rad_s"],
                          "rotation_difference_rad": float(np.linalg.norm(v)), "pairs": e["pairs"]}
                         for e, v in zip(estimates, vectors)],
            "holdout": {"offset_error_ns": holdout_offset_error,
                        "rotation_error_rad": holdout_rotation_error,
                        "residual_rad_s": holdout["residual_rad_s"], "consistent_3_sigma": consistent}}


def calibration_entries(result, calibration=None, evidence=""):
    """Rig calibration entries for the solved link and clock offset."""
    today = datetime.date.today().isoformat()
    # Gyros do not observe position. The OAK IMU sits inside the housing, so a
    # hand-measured camera position bounds it; otherwise the position stays unknown.
    translation, translation_sigma = [0.0, 0.0, 0.0], None
    direct = None if calibration is None else calibration.transforms.get((BODY, LEFT))
    if direct is not None:
        translation = direct.translation.tolist()
        if direct.covariance is not None:
            housing = 0.03
            translation_sigma = np.sqrt(np.diag(direct.covariance)[3:] + housing ** 2).tolist()
    return [
        {"parent": BODY, "child": OAK_IMU, "source": "estimated",
         "rotation_xyzw": xyzw_from_rotation(result["rotation"]), "translation_m": translation,
         "rotation_sigma_rad": result["rotation_sigma_rad"], "translation_sigma_m": translation_sigma,
         "evidence": f"{SOLVER}: gyro rotation; translation from hand-measured camera position "
                     f"within the OAK housing (0.03 m) if available. {evidence}".strip(),
         "date": today},
        {"clock": "oak_ros_stamp", "reference": "px4_ros_stamp", "source": "estimated",
         "offset_ns": int(result["offset_ns"]), "sigma_ns": int(result["offset_sigma_ns"]),
         "evidence": f"{SOLVER}: OAK vs PX4 gyro timestamps; includes any filter-delay "
                     f"difference, not exposure timing. {evidence}".strip(),
         "date": today},
    ]


def solve_session(session, **options):
    oak, px4 = read_gyros(session)
    result = solve_gyros(oak, px4, **options)
    result["rates_hz"] = {name: float((len(t) - 1) / ((t[-1] - t[0]) / 1e9))
                          for name, (t, _) in (("oak", oak), ("px4", px4))}
    return result
