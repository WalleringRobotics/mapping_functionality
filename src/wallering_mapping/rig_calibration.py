"""Rig calibration: camera/IMU/antenna transforms and clock offsets with uncertainty.

A transform ``parent -> child`` maps child coordinates into the parent frame,
``p_parent = R @ p_child + t``; ``t`` is the child origin expressed in the parent.
Uncertainty is a first-order 6-vector ``[rotation (rad), translation (m)]`` of
perturbations expressed in the parent frame. ``None`` sigmas mean "unknown" and
make every composed covariance unknown rather than silently zero.
"""

import copy
import json
import math
from pathlib import Path

import numpy as np

BODY = "base_link"
OAK_IMU = "oak_imu_frame"
CAMERA_SOCKETS = {0: "oak_rgb_camera_optical_frame", 1: "oak_left_camera_optical_frame",
                  2: "oak_right_camera_optical_frame"}
LEFT = CAMERA_SOCKETS[1]
ANTENNA = "gnss_antenna_arp"
FRAMES = {BODY, OAK_IMU, ANTENNA, *CAMERA_SOCKETS.values()}
# Links a person or a solver supplies; camera-to-camera links come from OAK factory data.
# body -> left camera can be measured by hand; the IMU chain comes from the solvers and
# takes precedence when complete. Both may be present and are then cross-checked.
LINKS = [(BODY, OAK_IMU), (OAK_IMU, LEFT), (BODY, LEFT), (BODY, ANTENNA)]
TRANSFORM_SOURCES = {"unset", "manual_measurement", "cad", "estimated", "factory"}
CLOCKS = {"oak_ros_stamp", "px4_ros_stamp", "oak_camera_exposure", "oak_imu"}
REQUIRED_OFFSETS = [("oak_ros_stamp", "px4_ros_stamp"), ("oak_camera_exposure", "oak_imu")]


def rotation_from_xyzw(xyzw):
    q = np.asarray(xyzw, float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 1e-6:
        raise ValueError("rotation_xyzw must be a unit quaternion [x, y, z, w]")
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rotation_from_rpy_deg(rpy):
    """ROS convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll), angles about the parent axes."""
    values = np.asarray(rpy, float)
    if values.shape != (3,) or not np.isfinite(values).all():
        raise ValueError("rotation_rpy_deg must be [roll, pitch, yaw] in degrees")
    r, p, y = np.radians(values)
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def xyzw_from_rotation(rotation):
    m = np.asarray(rotation, float)
    trace = np.trace(m)
    if trace > 0:
        s = 2 * math.sqrt(trace + 1)
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4]
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * math.sqrt(1 + m[i, i] - m[j, j] - m[k, k])
        q = [0.0, 0.0, 0.0, (m[k, j] - m[j, k]) / s]
        q[i] = s / 4
        q[j] = (m[j, i] + m[i, j]) / s
        q[k] = (m[k, i] + m[i, k]) / s
    q = np.asarray(q)
    q = q / np.linalg.norm(q)
    return (q if q[3] >= 0 else -q).tolist()


def skew(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


class Transform:
    def __init__(self, parent, child, rotation, translation, covariance=None, sources=()):
        self.parent, self.child = parent, child
        self.rotation = np.asarray(rotation, float)
        self.translation = np.asarray(translation, float)
        self.covariance = None if covariance is None else np.asarray(covariance, float)
        self.sources = tuple(sources)

    def compose(self, other):
        """self (a -> b) then other (b -> c) gives a -> c."""
        if self.child != other.parent:
            raise ValueError(f"Cannot compose {self.parent}->{self.child} with {other.parent}->{other.child}")
        rotation = self.rotation @ other.rotation
        translation = self.rotation @ other.translation + self.translation
        covariance = None
        if self.covariance is not None and other.covariance is not None:
            outer = np.eye(6)
            outer[3:, :3] = -skew(self.rotation @ other.translation)
            inner = np.zeros((6, 6))
            inner[:3, :3] = inner[3:, 3:] = self.rotation
            covariance = outer @ self.covariance @ outer.T + inner @ other.covariance @ inner.T
        return Transform(self.parent, other.child, rotation, translation, covariance,
                         self.sources + other.sources)

    def inverse(self):
        rotation = self.rotation.T
        translation = -rotation @ self.translation
        covariance = None
        if self.covariance is not None:
            # Perturbations move from the old parent frame into the old child frame.
            jacobian = np.zeros((6, 6))
            jacobian[:3, :3] = -rotation
            jacobian[3:, :3] = -rotation @ skew(self.translation)
            jacobian[3:, 3:] = -rotation
            covariance = jacobian @ self.covariance @ jacobian.T
        return Transform(self.child, self.parent, rotation, translation, covariance, self.sources)

    def report(self):
        sigma = None if self.covariance is None else np.sqrt(np.clip(np.diag(self.covariance), 0, None))
        return {"parent": self.parent, "child": self.child,
                "rotation_xyzw": xyzw_from_rotation(self.rotation),
                "rotation_matrix": self.rotation.tolist(), "translation_m": self.translation.tolist(),
                "rotation_sigma_rad": None if sigma is None else sigma[:3].tolist(),
                "translation_sigma_m": None if sigma is None else sigma[3:].tolist(),
                "sources": sorted(set(self.sources))}


def sigma_vector(value, label):
    if value is None:
        return None
    values = np.asarray(value, float)
    if values.shape != (3,) or not np.isfinite(values).all() or (values < 0).any():
        raise ValueError(f"{label} must be three nonnegative values or null")
    return values


def parse_transform(entry):
    parent, child, source = entry.get("parent"), entry.get("child"), entry.get("source")
    if parent not in FRAMES or child not in FRAMES or parent == child:
        raise ValueError(f"Unknown or repeated frame in transform {parent}->{child}")
    if source not in TRANSFORM_SOURCES:
        raise ValueError(f"Transform {parent}->{child} source must be one of {sorted(TRANSFORM_SOURCES)}")
    if source == "unset":
        return None
    quaternion, rpy = entry.get("rotation_xyzw"), entry.get("rotation_rpy_deg")
    if (quaternion is None) == (rpy is None):
        raise ValueError(f"Transform {parent}->{child} needs exactly one of rotation_xyzw or rotation_rpy_deg")
    rotation = rotation_from_xyzw(quaternion) if quaternion is not None else rotation_from_rpy_deg(rpy)
    translation = np.asarray(entry.get("translation_m"), float)
    if translation.shape != (3,) or not np.isfinite(translation).all() or np.linalg.norm(translation) > 20:
        raise ValueError(f"Transform {parent}->{child} translation_m must be three finite metres (< 20 m)")
    rotation_sigma = sigma_vector(entry.get("rotation_sigma_rad"), "rotation_sigma_rad")
    translation_sigma = sigma_vector(entry.get("translation_sigma_m"), "translation_sigma_m")
    if rotation_sigma is not None and rotation_sigma.max() > 0.2:
        raise ValueError("rotation_sigma_rad above 0.2 rad breaks the small-angle model")
    covariance = None
    if rotation_sigma is not None and translation_sigma is not None:
        covariance = np.diag(np.concatenate([rotation_sigma, translation_sigma]) ** 2)
    return Transform(parent, child, rotation, translation, covariance, [source])


def parse_offset(entry):
    clock, reference, source = entry.get("clock"), entry.get("reference"), entry.get("source")
    if clock not in CLOCKS or reference not in CLOCKS or clock == reference:
        raise ValueError(f"Unknown clock pair {clock}->{reference}")
    if source not in TRANSFORM_SOURCES - {"factory", "cad"}:
        raise ValueError(f"Offset {clock}->{reference} source must be unset, manual_measurement or estimated")
    if source == "unset":
        return None
    offset, sigma = entry.get("offset_ns"), entry.get("sigma_ns")
    if type(offset) is not int or abs(offset) > 10**9:
        raise ValueError(f"Offset {clock}->{reference} offset_ns must be an integer within one second")
    if sigma is not None and (type(sigma) is not int or sigma < 0):
        raise ValueError(f"Offset {clock}->{reference} sigma_ns must be a nonnegative integer or null")
    return {"clock": clock, "reference": reference, "offset_ns": offset, "sigma_ns": sigma,
            "source": source}


def factory_camera_links(calibration):
    """OAK factory extrinsics (cm, p_dst = R p_src + t) as left-camera -> camera transforms."""
    links = {}
    for socket, data in calibration["cameraData"]:
        extrinsics = data["extrinsics"]
        target = extrinsics["toCameraSocket"]
        if socket in CAMERA_SOCKETS and target in CAMERA_SOCKETS:
            translation = np.array([extrinsics["translation"][axis] for axis in "xyz"]) / 100
            links[(socket, target)] = Transform(CAMERA_SOCKETS[target], CAMERA_SOCKETS[socket],
                                                extrinsics["rotationMatrix"], translation,
                                                None, ["factory"])
    result = {}
    frontier = [LEFT]
    known = {LEFT: Transform(LEFT, LEFT, np.eye(3), np.zeros(3), np.zeros((6, 6)), [])}
    while frontier:
        frame = frontier.pop()
        for link in links.values():
            for edge in (link, link.inverse()):
                if edge.parent == frame and edge.child not in known:
                    known[edge.child] = known[frame].compose(edge)
                    frontier.append(edge.child)
    for frame, transform in known.items():
        if frame != LEFT:
            result[frame] = transform
    return result


class RigCalibration:
    def __init__(self, data):
        if data.get("schema_version") != 1:
            raise ValueError("Unsupported rig calibration schema_version")
        self.data = copy.deepcopy(data)
        self.transforms, self.offsets, self.unset = {}, {}, []
        for entry in data.get("transforms", []):
            key = (entry.get("parent"), entry.get("child"))
            if key not in LINKS:
                raise ValueError(f"Unsupported transform {key[0]}->{key[1]}; expected {LINKS}")
            if key in self.transforms or key in self.unset:
                raise ValueError(f"Duplicate transform {key[0]}->{key[1]}")
            parsed = parse_transform(entry)
            if parsed is None:
                self.unset.append(key)
            else:
                self.transforms[key] = parsed
        for entry in data.get("time_offsets", []):
            key = (entry.get("clock"), entry.get("reference"))
            if key not in REQUIRED_OFFSETS:
                raise ValueError(f"Unsupported time offset {key[0]}->{key[1]}; expected {REQUIRED_OFFSETS}")
            if key in self.offsets or key in self.unset:
                raise ValueError(f"Duplicate time offset {key[0]}->{key[1]}")
            parsed = parse_offset(entry)
            if parsed is None:
                self.unset.append(key)
            else:
                self.offsets[key] = parsed
        missing = [key for key in LINKS + REQUIRED_OFFSETS
                   if key not in self.transforms and key not in self.offsets and key not in self.unset]
        if missing:
            raise ValueError(f"Rig calibration must list every link (unset is allowed): {missing}")

    @classmethod
    def read(cls, path):
        return cls(json.loads(Path(path).read_text()))

    @property
    def oak_device_id(self):
        return self.data.get("hardware", {}).get("oak_device_id")

    def imu_chain(self):
        first, second = self.transforms.get((BODY, OAK_IMU)), self.transforms.get((OAK_IMU, LEFT))
        return None if first is None or second is None else first.compose(second)

    def camera_to_body(self, camera=LEFT, factory=None):
        """Transform base_link -> camera optical frame, or None while no path is set."""
        result = self.imu_chain() or self.transforms.get((BODY, LEFT))
        if result is None:
            return None
        if camera != LEFT:
            if factory is None or camera not in factory:
                raise ValueError(f"{camera} needs OAK factory extrinsics from a session")
            result = result.compose(factory[camera])
        return result

    def antenna_to_camera(self, camera=LEFT, factory=None):
        """Lever from antenna ARP to camera centre in body FLU, as gnss_accuracy expects."""
        body_camera = self.camera_to_body(camera, factory)
        antenna = self.transforms.get((BODY, ANTENNA))
        if body_camera is None or antenna is None:
            return None
        lever = body_camera.translation - antenna.translation
        covariance = None
        if body_camera.covariance is not None and antenna.covariance is not None:
            covariance = body_camera.covariance[3:, 3:] + antenna.covariance[3:, 3:]
        return {"antenna_to_camera_flu_m": lever.tolist(),
                "lever_covariance_flu_m2": None if covariance is None else covariance.tolist(),
                "sources": sorted(set(body_camera.sources + antenna.sources))}


def check(calibration, session=None):
    factory, session_report = None, None
    if session is not None:
        files = sorted(Path(session).glob("*_calibration.json"))
        if len(files) != 1:
            raise ValueError("Session must contain exactly one OAK *_calibration.json")
        device = files[0].name.removesuffix("_calibration.json")
        if calibration.oak_device_id not in (None, device):
            raise ValueError(f"Calibration is for OAK {calibration.oak_device_id}, session used {device}")
        oak = json.loads(files[0].read_text())
        factory = factory_camera_links(oak)
        session_report = {"oak_device_id": device, "board": oak.get("boardName"),
                          "factory_imu_extrinsics": oak.get("imuExtrinsics", {}).get("toCameraSocket", -1) != -1}
    cameras = {}
    for camera in ([LEFT] + sorted(factory) if factory else [LEFT]):
        transform = calibration.camera_to_body(camera, factory)
        cameras[camera] = None if transform is None else {
            "body_to_camera": transform.report(),
            "antenna_lever": calibration.antenna_to_camera(camera, factory)}
    unset = [f"{a}->{b}" for a, b in calibration.unset]
    unknown_sigma = sorted(f"{t.parent}->{t.child}" for t in calibration.transforms.values()
                           if t.covariance is None)
    unknown_sigma += sorted(f"{o['clock']}->{o['reference']}" for o in calibration.offsets.values()
                            if o["sigma_ns"] is None)
    left = calibration.camera_to_body()
    lever = calibration.antenna_to_camera()
    blocking = []
    if left is None:
        blocking.append(f"{BODY}->{LEFT} (directly or via {OAK_IMU})")
    elif left.covariance is None:
        blocking.append(f"{BODY}->{LEFT} uncertainty")
    if lever is None:
        blocking.append(f"{BODY}->{ANTENNA}")
    elif lever["lever_covariance_flu_m2"] is None:
        blocking.append(f"{BODY}->{ANTENNA} uncertainty")
    for key in REQUIRED_OFFSETS:
        offset = calibration.offsets.get(key)
        if offset is None or offset["sigma_ns"] is None:
            blocking.append(f"time offset {key[0]}->{key[1]}")
    agreement = consistency(calibration)
    if agreement is not None and agreement.get("consistent_3_sigma") is False:
        blocking.append("Direct camera transform and IMU chain disagree beyond 3 sigma")
    return {"complete": not blocking, "blocking": blocking, "unset": unset,
            "unknown_uncertainty": unknown_sigma,
            "camera_path": None if left is None else (
                "imu_chain" if calibration.imu_chain() is not None else "direct"),
            "direct_vs_imu_chain": agreement,
            "sources": {f"{a}->{b}": t.sources[0] for (a, b), t in calibration.transforms.items()},
            "time_offsets": list(calibration.offsets.values()),
            "cameras": cameras, "session": session_report}


def consistency(calibration):
    """Disagreement between a hand-measured camera link and the IMU chain, in sigmas."""
    chain, direct = calibration.imu_chain(), calibration.transforms.get((BODY, LEFT))
    if chain is None or direct is None:
        return None
    delta = direct.rotation @ chain.rotation.T
    angle = math.acos(max(-1.0, min(1.0, (np.trace(delta) - 1) / 2)))
    distance = float(np.linalg.norm(direct.translation - chain.translation))
    result = {"rotation_difference_rad": angle, "translation_difference_m": distance}
    if chain.covariance is not None and direct.covariance is not None:
        combined = chain.covariance + direct.covariance
        result["rotation_sigma_rad"] = math.sqrt(np.trace(combined[:3, :3]))
        result["translation_sigma_m"] = math.sqrt(np.trace(combined[3:, 3:]))
        result["consistent_3_sigma"] = (angle <= 3 * result["rotation_sigma_rad"]
                                        and distance <= 3 * result["translation_sigma_m"])
    return result


def apply_entries(data, entries):
    """Copy of calibration data with matching transform/offset entries replaced and re-validated."""
    result = copy.deepcopy(data)
    for entry in entries:
        if "parent" in entry:
            section, key = "transforms", ("parent", "child")
        else:
            section, key = "time_offsets", ("clock", "reference")
        matches = [i for i, old in enumerate(result[section])
                   if all(old.get(k) == entry[k] for k in key)]
        if len(matches) != 1:
            raise ValueError(f"Calibration has no single {section} entry for {[entry[k] for k in key]}")
        result[section][matches[0]] = copy.deepcopy(entry)
    RigCalibration(result)
    return result
