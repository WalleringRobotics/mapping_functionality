"""Corrected rover evidence and conditional camera-position uncertainty budgets."""

import bisect
import csv
import html
import json
import math
from pathlib import Path

import numpy as np

from .association import ClockMap
from .dataset import jsonl, safe_path, sha256_file, write_json
from .reconstruct import load_project
from .rtcm import RTCMStream, describe
from .validate import distribution, validate


PROFILE = {
    "schema_version": 1,
    "base": {
        "station_id": None,
        "wgs84_lat_lon_h": None,
        "covariance_enu_m2": None,
        "survey_evidence": None,
        "datum_epoch_evidence": None,
        "position_reference": "ARP",
        "rover_uncertainty_includes_base": None,
        "rtcm_coordinate_tolerance_m": 0.05,
    },
    "receiver": {
        "model_firmware_evidence": None,
        "position_reference": "unknown",
        "gpsraw_time_mode": "unknown",
        "timestamp_validation_evidence": None,
        "horizontal_accuracy_model": "unknown",
        "vertical_accuracy_model": "unknown",
        "accuracy_validation_evidence": None,
        "ellipsoidal_height_verified": False,
        "correction_age_field_verified": False,
    },
    "rig": {
        "antenna_to_camera_flu_m": None,
        "lever_covariance_flu_m2": None,
        "attitude_sigma_rad": None,
        "calibration_evidence": None,
        "orientation_in_base_tangent_enu_verified": False,
        "lever_from_receiver_reference_verified": False,
    },
    "motion": {
        "speed_bound_m_s": None,
        "acceleration_bound_m_s2": None,
        "angular_rate_bound_rad_s": None,
        "receiver_latency_bound_ms": None,
        "camera_latency_bound_ms": None,
        "bound_validation_evidence": None,
    },
    "policy": {
        "max_gnss_age_ms": 200,
        "max_transport_age_seconds": 2,
        "max_base_distance_m": 5000,
        "max_receiver_correction_age_ms": 5000,
        "require_receiver_correction_age": True,
        "horizontal_limit_m": 0.1,
        "vertical_limit_m": 0.15,
        "min_qualified_fraction": 0.9,
        "output_crs": None,
    },
}


def finite(value, label, positive=False):
    if (
        type(value) not in (int, float)
        or not math.isfinite(value)
        or value < 0
        or (positive and value == 0)
    ):
        raise ValueError(f"{label} must be finite and {'positive' if positive else 'nonnegative'}")
    return float(value)


def covariance(value, label):
    matrix = np.asarray(value, float)
    if (
        matrix.shape != (3, 3)
        or not np.isfinite(matrix).all()
        or not np.allclose(matrix, matrix.T, atol=1e-12, rtol=0)
        or np.linalg.eigvalsh(matrix).min() < -1e-12
    ):
        raise ValueError(f"{label} must be a symmetric positive semidefinite 3x3 covariance in m²")
    return matrix


def geodetic(value):
    point = np.asarray(value, float)
    if (
        point.shape != (3,)
        or not np.isfinite(point).all()
        or not -90 <= point[0] <= 90
        or not -180 <= point[1] <= 180
        or not -1000 < point[2] < 20000
    ):
        raise ValueError("Provide WGS84 latitude/longitude degrees and ellipsoidal height metres")
    return point


def read_profile(path):
    profile = json.loads(path.read_text())
    if set(profile) != set(PROFILE) or profile["schema_version"] != 1:
        raise ValueError("Unsupported GNSS accuracy profile")
    for section in PROFILE:
        if section == "schema_version":
            continue
        if not isinstance(profile[section], dict) or set(profile[section]) != set(PROFILE[section]):
            raise ValueError(f"Invalid accuracy-profile keys: {section}")
    base, receiver, rig, motion, policy = (
        profile[k] for k in ("base", "receiver", "rig", "motion", "policy")
    )
    if base["position_reference"] != "ARP":
        raise ValueError(
            "Base coordinates must refer to the surveyed antenna reference point (ARP)"
        )
    if base["wgs84_lat_lon_h"] is not None:
        geodetic(base["wgs84_lat_lon_h"])
    if base["station_id"] is not None and (
        type(base["station_id"]) is not int or not 0 <= base["station_id"] <= 4095
    ):
        raise ValueError("Invalid RTCM station ID")
    for value, label in (
        (base["covariance_enu_m2"], "base covariance"),
        (rig["lever_covariance_flu_m2"], "lever covariance"),
    ):
        if value is not None:
            covariance(value, label)
    if rig["antenna_to_camera_flu_m"] is not None:
        lever = np.asarray(rig["antenna_to_camera_flu_m"], float)
        if lever.shape != (3,) or not np.isfinite(lever).all() or np.linalg.norm(lever) > 20:
            raise ValueError("Invalid measured antenna-to-camera lever arm in body FLU metres")
    for section, names in (
        (rig, ["attitude_sigma_rad"]),
        (
            motion,
            [
                "speed_bound_m_s",
                "acceleration_bound_m_s2",
                "angular_rate_bound_rad_s",
                "receiver_latency_bound_ms",
                "camera_latency_bound_ms",
            ],
        ),
    ):
        for name in names:
            if section[name] is not None:
                finite(section[name], name)
    if rig["attitude_sigma_rad"] is not None and rig["attitude_sigma_rad"] > 0.1:
        raise ValueError("Small-angle lever covariance requires attitude_sigma_rad <= 0.1 radians")
    for name in (
        "max_gnss_age_ms",
        "max_transport_age_seconds",
        "max_receiver_correction_age_ms",
        "horizontal_limit_m",
        "vertical_limit_m",
        "max_base_distance_m",
    ):
        finite(policy[name], name, positive=True)
    finite(base["rtcm_coordinate_tolerance_m"], "rtcm_coordinate_tolerance_m", positive=True)
    if (
        type(policy["min_qualified_fraction"]) not in (int, float)
        or not 0 <= policy["min_qualified_fraction"] <= 1
    ):
        raise ValueError("min_qualified_fraction must be finite in [0,1]")
    for value in (base["rover_uncertainty_includes_base"],):
        if value is not None and type(value) is not bool:
            raise ValueError("Specify whether receiver uncertainty already includes the base")
    for value in (
        receiver["ellipsoidal_height_verified"],
        receiver["correction_age_field_verified"],
        rig["orientation_in_base_tangent_enu_verified"],
        rig["lever_from_receiver_reference_verified"],
        policy["require_receiver_correction_age"],
    ):
        if type(value) is not bool:
            raise ValueError("Accuracy evidence switches must be booleans")
    if receiver["gpsraw_time_mode"] not in ("unknown", "px4_boot_via_mavros", "unix_via_mavros"):
        raise ValueError("Unsupported GPSRAW timestamp convention")
    if receiver["position_reference"] not in ("unknown", "ARP", "APC"):
        raise ValueError("Receiver position reference must be ARP, APC or unknown")
    if receiver["horizontal_accuracy_model"] not in (
        "unknown",
        "axis_1sigma",
        "horizontal_rms",
        "radial_95",
    ):
        raise ValueError("Unsupported horizontal receiver-uncertainty model")
    if receiver["vertical_accuracy_model"] not in ("unknown", "axis_1sigma", "absolute_95"):
        raise ValueError("Unsupported vertical receiver-uncertainty model")
    for section, names in (
        (base, ["survey_evidence", "datum_epoch_evidence"]),
        (
            receiver,
            [
                "model_firmware_evidence",
                "timestamp_validation_evidence",
                "accuracy_validation_evidence",
            ],
        ),
        (rig, ["calibration_evidence"]),
        (motion, ["bound_validation_evidence"]),
    ):
        for name in names:
            if section[name] is not None and (
                not isinstance(section[name], str) or not section[name].strip()
            ):
                raise ValueError(f"{name} must identify evidence, or be null")
    if policy["output_crs"] is not None:
        projected_crs(policy["output_crs"])
    return profile


def projected_crs(value):
    from pyproj import CRS

    crs = CRS.from_user_input(value)
    if (
        not crs.is_projected
        or crs.is_compound
        or len(crs.axis_info) != 2
        or any(a.unit_conversion_factor != 1 for a in crs.axis_info[:2])
    ):
        raise ValueError("Accuracy output requires a projected CRS with metre horizontal axes")
    return crs


def ecef_and_enu(lat_lon_h):
    lat, lon, height = geodetic(lat_lon_h)
    lat, lon = math.radians(lat), math.radians(lon)
    s, c, sl, cl = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
    n = 6378137 / math.sqrt(1 - 6.6943799901413165e-3 * s * s)
    xyz = np.array(
        [
            (n + height) * c * cl,
            (n + height) * c * sl,
            (n * (1 - 6.6943799901413165e-3) + height) * s,
        ]
    )
    rotation = np.array([[-sl, cl, 0], [-s * cl, -s * sl, c], [c * cl, c * sl, s]])
    return xyz, rotation


def quaternion_rotation(xyzw):
    q = np.asarray(xyzw, float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 0.01:
        raise ValueError("Invalid orientation quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def position_budget(
    rover_cov,
    base_cov,
    lever_cov,
    rotation,
    lever,
    attitude_sigma,
    motion_allowance,
    quantization_allowance=(0, 0),
    base_coordinate_allowance=0,
):
    lever = np.asarray(lever, float)
    x, y, z = lever
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    attitude = rotation @ skew @ (np.eye(3) * attitude_sigma**2) @ skew.T @ rotation.T
    parts = {
        "rover": rover_cov,
        "shared_base": base_cov,
        "lever_calibration": rotation @ lever_cov @ rotation.T,
        "attitude_on_lever": attitude,
    }
    total = sum(parts.values())
    # Conservative enclosing horizontal 95% ellipse radius, not circular CEP.
    h95 = math.sqrt(5.991464547107979 * max(0, np.linalg.eigvalsh(total[:2, :2]).max()))
    v95 = 1.959963984540054 * math.sqrt(max(0, total[2, 2]))
    return {
        "covariance_enu_m2": total.tolist(),
        "components_covariance_enu_m2": {k: v.tolist() for k, v in parts.items()},
        "conditional_gaussian_horizontal_95_m": h95,
        "conditional_gaussian_vertical_95_m": v95,
        "deterministic_motion_allowance_m": motion_allowance,
        "coordinate_quantization_allowance_horizontal_vertical_m": list(quantization_allowance),
        "base_coordinate_discrepancy_allowance_m": base_coordinate_allowance,
        "conditional_horizontal_95_plus_allowances_m": h95
        + motion_allowance
        + quantization_allowance[0]
        + base_coordinate_allowance,
        "conditional_vertical_95_plus_allowances_m": v95
        + motion_allowance
        + quantization_allowance[1]
        + base_coordinate_allowance,
        "model": "Gaussian zero-mean covariance model plus explicit deterministic allowances; not a guaranteed bound",
    }


def gps_source_ros_ns(row, mode, offset):
    stamp = row["source_stamp_ros_ns"]
    if stamp is None:
        raise ValueError("GPSRAW has no source timestamp")
    if mode == "px4_boot_via_mavros":
        return stamp
    if mode == "unix_via_mavros":
        # builtin_interfaces/Time.sec is int32; a doubled epoch can wrap its seconds.
        if stamp < 0:
            stamp += 2**32 * 10**9
        return stamp - offset
    raise ValueError("GPSRAW UTC/boot timestamp convention is unverified")


def correction_history(root, profile):
    decoder, snapshots = RTCMStream(), []
    base = profile["base"]
    expected = (
        ecef_and_enu(base["wgs84_lat_lon_h"])[0] if base["wgs84_lat_lon_h"] is not None else None
    )
    state = {
        "verified_base": False,
        "reason": "No verified RTCM base coordinates",
        "station_id": None,
        "latest_observation_monotonic_ns": None,
    }
    for row in jsonl(root / "telemetry.jsonl"):
        if row.get("role") != "rtcm":
            continue
        try:
            frames = decoder.feed(bytes(row["fields"]["data"]))
            for frame in frames:
                details = describe(frame)
                if "station_id" in details and details["station_id"] != base["station_id"]:
                    raise ValueError("Correction station differs from surveyed base")
                if "base_arp_ecef_m" in details:
                    if expected is None or base["station_id"] is None:
                        raise ValueError("Surveyed base coordinates/station ID are missing")
                    difference = float(np.linalg.norm(expected - details["base_arp_ecef_m"]))
                    if difference > base["rtcm_coordinate_tolerance_m"]:
                        raise ValueError("RTCM base coordinates differ from surveyed ARP")
                    state = {
                        **state,
                        "verified_base": True,
                        "reason": None,
                        **details,
                        "base_coordinate_difference_m": difference,
                    }
                kind = details["message_type"]
                if (
                    1001 <= kind <= 1004
                    or 1009 <= kind <= 1012
                    or any(
                        first <= kind <= first + 6 for first in (1071, 1081, 1091, 1101, 1111, 1121)
                    )
                ):
                    state["latest_observation_monotonic_ns"] = row["received_monotonic_ns"]
            if frames:
                snapshots.append({**state, "mapped_monotonic_ns": row["received_monotonic_ns"]})
        except (ValueError, KeyError, TypeError) as error:
            state = {
                "verified_base": False,
                "reason": str(error),
                "station_id": None,
                "latest_observation_monotonic_ns": None,
            }
            snapshots.append({**state, "mapped_monotonic_ns": row["received_monotonic_ns"]})
            decoder = RTCMStream()
    return snapshots, [s["mapped_monotonic_ns"] for s in snapshots]


def gps_bracket(rows, times, exposure, max_age_ns):
    """Use exact or bracketing source measurements, never receipt time or extrapolation."""
    index = bisect.bisect_left(times, exposure)
    if index < len(times) and times[index] == exposure:
        before = after = rows[index]
        weight = 0.0
    elif index == 0 or index == len(times):
        return None
    else:
        before, after = rows[index - 1], rows[index]
        weight = (exposure - times[index - 1]) / (times[index] - times[index - 1])
    before_dt = exposure - before["mapped_monotonic_ns"]
    after_dt = after["mapped_monotonic_ns"] - exposure
    if max(before_dt, after_dt) > max_age_ns:
        return None
    return {
        "samples": [before, after] if before is not after else [before],
        "after_weight": weight,
        "before_interval_seconds": before_dt / 1e9,
        "after_interval_seconds": after_dt / 1e9,
        "clock_budget_ns": max(before["clock_budget_ns"], after["clock_budget_ns"]),
    }


def image_accuracy(session, alignment, profile_path, output, project=None):
    session, alignment, output = session.resolve(), alignment.resolve(), output.resolve()
    for source in (session, alignment, project.resolve() if project else None):
        if source and (output.is_relative_to(source) or source.is_relative_to(output)):
            raise ValueError("Accuracy outputs must be separate from immutable inputs")
    audit = validate(session)
    if not audit["valid"]:
        raise ValueError(f"Dataset failed validation: {audit['errors']}")
    profile = read_profile(profile_path)
    alignment_report = json.loads((alignment / "report.json").read_text())
    source_hash = sha256_file(session / "manifest.json")
    if (
        alignment_report["status"] != "complete"
        or alignment_report["source_manifest_sha256"] != source_hash
    ):
        raise ValueError("Alignment is incomplete or belongs to another capture")
    for name, expected in alignment_report["output_hashes"].items():
        if sha256_file(safe_path(alignment, name)) != expected:
            raise ValueError("Alignment output integrity mismatch")
    selected = None
    if project:
        metadata = load_project(project)
        if metadata["source_manifest_sha256"] != source_hash:
            raise ValueError("Selected image project belongs to another capture")
        selected = {image["name"] for image in metadata["images"]}
    base, receiver, rig, motion, policy = (
        profile[k] for k in ("base", "receiver", "rig", "motion", "policy")
    )
    config = json.loads((session / "manifest.json").read_text())["telemetry"]["config"]
    ros = ClockMap([r for r in jsonl(session / "telemetry.jsonl") if r["record_type"] == "clock"])
    syncs = [r for r in jsonl(session / "telemetry.jsonl") if r.get("role") == "timesync"]
    sync_times = [r["received_monotonic_ns"] for r in syncs]
    gps_rows, unavailable = [], []
    for row in jsonl(session / "telemetry.jsonl"):
        if row.get("role") != "gps_raw":
            continue
        try:
            index = bisect.bisect_right(sync_times, row["received_monotonic_ns"]) - 1
            if (
                index < 0
                or row["received_monotonic_ns"] - sync_times[index]
                > config["max_sync_age_ms"] * 1e6
                or syncs[index]["sync_quality"]["qualification"] != "qualified"
            ):
                raise ValueError("GPSRAW received without qualified clock evidence")
            source = gps_source_ros_ns(
                row, receiver["gpsraw_time_mode"], syncs[index]["fields"]["estimated_offset_ns"]
            )
            mono, bracket, _ = ros.to_monotonic(source, int(config["max_clock_age_ms"] * 1e6))
            gps_rows.append(
                {
                    "ordinal": row["ordinal"],
                    "source_stamp_ros_ns": source,
                    "original_header_ns": row["source_stamp_ros_ns"],
                    "fields": row["fields"],
                    "clock_budget_ns": bracket,
                    "mapped_monotonic_ns": mono,
                }
            )
        except ValueError as error:
            unavailable.append(str(error))
    gps_rows.sort(key=lambda r: r["mapped_monotonic_ns"])
    gps_times = [r["mapped_monotonic_ns"] for r in gps_rows]
    if any(b <= a for a, b in zip(gps_times, gps_times[1:])):
        raise ValueError("GPSRAW source timestamps are duplicated/nonmonotonic")
    corrections, correction_times = correction_history(session, profile)
    from pyproj import Transformer

    projector = (
        Transformer.from_crs(4978, policy["output_crs"], always_xy=True)
        if policy["output_crs"]
        else None
    )
    output.mkdir(parents=True, exist_ok=False)
    report = {
        "schema_version": 1,
        "status": "running",
        "source_manifest_sha256": source_hash,
        "stream": alignment_report["stream"],
        "alignment_report_sha256": sha256_file(alignment / "report.json"),
        "profile_sha256": sha256_file(profile_path),
        "profile": profile,
        "project_sha256": sha256_file(project / "project.json") if project else None,
        "accuracy_kind": "Conditional camera-position uncertainty, not per-pixel or measured map accuracy",
        "unmodelled": [
            "Wrong integer ambiguities and non-Gaussian multipath",
            "Scene depth, camera ray/calibration uncertainty",
            "Bundle adjustment and surface reconstruction error",
        ],
        "base_correlation": "Shared base error does not decrease with number of photographs",
        "gps_unavailable_samples": len(unavailable),
        "gps_unavailable_reasons": sorted(set(unavailable)),
    }
    write_json(output / "report.json", report)
    results = []
    try:
        for association in jsonl(alignment / "associations.jsonl"):
            name = Path(association["image"]).name
            if selected is not None and name not in selected:
                continue
            result = {
                "image": association["image"],
                "name": name,
                "qualified": False,
                "position_available": False,
                "reasons": [],
                "limitations": [],
                "budget": None,
            }
            reasons = result["reasons"]
            if association["status"] != "associated":
                reasons.append("Exposure lacks qualified pose/timing association")
                results.append(result)
                continue
            mono = association["exposure_monotonic_ns"]
            gps = gps_bracket(gps_rows, gps_times, mono, int(policy["max_gnss_age_ms"] * 1e6))
            index = bisect.bisect_right(correction_times, mono) - 1
            correction = corrections[index] if index >= 0 else None
            if not correction or not correction["verified_base"]:
                reasons.append(
                    correction["reason"]
                    if correction
                    else "No correction transport/base evidence at exposure"
                )
            if correction:
                observation_time = correction.get("latest_observation_monotonic_ns")
                if (
                    observation_time is None
                    or mono - observation_time > policy["max_transport_age_seconds"] * 1e9
                ):
                    reasons.append("Observation-correction transport missing/stale at exposure")
            result["correction_transport"] = correction
            if gps is None:
                reasons.append(
                    "No fresh bracketing GPSRAW samples with a verified timestamp convention"
                )
                results.append(result)
                continue
            samples = gps["samples"]
            source_fields = [s["fields"] for s in samples]
            # Worst endpoint quality; these are derived gates, not an invented GPSRAW message.
            fields = {
                "fix_type": next(
                    (s.get("fix_type") for s in source_fields if s.get("fix_type") != 6), 6
                ),
                "satellites_visible": min(s.get("satellites_visible", 255) for s in source_fields),
                "dgps_age": max(s.get("dgps_age", 2**32 - 1) for s in source_fields),
            }
            for field in ("h_acc", "v_acc"):
                values = [s.get(field, 0) for s in source_fields]
                fields[field] = (
                    max(values) if all(type(v) is int and 0 < v < 2**32 - 1 for v in values) else 0
                )
            result.update(
                receiver_interpolation=gps,
                fix_type=fields.get("fix_type"),
                satellites=fields.get("satellites_visible"),
            )
            if fields.get("fix_type") != 6:
                reasons.append("Rover is not RTK fixed (fix_type=6)")
            correction_age = fields.get("dgps_age")
            if not receiver["correction_age_field_verified"] or correction_age in (None, 2**32 - 1):
                result["limitations"].append(
                    "Receiver-applied correction age unavailable; transport age is distinct"
                )
                if policy["require_receiver_correction_age"]:
                    reasons.append("Receiver correction age is required but unverified/unavailable")
            elif correction_age > policy["max_receiver_correction_age_ms"]:
                reasons.append("Receiver-applied corrections are stale")
            result["receiver_correction_age_ms"] = (
                correction_age
                if receiver["correction_age_field_verified"] and correction_age != 2**32 - 1
                else None
            )
            required_evidence = [
                (base["survey_evidence"], "Base survey evidence"),
                (base["datum_epoch_evidence"], "Datum/epoch evidence"),
                (receiver["model_firmware_evidence"], "Receiver/firmware evidence"),
                (receiver["timestamp_validation_evidence"], "GPS timestamp validation"),
                (receiver["accuracy_validation_evidence"], "Receiver uncertainty interpretation"),
                (rig["calibration_evidence"], "Rig calibration evidence"),
                (motion["bound_validation_evidence"], "Motion/latency bounds evidence"),
            ]
            reasons.extend(
                label + " missing" for evidence, label in required_evidence if not evidence
            )
            if not receiver["ellipsoidal_height_verified"]:
                reasons.append("GPSRAW ellipsoidal-height extension unverified")
            if (
                receiver["position_reference"] == "unknown"
                or not rig["lever_from_receiver_reference_verified"]
            ):
                reasons.append("Antenna reference/lever-arm convention is unverified")
            if not rig["orientation_in_base_tangent_enu_verified"]:
                reasons.append("Body orientation is not verified in base-tangent ENU")
            if base["rover_uncertainty_includes_base"] is None:
                reasons.append("Receiver/base uncertainty overlap unknown")
            missing = (
                base["wgs84_lat_lon_h"] is None
                or base["covariance_enu_m2"] is None
                or rig["antenna_to_camera_flu_m"] is None
                or rig["lever_covariance_flu_m2"] is None
                or rig["attitude_sigma_rad"] is None
                or any(v is None for v in motion.values())
            )
            if missing:
                reasons.append("Base, rig or motion uncertainty components are missing")
            hacc, vacc = fields.get("h_acc", 0), fields.get("v_acc", 0)
            if (
                type(hacc) is not int
                or type(vacc) is not int
                or hacc <= 0
                or vacc <= 0
                or max(hacc, vacc) >= 2**32 - 1
            ):
                reasons.append("Receiver h_acc/v_acc unavailable; HDOP is not metre accuracy")
            if "unknown" in (
                receiver["horizontal_accuracy_model"],
                receiver["vertical_accuracy_model"],
            ):
                reasons.append("Receiver accuracy confidence model is unknown")
            if missing or any(
                "unavailable; HDOP" in reason or "confidence model" in reason for reason in reasons
            ):
                results.append(result)
                continue
            endpoints = [
                ecef_and_enu([s["lat"] / 1e7, s["lon"] / 1e7, s["alt_ellipsoid"] / 1000])
                for s in source_fields
            ]
            weights = [1 - gps["after_weight"], gps["after_weight"]] if len(samples) == 2 else [1.0]
            rover_xyz = sum(w * point[0] for w, point in zip(weights, endpoints))
            base_xyz, base_axes = ecef_and_enu(base["wgs84_lat_lon_h"])
            result["base_distance_m"] = float(np.linalg.norm(rover_xyz - base_xyz))
            if result["base_distance_m"] > policy["max_base_distance_m"]:
                reasons.append("Rover exceeds configured base-distance limit")
            rotation = quaternion_rotation(
                association["body_pose"]["orientation_body_flu_to_enu_xyzw"]
            )
            lever = np.asarray(rig["antenna_to_camera_flu_m"])
            camera_xyz = rover_xyz + base_axes.T @ rotation @ lever
            camera_enu = base_axes @ (camera_xyz - base_xyz)
            hscale = {
                "axis_1sigma": 1,
                "horizontal_rms": math.sqrt(2),
                "radial_95": math.sqrt(5.991464547107979),
            }[receiver["horizontal_accuracy_model"]]
            vscale = (
                1 if receiver["vertical_accuracy_model"] == "axis_1sigma" else 1.959963984540054
            )
            rover_cov = np.zeros((3, 3))
            for weight, (_, axes), sample in zip(weights, endpoints, source_fields):
                hs, vs = sample["h_acc"] / 1000 / hscale, sample["v_acc"] / 1000 / vscale
                transform = base_axes @ axes.T
                # Linear covariance envelope allows unknown endpoint correlation;
                # squared weights would silently assume independent GNSS errors.
                rover_cov += weight * transform @ np.diag([hs * hs, hs * hs, vs * vs]) @ transform.T
            base_cov = covariance(base["covariance_enu_m2"], "base covariance")
            if base["rover_uncertainty_includes_base"]:
                base_cov = np.zeros((3, 3))
                result["limitations"].append(
                    "Receiver covariance declared to include base; shared component is not added twice"
                )
            dt = (
                association["estimated_alignment_budget_ms"]
                + gps["clock_budget_ns"] / 1e6
                + motion["receiver_latency_bound_ms"]
                + motion["camera_latency_bound_ms"]
            ) / 1000
            interpolation_allowance = (
                0.5
                * motion["acceleration_bound_m_s2"]
                * gps["before_interval_seconds"]
                * gps["after_interval_seconds"]
            )
            allowance = (
                motion["speed_bound_m_s"]
                + motion["angular_rate_bound_rad_s"] * np.linalg.norm(lever)
            ) * dt + interpolation_allowance
            budget = position_budget(
                rover_cov,
                base_cov,
                covariance(rig["lever_covariance_flu_m2"], "lever covariance"),
                rotation,
                lever,
                rig["attitude_sigma_rad"],
                float(allowance),
                (math.sqrt(2) * 111700 * 0.5e-7, 0.0005),
                correction.get("base_coordinate_difference_m", 0) if correction else 0,
            )
            if budget["conditional_horizontal_95_plus_allowances_m"] > policy["horizontal_limit_m"]:
                reasons.append("Conditional horizontal budget exceeds target")
            if budget["conditional_vertical_95_plus_allowances_m"] > policy["vertical_limit_m"]:
                reasons.append("Conditional vertical budget exceeds target")
            result.update(
                position_available=True,
                camera_ecef_m=camera_xyz.tolist(),
                camera_base_enu_m=camera_enu.tolist(),
                projected_camera_xyz_m=list(projector.transform(*camera_xyz))
                if projector
                else None,
                budget=budget,
                qualified=not reasons,
                exposure_monotonic_ns=mono,
                time_motion_interval_seconds=dt,
                interpolation_curvature_allowance_m=interpolation_allowance,
            )
            results.append(result)
        with (output / "images.jsonl").open("x") as file:
            for row in results:
                file.write(json.dumps(row, allow_nan=False) + "\n")
        with (output / "images.csv").open("x", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(
                [
                    "image",
                    "qualified",
                    "fix_type",
                    "h95_plus_allowances_m",
                    "v95_plus_allowances_m",
                    "reasons",
                    "limitations",
                ]
            )
            for row in results:
                b = row["budget"] or {}
                writer.writerow(
                    [
                        row["image"],
                        row["qualified"],
                        row.get("fix_type"),
                        b.get("conditional_horizontal_95_plus_allowances_m"),
                        b.get("conditional_vertical_95_plus_allowances_m"),
                        "; ".join(row["reasons"]),
                        "; ".join(row["limitations"]),
                    ]
                )
        page = '<!doctype html><meta charset="utf-8"><title>Image navigation uncertainty</title><style>body{font:16px system-ui;margin:32px;line-height:1.5}td,th{padding:8px;border-bottom:1px solid #ddd;text-align:left}table{border-collapse:collapse}pre{white-space:pre-wrap}input{padding:8px;width:300px}</style>'
        page += '<h1>Image navigation uncertainty</h1><p>Conditional camera-position estimates, not per-pixel ground accuracy or measured map accuracy. Shared base uncertainty does not average away across photographs. Missing evidence and receiver correction age remain visible.</p><p><a href="images.csv">CSV</a> · <a href="images.jsonl">Complete image evidence</a> · <a href="report.json">Profile and summary</a></p>'
        page += '<input aria-label="Filter images" placeholder="Filter image or reason" oninput="for(const r of document.querySelectorAll(\'tbody tr\'))r.hidden=!r.textContent.toLowerCase().includes(this.value.toLowerCase())"><table><thead><tr><th>Image</th><th>Prior qualified</th><th>H95 + allowances (m)</th><th>V95 + allowances (m)</th><th>Evidence</th></tr></thead><tbody>'
        for row in results:
            b = row["budget"] or {}
            h, v = (
                b.get("conditional_horizontal_95_plus_allowances_m"),
                b.get("conditional_vertical_95_plus_allowances_m"),
            )
            page += (
                "<tr><td>"
                + html.escape(row["name"])
                + "</td><td>"
                + str(row["qualified"])
                + "</td><td>"
                + ("unknown" if h is None else f"{h:.4f}")
                + "</td><td>"
                + ("unknown" if v is None else f"{v:.4f}")
                + "</td><td><details><summary>"
                + html.escape("; ".join(row["reasons"] + row["limitations"]) or "Profile gates met")
                + "</summary><pre>"
                + html.escape(json.dumps(b, indent=2))
                + "</pre></details></td></tr>"
            )
        (output / "report.html").write_text(page + "</tbody></table>")
        good = [row for row in results if row["qualified"]]
        geo = [row for row in good if row["projected_camera_xyz_m"] is not None]
        if len(geo) >= 3:
            xy = np.array([r["projected_camera_xyz_m"][:2] for r in geo])
            if np.linalg.matrix_rank(xy - xy.mean(axis=0), tol=1e-6) >= 2:
                with (output / "camera-geo.txt").open("x") as file:
                    file.write(str(policy["output_crs"]) + "\n")
                    for row in geo:
                        # Four-column geo: no unsupported camera yaw/pitch/roll conversion.
                        file.write(
                            row["name"]
                            + " "
                            + " ".join(format(v, ".12g") for v in row["projected_camera_xyz_m"])
                            + "\n"
                        )
        report.update(
            status="complete",
            images=len(results),
            qualified_images=len(good),
            qualified_fraction=len(good) / len(results) if results else 0,
            passed=bool(good) and len(good) / len(results) >= policy["min_qualified_fraction"],
            fully_documented_receiver_age=all(
                r["receiver_correction_age_ms"] is not None for r in good
            )
            if good
            else False,
            horizontal_budget_m=distribution(
                [r["budget"]["conditional_horizontal_95_plus_allowances_m"] for r in good]
            ),
            vertical_budget_m=distribution(
                [r["budget"]["conditional_vertical_95_plus_allowances_m"] for r in good]
            ),
            geo_available=(output / "camera-geo.txt").exists(),
            vertical_datum="WGS84 ellipsoidal height",
            output_hashes={
                p.name: sha256_file(p) for p in output.iterdir() if p.name != "report.json"
            },
        )
    except BaseException as error:
        report.update(status="failed", error=str(error))
        raise
    finally:
        write_json(output / "report.json", report)
    return report
