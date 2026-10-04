"""Read-only OAK/PX4 clock and mounting drift checks for recorded sessions."""

import math

import numpy as np

from .calibration_imu import InsufficientMotion, rotation_vector, solve_session
from .rig_calibration import BODY, OAK_IMU, RigCalibration, check, xyzw_from_rotation


def thresholds(warn_sigma, fail_sigma):
    if not all(math.isfinite(v) for v in (warn_sigma, fail_sigma)) or not 0 < warn_sigma < fail_sigma:
        raise ValueError("Calibration thresholds require 0 < warn_sigma < fail_sigma and finite values")
    return {"warn_sigma": float(warn_sigma), "fail_sigma": float(fail_sigma)}


def compare(result, calibration, *, warn_sigma=3.0, fail_sigma=5.0):
    """Compare independent estimates in body axes using combined one-sigma errors.

    The rotation score is the length of the three normalized small-angle errors;
    offset uses absolute error divided by combined sigma. Unknown reference sigmas
    prevent qualification; translation uncertainty is irrelevant to this check.
    """
    limits = thresholds(warn_sigma, fail_sigma)
    report = {"status": "unqualified", "thresholds": limits,
              "estimated": {"offset_ns": int(result["offset_ns"]),
                            "offset_sigma_ns": int(result["offset_sigma_ns"]),
                            "rotation_xyzw": xyzw_from_rotation(result["rotation"]),
                            "rotation_sigma_rad": list(result["rotation_sigma_rad"])},
              "recommendation": "Supply a calibrated OAK-to-PX4 rotation and offset with uncertainties."}
    transform = calibration.transforms.get((BODY, OAK_IMU))
    offset = calibration.offsets.get(("oak_ros_stamp", "px4_ros_stamp"))
    entry = next((entry for entry in calibration.data["transforms"]
                  if (entry["parent"], entry["child"]) == (BODY, OAK_IMU)), {})
    rotation_sigma = entry.get("rotation_sigma_rad")
    if transform is None or offset is None or offset["sigma_ns"] is None or rotation_sigma is None:
        report["reason"] = "Stored calibration lacks the OAK/PX4 rotation, offset or their uncertainty"
        return report
    offset_error = int(result["offset_ns"] - offset["offset_ns"])
    offset_sigma = math.hypot(offset["sigma_ns"], result["offset_sigma_ns"])
    delta = rotation_vector(result["rotation"] @ transform.rotation.T)
    combined = np.hypot(rotation_sigma, result["rotation_sigma_rad"])
    if offset_sigma <= 0 or np.any(combined <= 0):
        report["reason"] = "Combined calibration uncertainty must be positive"
        return report
    rotation_score = float(np.linalg.norm(delta / combined))
    offset_score = abs(offset_error) / offset_sigma
    score = max(rotation_score, offset_score)
    report.update(status="fail" if score >= fail_sigma else "warn" if score >= warn_sigma else "pass",
                  stored={"offset_ns": offset["offset_ns"], "offset_sigma_ns": offset["sigma_ns"],
                          "rotation_xyzw": xyzw_from_rotation(transform.rotation),
                          "rotation_sigma_rad": rotation_sigma},
                  difference={"offset_ns": offset_error, "offset_combined_sigma_ns": offset_sigma,
                              "offset_sigmas": offset_score, "rotation_vector_rad": delta.tolist(),
                              "rotation_angle_rad": float(np.linalg.norm(delta)),
                              "rotation_combined_sigma_rad": combined.tolist(), "rotation_sigmas": rotation_score})
    report["recommendation"] = ("No calibration drift detected." if report["status"] == "pass" else
                                "Inspect the mounting and clocks and record a new calibration session.")
    return report


def check_session(session, calibration, *, warn_sigma=3.0, fail_sigma=5.0, **solver_options):
    """Never mutate a stored calibration, including when a drift threshold is exceeded."""
    limits = thresholds(warn_sigma, fail_sigma)
    try:
        rig = calibration if isinstance(calibration, RigCalibration) else RigCalibration.read(calibration)
        check(rig, session)  # device identity must agree before comparing estimates
        result = solve_session(session, **solver_options)
        report = compare(result, rig, **limits)
        report["evidence"] = {"seal_sha256": result["seal_sha256"],
                              "excitation": result["excitation"], "holdout": result["holdout"],
                              "residual_rad_s": result["residual_rad_s"]}
        return report
    except InsufficientMotion as error:
        return {"status": "insufficient_motion", "reason": str(error), "thresholds": limits,
                "recommendation": "Record longer rotation about all three axes to check calibration drift."}
    except (ValueError, OSError) as error:
        return {"status": "fail", "reason": str(error), "thresholds": limits,
                "recommendation": "Resolve the recording or calibration error before checking drift."}


def add_to_validation(report, session, calibration, **options):
    """Attach drift qualification without upgrading an invalid or unqualified bag audit."""
    drift = check_session(session, calibration, **options)
    report["calibration_check"] = drift
    status = drift["status"]
    if status == "fail":
        report["errors"].append("Calibration drift check failed: " + drift.get("reason", drift["recommendation"]))
        report["valid"] = report["survey_ready"] = False
    elif status != "pass":
        report["warnings"].append(f"Calibration drift check: {status}: " + drift.get("reason", drift["recommendation"]))
    return report
