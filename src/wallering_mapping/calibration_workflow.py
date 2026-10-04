"""Publish reviewed camera calibration estimates outside their immutable source session."""

import shutil
import tempfile
from pathlib import Path

from .calibration_camera import solve_camera_imu
from .dataset import write_json
from .rig_calibration import RigCalibration, apply_entries, check


def solve_camera(session, calibration_path, output, **options):
    session, output = Path(session), Path(output)
    if output.exists():
        raise ValueError("Output must be a new directory")
    if output.resolve().is_relative_to(session.resolve()):
        raise ValueError("Write calibration results outside the immutable source session")
    calibration = RigCalibration.read(calibration_path)
    check(calibration, session)
    solved = solve_camera_imu(session, **options)
    if not solved["qualified"]:
        raise ValueError("Camera calibration requires a successful independent stereo cross-check")
    updated = apply_entries(calibration.data, solved["entries"])
    calibration_report = check(RigCalibration(updated), session)
    consistency = calibration_report["direct_vs_imu_chain"]
    if consistency is not None and consistency.get("consistent_3_sigma") is False:
        raise ValueError("Solved IMU chain disagrees with the manual camera mounting beyond 3 sigma")
    report = {"camera_to_imu": solved, "calibration": calibration_report,
              "outputs": {"calibration": str(output / "rig-calibration.json")}}
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=".calibrate-camera-", dir=output.parent))
    try:
        write_json(staging / "rig-calibration.json", updated)
        write_json(staging / "report.json", report)
        staging.rename(output)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return report
