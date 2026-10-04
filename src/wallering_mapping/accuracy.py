"""Independent checkpoint statistics in a pre-established metric coordinate frame."""

import csv

import numpy as np


def checkpoints(path):
    with path.open(newline="") as file:
        rows = list(csv.DictReader(file))
    if len(rows) < 3:
        raise ValueError("At least three independent checkpoints required")
    if len({row["id"] for row in rows}) != len(rows):
        raise ValueError("Checkpoint IDs must be unique")
    if any(row["role"] != "check" for row in rows):
        raise ValueError("Only withheld checkpoints (role=check), not fitted control points")
    residuals = np.array([
        [float(row[f"model_{axis}_m"]) - float(row[f"reference_{axis}_m"]) for axis in "xyz"]
        for row in rows
    ])
    if not np.isfinite(residuals).all():
        raise ValueError("Checkpoint coordinates must be finite")
    xy, xyz = np.linalg.norm(residuals[:, :2], axis=1), np.linalg.norm(residuals, axis=1)
    return {
        "points": len(rows), "units": "metres",
        "bias_xyz_m": residuals.mean(axis=0).tolist(),
        "rmse_xyz_axes_m": np.sqrt(np.mean(residuals**2, axis=0)).tolist(),
        "rmse_horizontal_m": float(np.sqrt(np.mean(xy**2))),
        "rmse_3d_m": float(np.sqrt(np.mean(xyz**2))),
        "p95_3d_m": float(np.percentile(xyz, 95)), "max_3d_m": float(xyz.max()),
        "note": "No alignment fitted here. Inputs must already share a metric CRS and vertical datum. "
                "This report does not verify independence or reference survey uncertainty.",
    }

