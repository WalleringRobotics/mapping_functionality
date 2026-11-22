"""Coverage of a field by a finished Survey plan, independent of how its lines were made.

``survey-check`` confirms a plan is consistent with the camera and profile. It does not
confirm that the lines cover the field. This module reads only the saved plan and counts,
for a grid of ground points on the field, the flight lines that see each point with
enough frames. It imports nothing from the line generator.
"""

import math

import numpy as np

from .survey_plan import EARTH_RADIUS_M, load_plan, local_xy, mission_speed


def _views(overlap):
    return max(1, math.floor(1 / (1 - overlap / 100) + 1e-9))


def _inside(polygon, x, y):
    """Vectorised even-odd test of points against polygon vertices (N, 2)."""
    inside = np.zeros(x.shape, dtype=bool)
    for (x0, y0), (x1, y1) in zip(polygon, np.roll(polygon, -1, axis=0)):
        if y0 == y1:
            continue
        crosses = (y0 > y) != (y1 > y)
        inside ^= crosses & (x < x0 + (y - y0) * (x1 - x0) / (y1 - y0))
    return inside


def _edge_samples(polygon, step):
    points = []
    for start, end in zip(polygon, np.roll(polygon, -1, axis=0)):
        count = max(1, math.ceil(float(np.hypot(*(end - start))) / step))
        points.extend(start + (end - start) * n / count for n in range(count))
    return np.array(points)


def check_coverage(plan_path, boundary, spec, profile, *, relief_m, acceptance_radius_m,
                   front, side, grid_m=1.0):
    """Count lines and frames seeing each ground sample of ``boundary`` (lat, lon vertices).

    A line sees a sample when the sample lies within the across-track footprint and at
    least the required number of frames contain it. Frames are spaced at speed over frame
    rate with the worst-case phase, and are counted only from the leg entry to the exit
    less the acceptance radius, the window ``survey-legs`` keeps. Footprints use the
    lowest height above ground, plan height minus relief, where they are smallest.
    Returns the report and the grid of line counts (rows north to south, -1 off the field).
    """
    loaded = load_plan(plan_path)
    home, items = loaded["home"], loaded["items"]
    lines_needed, frames_needed = _views(side), _views(front)
    report = {"kind": "survey_coverage_check", "passed": False, "errors": list(loaded["errors"]),
              "lines_needed": lines_needed, "frames_needed": frames_needed, "grid_m": grid_m,
              "relief_m": relief_m, "acceptance_radius_m": acceptance_radius_m}
    speed = mission_speed(items)
    if speed is None:
        report["errors"].append("Plan has no commanded speed; frame spacing is unknown")
    if not loaded["legs"]:
        report["errors"].append("Plan has no survey legs")
    polygon = np.array([local_xy(home, vertex) for vertex in boundary])
    west, south = polygon.min(axis=0)
    east, north = polygon.max(axis=0)
    columns = max(1, math.ceil((east - west) / grid_m - 1e-6))
    rows = max(1, math.ceil((north - south) / grid_m - 1e-6))
    grid_x, grid_y = np.meshgrid(west + (np.arange(columns) + 0.5) * grid_m,
                                 north - (np.arange(rows) + 0.5) * grid_m)
    mask = _inside(polygon, grid_x, grid_y)
    edge = _edge_samples(polygon, grid_m)
    x = np.concatenate([grid_x[mask], edge[:, 0]])
    y = np.concatenate([grid_y[mask], edge[:, 1]])
    lines = np.zeros(x.shape, dtype=int)
    frames = np.zeros(x.shape, dtype=int)
    by_seq = {item["mission_seq"]: item for item in items}
    if not report["errors"]:
        frame_spacing = speed / profile["fps"]
        for leg in loaded["legs"]:
            calc = loaded["surveys"][leg["survey"]]["calc"]
            low = float(calc["DistanceToSurface"]) - relief_m
            if low <= 0:
                report["errors"].append("Relief is not below the survey height")
                break
            gsd = low * spec["SensorWidth"] / (spec["ImageWidth"] * spec["FocalLength"])
            across, along = ((spec["ImageWidth"], spec["ImageHeight"]) if spec["Landscape"]
                             else (spec["ImageHeight"], spec["ImageWidth"]))
            half_across, half_along = across * gsd / 2, along * gsd / 2
            entry, exit_ = (np.array(local_xy(home, (by_seq[leg[key]]["lat"], by_seq[leg[key]]["lon"])))
                            for key in ("entry_seq", "exit_seq"))
            length = float(np.hypot(*(exit_ - entry)))
            if length <= 0:
                continue
            unit = (exit_ - entry) / length
            distance = (x - entry[0]) * unit[0] + (y - entry[1]) * unit[1]
            offset = (x - entry[0]) * unit[1] - (y - entry[1]) * unit[0]
            first = np.maximum(distance - half_along, 0.0)
            last = np.minimum(distance + half_along, length - acceptance_radius_m)
            count = np.floor(np.clip(last - first, 0.0, None) / frame_spacing + 1e-9).astype(int)
            count[np.abs(offset) > half_across + 1e-6] = 0
            frames += count
            lines += count >= frames_needed
        report["frame_spacing_m"] = frame_spacing
    failing = lines < lines_needed
    report.update(samples=int(x.size), failing_samples=int(failing.sum()),
                  min_lines=int(lines.min()), mean_lines=float(lines.mean()), min_frames=int(frames.min()),
                  mean_frames=float(frames.mean()))
    if not report["errors"] and failing.any():
        worst = int(np.argmin(lines))
        report["errors"].append(
            f"{int(failing.sum())} of {x.size} ground samples are seen from fewer than {lines_needed} lines "
            f"with {frames_needed} frames each; the worst is {x[worst]:.1f} m east, {y[worst]:.1f} m north "
            f"of the take-off point with {int(lines[worst])}")
    report["passed"] = not report["errors"]
    raster = np.full(mask.shape, -1, dtype=int)
    raster[mask] = lines[:int(mask.sum())]

    def latlon(east_m, north_m):
        return [home[0] + math.degrees(north_m / EARTH_RADIUS_M),
                home[1] + math.degrees(east_m / (EARTH_RADIUS_M * math.cos(math.radians(home[0]))))]
    report["grid"] = {"rows": int(rows), "columns": int(columns),
                      "south_west": latlon(west, north - rows * grid_m),
                      "north_east": latlon(west + columns * grid_m, north)}
    return report, raster
