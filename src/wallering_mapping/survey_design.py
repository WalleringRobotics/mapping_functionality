"""Survey design rules: optics, aircraft and target to height, spacing, speed and margins.

No vehicle IO and no geometry. Formulas follow ``survey_plan.check_survey`` so a
design that satisfies these rules also satisfies the checker.
"""

import json
import math
from pathlib import Path

from .survey_plan import DEFAULT_LIMITS, MULTIROTOR_TYPES, finite

AIRCRAFT_SCHEMA = "wallering.px4-multirotor/v1"
# Field: (PX4 parameter it mirrors, lowest accepted, highest accepted).
AIRCRAFT_FIELDS = {"max_survey_speed_m_s": ("MPC_XY_CRUISE", 0.5, 20.0),
                   "accel_m_s2": ("MPC_ACC_HOR", 0.5, 15.0),
                   "acceptance_radius_m": ("NAV_ACC_RAD", 0.1, 20.0),
                   "climb_m_s": ("MPC_Z_V_AUTO_UP", 0.2, 10.0),
                   "descent_m_s": ("MPC_Z_V_AUTO_DN", 0.2, 10.0),
                   "return_height_m": ("RTL_RETURN_ALT", 0.0, 500.0)}


def read_aircraft(path):
    """A reviewed PX4 multirotor definition; template nulls and out-of-range values fail."""
    data = json.loads(Path(path).read_text())
    if data.get("schema") != AIRCRAFT_SCHEMA or data.get("schema_version") != 1:
        raise ValueError(f"Aircraft definition must be {AIRCRAFT_SCHEMA}, schema_version 1")
    for key in ("aircraft_id", "source"):
        if not isinstance(data.get(key), str) or not data[key].strip():
            raise ValueError(f"Aircraft definition needs {key}: name the vehicle and where its values came from")
    if data.get("vehicle_type") not in MULTIROTOR_TYPES:
        raise ValueError("Aircraft vehicle_type must be a multirotor MAV_TYPE: "
                         + ", ".join(f"{k} ({v})" for k, v in sorted(MULTIROTOR_TYPES.items())))
    for key, (parameter, lower, upper) in AIRCRAFT_FIELDS.items():
        if data.get(key) is None:
            raise ValueError(f"Aircraft definition needs {key} (PX4 {parameter})")
        if not lower <= finite(data[key], key) <= upper:
            raise ValueError(f"Aircraft {key} must be in [{lower}, {upper}]")
    return data


TARGET_TYPES = {"terrain": {"front": 80.0, "side": 70.0, "crosshatch": False},
                "building": {"front": 85.0, "side": 75.0, "crosshatch": True}}
TURNAROUND_MARGIN_M = 2.0
EDGE_ALLOWANCE_M = 0.5  # Added to both derived edge margins; the coverage test has the last word.


def round_down(value, step):
    return math.floor(value / step + 1e-9) * step


def round_up(value, step):
    return math.ceil(value / step - 1e-9) * step


def required_views(overlap):
    """Images that see an interior ground point at this overlap, e.g. 3 at 70 % and 5 at 80 %."""
    return max(1, math.floor(1 / (1 - overlap / 100) + 1e-9))


def footprint(spec, height):
    """GSD in metres and the across- and along-track ground footprint at one height."""
    gsd = height * spec["SensorWidth"] / (spec["ImageWidth"] * spec["FocalLength"])
    across, along = ((spec["ImageWidth"], spec["ImageHeight"]) if spec["Landscape"]
                     else (spec["ImageHeight"], spec["ImageWidth"]))
    return gsd, across * gsd, along * gsd


def design(spec, profile, aircraft, *, relief_m, gsd_cm=None, height_m=None, target="terrain",
           front=None, side=None, limits=None):
    """Apply the planning rules and return every derived value with the limit that binds.

    ``relief_m`` is the terrain relief about take-off level and must be stated. Overlap
    and blur are held over the highest ground (height minus relief); GSD is held over
    the lowest ground (height plus relief).
    """
    limits = {**DEFAULT_LIMITS, **(limits or {})}
    if target not in TARGET_TYPES:
        raise ValueError(f"Target type must be one of {', '.join(TARGET_TYPES)}")
    defaults = TARGET_TYPES[target]
    front = finite(defaults["front"] if front is None else front, "Forward overlap")
    side = finite(defaults["side"] if side is None else side, "Side overlap")
    if not 50 <= front <= 95 or not 30 <= side <= 90:
        raise ValueError("Forward overlap must be in [50, 95] % and side overlap in [30, 90] %")
    relief = finite(relief_m, "Terrain relief")
    if relief < 0:
        raise ValueError("Terrain relief must be zero or positive")
    if (gsd_cm is None) == (height_m is None):
        raise ValueError("Give exactly one of the target GSD and the height")
    if gsd_cm is not None:
        target_gsd = finite(gsd_cm, "Target GSD") / 100
        if target_gsd <= 0:
            raise ValueError("Target GSD must be positive")
        ceiling = target_gsd * spec["ImageWidth"] * spec["FocalLength"] / spec["SensorWidth"]
        height = round(round_down(ceiling - relief, 0.1), 1)
    else:
        height = finite(height_m, "Height")
    low, high = height - relief, height + relief
    if not limits["min_height_m"] <= height <= limits["max_height_m"]:
        raise ValueError(f"Height {height:.1f} m is outside [{limits['min_height_m']}, "
                         f"{limits['max_height_m']}] m; change the GSD or the height")
    if low < limits["min_height_m"]:
        raise ValueError(f"Height {height:.1f} m leaves {low:.1f} m over the highest ground, below "
                         f"{limits['min_height_m']} m; fly higher or correct the relief")
    gsd, across, along = footprint(spec, height)
    gsd_low, across_low, along_low = footprint(spec, low)
    fps, exposure_s = profile["fps"], profile["exposure_us"] * 1e-6

    # QGroundControl derives spacing from the overlap at the nominal height, so the saved
    # overlap is the nominal-height value that gives the spacing needed over high ground.
    saved_side = min(99.0, round_up(100 * (1 - across_low * (1 - side / 100) / across), 0.01))
    spacing = across * (1 - saved_side / 100)
    speed_limits = {"blur": limits["max_blur_px"] * gsd_low / exposure_s,
                    "forward_overlap": fps * along_low * (1 - front / 100),
                    "aircraft": aircraft["max_survey_speed_m_s"]}
    binding = min(speed_limits, key=speed_limits.get)
    speed = round(round_down(speed_limits[binding], 0.1), 1)
    if speed < 0.5:
        raise ValueError(f"The {binding} limit allows only {speed_limits[binding]:.2f} m/s; "
                         "fly higher, shorten the exposure or lower the overlap")
    frame_spacing = speed / fps
    lines_needed, frames_needed = required_views(side), required_views(front)
    radius = aircraft["acceptance_radius_m"]
    turnaround = round(round_up(speed ** 2 / (2 * aircraft["accel_m_s2"]) + radius
                                + TURNAROUND_MARGIN_M, 0.1), 1)
    # Smallest margins that give an edge point the interior's minimum number of views.
    # Across: lines continue this far past the field's sides; each line is flown wherever
    # its footprint touches the field.
    across_extent = max(0.0, (lines_needed - 1) * spacing - across_low / 2) + EDGE_ALLOWANCE_M
    # Along: frames count from the leg entry to the exit less the acceptance radius.
    along_margin = max(radius, frames_needed * frame_spacing - along_low / 2 + radius) + EDGE_ALLOWANCE_M

    def thinned(step):
        return 100 * (1 - step * frame_spacing / along_low)

    return {
        "target": target, "crosshatch": defaults["crosshatch"],
        "requested": {"gsd_cm": gsd_cm, "height_m": height_m, "front_overlap": front,
                      "side_overlap": side, "relief_m": relief},
        "height_m": height, "lowest_height_above_ground_m": low,
        "gsd_cm": gsd * 100, "gsd_cm_worst": footprint(spec, high)[0] * 100,
        "footprint_across_m": across, "footprint_along_m": along,
        "footprint_across_low_m": across_low, "footprint_along_low_m": along_low,
        "line_spacing_m": spacing, "saved_side_overlap": saved_side, "saved_front_overlap": front,
        "speed_m_s": speed, "speed_limits_m_s": speed_limits, "binding_speed_limit": binding,
        "frame_spacing_m": frame_spacing,
        "front_overlap_worst": thinned(1), "blur_px_worst": speed * exposure_s / gsd_low,
        "lines_needed": lines_needed, "frames_needed": frames_needed,
        "turnaround_m": turnaround, "acceleration_distance_m": speed ** 2 / (2 * aircraft["accel_m_s2"]),
        "across_extent_m": across_extent, "along_margin_m": along_margin,
        "sensitivities": {
            "speed_at_half_exposure_m_s": min(2 * speed_limits["blur"], speed_limits["forward_overlap"],
                                              speed_limits["aircraft"]),
            "rolling_shutter_skew_px_per_ms": speed * 1e-3 / gsd_low,
            "front_overlap_every_second_frame": thinned(2),
            "front_overlap_every_third_frame": thinned(3)},
    }
