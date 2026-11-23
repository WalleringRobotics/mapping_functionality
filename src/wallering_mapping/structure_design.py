"""Fly-by design rules: optics, mount, aircraft and structure to stand-off, heights and speed.

No vehicle IO and no geometry on the map. The camera looks along the nose, pitched
down, and the aircraft flies sideways along a wall with the nose square to it.

For a ray ``delta`` below the horizon in the vertical plane of the nose, at stand-off
``d`` from the wall and camera pitch ``q``:

- it meets the wall ``d * tan(delta)`` below the aircraft;
- a pixel covers ``d * cos(delta - q) / (f * cos(delta))`` along the wall and
  ``d * cos(delta - q)**2 / (f * cos(delta)**2)`` up it, with ``f`` in pixels.

So the top of the image looks at the wall most squarely and resolves it best, and the
usable band runs from the top of the image down to the steepest usable ray. Every rule
is evaluated at both ends of the mount's pitch tolerance and the worse value is used.
"""

import math

from .structure_check import optics, requirements_from, views_needed
from .survey_design import TURNAROUND_MARGIN_M, round_down, round_up
from .survey_plan import DEFAULT_LIMITS, finite

BAND_MARGIN_M = 0.1      # Taken off both ends of the usable band on the wall.
EDGE_ALLOWANCE_M = 0.5   # Passes run this far past a wall's ends, plus what the frame count needs.
OVERLAP_ALLOWANCE = 0.98
STANDOFF_RANGE_M = (5.0, 100.0)
FRAMES_PER_STOP = 2
MIN_SPEED_M_S = 0.3


def band_angles(mount, lens, requirements):
    """Highest and steepest usable ray, degrees below the horizon, at the worst pitch for each."""
    focal, _, half_height = lens
    half_view = math.degrees(math.atan(half_height / focal))
    pitch, tolerance = mount["pitch_down_deg"], mount["pitch_tolerance_deg"]
    top = pitch + tolerance - half_view
    bottom = min(pitch - tolerance + half_view, requirements["max_depression_deg"])
    if bottom - top < 5:
        raise ValueError(f"The usable band is only {bottom - top:.1f} degrees high; raise the steepest usable "
                         "ray or correct the camera pitch")
    return top, bottom


def wall_scale(mount, lens, requirements):
    """Per metre of stand-off: coarsest GSD in the band, and finest GSD along the wall (metres)."""
    focal, _, half_height = lens
    half_view = math.degrees(math.atan(half_height / focal))
    pitch, tolerance = mount["pitch_down_deg"], mount["pitch_tolerance_deg"]
    coarsest, finest = 0.0, math.inf
    for q in (pitch - tolerance, pitch, pitch + tolerance):
        steepest = math.radians(min(q + half_view, requirements["max_depression_deg"]))
        coarsest = max(coarsest, math.cos(steepest - math.radians(q)) ** 2 / (focal * math.cos(steepest) ** 2))
        finest = min(finest, math.cos(math.radians(half_view)) / (focal * math.cos(math.radians(q - half_view))))
    return coarsest, finest


def design(spec, profile, aircraft, mount, *, structure_height_m, standoff_m=None, gsd_cm=None,
           requirements=None, limits=None):
    """Apply the fly-by rules and return every derived value with the limit that binds.

    Heights in the result are above the ground at the structure's base.
    """
    limits = {**DEFAULT_LIMITS, **(limits or {})}
    requirements = requirements_from(requirements)
    lens = optics(spec)
    wall = finite(structure_height_m, "Structure height")
    if (standoff_m is None) == (gsd_cm is None):
        raise ValueError("Give exactly one of the stand-off distance and the target GSD")
    coarsest, finest = wall_scale(mount, lens, requirements)
    if gsd_cm is not None:
        if finite(gsd_cm, "Target GSD") <= 0:
            raise ValueError("Target GSD must be positive")
        standoff = round(round_down(gsd_cm / 100 / coarsest, 0.1), 1)
    else:
        standoff = finite(standoff_m, "Stand-off")
    low, high = max(STANDOFF_RANGE_M[0], requirements["min_clearance_m"]), STANDOFF_RANGE_M[1]
    if not low <= standoff <= high:
        raise ValueError(f"Stand-off {standoff:.1f} m is outside [{low}, {high}] m; change the GSD, the "
                         "stand-off or the minimum clearance")

    top, bottom = band_angles(mount, lens, requirements)
    lines_needed = views_needed(requirements["vertical_overlap"])
    frames_needed = views_needed(requirements["front_overlap"])

    def band(distance):
        return (distance * math.tan(math.radians(top)) + BAND_MARGIN_M,
                distance * math.tan(math.radians(bottom)) - BAND_MARGIN_M)

    def lowest_pass(distance):
        above, below = band(distance)
        step = (below - above) * (1 - requirements["vertical_overlap"] / 100)
        first = round(round_up(max(requirements["min_height_m"], above), 0.1), 1)
        return first, step, first + (lines_needed - 1) * step <= below

    # A pass sees the wall from `above` to `below` beneath it. The wall's foot needs the
    # required number of passes no higher than `below`, and none may fly under the minimum.
    above, below = band(standoff)
    first, step, reaches_foot = lowest_pass(standoff)
    if not reaches_foot:
        needed = standoff
        while needed < high and not lowest_pass(needed)[2]:
            needed = round(needed + 0.5, 1)
        raise ValueError(
            f"At {standoff:.1f} m stand-off the foot of the wall is not seen from {lines_needed} passes at or "
            f"above {requirements['min_height_m']} m within {requirements['max_depression_deg']} degrees; "
            f"stand off at least {needed:.1f} m, or lower the minimum height")
    heights = [first]
    while sum(wall + above <= z <= wall + below for z in heights) < lines_needed:
        heights.append(heights[-1] + step)
        if len(heights) > 200:
            raise ValueError("More than 200 passes per face; stand off further")
    # An odd number of passes ends each face at its far corner, where the next one starts.
    if len(heights) % 2 == 0:
        count = len(heights) + 1
        heights = [first + (heights[-1] - first) * n / (count - 1) for n in range(count)]
    heights = [round(z, 2) for z in heights]
    safe = round(round_up(max(heights[-1], wall + requirements["vertical_clearance_m"]), 0.1), 1)

    fps, exposure_s = profile["fps"], profile["exposure_us"] * 1e-6
    width = 2 * lens[1] * standoff * finest  # narrowest strip of wall across the usable band
    speed_limits = {"blur": limits["max_blur_px"] * standoff * finest / exposure_s,
                    "forward_overlap": fps * width * (1 - requirements["front_overlap"] / 100) * OVERLAP_ALLOWANCE,
                    "aircraft": aircraft["max_survey_speed_m_s"]}
    binding = min(speed_limits, key=speed_limits.get)
    speed = round(round_down(speed_limits[binding], 0.1), 1)
    if speed < MIN_SPEED_M_S:
        raise ValueError(f"The {binding} limit allows only {speed_limits[binding]:.2f} m/s; stand off further "
                         "or shorten the exposure")
    spacing = speed / fps
    radius = aircraft["acceptance_radius_m"]
    lead = round(round_up(speed ** 2 / (2 * aircraft["accel_m_s2"]) + radius + TURNAROUND_MARGIN_M, 0.1), 1)
    end_margin = max(0.0, frames_needed * spacing - width / 2 - lead + radius) + EDGE_ALLOWANCE_M
    settle = requirements["yaw_settle_s"]
    return {
        "requested": {"standoff_m": standoff_m, "gsd_cm": gsd_cm},
        "requirements": requirements, "structure_height_m": wall,
        "standoff_m": standoff, "gsd_cm_coarsest": coarsest * standoff * 100,
        "gsd_cm_finest_along_wall": finest * standoff * 100,
        "band_top_deg": top, "band_bottom_deg": bottom,
        "band_below_aircraft_m": [above, below], "band_height_m": below - above,
        "pass_heights_m": heights, "pass_step_m": heights[1] - heights[0] if len(heights) > 1 else 0.0,
        "safe_height_m": safe, "lines_needed": lines_needed, "frames_needed": frames_needed,
        "strip_width_m": width, "speed_m_s": speed, "speed_limits_m_s": speed_limits,
        "binding_speed_limit": binding, "frame_spacing_m": spacing,
        "front_overlap_achieved": 100 * (1 - spacing / width),
        "blur_px": speed * exposure_s / (standoff * finest),
        "lead_m": lead, "end_margin_m": end_margin,
        "settle_hold_s": settle, "stop_hold_s": settle + FRAMES_PER_STOP / fps,
        "sensitivities": {
            "speed_at_half_exposure_m_s": min(2 * speed_limits["blur"], speed_limits["forward_overlap"],
                                              speed_limits["aircraft"]),
            "band_shift_per_degree_of_pitch_m": standoff * (
                math.tan(math.radians(bottom)) - math.tan(math.radians(bottom - 1)))},
    }
