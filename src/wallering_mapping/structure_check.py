"""Independent test of a fixed-yaw fly-by plan against a structure (no vehicle IO).

The platform has one camera, fixed to the airframe, facing forward and pitched down.
A fly-by plan is plain waypoints, each carrying a yaw: straight passes flown sideways
along each face of the structure with the nose square to it, and stops around each
corner with the yaw stepped from one face to the next.

This module reads only the finished ``.plan`` and the structure it is meant for. It
imports nothing from the route generator. It simulates the flight, projects every wall
sample into every frame the camera would take, and passes the plan only if each sample
is seen sharply from enough lines, the path keeps its distance from the structure, the
yaw is settled before imaging starts, and the flight fits the time budget.
"""

import json
import math
from pathlib import Path

import numpy as np

from .survey_design import read_aircraft
from .survey_plan import (CHANGE_SPEED, DEFAULT_LIMITS, FRAME_CLASSES, MULTIROTOR_TYPES, PX4_AUTOPILOT,
                          RETURN_TO_LAUNCH, TAKEOFF, check_fence, finite, local_xy, read_camera, read_profile,
                          sha256_bytes, simple)

WAYPOINT = 16
MOUNT_SCHEMA = "wallering.camera-mount/v1"
# What a usable view is, and how close the aircraft may come. Every value is in the report.
REQUIREMENTS = {"front_overlap": 80.0, "vertical_overlap": 60.0, "max_depression_deg": 45.0,
                "max_incidence_deg": 60.0, "max_yaw_step_deg": 30.0, "min_height_m": 5.0,
                "min_clearance_m": 5.0, "vertical_clearance_m": 5.0, "yaw_settle_s": 2.0}
REQUIREMENT_RANGES = {"front_overlap": (50, 95), "vertical_overlap": (30, 90), "max_depression_deg": (20, 70),
                      "max_incidence_deg": (30, 80), "max_yaw_step_deg": (10, 60), "min_height_m": (2, 60),
                      "min_clearance_m": (3, 100), "vertical_clearance_m": (2, 50), "yaw_settle_s": (0.5, 10)}
MIN_FACE_M = 2.0
MAX_SAMPLES = 40000
COLLINEAR_DEG = 1.0
SAME_TRACK_M = 1.0


def read_mount(path):
    """How the camera is fixed to the airframe: forward, pitched down by a stated angle."""
    data = json.loads(Path(path).read_text())
    if data.get("schema") != MOUNT_SCHEMA or data.get("schema_version") != 1:
        raise ValueError(f"Camera mount must be {MOUNT_SCHEMA}, schema_version 1")
    if data.get("direction") != "forward" or data.get("image_width") != "horizontal":
        raise ValueError("Camera mount must face forward with the image width horizontal")
    if not isinstance(data.get("pitch_source"), str) or not data["pitch_source"].strip():
        raise ValueError("Camera mount needs pitch_source: say whether the pitch is designed or measured")
    pitch = finite(data.get("pitch_down_deg"), "pitch_down_deg")
    tolerance = finite(data.get("pitch_tolerance_deg"), "pitch_tolerance_deg")
    if not 5 <= pitch <= 75 or not 0 <= tolerance <= 10:
        raise ValueError("Camera pitch must be in [5, 75] degrees below the horizon, tolerance in [0, 10]")
    return data


def optics(spec):
    """Focal length and half image size in pixels; the image width must be horizontal."""
    if not spec["Landscape"]:
        raise ValueError("Fly-bys need the image width horizontal (Landscape true)")
    return (spec["FocalLength"] * spec["ImageWidth"] / spec["SensorWidth"], spec["ImageWidth"] / 2,
            spec["ImageHeight"] / 2)


def requirements_from(values=None):
    merged = {**REQUIREMENTS, **{k: v for k, v in (values or {}).items() if v is not None}}
    for key, (lower, upper) in REQUIREMENT_RANGES.items():
        if not lower <= finite(merged[key], key) <= upper:
            raise ValueError(f"{key} must be in [{lower}, {upper}]")
    if set(merged) != set(REQUIREMENTS):
        raise ValueError("Unknown requirement: " + ", ".join(sorted(set(merged) - set(REQUIREMENTS))))
    return merged


def views_needed(overlap):
    return max(1, math.floor(1 / (1 - overlap / 100) + 1e-9))


def convex_hull(points):
    """Counter-clockwise convex hull (east, north) without its collinear points."""
    points = sorted({(round(float(x), 4), round(float(y), 4)) for x, y in points})
    if len(points) < 3:
        raise ValueError("Structure outline needs at least three distinct vertices")

    def half(sequence):
        chain = []
        for point in sequence:
            while len(chain) >= 2 and ((chain[-1][0] - chain[-2][0]) * (point[1] - chain[-2][1])
                                       - (chain[-1][1] - chain[-2][1]) * (point[0] - chain[-2][0])) <= 1e-9:
                chain.pop()
            chain.append(point)
        return chain[:-1]

    hull = half(points) + half(points[::-1])
    if len(hull) < 3:
        raise ValueError("Structure outline has no area")
    return hull


def structure_faces(home, outline):
    """The walls to image: each edge of the outline's convex hull, with its outward normal.

    Returns the hull, one face per edge, and whether the outline was concave. A face's
    ``yaw_deg`` is the compass heading that points the nose square at it.
    """
    xy = [local_xy(home, vertex) for vertex in outline]
    hull = convex_hull(xy)
    area = abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(xy, xy[1:] + xy[:1]))) / 2
    hull_area = abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(hull, hull[1:] + hull[:1]))) / 2
    faces = []
    for index, (start, end) in enumerate(zip(hull, hull[1:] + hull[:1])):
        length = math.dist(start, end)
        along = ((end[0] - start[0]) / length, (end[1] - start[1]) / length)
        normal = (along[1], -along[0])
        faces.append({"index": index, "start": start, "end": end, "length_m": length, "along": along,
                      "normal": normal, "yaw_deg": math.degrees(math.atan2(-normal[0], -normal[1])) % 360})
    return hull, faces, hull_area - area > 0.02 * hull_area


def camera_axes(yaw_deg, pitch_deg):
    """Unit vectors of the image's right, down and forward directions in east, north, up."""
    yaw, pitch = math.radians(yaw_deg), math.radians(pitch_deg)
    nose = np.array([math.sin(yaw), math.cos(yaw), 0.0])
    right = np.array([math.cos(yaw), -math.sin(yaw), 0.0])
    up = np.array([0.0, 0.0, 1.0])
    return right, -math.sin(pitch) * nose - math.cos(pitch) * up, math.cos(pitch) * nose - math.sin(pitch) * up


def wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def _observe(positions, samples, face, yaw, pitch, lens, requirements, velocity=None, exposure_s=0.0):
    """For camera positions (P, 3) and wall samples (S, 3): usable view, blur in px, GSD in m.

    A view is usable when the sample is in front of the camera, inside the image width,
    between the top of the image and the row of the steepest usable ray, and seen at
    no more than the allowed angle from the wall's normal.
    """
    focal, half_width, half_height = lens
    right, down, forward = camera_axes(yaw, pitch)
    ray = samples[None, :, :] - positions[:, None, :]
    x, y, z = ray @ right, ray @ down, ray @ forward
    safe = np.where(z > 0.1, z, 1.0)
    u, v = focal * x / safe, focal * y / safe
    lowest = min(half_height, focal * math.tan(math.radians(requirements["max_depression_deg"] - pitch)))
    normal = np.array([*face["normal"], 0.0])
    facing = -(ray @ normal) / np.linalg.norm(ray, axis=2)
    usable = ((z > 0.1) & (np.abs(u) <= half_width + 1e-6) & (v >= -half_height - 1e-6) & (v <= lowest + 1e-6)
              & (facing >= math.cos(math.radians(requirements["max_incidence_deg"])) - 1e-9))

    def image_motion(direction):
        dx, dy, dz = direction @ right, direction @ down, direction @ forward
        return focal * (dx * safe - x * dz) / safe ** 2, focal * (dy * safe - y * dz) / safe ** 2

    blur = np.zeros(z.shape)
    if velocity is not None:
        du, dv = image_motion(-np.asarray(velocity))
        blur = np.hypot(du, dv) * exposure_s
    # Pixels per metre along the wall and up it; the coarser direction sets the GSD.
    a = image_motion(np.array([*face["along"], 0.0]))
    b = image_motion(np.array([0.0, 0.0, 1.0]))
    total = a[0] ** 2 + a[1] ** 2 + b[0] ** 2 + b[1] ** 2
    det = a[0] * b[1] - a[1] * b[0]
    weakest = np.sqrt(np.clip((total - np.sqrt(np.clip(total ** 2 - 4 * det ** 2, 0, None))) / 2, 1e-12, None))
    return usable, blur, 1.0 / weakest


def _slew_seconds(angle_deg, rate, acceleration):
    """Time to turn through an angle with a rate and an acceleration limit."""
    angle = abs(angle_deg)
    if angle < 1e-6:
        return 0.0
    if angle >= rate ** 2 / acceleration:
        return angle / rate + rate / acceleration
    return 2 * math.sqrt(angle / acceleration)


def _direction(a, b):
    delta = np.asarray(b, dtype=float) - np.asarray(a, dtype=float)
    length = float(np.linalg.norm(delta))
    return (delta / length if length > 1e-6 else None), length


def _collinear(first, second):
    return first is not None and second is not None and float(first @ second) >= math.cos(math.radians(COLLINEAR_DEG))


def _leg_velocity(start, end, speed, aircraft):
    """Velocity on a straight leg: the commanded groundspeed, or the climb or descent rate if slower."""
    delta = np.asarray(end, dtype=float) - np.asarray(start, dtype=float)
    horizontal, vertical = float(np.hypot(delta[0], delta[1])), float(delta[2])
    rate = aircraft["climb_m_s"] if vertical > 0 else aircraft["descent_m_s"]
    seconds = max(horizontal / speed, abs(vertical) / rate)
    return (delta / seconds if seconds > 0 else np.zeros(3)), seconds


def simulate(waypoints, takeoff_height, speed, aircraft, limits):
    """Fly the plan: key frames [t, east, north, up, yaw] and one segment per phase.

    Position and yaw are linear between key frames. The yaw turns towards a waypoint's
    yaw while flying to it, as PX4 does, and the aircraft waits there until the turn is
    complete. The yaw is unwrapped so that interpolation turns the short way.
    """
    keys, segments = [], []
    position, yaw, time_s, path = np.zeros(3), float(waypoints[0]["yaw"]) if waypoints else 0.0, 0.0, 0.0

    def key():
        keys.append([round(time_s, 3), *(round(float(c), 3) for c in position), round(yaw, 3)])

    def phase(kind, seconds, **fields):
        nonlocal time_s
        if seconds > 1e-9:
            segments.append({"kind": kind, "t0": round(time_s, 3), "t1": round(time_s + seconds, 3), **fields})
            time_s += seconds
            key()

    key()
    position = np.array([0.0, 0.0, takeoff_height])
    phase("takeoff", takeoff_height / aircraft["climb_m_s"])
    for number, waypoint in enumerate(waypoints):
        target = np.array(waypoint["position"])
        velocity, seconds = _leg_velocity(position, target, speed, aircraft)
        turn = wrap_deg(waypoint["yaw"] - yaw)
        slew = _slew_seconds(turn, aircraft["yaw_rate_deg_s"], aircraft["yaw_accel_deg_s2"])
        heading, length = _direction(position, target)
        path += float(np.hypot(*(target - position)[:2]))
        start, start_yaw, start_time = position.copy(), yaw, time_s
        fields = {"to_seq": waypoint["seq"], "from_seq": waypoints[number - 1]["seq"] if number else None}
        if 1e-9 < slew < seconds:
            position, yaw = start + velocity * slew, start_yaw + turn
            phase("move", slew, turning=True, **fields)
            position = target
            phase("move", seconds - slew, turning=False, **fields)
        else:
            position, yaw = target, start_yaw + turn
            phase("move", seconds, turning=slew > 1e-9, **fields)
            # PX4 does not accept a waypoint until the yaw is within tolerance of its value.
            phase("yaw", slew - (time_s - start_time), seq=waypoint["seq"])
        phase("hold", waypoint["hold"], seq=waypoint["seq"])
        following = waypoints[number + 1]["position"] if number + 1 < len(waypoints) else None
        outgoing = _direction(target, following)[0] if following is not None else None
        if waypoint["hold"] > 0 or not (length < 1e-6 or _collinear(heading, outgoing)):
            phase("settle", limits["waypoint_penalty_s"], seq=waypoint["seq"])
    home = np.array([0.0, 0.0, position[2]])
    distance = float(np.hypot(*position[:2]))
    path += distance
    position = home
    phase("return", distance / speed)
    height = float(position[2])
    position = np.zeros(3)
    phase("land", height / limits["land_m_s"])
    return keys, segments, time_s, path


def _polygon_distance(points, hull):
    """Horizontal distance from points (N, 2) to a convex polygon; zero inside it."""
    hull = np.asarray(hull)
    nearest = np.full(len(points), np.inf)
    inside = np.ones(len(points), dtype=bool)
    for start, end in zip(hull, np.roll(hull, -1, axis=0)):
        edge = end - start
        offset = points - start
        along = np.clip((offset @ edge) / (edge @ edge), 0.0, 1.0)
        nearest = np.minimum(nearest, np.linalg.norm(offset - along[:, None] * edge, axis=1))
        inside &= edge[0] * offset[:, 1] - edge[1] * offset[:, 0] >= 0
    nearest[inside] = 0.0
    return nearest


def _clearance(nodes, hull, top, requirements):
    """Closest approach to the structure of any path point below the safe height above it."""
    closest, where = math.inf, None
    for (first, a), (second, b) in zip(nodes, nodes[1:]):
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        count = max(2, math.ceil(float(np.linalg.norm(b - a)) / 0.25) + 1)
        points = a + (b - a) * np.linspace(0.0, 1.0, count)[:, None]
        low = points[:, 2] < top + requirements["vertical_clearance_m"] - 1e-6
        if not low.any():
            continue
        distance = _polygon_distance(points[low, :2], hull)
        if distance.min() < closest:
            closest, where = float(distance.min()), f"{first} to {second}"
    return closest, where


def _runs(waypoints):
    """Consecutive waypoints that share one yaw: the aircraft does not turn within a run."""
    runs = []
    for waypoint in waypoints:
        if runs and abs(wrap_deg(waypoint["yaw"] - runs[-1][-1]["yaw"])) < 0.01:
            runs[-1].append(waypoint)
        else:
            runs.append([waypoint])
    return runs


def _lines(run, speed, aircraft):
    """Straight lines flown within one run; collinear waypoints in a row are one line."""
    lines, previous = [], None
    for first, second in zip(run, run[1:]):
        heading, length = _direction(first["position"], second["position"])
        if heading is None:
            continue
        if lines and _collinear(previous, heading):
            lines[-1].update(end=np.array(second["position"]), last_seq=second["seq"])
        else:
            velocity, _ = _leg_velocity(first["position"], second["position"], speed, aircraft)
            lines.append({"start": np.array(first["position"]), "end": np.array(second["position"]),
                          "first_seq": first["seq"], "last_seq": second["seq"], "velocity": velocity,
                          "yaw": first["yaw"]})
        previous = heading
    for line in lines:
        line["length"] = float(np.linalg.norm(line["end"] - line["start"]))
    return lines


def _same_track(track, line, unit):
    """Whether a line lies along a track already flown with the same yaw, within a metre."""
    if abs(wrap_deg(track["yaw"] - line["yaw"])) > 0.01 or abs(float(track["unit"] @ unit)) < math.cos(
            math.radians(COLLINEAR_DEG)):
        return False
    offset = line["start"] - track["start"]
    return float(np.linalg.norm(offset - (offset @ track["unit"]) * track["unit"])) < SAME_TRACK_M


def check_structure(plan_path, outline, *, structure_height_m, base_m, camera_path, mount_path, profile_path,
                    aircraft_path, flight_time_budget_min, requirements=None, limits=None, target_gsd_cm=None,
                    gsd_tolerance=0.1, grid_m=0.5):
    """Report on one fly-by plan; ``passed`` only when pointing, clearance and time all hold.

    ``outline`` is the structure's footprint as (lat, lon) vertices. ``structure_height_m``
    is the wall height above the ground at its base and ``base_m`` is that ground relative
    to the take-off point. Returns the report and, per face, the grid of lines that see
    each wall sample (rows top to bottom).
    """
    limits = {**DEFAULT_LIMITS, **(limits or {})}
    requirements = requirements_from(requirements)
    budget = finite(flight_time_budget_min, "Flight-time budget")
    wall, base = finite(structure_height_m, "Structure height"), finite(base_m, "Base height")
    if budget <= 0 or not 1 <= wall <= 100 or abs(base) > 100:
        raise ValueError("Budget must be positive, structure height in [1, 100] m and base within 100 m of take-off")
    camera, mount = read_camera(camera_path), read_mount(mount_path)
    profile, aircraft = read_profile(profile_path, camera["stream"]), read_aircraft(aircraft_path)
    lens = optics(camera["qgc_custom_camera"])
    data = Path(plan_path).read_bytes()
    plan = json.loads(data)
    if plan.get("fileType") != "Plan" or plan.get("version") != 1:
        raise ValueError("Not a QGroundControl Plan file version 1")
    mission = plan.get("mission")
    if not isinstance(mission, dict) or mission.get("version") != 2:
        raise ValueError("Plan mission section must be version 2")
    home = mission.get("plannedHomePosition")
    if not isinstance(home, list) or len(home) != 3 or not all(
            isinstance(v, (int, float)) and math.isfinite(v) for v in home):
        raise ValueError("Plan needs a finite plannedHomePosition")
    home = tuple(map(float, home))
    errors, warnings = [], []
    if profile["resolution"] != camera["profile_resolution"]:
        errors.append(f"Profile {camera['stream']} resolution {profile['resolution']} does not match the "
                      f"camera definition ({camera['profile_resolution']})")
    if mission.get("firmwareType") != PX4_AUTOPILOT:
        errors.append("Plan was not made for PX4 firmware (firmwareType 12)")
    if mission.get("vehicleType") not in MULTIROTOR_TYPES:
        errors.append(f"Plan vehicle type {mission.get('vehicleType')} is not a multirotor")

    items = []
    for index, entry in enumerate(mission.get("items") or []):
        if entry.get("type") != "SimpleItem":
            errors.append(f"Plan item {index} is a {entry.get('complexItemType') or entry.get('type')!r} pattern; "
                          "a fly-by plan is plain waypoints, because a pattern cannot carry a yaw")
            continue
        items.append(simple(entry, len(items), index, None, "simple"))
    commands = [item["command"] for item in items]
    for item in items:
        if item["command"] not in (CHANGE_SPEED, TAKEOFF, WAYPOINT, RETURN_TO_LAUNCH):
            errors.append(f"Item {item['mission_seq']} has command {item['command']}; a fly-by plan holds only "
                          "a speed, a take-off, waypoints and a return")
        if item["auto_continue"] is not True:
            errors.append(f"Item {item['mission_seq']} has an unbounded manual continuation")
    speeds = {float(item["params"][1]) for item in items if item["command"] == CHANGE_SPEED}
    absolute = all(item["params"][0] == 1 and item["params"][2] == -1 and item["params"][3] == 0
                   for item in items if item["command"] == CHANGE_SPEED)
    first_waypoint = commands.index(WAYPOINT) if WAYPOINT in commands else len(commands)
    if len(speeds) != 1 or not absolute or min(speeds) <= 0 or CHANGE_SPEED not in commands[:first_waypoint]:
        errors.append("Plan needs exactly one absolute groundspeed, commanded before the first waypoint")
    speed = max(speeds) if len(speeds) == 1 and min(speeds) > 0 else None
    if commands.count(TAKEOFF) != 1 or TAKEOFF not in commands[:first_waypoint]:
        errors.append("Plan needs one take-off before the first waypoint")
    if not commands or commands[-1] != RETURN_TO_LAUNCH or commands.count(RETURN_TO_LAUNCH) != 1:
        errors.append("Plan must end with one Return")

    hull, faces, concave = structure_faces(home, outline)
    if concave:
        warnings.append("The outline is concave: the passes follow its convex hull, and recessed walls are "
                        "further from the camera than tested")
    for face in faces:
        if face["length_m"] < MIN_FACE_M:
            errors.append(f"Face {face['index']} is {face['length_m']:.1f} m long; simplify the outline so "
                          f"every face is at least {MIN_FACE_M} m")
    top = base + wall

    waypoints = []
    for item in items:
        if item["command"] not in (WAYPOINT, TAKEOFF):
            continue
        if "lat" not in item or item.get("alt") is None or FRAME_CLASSES.get(item["frame"]) != "relative":
            errors.append(f"Item {item['mission_seq']} needs a position and a height relative to take-off")
            continue
        if item["alt"] > limits["max_height_m"]:
            errors.append(f"Item {item['mission_seq']} height {item['alt']} m exceeds {limits['max_height_m']} m")
        if item["alt"] - base < requirements["min_height_m"] - 1e-6 or item["alt"] < 2.0:
            errors.append(f"Item {item['mission_seq']} is {item['alt'] - base:.1f} m above the ground at the "
                          f"structure; the minimum is {requirements['min_height_m']} m")
        if item["command"] == TAKEOFF:
            continue
        yaw, hold = item["params"][3], item["params"][0]
        if not isinstance(yaw, (int, float)) or isinstance(yaw, bool) or not math.isfinite(yaw):
            errors.append(f"Waypoint {item['mission_seq']} carries no yaw; PX4 would turn the nose along "
                          "the path and the camera would not see the structure")
            continue
        waypoints.append({"seq": item["mission_seq"], "yaw": float(yaw) % 360, "hold": float(hold or 0),
                          "position": (*local_xy(home, (item["lat"], item["lon"])), item["alt"])})
    takeoff = next((item for item in items if item["command"] == TAKEOFF and item.get("alt") is not None), None)
    if not waypoints:
        errors.append("Plan has no waypoints with a yaw")
    fence = check_fence(plan, home, items, errors)

    lines_needed = views_needed(requirements["vertical_overlap"])
    frames_needed = views_needed(requirements["front_overlap"])
    nominal, tolerance = mount["pitch_down_deg"], mount["pitch_tolerance_deg"]
    pitches = sorted({nominal - tolerance, nominal, nominal + tolerance})
    exposure_s, fps = profile["exposure_us"] * 1e-6, profile["fps"]
    settle = requirements["yaw_settle_s"]
    flight = {"speed_m_s": speed, "frame_rate_hz": fps, "exposure_us": profile["exposure_us"],
              "budget_minutes": budget}
    pointing = {"pitch_down_deg": nominal, "pitches_tested_deg": pitches, "lines_needed": lines_needed,
                "frames_needed": frames_needed, "grid_m": grid_m}
    report_runs, coverage, timeline, clearance = [], [], None, None

    if not errors:
        # The camera never turns while it images: every run starts with time for the yaw to settle.
        runs = _runs(waypoints)
        steps = [abs(wrap_deg(b[0]["yaw"] - a[0]["yaw"])) for a, b in zip(runs, runs[1:] + runs[:1])]
        pointing["largest_yaw_step_deg"] = max(steps)
        if len(runs) > 1 and max(steps) > requirements["max_yaw_step_deg"] + 0.5:
            errors.append(f"The yaw steps {max(steps):.1f} degrees between neighbouring views; above "
                          f"{requirements['max_yaw_step_deg']} degrees the images of one face may not join "
                          "the next")
        samples_total = sum((math.ceil(f["length_m"] / grid_m) + 1) * (math.ceil(wall / grid_m) + 1) for f in faces)
        if samples_total > MAX_SAMPLES:
            grid_m = grid_m * math.sqrt(samples_total / MAX_SAMPLES)
            pointing["grid_m"] = grid_m
        grids = []
        for face in faces:
            columns, rows = math.ceil(face["length_m"] / grid_m) + 1, math.ceil(wall / grid_m) + 1
            along = np.linspace(0.0, face["length_m"], columns)
            height = np.linspace(top, base, rows)
            east = face["start"][0] + face["along"][0] * along
            north = face["start"][1] + face["along"][1] * along
            points = np.stack([np.tile(east, rows), np.tile(north, rows), np.repeat(height, columns)], axis=1)
            grids.append({"face": face, "shape": (rows, columns), "points": points,
                          "lines": np.full((len(pitches), len(points)), 0),
                          "gsd": np.full((len(pitches), lines_needed, len(points)), np.inf),
                          "frames": np.zeros(len(points), dtype=int), "stops": np.zeros(len(points), dtype=int)})
        sharpest_refused, worst_blur, tracks = None, 0.0, []
        for number, run in enumerate(runs):
            lines = _lines(run, speed, aircraft)
            stops = [w for w in run if w["hold"] >= settle + 1 / fps - 1e-9]
            row = {"run": number, "yaw_deg": run[0]["yaw"], "first_seq": run[0]["seq"], "last_seq": run[-1]["seq"],
                   "settle_hold_s": run[0]["hold"], "stops": [w["seq"] for w in stops], "lines": []}
            report_runs.append(row)
            if run[0]["hold"] < settle - 1e-9:
                errors.append(f"Waypoint {run[0]['seq']} starts a new yaw without a {settle} s hold; the yaw "
                              "may still be settling when imaging starts")
            for line in lines:
                rate = float(np.linalg.norm(line["velocity"]))
                spacing = rate / fps
                step = min(0.1, spacing / 8)
                radius = aircraft["acceptance_radius_m"]
                # Within the acceptance radius of a turn the aircraft is off the line.
                distances = np.arange(radius + step / 2, line["length"] - radius, step)
                unit = (line["end"] - line["start"]) / line["length"]
                positions = line["start"] + distances[:, None] * unit
                entry = {"first_seq": line["first_seq"], "last_seq": line["last_seq"],
                         "length_m": line["length"], "speed_m_s": rate, "frame_spacing_m": spacing,
                         "vertical": bool(abs(unit[2]) > 0.5), "blur_px": 0.0, "samples_seen": 0}
                row["lines"].append(entry)
                track = next((t for t in tracks if _same_track(t, line, unit)), None)
                if track is None:
                    track = {"yaw": line["yaw"], "start": line["start"], "unit": unit, "views": {}}
                    tracks.append(track)
                for number_of_grid, grid in enumerate(grids):
                    for index, pitch in enumerate(pitches):
                        seen = np.zeros(len(grid["points"]), dtype=int)
                        line_gsd = np.zeros(len(grid["points"]))
                        for chunk in range(0, len(positions), 256):
                            usable, blur, gsd = _observe(positions[chunk:chunk + 256], grid["points"], grid["face"],
                                                         line["yaw"], pitch, lens, requirements, line["velocity"],
                                                         exposure_s)
                            sharp = usable & (blur <= limits["max_blur_px"] + 1e-6)
                            if usable.any():
                                entry["blur_px"] = max(entry["blur_px"], float(blur[usable].max()))
                            if sharp.any():
                                worst_blur = max(worst_blur, float(blur[sharp].max()))
                                line_gsd = np.maximum(line_gsd, np.where(sharp, gsd, 0.0).max(axis=0))
                            seen += sharp.sum(axis=0)
                        frames = np.floor(seen * step / spacing + 1e-9).astype(int)
                        enough = frames >= frames_needed
                        if enough.any():
                            held = track["views"].get((number_of_grid, index))
                            fine = np.where(enough, line_gsd, np.inf)
                            track["views"][number_of_grid, index] = (enough, fine) if held is None else (
                                held[0] | enough, np.minimum(held[1], fine))
                        if pitch == nominal:
                            grid["frames"] += frames
                            entry["samples_seen"] += int(enough.sum())
                if entry["blur_px"] > limits["max_blur_px"] + 1e-6:
                    sharpest_refused = max(sharpest_refused or 0.0, entry["blur_px"])
            for stop in stops:
                for grid in grids:
                    usable, _, _ = _observe(np.array([stop["position"]]), grid["points"], grid["face"], stop["yaw"],
                                            nominal, lens, requirements)
                    grid["stops"] += usable[0] * math.floor((stop["hold"] - settle) * fps + 1e-9)
        # Two passes along the same line are one view of the wall, however often it is flown.
        for track in tracks:
            for (number_of_grid, index), (enough, fine) in track["views"].items():
                grid = grids[number_of_grid]
                grid["lines"][index] += enough
                # Keep each sample's finest lines: the GSD of the views the count relies on.
                ranked = np.vstack([grid["gsd"][index], fine[None]])
                ranked.sort(axis=0)
                grid["gsd"][index] = ranked[:lines_needed]
        counts = np.concatenate([grid["lines"].min(axis=0) for grid in grids])
        failing = counts < lines_needed
        relied_on = np.concatenate([grid["gsd"][:, -1, :].ravel() for grid in grids])
        coarsest = float(relied_on[np.isfinite(relied_on)].max()) if np.isfinite(relied_on).any() else 0.0
        pointing.update(samples=int(counts.size), failing_samples=int(failing.sum()), min_lines=int(counts.min()),
                        mean_lines=float(counts.mean()), coarsest_gsd_cm=coarsest * 100,
                        blur_px=worst_blur, blur_px_on_uncounted_legs=sharpest_refused,
                        frames_per_sample_mean=float(np.concatenate([g["frames"] for g in grids]).mean()))
        if failing.any():
            worst = max(grids, key=lambda g: int((g["lines"].min(axis=0) < lines_needed).sum()))
            errors.append(f"{int(failing.sum())} of {counts.size} wall samples are seen sharply from fewer than "
                          f"{lines_needed} lines with {frames_needed} frames each; face {worst['face']['index']} "
                          "is the worst")
        if target_gsd_cm is not None and coarsest * 100 > target_gsd_cm * (1 + gsd_tolerance):
            errors.append(f"GSD of the views relied on is {coarsest * 100:.2f} cm, above the target "
                          f"{target_gsd_cm} cm")
        for grid in grids:
            rows, columns = grid["shape"]
            coverage.append({"face": grid["face"]["index"],
                             "lines": grid["lines"].min(axis=0).reshape(rows, columns),
                             "frames": grid["frames"].reshape(rows, columns),
                             "stops": grid["stops"].reshape(rows, columns)})

        keys, segments, seconds, path = simulate(waypoints, takeoff["alt"], speed, aircraft, limits)
        timeline = {"keys": keys, "segments": segments}
        flight.update(estimated_minutes=seconds / 60, path_length_m=path)
        if seconds / 60 > budget:
            errors.append(f"Estimated flight time {seconds / 60:.1f} min exceeds the {budget} min budget")
        nodes = ([("take-off", (0.0, 0.0, 0.0)), ("take-off", (0.0, 0.0, takeoff["alt"]))]
                 + [(f"item {w['seq']}", w["position"]) for w in waypoints])
        nodes.append(("return", (0.0, 0.0, nodes[-1][1][2])))
        closest, where = _clearance(nodes, hull, top, requirements)
        clearance = {"closest_m": None if math.isinf(closest) else closest, "where": where,
                     "required_m": requirements["min_clearance_m"],
                     "applies_below_m": top + requirements["vertical_clearance_m"]}
        if closest < requirements["min_clearance_m"] - 1e-6:
            errors.append(f"The path comes within {closest:.1f} m of the structure between {where}, below "
                          f"{clearance['applies_below_m']:.1f} m; the minimum is {requirements['min_clearance_m']} m")

    for key in ("climb_m_s", "descent_m_s"):
        if aircraft[key] < limits[key]:
            warnings.append(f"The aircraft's {key} {aircraft[key]} is below the {limits[key]} m/s default")
    report = {
        "schema_version": 1, "kind": "structure_plan_check", "passed": not errors, "errors": errors,
        "warnings": warnings,
        "plan": {"sha256": sha256_bytes(data), "file": "structure.plan",
                 "ground_station": plan.get("groundStation"),
                 "vehicle_type": MULTIROTOR_TYPES.get(mission.get("vehicleType")),
                 "mission_items": len(items), "planned_home": list(home)},
        "inputs": {"camera": camera.get("name"), "camera_sha256": sha256_bytes(Path(camera_path).read_bytes()),
                   "mount_sha256": sha256_bytes(Path(mount_path).read_bytes()),
                   "profile_sha256": sha256_bytes(Path(profile_path).read_bytes()),
                   "aircraft": aircraft["aircraft_id"],
                   "aircraft_sha256": sha256_bytes(Path(aircraft_path).read_bytes()),
                   "outline": [list(vertex) for vertex in outline], "structure_height_m": wall, "base_m": base,
                   "requirements": requirements, "limits": limits, "target_gsd_cm": target_gsd_cm},
        "structure": {"hull_east_north_m": [list(point) for point in hull], "concave_outline": concave,
                      "faces": [{"face": f["index"], "length_m": f["length_m"], "yaw_deg": f["yaw_deg"]}
                                for f in faces]},
        "flight": flight, "pointing": pointing, "runs": report_runs, "clearance": clearance, "geofence": fence,
        "mission_items": [{k: item.get(k) for k in ("mission_seq", "plan_index", "command", "frame", "lat",
                                                    "lon", "alt")}
                          | ({"yaw_deg": item["params"][3], "hold_s": item["params"][0]}
                             if item["command"] == WAYPOINT else {}) for item in items],
        "timeline": timeline, "flight_ready": False,
        "limitations": [
            "Not flown in PX4 SIH: the yaw behaviour is read from PX4 v1.17.0 source, not measured",
            "The structure is taken as its convex outline with vertical walls of one height; the roof, "
            "overhangs and recesses are not tested",
            "Only the structure itself is an obstacle here; trees, wires and neighbouring buildings are not known",
            "The camera pitch is the mount's stated value and tolerance, not a measurement of this airframe",
            "Flight time is a planning estimate; wind, battery state and PX4 tuning are not modelled",
            "RGB rolling-shutter readout is unmeasured (#14); blur covers exposure only",
            "A passing check is not airspace, site or operator approval"]}
    return report, coverage


def write_check(plan_path, output, report):
    """Publish the report beside an exact copy of the checked plan, refusing replacement."""
    data = Path(plan_path).read_bytes()
    if sha256_bytes(data) != report["plan"]["sha256"]:
        raise ValueError("Plan changed after checking")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    (output / "structure.plan").write_bytes(data)
    (output / "structure-check.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    return output


def add_requirement_arguments(parser):
    parser.add_argument("--front", type=float, help="Overlap between frames along a pass, percent (default 80)")
    parser.add_argument("--vertical", type=float, help="Overlap between passes up the wall, percent (default 60)")
    parser.add_argument("--max-depression-deg", type=float,
                        help="Steepest usable ray below the horizon (default 45)")
    parser.add_argument("--max-yaw-step-deg", type=float,
                        help="Largest yaw step between neighbouring views round a corner (default 30)")
    parser.add_argument("--min-height-m", type=float, help="Lowest flight above the ground at the structure (default 5)")
    parser.add_argument("--min-clearance-m", type=float, help="Closest approach to the structure (default 5)")


def requirements_from_arguments(args):
    return {"front_overlap": args.front, "vertical_overlap": args.vertical,
            "max_depression_deg": args.max_depression_deg, "max_yaw_step_deg": args.max_yaw_step_deg,
            "min_height_m": args.min_height_m, "min_clearance_m": args.min_clearance_m}


def add_arguments(parser):
    parser.add_argument("plan", type=Path, help="Fly-by plan to test")
    parser.add_argument("--outline", type=Path, required=True, help="Structure footprint: .kml or .geojson")
    parser.add_argument("--structure-height-m", type=float, required=True,
                        help="Wall height above the ground at its base")
    parser.add_argument("--base-m", type=float, required=True,
                        help="Ground at the structure relative to the take-off point; state 0 if level")
    parser.add_argument("--aircraft", type=Path, required=True,
                        help="Filled copy of configs/px4-multirotor.template.json")
    parser.add_argument("--camera", type=Path, default=Path("configs/qgc-oak-rgb-12mp.json"))
    parser.add_argument("--mount", type=Path, default=Path("configs/oak-forward-mount.json"))
    parser.add_argument("--profile", type=Path, default=Path("configs/oakd-ros.yaml"))
    parser.add_argument("--flight-time-budget-min", type=float, required=True,
                        help="Usable flight minutes after the battery reserve")
    parser.add_argument("--target-gsd-cm", type=float)
    add_requirement_arguments(parser)
    parser.add_argument("--output", type=Path, required=True, help="New directory for the checked plan")


def run(args):
    from .survey_lines import read_boundary
    report, _ = check_structure(
        args.plan, read_boundary(args.outline), structure_height_m=args.structure_height_m, base_m=args.base_m,
        camera_path=args.camera, mount_path=args.mount, profile_path=args.profile, aircraft_path=args.aircraft,
        flight_time_budget_min=args.flight_time_budget_min, requirements=requirements_from_arguments(args),
        target_gsd_cm=args.target_gsd_cm)
    write_check(args.plan, args.output, report)
    report.pop("timeline")
    return report
