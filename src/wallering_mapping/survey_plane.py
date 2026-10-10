"""Conservative offline Plane checks through the existing QGC planner connection.

Only continuous Survey missions with explicit airspeed and relative, level-terrain
altitudes are implemented. Other trajectories remain refused, never approximated
into flight approval. No vehicle connection or autopilot parameters are written.
"""

import hashlib
import json
import math
from pathlib import Path
import re

from .capabilities import camera_definition, fingerprints, fraction, load_handoff, number
from .dataset import sha256_file, write_json
from .survey_plan import (
    CAMERA_KEYS, DEFAULT_LIMITS, FRAME_CLASSES, distance_m, load_plan,
    local_xy, write_check,
)


def load_aircraft(path, handoff_hash):
    data = json.loads(Path(path).read_text())
    if (data.get("schema") != "wallering.plane-limits/v1"
            or type(data.get("schema_version")) is not int or data["schema_version"] != 1):
        raise ValueError("Expected wallering.plane-limits/v1")
    if data.get("handoff_sha256") != handoff_hash:
        raise ValueError("Aircraft limits bind a stale handoff hash (camera/payload changed)")
    if not re.fullmatch(r"[0-9a-f]{64}", data.get("calibration_sha256", "")):
        raise ValueError("Aircraft limits require the selected calibration profile SHA-256")
    for key in ("aircraft_id", "pack_id", "payload_id", "calibration_id"):
        if not isinstance(data.get(key), str) or not data[key].strip():
            raise ValueError(f"Aircraft limits require {key}")
    fingerprints(data.get("source_files"))
    if data.get("endurance_basis") not in {"estimated", "measured"}:
        raise ValueError("Endurance must be estimated or measured for this pack/payload, not a design target")
    for key in ("endurance_min", "stall_speed_m_s", "stall_margin", "min_airspeed_m_s",
                "max_airspeed_m_s", "bank_limit_deg", "min_turn_radius_m", "climb_m_s",
                "descent_m_s", "launch_land_allowance_s", "max_agl_m"):
        number(data.get(key), key)
    number(data.get("wind_bound_m_s"), "wind_bound_m_s", zero=True)
    fraction(data.get("reserve_fraction"), "reserve_fraction", zero=False)
    if (data["stall_margin"] < 1 or not 0 < data["bank_limit_deg"] < 80
            or data["min_airspeed_m_s"] > data["max_airspeed_m_s"]):
        raise ValueError("Inconsistent stall margin, bank or airspeed limits")
    terrain = data.get("terrain") or {}
    if (terrain.get("model") != "level_at_home" or not terrain.get("evidence")
            or not isinstance(terrain["evidence"], str)):
        raise ValueError("Supply evidenced level_at_home terrain; terrain-following is not supported yet")
    number(terrain.get("height_uncertainty_m"), "terrain.height_uncertainty_m", zero=True)
    return data


def _regions(plan, home):
    fence = plan.get("geoFence") or {}
    regions = []
    for raw in fence.get("polygons", []):
        points = raw.get("polygon", [])
        if len(points) < 3:
            raise ValueError("Fence polygons need at least three vertices")
        if any(not isinstance(p, list) or len(p) != 2 or any(
                type(v) not in (int, float) or not math.isfinite(v) for v in p)
                or not -80 <= p[0] <= 80 or not -180 <= p[1] <= 180 for p in points):
            raise ValueError("Fence has invalid coordinates")
        vertices = [local_xy(home, p) for p in points]
        if len(set(vertices)) != len(vertices):
            raise ValueError("Repeated geofence vertices are unsupported")
        edges = list(zip(vertices, vertices[1:] + vertices[:1]))
        for index, (a, b) in enumerate(edges):
            for other, (p, q) in enumerate(edges[index+1:], index+1):
                if other == index+1 or (index == 0 and other == len(edges)-1):
                    continue
                d, e = _difference(b, a), _difference(q, p)
                denominator = _cross(d, e)
                if abs(denominator) > 1e-12:
                    t = _cross(_difference(p, a), e) / denominator
                    u = _cross(_difference(p, a), d) / denominator
                    if 0 <= t <= 1 and 0 <= u <= 1:
                        raise ValueError("Self-intersecting geofence polygons are unsupported")
                elif min(_edge_distance(p, a, b), _edge_distance(q, a, b),
                         _edge_distance(a, p, q), _edge_distance(b, p, q)) < 1e-7:
                    raise ValueError("Overlapping geofence polygon edges are unsupported")
        if any(math.hypot(*p) > 20_000 for p in vertices):
            raise ValueError("Plane local-path model supports fences within 20 km of home")
        area = abs(sum(_cross(a, b) for a, b in zip(vertices, vertices[1:] + vertices[:1]))) / 2
        if area < 1:
            raise ValueError("Degenerate geofence polygon")
        if type(raw.get("inclusion", True)) is not bool:
            raise ValueError("Fence inclusion must be boolean")
        regions.append((raw.get("inclusion", True), "polygon", vertices))
    for raw in fence.get("circles", []):
        circle = raw["circle"]
        if type(raw.get("inclusion", True)) is not bool:
            raise ValueError("Fence inclusion must be boolean")
        center = circle["center"]
        if (not isinstance(center, list) or len(center) != 2
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in center)
                or not -80 <= center[0] <= 80 or not -180 <= center[1] <= 180
                or distance_m(home, center) + circle["radius"] > 20_000):
            raise ValueError("Fence circle is outside the local-path model")
        regions.append((raw.get("inclusion", True), "circle",
                        (local_xy(home, circle["center"]), number(circle["radius"], "fence radius"))))
    if not any(r[0] for r in regions):
        raise ValueError("Plan has no inclusion geofence")
    return regions


def _cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _difference(a, b):
    return a[0] - b[0], a[1] - b[1]


def _edge_distance(point, a, b):
    delta = _difference(b, a)
    length2 = sum(x * x for x in delta)
    t = 0 if not length2 else max(0, min(1, sum(x * y for x, y in zip(
        _difference(point, a), delta)) / length2))
    return math.dist(point, (a[0] + t * delta[0], a[1] + t * delta[1]))


def _contains(kind, shape, point):
    if kind == "circle":
        center, radius = shape
        return math.dist(center, point) <= radius
    x, y = point
    inside = False
    for a, b in zip(shape, shape[1:] + shape[:1]):
        if _edge_distance(point, a, b) < 1e-7:
            return True
        if (a[1] > y) != (b[1] > y) and x < a[0] + (y-a[1]) * (b[0]-a[0]) / (b[1]-a[1]):
            inside = not inside
    return inside


def _safe(regions, point):
    return (any(include and _contains(kind, shape, point) for include, kind, shape in regions)
            and not any(not include and _contains(kind, shape, point)
                        for include, kind, shape in regions))


def segment_safe(regions, a, b):
    """Check every interval cut by a polygon edge/circle, including thin exclusions."""
    cuts = {0., 1.}
    d = _difference(b, a)
    for _, kind, shape in regions:
        if kind == "polygon":
            for p, q in zip(shape, shape[1:] + shape[:1]):
                edge = _difference(q, p)
                denominator = _cross(d, edge)
                if abs(denominator) > 1e-12:
                    t = _cross(_difference(p, a), edge) / denominator
                    u = _cross(_difference(p, a), d) / denominator
                    if 0 <= t <= 1 and 0 <= u <= 1:
                        cuts.add(t)
                else:
                    # Collinear boundary travel still needs its endpoints checked.
                    norm = sum(v * v for v in d)
                    if norm and abs(_cross(_difference(p, a), d)) < 1e-8:
                        for point in (p, q):
                            t = sum(x * y for x, y in zip(_difference(point, a), d)) / norm
                            if 0 <= t <= 1:
                                cuts.add(t)
        else:
            center, radius = shape
            offset = _difference(a, center)
            aa = sum(v * v for v in d)
            bb = 2 * sum(x * y for x, y in zip(offset, d))
            cc = sum(v * v for v in offset) - radius * radius
            disc = bb * bb - 4 * aa * cc
            if aa and disc >= 0:
                cuts.update(t for t in ((-bb - math.sqrt(disc)) / (2 * aa),
                                        (-bb + math.sqrt(disc)) / (2 * aa)) if 0 <= t <= 1)
    ordered = sorted(cuts)
    cuts.update((lo + hi) / 2 for lo, hi in zip(ordered, ordered[1:]))
    return all(_safe(regions, (a[0] + t * d[0], a[1] + t * d[1])) for t in cuts)


def disk_safe(regions, point, radius):
    """A conservative whole-turn envelope must fit one inclusion and avoid all exclusions."""
    contained = False
    for include, kind, shape in regions:
        inside = _contains(kind, shape, point)
        if kind == "circle":
            center, r = shape
            clearance = abs(r - math.dist(center, point))
        else:
            clearance = min(_edge_distance(point, a, b)
                            for a, b in zip(shape, shape[1:] + shape[:1]))
        if include:
            contained |= inside and clearance >= radius
        elif inside or clearance <= radius:
            return False
    return contained


def check_plane(plan_path, handoff_path, aircraft_path):
    handoff_hash = sha256_file(Path(handoff_path))
    aircraft_hash = sha256_file(Path(aircraft_path))
    handoff, derived = load_handoff(handoff_path)
    limits = load_aircraft(aircraft_path, handoff_hash)
    if sha256_file(Path(handoff_path)) != handoff_hash or sha256_file(Path(aircraft_path)) != aircraft_hash:
        raise ValueError("Plane profiles changed while checking")
    loaded = load_plan(plan_path, platform="ardupilot_plane")
    plan, home, items = loaded["plan"], loaded["home"], loaded["items"]
    errors = list(loaded["errors"])
    camera = camera_definition(handoff)["qgc_custom_camera"]
    capture, mission = handoff["capture"], handoff["mission"]
    for key, value in {**derived["resources"], **derived["geometry"]}.items():
        if key.endswith("_pass") and value is not True:
            errors.append(f"Handoff {key} is {value}; capture budget is not established")
    if plan.get("rallyPoints", {}).get("points"):
        errors.append("Rally-point return paths are unsupported")
    allowed = {16, 21, 22, 178, 206, 2000, 2001}
    speed = None
    speeds = []
    navigation = []
    for item in items[1:]:
        seq, command, p = item["mission_seq"], item["command"], item["params"]
        if item["auto_continue"] is not True:
            errors.append(f"Item {seq} does not continue automatically")
        if command not in allowed:
            errors.append(f"Unsupported Plane command {command} at item {seq}")
        if any(v is not None and (type(v) not in (int, float) or not math.isfinite(v)) for v in p):
            errors.append(f"Nonfinite/nonnumeric parameters at item {seq}")
            continue
        if command == 178:
            if p[0] != 0 or p[2] != -1 or p[3] != 0:
                errors.append(f"Item {seq} must set absolute airspeed with unchanged throttle")
            speed = number(p[1], "mission airspeed")
            speeds.append(speed)
        elif command in {16, 21, 22}:
            if ("lat" not in item or not -80 <= item["lat"] <= 80
                    or not -180 <= item["lon"] <= 180 or item["alt"] is None
                    or not math.isfinite(item["alt"])):
                errors.append(f"Invalid navigation position at item {seq}")
                continue
            if FRAME_CLASSES.get(item["frame"]) != "relative":
                errors.append(f"Item {seq} altitude frame must match level-at-home relative terrain")
            if not 0 <= item["alt"] <= limits["max_agl_m"]:
                errors.append(f"Item {seq} altitude outside aircraft limits")
            if command == 16 and p[0] != 0:
                errors.append(f"Waypoint hold at item {seq} needs an explicit modelled loiter")
            if command == 16 and (p[1] != 0 or p[2] != 0):
                errors.append(f"Item {seq} custom acceptance/pass radius is unsupported")
            if item["survey"] is not None and speed is None:
                errors.append("Set explicit airspeed before entering the survey")
            navigation.append(item)
    if not navigation or navigation[0]["command"] != 22 or navigation[-1]["command"] != 21:
        errors.append("Plane requires explicit takeoff and landing; RTL/loiter is not a landing model")
    if sum(i["command"] == 22 for i in navigation) != 1 or sum(i["command"] == 21 for i in navigation) != 1:
        errors.append("Exactly one takeoff and one final landing are supported")
    if not speeds:
        errors.append("Plane airspeed is unknown")
        speeds = [limits["min_airspeed_m_s"]]
    wind = limits["wind_bound_m_s"]
    # Level-turn stall speed rises with sqrt(load factor); check it separately from camera speed.
    bank = math.radians(limits["bank_limit_deg"])
    minimum = max(limits["min_airspeed_m_s"], limits["stall_speed_m_s"] * limits["stall_margin"] /
                  math.sqrt(math.cos(bank)))
    if min(speeds) < minimum or max(speeds) > limits["max_airspeed_m_s"]:
        errors.append(f"Commanded airspeed outside safe aircraft interval [{minimum:.3f}, "
                      f"{limits['max_airspeed_m_s']}] m/s")
    ground_max, ground_min = max(speeds) + wind, min(speeds) - wind
    if ground_min <= 0:
        errors.append("Wind permits zero/negative headway; return time is unbounded")
    if minimum + wind > derived["max_camera_groundspeed_m_s"]:
        errors.append("Wind/airspeed and camera blur/overlap limits are incompatible")
    radius = max(limits["min_turn_radius_m"], max(speeds) ** 2 / (9.80665 * math.tan(bank)))
    # Bound a complete coordinated turn plus the worst drift at the supplied wind bound.
    orbit_seconds = 2 * math.pi * radius / min(speeds)
    envelope = 2 * radius + wind * orbit_seconds
    surveys = []
    for survey in loaded["surveys"]:
        calc, raw = survey["calc"], survey["raw"]["TransectStyleComplexItem"]
        if calc.get("DistanceMode") != 1:
            errors.append("Plane supports relative altitude with evidenced level terrain only")
        if calc.get("CameraName") != "Custom Camera" or calc.get("Landscape") is not True:
            errors.append("Survey camera must match the landscape custom camera from the handoff")
        for key in CAMERA_KEYS:
            value = calc.get(key)
            if type(value) not in (int, float) or not math.isfinite(value) or not math.isclose(
                    value, camera[key], rel_tol=.005):
                errors.append(f"Survey camera {key} differs from the selected SKU geometry")
        agl = number(calc.get("DistanceToSurface"), "survey height")
        if not math.isclose(agl, mission["agl_m"], rel_tol=1e-6):
            errors.append("Survey height differs from the handoff geometry")
        uncertainty = limits["terrain"]["height_uncertainty_m"]
        if agl <= uncertainty or agl + uncertainty > limits["max_agl_m"]:
            errors.append("Terrain uncertainty exceeds altitude clearance/ceiling limits")
        for item in items:
            if item["survey"] == survey["index"] and "alt" in item and not math.isclose(
                    item["alt"], agl, abs_tol=.05):
                errors.append("Saved mission altitude differs from the survey camera calculation")
                break
        gsd = max(1e-12, (agl - uncertainty) * handoff["hardware"]["camera"]["geometry"]["pixel_um"]
                  * 1e-3 / camera["FocalLength"])
        frontal = number(calc.get("FrontalOverlap"), "FrontalOverlap", zero=True) / 100
        side = number(calc.get("SideOverlap"), "SideOverlap", zero=True) / 100
        if not 0 <= frontal < 1 or not 0 <= side < 1:
            errors.append("Survey overlap must be in [0, 100) percent")
        if frontal < mission["forward_overlap"] or side < mission["side_overlap"]:
            errors.append("Survey overlap is below the design handoff requirement")
        achieved = 1 - ground_max / capture["requested_fps"] / (camera["ImageHeight"] * gsd)
        blur = ground_max * mission["exposure_s"] / gsd
        if achieved < frontal or blur > mission["blur_limit_px"] * (1 + 1e-12):
            errors.append("Tailwind groundspeed exceeds camera blur/forward-overlap limits")
        nominal_gsd = agl * camera["SensorWidth"] / (camera["ImageWidth"] * camera["FocalLength"])
        spacing = camera["ImageWidth"] * nominal_gsd * (1 - side)
        if not math.isclose(number(calc.get("AdjustedFootprintSide"), "line spacing"), spacing, rel_tol=.01):
            errors.append("Survey line spacing differs from camera geometry")
        if number(raw.get("TurnAroundDistance"), "turnaround", zero=True) < radius:
            errors.append("Insufficient turnaround distance for the aircraft turn radius")
        surveys.append({"index": survey["index"], "gsd_m": gsd, "blur_px": blur,
                        "frontal_overlap_at_frame_rate": achieved * 100})
    regions = _regions(plan, home)
    path_points = [home[:2]] + [(i["lat"], i["lon"]) for i in navigation]
    xy = [local_xy(home, p) for p in path_points]
    # Include the direct emergency return corridor from every mission vertex.
    segments = list(zip(xy, xy[1:])) + [(p, xy[0]) for p in xy[1:]]
    unsafe = [n for n, (a, b) in enumerate(segments) if not segment_safe(regions, a, b)]
    if unsafe:
        errors.append("Unsafe path segments cross a geofence boundary/exclusion (including return)")
    turn_time = 0.
    tangents = [0.] * len(xy)
    for n in range(1, len(xy) - 1):
        before, after = _difference(xy[n], xy[n-1]), _difference(xy[n+1], xy[n])
        lengths = math.hypot(*before), math.hypot(*after)
        if min(lengths) < 1e-6:
            continue
        cosine = max(-1, min(1, sum(a*b for a, b in zip(before, after)) / math.prod(lengths)))
        angle = math.acos(cosine)
        if angle > 1e-3:
            tangents[n] = radius * math.tan(min(angle, math.pi - 1e-6) / 2)
            turn_time += orbit_seconds * angle / (2 * math.pi)
            if not disk_safe(regions, xy[n], envelope):
                errors.append(f"Turn envelope at path vertex {n} crosses the geofence")
    if any(tangents[n] + tangents[n+1] > math.dist(a, b) + .01
           for n, (a, b) in enumerate(zip(xy, xy[1:]))):
        errors.append("Insufficient segment length/line separation for minimum turn radius")
    horizontal = sum(distance_m(a, b) for a, b in zip(path_points, path_points[1:]))
    altitudes = [0.] + [i["alt"] for i in navigation]
    vertical = sum(abs(b-a) / (limits["climb_m_s"] if b >= a else limits["descent_m_s"])
                   for a, b in zip(altitudes, altitudes[1:]))
    # Add a conservative full return allowance in addition to the planned landing route.
    return_distance = max((distance_m(home, p) for p in path_points), default=0)
    seconds = ((horizontal + return_distance) / ground_min + vertical + turn_time
               + limits["launch_land_allowance_s"]) if ground_min > 0 else None
    usable = limits["endurance_min"] * 60 * (1 - limits["reserve_fraction"])
    if seconds is not None and seconds > usable:
        errors.append("Mission plus return/turn/climb allowances exceeds pack endurance after reserve")
    return {"schema_version": 1, "kind": "survey_plan_check", "platform": "ardupilot_plane",
            "passed": not errors, "errors": errors, "warnings": [], "flight_ready": False,
            "survey_ready": False, "plan": {"sha256": loaded["sha256"], "file": "survey.plan",
                "vehicle_type": "fixed_wing", "mission_items": len(items), "planned_home": list(home)},
            "inputs": {"handoff_sha256": handoff_hash, "aircraft_sha256": aircraft_hash,
                       "calibration_sha256": limits["calibration_sha256"],
                       "source_files": handoff["source_files"], "limits": DEFAULT_LIMITS,
                       "pack_id": limits["pack_id"], "calibration_id": limits["calibration_id"]},
            "flight": {"safe_min_airspeed_m_s": minimum, "max_groundspeed_m_s": ground_max,
                       "min_groundspeed_m_s": ground_min, "turn_radius_m": radius,
                       "turn_envelope_m": envelope, "estimated_minutes": None if seconds is None else seconds/60,
                       "budget_minutes": usable/60, "endurance_basis": limits["endurance_basis"]},
            "geofence": {"unsafe_segments": unsafe}, "surveys": surveys,
            "mission_items": items, "legs": loaded["legs"],
            "limitations": ["ArduPilot sequence 0 is home; the downloaded vehicle mission must confirm numbering",
                "Only level terrain and continuous QGC Survey with explicit takeoff/landing are supported",
                "Conservative turn envelopes are planning bounds, not an autopilot trajectory prediction",
                "QGC/Mission Planner saved Plane fixtures, SITL and operator flight acceptance remain pending",
                "Design intrinsics, timing and cage lever arms are not measured calibration"]}


def write_plane_check(plan, handoff, aircraft, output, report):
    sources = {"mapping_handoff.json": (Path(handoff), "handoff_sha256"),
               "aircraft-limits.json": (Path(aircraft), "aircraft_sha256")}
    snapshots = {}
    for name, (path, key) in sources.items():
        snapshots[name] = path.read_bytes()
        if hashlib.sha256(snapshots[name]).hexdigest() != report["inputs"][key]:
            raise ValueError(f"{name} changed after checking")
    write_check(plan, output, report)
    for name, data in snapshots.items():
        (Path(output) / name).write_bytes(data)
    data = json.loads(snapshots["mapping_handoff.json"])
    write_json(Path(output) / "camera.json", camera_definition(data))
