"""Offline checks of QGroundControl Survey plans against the OAK capture profile.

No vehicle IO. The planner and its map UI are QGroundControl's Survey pattern; this
module parses the saved ``.plan`` exactly as QGroundControl 5.1 writes it (older
CameraCalc/transect versions are also read), labels every PX4 mission item, and
refuses plans the free-running OAK capture cannot cover with the requested overlap.
"""

import hashlib
import json
import math
from pathlib import Path

EARTH_RADIUS_M = 6_371_008.8
PX4_AUTOPILOT = 12  # MAV_AUTOPILOT_PX4
MULTIROTOR_TYPES = {2: "quadrotor", 13: "hexarotor", 14: "octorotor", 15: "tricopter"}
POSITIONAL_COMMANDS = {16: "waypoint", 17: "loiter_unlimited", 18: "loiter_turns",
                       19: "loiter_time", 21: "land", 22: "takeoff", 4501: "condition_gate"}
CAMERA_COMMANDS = {203: "digicam_control", 206: "set_camera_trigger_distance",
                   214: "set_camera_trigger_interval", 2000: "image_start_capture",
                   2001: "image_stop_capture"}
CHANGE_SPEED, RETURN_TO_LAUNCH, LAND, TAKEOFF = 178, 20, 21, 22
# QGroundControlQmlGlobal::AltitudeFrame, as saved in CameraCalc "DistanceMode".
DISTANCE_MODES = {1: "relative", 2: "absolute", 3: "calc_above_terrain", 4: "terrain"}
# MAV_FRAME classes; PX4 may return the *_INT variant of the frame QGC uploaded.
FRAME_CLASSES = {0: "global", 5: "global", 3: "relative", 6: "relative",
                 10: "terrain", 11: "terrain", 2: "mission"}
CAMERA_KEYS = ("SensorWidth", "SensorHeight", "ImageWidth", "ImageHeight", "FocalLength")
DEFAULT_LIMITS = {"max_height_m": 120.0, "min_height_m": 10.0, "max_blur_px": 0.5,
                  "camera_tolerance": 0.005, "climb_m_s": 3.0, "descent_m_s": 1.0,
                  "land_m_s": 0.7, "waypoint_penalty_s": 2.0, "match_tolerance_m": 0.05}


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def local_xy(origin, point):
    """Small-area east/north metres about origin (lat, lon); <0.1 % error over a few km."""
    lat0 = math.radians(origin[0])
    return (math.radians(point[1] - origin[1]) * EARTH_RADIUS_M * math.cos(lat0),
            math.radians(point[0] - origin[0]) * EARTH_RADIUS_M)


def distance_m(a, b):
    east, north = local_xy(a, b)
    return math.hypot(east, north)


def polygon_area_m2(origin, polygon):
    points = [local_xy(origin, p) for p in polygon]
    return abs(sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1)
                   in zip(points, points[1:] + points[:1]))) / 2


def inside_polygon(origin, polygon, point):
    x, y = local_xy(origin, point)
    vertices = [local_xy(origin, p) for p in polygon]
    inside = False
    for (x0, y0), (x1, y1) in zip(vertices, vertices[1:] + vertices[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def read_camera(path):
    """The reviewed QGC custom-camera values for one capture stream and resolution."""
    data = json.loads(Path(path).read_text())
    if data.get("schema_version") != 1:
        raise ValueError("Camera definition schema_version must be 1")
    spec = data.get("qgc_custom_camera")
    if not isinstance(spec, dict):
        raise ValueError("Camera definition needs a qgc_custom_camera object")
    for key in CAMERA_KEYS:
        if finite(spec.get(key), key) <= 0:
            raise ValueError(f"{key} must be positive")
    if not isinstance(spec.get("Landscape"), bool):
        raise ValueError("Landscape must be true or false")
    if data.get("stream") not in {"rgb", "left", "right"} or not data.get("profile_resolution"):
        raise ValueError("Camera definition must name its stream and profile_resolution")
    return data


def read_profile(path, stream):
    """Frame rate and fixed exposure for one stream of an official-driver YAML profile."""
    from ruamel.yaml import YAML
    parameters = YAML(typ="safe").load(Path(path).read_text())["/oak"]["ros__parameters"]
    settings = parameters[stream]
    fps = finite(settings.get("i_fps"), f"{stream} i_fps")
    if fps <= 0:
        raise ValueError(f"{stream} frame rate must be positive")
    if settings.get("r_set_man_exposure") is not True:
        raise ValueError(f"{stream} uses automatic exposure; motion blur cannot be bounded")
    exposure_us = finite(settings.get("r_exposure"), f"{stream} r_exposure")
    if exposure_us <= 0:
        raise ValueError(f"{stream} exposure must be positive")
    return {"stream": stream, "fps": fps, "exposure_us": exposure_us,
            "resolution": settings.get("i_resolution")}


def camera_calc(survey, errors):
    """Normalise CameraCalc v1/v2 to the v2 fields used here."""
    calc = dict(survey["TransectStyleComplexItem"].get("CameraCalc") or {})
    version = calc.get("version", 0)
    if version in (0, 1):
        follow = bool(survey["TransectStyleComplexItem"].get("FollowTerrain"))
        relative = calc.pop("DistanceToSurfaceRelative", True)
        calc["DistanceMode"] = 3 if follow else (1 if relative else 2)
    elif version != 2:
        errors.append(f"Unsupported CameraCalc version {version}")
    return calc


def flatten(plan, errors, tolerance_m, include_home=False):
    """Return PX4 mission items in upload order, each labelled with its survey role.

    QGroundControl's PX4 plugin does not upload the planned home, so the first plan
    item is PX4 sequence 0. That numbering is verified later against the mission the
    vehicle reports, never assumed for qualification.
    """
    items, surveys, legs = [], [], []
    if include_home:
        home = plan["mission"]["plannedHomePosition"]
        items.append(simple({"command": 16, "frame": 0, "autoContinue": True,
                             "params": [0, 0, 0, 0, *home]}, 0, -1, None, "home"))
    for index, entry in enumerate(plan["mission"]["items"]):
        kind = entry.get("type")
        if kind == "SimpleItem":
            items.append(simple(entry, len(items), index, None, "simple"))
            continue
        if kind != "ComplexItem":
            errors.append(f"Plan item {index} has unknown type {kind!r}")
            continue
        complex_type = entry.get("complexItemType")
        if complex_type != "survey":
            errors.append(f"Plan item {index} is a {complex_type!r} pattern; only Survey is supported")
            continue
        survey_index = len(surveys)
        transect = entry.get("TransectStyleComplexItem") or {}
        raw = transect.get("Items") or []
        if not raw:
            errors.append(f"Survey {survey_index} has no saved mission items; resave it in QGroundControl")
            continue
        first = len(items)
        survey_items = [simple(item, first + n, index, survey_index, "survey") for n, item in enumerate(raw)]
        survey = {"index": survey_index, "plan_index": index, "raw": entry,
                  "calc": camera_calc(entry, errors), "first_seq": first,
                  "last_seq": first + len(raw) - 1}
        survey_legs = label_survey(survey, survey_items, errors, tolerance_m)
        surveys.append(survey)
        legs.extend(survey_legs)
        items.extend(survey_items)
    return items, surveys, legs


def simple(item, seq, plan_index, survey_index, origin):
    params = item.get("params")
    if not isinstance(params, list) or len(params) != 7:
        raise ValueError(f"Mission item at plan index {plan_index} lacks seven params")
    command = item.get("command")
    if type(command) is not int:
        raise ValueError(f"Mission item at plan index {plan_index} has no integer command")
    row = {"mission_seq": seq, "plan_index": plan_index, "survey": survey_index,
           "command": command, "frame": item.get("frame"), "params": params,
           "auto_continue": item.get("autoContinue"),
           "role": "camera_command" if command in CAMERA_COMMANDS else origin, "transect": None}
    if command in POSITIONAL_COMMANDS and params[4] is not None and params[5] is not None:
        row.update(lat=float(params[4]), lon=float(params[5]),
                   alt=None if params[6] is None else float(params[6]))
    return row


def label_survey(survey, items, errors, tolerance_m):
    """Match each positional survey item to QGC's saved transects (entry, exit, turnarounds).

    VisualTransectPoints hold each transect as [entry, exit], or as [turnaround, entry,
    exit, turnaround] when TurnAroundDistance > 0. Terrain-following points between an
    entry and its exit are leg interior points.
    """
    transect = survey["raw"]["TransectStyleComplexItem"]
    if transect.get("HoverAndCapture"):
        errors.append(f"Survey {survey['index']} uses hover-and-capture; the OAK free-runs, "
                      "so fly continuous transects")
        return []
    points = transect.get("VisualTransectPoints") or []
    period = 4 if (transect.get("TurnAroundDistance") or 0) > 0 else 2
    if not points or len(points) % period:
        errors.append(f"Survey {survey['index']} transect points do not form whole transects")
        return []
    expected = []
    for number in range(len(points) // period):
        chunk = [tuple(map(float, p[:2])) for p in points[number * period:(number + 1) * period]]
        roles = ("turnaround", "leg_entry", "leg_exit", "turnaround") if period == 4 else (
            "leg_entry", "leg_exit")
        expected.extend((role, point, number) for role, point in zip(roles, chunk))
    cursor, in_leg, legs, entry = 0, False, [], None
    for item in items:
        if item["role"] == "camera_command":
            continue
        if "lat" not in item:
            errors.append(f"Survey {survey['index']} item {item['mission_seq']} is not a positional "
                          "or camera command")
            continue
        position = (item["lat"], item["lon"])
        if cursor < len(expected) and distance_m(expected[cursor][1], position) <= tolerance_m:
            role, _, number = expected[cursor]
            item.update(role=role, transect=number)
            cursor += 1
            if role == "leg_entry":
                in_leg, entry = True, item
            elif role == "leg_exit":
                in_leg = False
                legs.append({"survey": survey["index"], "transect": number,
                             "entry_seq": entry["mission_seq"], "exit_seq": item["mission_seq"],
                             "length_m": distance_m((entry["lat"], entry["lon"]), position)})
        elif in_leg:
            item.update(role="leg_interior", transect=expected[cursor - 1][2])
        else:
            errors.append(f"Survey {survey['index']} item {item['mission_seq']} does not follow the saved "
                          "transects; resave the plan in QGroundControl")
            return []
    if cursor != len(expected):
        errors.append(f"Survey {survey['index']} items end before all saved transects were matched")
        return []
    return legs


def load_plan(path, tolerance_m=DEFAULT_LIMITS["match_tolerance_m"], *, platform="px4_multirotor"):
    """Parse and label a QGroundControl plan; structural problems become errors."""
    data = Path(path).read_bytes()
    plan = json.loads(data)
    errors = []
    if plan.get("fileType") != "Plan" or plan.get("version") != 1:
        raise ValueError("Not a QGroundControl Plan file version 1")
    mission = plan.get("mission")
    if not isinstance(mission, dict) or mission.get("version") != 2:
        raise ValueError("Plan mission section must be version 2")
    if platform == "px4_multirotor":
        if mission.get("firmwareType") != PX4_AUTOPILOT:
            errors.append("Plan was not made for PX4 firmware (firmwareType 12)")
        if mission.get("vehicleType") not in MULTIROTOR_TYPES:
            errors.append(f"Plan vehicle type {mission.get('vehicleType')} is not a multirotor")
    elif platform == "ardupilot_plane":
        if mission.get("firmwareType") != 3 or mission.get("vehicleType") != 1:
            errors.append("Plan must explicitly identify ArduPilot Plane (firmwareType 3, vehicleType 1)")
    else:
        raise ValueError(f"Unsupported survey platform {platform}")
    home = mission.get("plannedHomePosition")
    if not isinstance(home, list) or len(home) != 3 or not all(
            isinstance(v, (int, float)) and math.isfinite(v) for v in home):
        raise ValueError("Plan needs a finite plannedHomePosition")
    items, surveys, legs = flatten(plan, errors, tolerance_m, platform == "ardupilot_plane")
    if not surveys:
        errors.append("Plan contains no Survey pattern")
    return {"plan": plan, "sha256": sha256_bytes(data), "home": tuple(map(float, home)),
            "items": items, "surveys": surveys, "legs": legs, "errors": errors}


def mission_speed(items):
    speeds = [float(item["params"][1]) for item in items
              if item["command"] == CHANGE_SPEED and item["params"][1] not in (None, -1)
              and float(item["params"][1]) > 0]
    return max(speeds) if speeds else None


def estimate_minutes(home, items, speed, limits, errors, warnings):
    """Conservative time: horizontal at mission speed, vertical at PX4 auto limits, holds and stops."""
    position, altitude, seconds, path = home[:2], 0.0, 0.0, 0.0
    ended = False
    for item in items:
        command = item["command"]
        if command == RETURN_TO_LAUNCH:
            horizontal = distance_m(position, home[:2])
            path += horizontal
            seconds += horizontal / speed + altitude / limits["land_m_s"] + limits["waypoint_penalty_s"]
            position, altitude, ended = home[:2], 0.0, True
            continue
        if "lat" not in item:
            continue
        frame = FRAME_CLASSES.get(item["frame"])
        target = item["alt"] or 0.0
        if frame == "global":
            target -= home[2]
        if command == LAND:
            target = 0.0
        horizontal = distance_m(position, (item["lat"], item["lon"]))
        climb = target - altitude
        rate = limits["climb_m_s"] if climb > 0 else (
            limits["land_m_s"] if command == LAND else limits["descent_m_s"])
        hold = float(item["params"][0] or 0) if command in (16, 19) else 0.0
        seconds += horizontal / speed + abs(climb) / rate + hold + limits["waypoint_penalty_s"]
        path += horizontal
        position, altitude = (item["lat"], item["lon"]), target
        ended = command == LAND
    if not ended:
        warnings.append("Mission does not end with Land or Return; the estimate adds a direct return")
        horizontal = distance_m(position, home[:2])
        path += horizontal
        seconds += horizontal / speed + altitude / limits["land_m_s"]
    return seconds / 60, path


def check_fence(plan, home, items, errors):
    fence = plan.get("geoFence") or {}
    polygons = fence.get("polygons") or []
    circles = fence.get("circles") or []
    inclusions = [p for p in polygons if p.get("inclusion", True)] + [
        c for c in circles if c.get("inclusion", True)]
    report = {"inclusion_polygons": sum(p.get("inclusion", True) for p in polygons),
              "inclusion_circles": sum(c.get("inclusion", True) for c in circles),
              "exclusions": len(polygons) + len(circles) - len(inclusions), "outside": []}
    if not inclusions:
        errors.append("Plan has no inclusion geofence; draw one in QGroundControl")
        return report
    points = [("planned_home", home[:2])] + [
        (f"item {item['mission_seq']}", (item["lat"], item["lon"])) for item in items if "lat" in item]
    for name, point in points:
        def contains(region):
            if "circle" in region:
                circle = region["circle"]
                return distance_m(tuple(circle["center"]), point) <= float(circle["radius"])
            return inside_polygon(home, [tuple(v) for v in region["polygon"]], point)
        inside = any(contains(r) for r in inclusions)
        excluded = any(contains(r) for r in polygons + circles if not r.get("inclusion", True))
        if not inside or excluded:
            report["outside"].append(name)
    if report["outside"]:
        errors.append("Mission points outside the inclusion geofence or inside an exclusion: "
                      + ", ".join(report["outside"][:10]))
    return report


def check_survey(plan_path, camera_path, profile_path, flight_time_budget_min, limits=None,
                 target_gsd_cm=None, gsd_tolerance=0.1):
    """Return a report; ``passed`` is true only when every capture and safety limit holds."""
    limits = {**DEFAULT_LIMITS, **(limits or {})}
    budget = finite(flight_time_budget_min, "Flight-time budget")
    if budget <= 0:
        raise ValueError("Flight-time budget must be positive")
    camera = read_camera(camera_path)
    profile = read_profile(profile_path, camera["stream"])
    loaded = load_plan(plan_path, limits["match_tolerance_m"])
    plan, home, items = loaded["plan"], loaded["home"], loaded["items"]
    errors, warnings = list(loaded["errors"]), []
    if profile["resolution"] != camera["profile_resolution"]:
        errors.append(f"Profile {camera['stream']} resolution {profile['resolution']} does not match the "
                      f"camera definition ({camera['profile_resolution']})")
    spec = camera["qgc_custom_camera"]
    speed = mission_speed(items)
    first_survey = min((s["first_seq"] for s in loaded["surveys"]), default=None)
    speed_before = [i for i in items if i["command"] == CHANGE_SPEED and first_survey is not None
                    and i["mission_seq"] < first_survey and (i["params"][1] or 0) > 0]
    if not speed_before:
        errors.append("Set an explicit flight speed before the survey (QGroundControl Mission Start); "
                      "otherwise PX4 flies at its own cruise speed")
    if speed is None:
        speed = float(plan["mission"].get("hoverSpeed") or 0) or None
    surveys = []
    for survey in loaded["surveys"]:
        calc, transect, number = survey["calc"], survey["raw"]["TransectStyleComplexItem"], survey["index"]
        mode = DISTANCE_MODES.get(calc.get("DistanceMode"))
        row = {"index": number, "mission_seq_range": [survey["first_seq"], survey["last_seq"]],
               "camera_name": calc.get("CameraName"), "distance_mode": mode,
               "legs": sum(leg["survey"] == number for leg in loaded["legs"]),
               "turnaround_m": transect.get("TurnAroundDistance"),
               "trigger_in_turnaround": transect.get("CameraTriggerInTurnAround"),
               "refly_90_degrees": transect.get("Refly90Degrees")}
        surveys.append(row)
        if calc.get("CameraName") != "Custom Camera":
            errors.append(f"Survey {number} camera is {calc.get('CameraName')!r}; select Custom Camera "
                          "and enter the values from the camera definition")
            continue
        for key in CAMERA_KEYS:
            planned, reviewed = calc.get(key), float(spec[key])
            if not isinstance(planned, (int, float)) or abs(planned - reviewed) > limits[
                    "camera_tolerance"] * reviewed:
                errors.append(f"Survey {number} {key} is {planned}, camera definition {reviewed}")
        if calc.get("Landscape") is not spec["Landscape"]:
            errors.append(f"Survey {number} Landscape does not match the mounted camera orientation")
        if mode is None:
            errors.append(f"Survey {number} has unsupported altitude mode {calc.get('DistanceMode')}")
        elif mode == "absolute":
            errors.append(f"Survey {number} altitude is above mean sea level; use relative or terrain "
                          "altitude so the height limit can be checked")
        elif mode == "relative":
            warnings.append(f"Survey {number} height is relative to take-off; ground under the field "
                            "must be level with the take-off point")
        height = calc.get("DistanceToSurface")
        if not isinstance(height, (int, float)) or not math.isfinite(height):
            errors.append(f"Survey {number} has no distance to surface")
            continue
        if not limits["min_height_m"] <= height <= limits["max_height_m"]:
            errors.append(f"Survey {number} height {height} m is outside "
                          f"[{limits['min_height_m']}, {limits['max_height_m']}] m")
        gsd_m = height * spec["SensorWidth"] / (spec["ImageWidth"] * spec["FocalLength"])
        across, along = ((spec["ImageWidth"], spec["ImageHeight"]) if spec["Landscape"]
                         else (spec["ImageHeight"], spec["ImageWidth"]))
        footprint_side, footprint_frontal = across * gsd_m, along * gsd_m
        frontal, side = calc.get("FrontalOverlap"), calc.get("SideOverlap")
        row.update(height_m=height, gsd_cm=gsd_m * 100, footprint_side_m=footprint_side,
                   footprint_frontal_m=footprint_frontal, planned_frontal_overlap=frontal,
                   planned_side_overlap=side, line_spacing_m=calc.get("AdjustedFootprintSide"),
                   area_ha=polygon_area_m2(home, [tuple(p) for p in survey["raw"].get("polygon", [])]) / 1e4,
                   leg_length_m=sum(leg["length_m"] for leg in loaded["legs"] if leg["survey"] == number))
        if isinstance(side, (int, float)):
            expected = footprint_side * (100 - side) / 100
            stored = calc.get("AdjustedFootprintSide")
            if not isinstance(stored, (int, float)) or abs(stored - expected) > 0.01 * expected:
                errors.append(f"Survey {number} line spacing {stored} m disagrees with the camera values "
                              f"({expected:.2f} m)")
        if target_gsd_cm is not None and gsd_m * 100 > target_gsd_cm * (1 + gsd_tolerance):
            errors.append(f"Survey {number} GSD {gsd_m * 100:.2f} cm exceeds target {target_gsd_cm} cm")
        survey_speed = float(transect.get("TerrainFlightSpeed") or 0) or speed
        if survey_speed and isinstance(frontal, (int, float)):
            spacing = survey_speed / profile["fps"]
            achieved = 100 * (1 - spacing / footprint_frontal)
            blur = survey_speed * profile["exposure_us"] * 1e-6 / gsd_m
            row.update(speed_m_s=survey_speed, frame_spacing_m=spacing,
                       frontal_overlap_at_frame_rate=achieved, blur_px=blur)
            if achieved < frontal:
                errors.append(f"Survey {number} forward overlap at {profile['fps']} fps is {achieved:.1f} %, "
                              f"below the planned {frontal} %; fly slower or raise the frame rate")
            if blur > limits["max_blur_px"]:
                errors.append(f"Survey {number} motion blur {blur:.2f} px exceeds {limits['max_blur_px']} px")
        if (transect.get("TurnAroundDistance") or 0) <= 0:
            warnings.append(f"Survey {number} has no turnaround; the vehicle accelerates within each leg")
    triggers = sorted({CAMERA_COMMANDS[i["command"]] for i in items if i["command"] in CAMERA_COMMANDS
                       and not (i["command"] == 206 and not i["params"][0])})
    if triggers:
        warnings.append("Plan contains PX4 camera trigger commands (" + ", ".join(triggers) + "); the OAK "
                        "free-runs and does not use them")
    for item in items:
        alt = item.get("alt")
        if alt is not None and FRAME_CLASSES.get(item["frame"]) in {"relative", "terrain"} and (
                alt > limits["max_height_m"]):
            errors.append(f"Item {item['mission_seq']} altitude {alt} m exceeds {limits['max_height_m']} m")
    fence = check_fence(plan, home, items, errors)
    flight = {"speed_m_s": speed, "frame_rate_hz": profile["fps"], "exposure_us": profile["exposure_us"],
              "budget_minutes": budget}
    if speed:
        minutes, path = estimate_minutes(home, items, speed, limits, errors, warnings)
        flight.update(estimated_minutes=minutes, path_length_m=path)
        if minutes > budget:
            errors.append(f"Estimated flight time {minutes:.1f} min exceeds the {budget} min budget")
    else:
        errors.append("Mission speed is unknown")
    passed = not errors
    return {"schema_version": 1, "kind": "survey_plan_check", "passed": passed,
            "errors": errors, "warnings": warnings,
            "plan": {"sha256": loaded["sha256"], "file": "survey.plan",
                     "ground_station": plan.get("groundStation"),
                     "vehicle_type": MULTIROTOR_TYPES.get(plan["mission"].get("vehicleType")),
                     "mission_items": len(items), "planned_home": list(home)},
            "inputs": {"camera": camera.get("name"), "camera_sha256": sha256_bytes(Path(camera_path).read_bytes()),
                       "profile_sha256": sha256_bytes(Path(profile_path).read_bytes()),
                       "stream": camera["stream"], "limits": limits, "target_gsd_cm": target_gsd_cm},
            "surveys": surveys, "flight": flight, "geofence": fence,
            "mission_items": [{k: item.get(k) for k in ("mission_seq", "plan_index", "survey", "command",
                                                        "frame", "role", "transect", "lat", "lon", "alt")}
                              for item in items],
            "legs": loaded["legs"], "flight_ready": False,
            "limitations": [
                "PX4 numbering is assumed zero-based from the first plan item until the recorded vehicle "
                "mission confirms it",
                "Relative height is above take-off, not terrain clearance",
                "Flight time is a planning estimate; wind, battery state and PX4 tuning are not modelled",
                "RGB rolling-shutter readout is unmeasured (#14); blur covers exposure only",
                "A passing check is not airspace, site or operator approval"]}


def write_check(plan_path, output, report):
    """Publish the report beside an exact copy of the checked plan, refusing replacement."""
    data = Path(plan_path).read_bytes()
    if sha256_bytes(data) != report["plan"]["sha256"]:
        raise ValueError("Plan changed after checking")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    (output / "survey.plan").write_bytes(data)
    (output / "survey-check.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    return output


def add_arguments(parser):
    parser.add_argument("plan", type=Path, help="Plan saved by QGroundControl")
    parser.add_argument("--camera", type=Path, default=Path("configs/qgc-oak-rgb-12mp.json"))
    parser.add_argument("--profile", type=Path, default=Path("configs/oakd-ros.yaml"))
    parser.add_argument("--handoff", type=Path, help="Versioned design handoff for ArduPilot Plane")
    parser.add_argument("--aircraft-limits", type=Path, help="Pack, payload, wind and flight limits for Plane")
    parser.add_argument("--flight-time-budget-min", type=float,
                        help="Usable flight minutes after the battery reserve")
    parser.add_argument("--output", type=Path, required=True, help="New directory for the checked plan")
    parser.add_argument("--max-height-m", type=float, default=DEFAULT_LIMITS["max_height_m"])
    parser.add_argument("--max-blur-px", type=float, default=DEFAULT_LIMITS["max_blur_px"])
    parser.add_argument("--target-gsd-cm", type=float)


def run(args):
    if args.handoff is not None:
        from .survey_plane import check_plane, write_plane_check
        if args.aircraft_limits is None:
            raise ValueError("Plane checking requires --aircraft-limits")
        report = check_plane(args.plan, args.handoff, args.aircraft_limits)
        write_plane_check(args.plan, args.handoff, args.aircraft_limits, args.output, report)
        return report
    if args.aircraft_limits is not None or args.flight_time_budget_min is None:
        raise ValueError("PX4 checking requires --flight-time-budget-min; Plane requires --handoff")
    report = check_survey(args.plan, args.camera, args.profile, args.flight_time_budget_min,
                          {"max_height_m": args.max_height_m, "max_blur_px": args.max_blur_px},
                          args.target_gsd_cm)
    write_check(args.plan, args.output, report)
    return report
