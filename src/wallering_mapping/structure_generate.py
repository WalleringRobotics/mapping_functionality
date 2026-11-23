"""Generate a checked fly-by plan for a structure, with its review and mission view.

No vehicle IO: nothing here uploads, arms or reads a vehicle. The plan must pass the
independent ``structure-check`` unmodified before anything is written.
"""

import json
import math
import tempfile
from pathlib import Path

from . import __version__
from .structure_check import (add_requirement_arguments, check_structure, optics, read_mount,
                              requirements_from, requirements_from_arguments, structure_faces)
from .structure_design import design
from .structure_route import build_plan, lay_route
from .survey_design import read_aircraft
from .survey_generate import SurveyRefused
from .survey_lines import MAX_SITE_M, to_latlon
from .survey_plan import DEFAULT_LIMITS, local_xy, read_camera, read_profile, sha256_bytes
from .survey_write import encode_plan

SIDES = ("north", "north-east", "east", "south-east", "south", "south-west", "west", "north-west")
IMAGERY = {"name": "Esri World Imagery",
           "url": "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
           "attribution": "Imagery: Esri, Maxar, Earthstar Geographics", "max_zoom": 19}
FILES = ["structure.plan", "structure-check.json", "review.json", "review.geojson", "review.kml", "mission.html"]


def _side(face):
    return SIDES[round(math.degrees(math.atan2(face["normal"][0], face["normal"][1])) % 360 / 45) % 8]


def plan_structure(*, outline, home, structure_height_m, base_m, camera, mount, profile, aircraft,
                   flight_time_budget_min, standoff_m=None, gsd_cm=None, requirements=None,
                   fence_margin_m=20.0, limits=None, grid_m=0.5):
    """Design, route, write and check one fly-by mission; return the bundle or raise SurveyRefused.

    ``outline`` is the structure's footprint as (lat, lon) vertices, ``home`` is
    (lat, lon, AMSL metres). ``structure_height_m`` is the wall height above the ground at
    its base and ``base_m`` that ground relative to the take-off point.
    """
    limits = {**DEFAULT_LIMITS, **(limits or {})}
    requirements = requirements_from(requirements)
    camera_data, mount_data = read_camera(camera), read_mount(mount)
    spec = camera_data["qgc_custom_camera"]
    capture, vehicle = read_profile(profile, camera_data["stream"]), read_aircraft(aircraft)
    if len(home) != 3 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in home):
        raise ValueError("Home must be latitude, longitude and elevation above mean sea level")
    if not 5 <= fence_margin_m <= 200:
        raise ValueError("Fence margin must be in [5, 200] m")
    hull, faces, _ = structure_faces(home, outline)
    if max(math.hypot(*point) for point in hull) > MAX_SITE_M:
        raise ValueError(f"The structure is more than {MAX_SITE_M:.0f} m from the take-off point")

    def attempt(grid, **changes):
        request = {"standoff_m": standoff_m, "gsd_cm": gsd_cm, **changes}
        chosen = design(spec, capture, vehicle, mount_data, structure_height_m=structure_height_m,
                        requirements=requirements, limits=limits, **request)
        if base_m + chosen["safe_height_m"] > limits["max_height_m"]:
            raise ValueError(f"The top pass would be {base_m + chosen['safe_height_m']:.1f} m above take-off, "
                             f"over the {limits['max_height_m']} m height limit; stand off less")
        route = lay_route(home, outline, chosen)
        data = encode_plan(build_plan(home, vehicle, chosen, route, base_m, fence_margin_m))
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "structure.plan"
            path.write_bytes(data)
            check, coverage = check_structure(
                path, outline, structure_height_m=structure_height_m, base_m=base_m, camera_path=camera,
                mount_path=mount, profile_path=profile, aircraft_path=aircraft,
                flight_time_budget_min=flight_time_budget_min, requirements=requirements, limits=limits,
                target_gsd_cm=request["gsd_cm"], grid_m=grid)
        return chosen, route, data, check, coverage

    try:
        chosen, route, data, check, coverage = attempt(grid_m)
    except ValueError as error:
        raise SurveyRefused([str(error)]) from error
    if not check["passed"]:
        alternatives = {}
        estimated = check["flight"].get("estimated_minutes")
        if estimated is not None and estimated > flight_time_budget_min:
            # Further away the camera sees more wall per frame and may fly faster.
            for further in range(1, 41):
                try:
                    other, _, _, quick, _ = attempt(2.0, standoff_m=chosen["standoff_m"] + further, gsd_cm=None)
                except ValueError:
                    break
                if quick["passed"]:
                    alternatives["further_stand_off"] = {
                        "standoff_m": other["standoff_m"], "gsd_cm_coarsest": other["gsd_cm_coarsest"],
                        "estimated_minutes": quick["flight"]["estimated_minutes"]}
                    break
        raise SurveyRefused(check["errors"], alternatives)

    warnings = list(check["warnings"])
    blurred = check["pointing"]["blur_px_on_uncounted_legs"]
    if blurred:
        warnings.append(f"Frames taken on the climbs and descents between passes and stops reach {blurred:.1f} px "
                        "of blur; they are not counted and should be left out of processing")
    if base_m == 0:
        warnings.append("The ground at the structure is stated as level with the take-off point")
    steps, windows = _steps(check["timeline"], route, faces, chosen, base_m)
    review = _review(home, outline, camera, mount, profile, aircraft, camera_data, mount_data, vehicle, chosen,
                     route, faces, check, warnings, data, fence_margin_m, base_m, steps)
    mission = _mission(home, hull, faces, spec, mount_data, capture, chosen, route, check, coverage, steps,
                       windows, review, json.loads(data), base_m)
    return {"plan": data, "check": check, "coverage": coverage, "review": review, "mission": mission,
            "geojson": _geojson(home, outline, hull, route, json.loads(data), base_m)}


def _steps(timeline, route, faces, chosen, base_m):
    """The flight as named steps, and the time windows in which frames are counted."""
    by_seq = {point["seq"]: point for point in route}
    order = {}
    for point in route:
        if "face" in point:
            order.setdefault(point["face"], len(order))
    sides = {face["index"]: _side(face) for face in faces}
    passes = len(chosen["pass_heights_m"])
    stations = {}
    for point in route:
        if point["role"] == "stop":
            stations[point["corner"]] = max(stations.get(point["corner"], 0), point["station"] + 1)

    def group(point):
        role = point["role"]
        if role == "approach":
            return ("transit", f"Fly to face {order[point['face']] + 1}",
                    f"{base_m + point['height_m']:.1f} m, above the top of the structure")
        if role == "lead_in":
            return ("reposition", f"Face {order[point['face']] + 1}: move to pass {point['pass'] + 1}",
                    f"to {base_m + point['height_m']:.1f} m, yaw {point['yaw_deg']:.0f}°")
        if role in ("entry", "exit", "lead_out"):
            return ("pass", f"Face {order[point['face']] + 1} ({sides[point['face']]}): pass {point['pass'] + 1} "
                            f"of {passes}",
                    f"{base_m + point['height_m']:.1f} m, yaw {point['yaw_deg']:.0f}°, sideways at "
                    f"{chosen['speed_m_s']} m/s")
        if role == "stop":
            return ("corner", f"Corner after face {order[point['corner']] + 1}: station {point['station'] + 1} "
                              f"of {stations[point['corner']]}",
                    f"yaw {point['yaw_deg']:.0f}°, {passes} stops of {chosen['stop_hold_s']:.0f} s")
        return ("climb_out", "Climb clear of the structure", f"to {base_m + point['height_m']:.1f} m")

    fixed = {"takeoff": ("takeoff", "Take off", f"climb to {base_m + chosen['safe_height_m']:.1f} m"),
             "return": ("return", "Return", "straight to the take-off point"),
             "land": ("land", "Land", "descend at the take-off point")}
    steps, windows = [], []
    for segment in timeline["segments"]:
        seq = segment.get("to_seq", segment.get("seq"))
        kind, label, detail = fixed[segment["kind"]] if segment["kind"] in fixed else group(by_seq[seq])
        if steps and steps[-1]["label"] == label:
            steps[-1]["t1"] = segment["t1"]
        else:
            steps.append({"kind": kind, "label": label, "detail": detail, "t0": segment["t0"], "t1": segment["t1"]})
        point = by_seq.get(seq)
        if segment["kind"] == "move" and point and point["role"] == "exit":
            windows.append([segment["t0"], segment["t1"], "pass"])
        if segment["kind"] == "hold" and point and point["role"] == "stop":
            windows.append([round(segment["t0"] + chosen["settle_hold_s"], 3), segment["t1"], "stop"])
    return steps, windows


def _review(home, outline, camera, mount, profile, aircraft, camera_data, mount_data, vehicle, chosen, route,
            faces, check, warnings, data, fence_margin_m, base_m, steps):
    flight, pointing = check["flight"], check["pointing"]
    passes = [point for point in route if point["role"] == "entry"]
    stops = [point for point in route if point["role"] == "stop"]
    seconds = {kind: sum(step["t1"] - step["t0"] for step in steps if step["kind"] == kind)
               for kind in sorted({step["kind"] for step in steps})}
    entries = {(point["face"], point["pass"]): point["seq"] for point in route if point["role"] == "entry"}
    exits = {(point["face"], point["pass"]): point["seq"] for point in route if point["role"] == "exit"}
    return {
        "schema_version": 1, "kind": "structure_plan_review", "planner_version": __version__,
        "flight_ready": False, "plan_sha256": sha256_bytes(data),
        "inputs": {
            "outline": [list(vertex) for vertex in outline], "home": list(home),
            "structure_height_m": chosen["structure_height_m"], "base_m": base_m,
            "camera": camera_data.get("name"), "camera_sha256": sha256_bytes(Path(camera).read_bytes()),
            "mount": mount_data.get("name"), "mount_sha256": sha256_bytes(Path(mount).read_bytes()),
            "pitch_down_deg": mount_data["pitch_down_deg"],
            "pitch_tolerance_deg": mount_data["pitch_tolerance_deg"], "pitch_source": mount_data["pitch_source"],
            "profile_sha256": sha256_bytes(Path(profile).read_bytes()),
            "aircraft": vehicle["aircraft_id"], "aircraft_sha256": sha256_bytes(Path(aircraft).read_bytes()),
            "aircraft_values": {key: vehicle[key] for key in sorted(vehicle) if key not in {"schema"}},
            "flight_time_budget_min": flight["budget_minutes"], "fence_margin_m": fence_margin_m},
        "design": chosen,
        "yaw": {"how": "Every waypoint carries a yaw (MAV_CMD_NAV_WAYPOINT param4). PX4 turns to a waypoint's "
                       "yaw while flying to it and does not accept the waypoint until the yaw is within "
                       "MIS_YAW_ERR of it. The first waypoint of each new yaw holds for the settle time.",
                "headings_deg": sorted({point["yaw_deg"] for point in route}),
                "largest_step_deg": pointing["largest_yaw_step_deg"],
                "yaw_rate_while_imaging_deg_s": 0.0},
        "totals": {"faces": len(faces), "passes": len(passes), "corner_stops": len(stops),
                   "waypoints": len(route), "estimated_minutes": flight["estimated_minutes"],
                   "budget_minutes": flight["budget_minutes"], "path_length_m": flight["path_length_m"],
                   "seconds_by_step_kind": seconds,
                   "frames_on_passes": sum(math.floor(row["length_m"] / chosen["frame_spacing_m"]) + 1
                                           for run in check["runs"] for row in run["lines"]
                                           if not row["vertical"]),
                   "max_distance_from_home_m": max(math.hypot(*point["xy"]) for point in route),
                   "highest_point_m": max(base_m + point["height_m"] for point in route),
                   "closest_to_structure_m": check["clearance"]["closest_m"]},
        "faces": [{"face": face["index"], "side": _side(face), "length_m": face["length_m"],
                   "yaw_deg": face["yaw_deg"],
                   "passes": [{"pass": number, "height_m": base_m + height, "entry_seq": entries[face["index"], number],
                               "exit_seq": exits[face["index"], number]}
                              for number, height in enumerate(chosen["pass_heights_m"])]} for face in faces],
        "steps": steps,
        "checks": {"structure_check": {"passed": check["passed"], "file": "structure-check.json"},
                   "pointing": pointing, "clearance": check["clearance"]},
        "warnings": warnings, "files": FILES,
        "limitations": check["limitations"] + [
            "QGroundControl has not opened one of these plans yet; that needs the operator's QGroundControl",
            "The capture workflow (capture --survey-check, survey-legs) does not accept fly-by plans yet",
            "Line of sight stays with the operator: the aircraft passes behind the structure"]}


def _mission(home, hull, faces, spec, mount, capture, chosen, route, check, coverage, steps, windows, review,
             plan, base_m):
    """Everything the mission view draws, in metres east/north/up of the take-off point."""
    focal, half_width, half_height = optics(spec)
    pitch = mount["pitch_down_deg"]
    lowest = min(half_height, focal * math.tan(math.radians(chosen["requirements"]["max_depression_deg"] - pitch)))
    order = {}
    for point in route:
        if "face" in point:
            order.setdefault(point["face"], len(order))
    fence = [list(local_xy(home, tuple(vertex))) for vertex in plan["geoFence"]["polygons"][0]["polygon"]]
    return {
        "title": "Structure fly-by", "plan_sha256": review["plan_sha256"], "passed": check["passed"],
        "home": list(home), "base_m": base_m, "wall_m": chosen["structure_height_m"],
        "hull": [list(point) for point in hull],
        "faces": [{"index": face["index"], "order": order[face["index"]], "side": _side(face),
                   "start": list(face["start"]), "end": list(face["end"]), "along": list(face["along"]),
                   "normal": list(face["normal"]), "length_m": face["length_m"], "yaw_deg": face["yaw_deg"],
                   "rows": grid["lines"].shape[0], "columns": grid["lines"].shape[1],
                   "lines": grid["lines"].ravel().tolist(), "stops": grid["stops"].ravel().tolist()}
                  for face, grid in zip(faces, coverage)],
        "camera": {"focal_px": focal, "half_width_px": half_width, "half_height_px": half_height,
                   "pitch_down_deg": pitch, "lowest_usable_row_px": lowest,
                   "max_incidence_deg": chosen["requirements"]["max_incidence_deg"], "fps": capture["fps"],
                   "exposure_us": capture["exposure_us"], "name": review["inputs"]["camera"]},
        "fence": fence,
        "waypoints": [{"seq": p["seq"], "x": p["xy"][0], "y": p["xy"][1], "z": round(base_m + p["height_m"], 2),
                       "yaw": p["yaw_deg"], "hold": p["hold_s"], "role": p["role"]} for p in route],
        "keys": check["timeline"]["keys"], "steps": steps, "windows": windows,
        "needed": {"lines": chosen["lines_needed"], "frames": chosen["frames_needed"]},
        "facts": {"standoff_m": chosen["standoff_m"], "speed_m_s": chosen["speed_m_s"],
                  "binding_speed_limit": chosen["binding_speed_limit"],
                  "gsd_cm_coarsest": chosen["gsd_cm_coarsest"], "gsd_cm_finest": chosen["gsd_cm_finest_along_wall"],
                  "pass_heights_m": [round(base_m + z, 2) for z in chosen["pass_heights_m"]],
                  "estimated_minutes": review["totals"]["estimated_minutes"],
                  "budget_minutes": review["totals"]["budget_minutes"], "passes": review["totals"]["passes"],
                  "corner_stops": review["totals"]["corner_stops"],
                  "closest_to_structure_m": review["totals"]["closest_to_structure_m"],
                  "min_lines": check["pointing"]["min_lines"], "blur_px": check["pointing"]["blur_px"],
                  "aircraft": review["inputs"]["aircraft"], "pitch_source": mount["pitch_source"],
                  "pitch_tolerance_deg": mount["pitch_tolerance_deg"]},
        "warnings": review["warnings"], "limitations": review["limitations"], "imagery": IMAGERY}


def _geojson(home, outline, hull, route, plan, base_m):
    origin = home[:2]

    def lonlat(xy):
        lat, lon = to_latlon(origin, xy)
        return [round(lon, 9), round(lat, 9)]

    def feature(layer, kind, coordinates, **properties):
        return {"type": "Feature", "properties": {"layer": layer, **properties},
                "geometry": {"type": kind, "coordinates": coordinates}}

    def ring(vertices):
        return [[*vertices, vertices[0]]]

    features = [feature("outline", "Polygon", ring([[lon, lat] for lat, lon in outline])),
                feature("hull", "Polygon", ring([lonlat(xy) for xy in hull])),
                feature("home", "Point", [home[1], home[0]])]
    for polygon in plan["geoFence"]["polygons"]:
        features.append(feature("fence", "Polygon", ring([[lon, lat] for lat, lon in polygon["polygon"]])))
    features.append(feature("path", "LineString",
                            [[home[1], home[0], 0.0]] + [[*lonlat(p["xy"]), round(base_m + p["height_m"], 2)]
                                                         for p in route] + [[home[1], home[0], 0.0]]))
    entries = {(p["face"], p["pass"]): p for p in route if p["role"] == "entry"}
    for point in route:
        height = round(base_m + point["height_m"], 2)
        if point["role"] == "exit":
            entry = entries[point["face"], point["pass"]]
            features.append(feature("passes", "LineString", [lonlat(entry["xy"]), lonlat(point["xy"])],
                                    face=point["face"], height_m=height, yaw_deg=point["yaw_deg"],
                                    entry_seq=entry["seq"], exit_seq=point["seq"], **{"pass": point["pass"]}))
        elif point["role"] == "stop":
            features.append(feature("stops", "Point", lonlat(point["xy"]), corner=point["corner"],
                                    station=point["station"], height_m=height, yaw_deg=point["yaw_deg"],
                                    seq=point["seq"]))
    return {"type": "FeatureCollection", "features": features}


def review_kml(review, geojson):
    """The structure as a block and the path at height, for Google Earth."""
    top = review["inputs"]["structure_height_m"]

    def coordinates(points, height=None):
        return " ".join(f"{p[0]:.9f},{p[1]:.9f},{(p[2] if height is None else height):.1f}" for p in points)

    parts = ['<?xml version="1.0" encoding="UTF-8"?>', '<kml xmlns="http://www.opengis.net/kml/2.2">',
             "<Document>", f"<name>Structure fly-by {review['plan_sha256'][:12]}</name>",
             '<Style id="path"><LineStyle><color>ff00ffff</color><width>2</width></LineStyle></Style>',
             '<Style id="fence"><LineStyle><color>ff0000ff</color><width>2</width></LineStyle></Style>',
             '<Style id="structure"><PolyStyle><color>99ffaa55</color></PolyStyle></Style>']
    for item in geojson["features"]:
        layer, geometry = item["properties"]["layer"], item["geometry"]
        if layer == "hull":
            parts.append("<Placemark><name>structure</name><styleUrl>#structure</styleUrl><Polygon>"
                         "<extrude>1</extrude><altitudeMode>relativeToGround</altitudeMode><outerBoundaryIs>"
                         f"<LinearRing><coordinates>{coordinates(geometry['coordinates'][0], top)}</coordinates>"
                         "</LinearRing></outerBoundaryIs></Polygon></Placemark>")
        elif layer == "fence":
            parts.append("<Placemark><name>fence</name><styleUrl>#fence</styleUrl><LineString><tessellate>1"
                         f"</tessellate><coordinates>{coordinates(geometry['coordinates'][0], 0.0)}</coordinates>"
                         "</LineString></Placemark>")
        elif layer == "path":
            parts.append("<Placemark><name>path</name><styleUrl>#path</styleUrl><LineString>"
                         "<altitudeMode>relativeToGround</altitudeMode>"
                         f"<coordinates>{coordinates(geometry['coordinates'])}</coordinates></LineString></Placemark>")
        elif layer == "home":
            lon, lat = geometry["coordinates"]
            parts.append(f"<Placemark><name>take-off</name><Point><coordinates>{lon:.9f},{lat:.9f},0"
                         "</coordinates></Point></Placemark>")
    parts += ["</Document>", "</kml>", ""]
    return "\n".join(parts)


def mission_html(mission):
    """The mission view: one self-contained page with the mission data embedded."""
    template = Path(__file__).with_name("structure_mission.html").read_text()
    data = json.dumps(mission, allow_nan=False, separators=(",", ":")).replace("</", "<\\/")
    if template.count("__MISSION_DATA__") != 1:
        raise RuntimeError("Mission view template is damaged")
    return template.replace("__MISSION_DATA__", data)


def write_bundle(bundle, output):
    """Publish the checked plan, its review and the mission view in a new directory."""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    (output / "structure.plan").write_bytes(bundle["plan"])
    for name, content in (("structure-check.json", bundle["check"]), ("review.json", bundle["review"]),
                          ("review.geojson", bundle["geojson"])):
        (output / name).write_text(json.dumps(content, indent=2, allow_nan=False) + "\n")
    (output / "review.kml").write_text(review_kml(bundle["review"], bundle["geojson"]))
    (output / "mission.html").write_text(mission_html(bundle["mission"]))
    return output


def add_arguments(parser):
    parser.add_argument("--outline", type=Path, required=True, help="Structure footprint: .kml or .geojson")
    parser.add_argument("--structure-height-m", type=float, required=True,
                        help="Wall height above the ground at its base")
    parser.add_argument("--base-m", type=float, required=True,
                        help="Ground at the structure relative to the take-off point; state 0 if level")
    parser.add_argument("--home", required=True, help="Take-off point as latitude,longitude in degrees")
    parser.add_argument("--home-amsl-m", type=float, required=True, help="Take-off elevation above mean sea level")
    parser.add_argument("--aircraft", type=Path, required=True,
                        help="Filled copy of configs/px4-multirotor.template.json")
    parser.add_argument("--camera", type=Path, default=Path("configs/qgc-oak-rgb-12mp.json"))
    parser.add_argument("--mount", type=Path, default=Path("configs/oak-forward-mount.json"))
    parser.add_argument("--profile", type=Path, default=Path("configs/oakd-ros.yaml"))
    size = parser.add_mutually_exclusive_group(required=True)
    size.add_argument("--standoff-m", type=float, help="Horizontal distance from the walls")
    size.add_argument("--gsd-cm", type=float, help="Coarsest GSD allowed on the walls; sets the stand-off")
    add_requirement_arguments(parser)
    parser.add_argument("--fence-margin-m", type=float, default=20.0)
    parser.add_argument("--flight-time-budget-min", type=float, required=True,
                        help="Usable flight minutes after the battery reserve")
    parser.add_argument("--max-height-m", type=float, default=DEFAULT_LIMITS["max_height_m"])
    parser.add_argument("--max-blur-px", type=float, default=DEFAULT_LIMITS["max_blur_px"])
    parser.add_argument("--output", type=Path, required=True, help="New directory for the plan and its review")


def run(args):
    from .survey_lines import read_boundary
    try:
        latitude, longitude = (float(part) for part in args.home.split(","))
    except ValueError as error:
        raise ValueError("--home must be latitude,longitude") from error
    if args.output.exists():
        raise FileExistsError(f"Output {args.output} already exists")
    try:
        bundle = plan_structure(
            outline=read_boundary(args.outline), home=(latitude, longitude, args.home_amsl_m),
            structure_height_m=args.structure_height_m, base_m=args.base_m, camera=args.camera, mount=args.mount,
            profile=args.profile, aircraft=args.aircraft, flight_time_budget_min=args.flight_time_budget_min,
            standoff_m=args.standoff_m, gsd_cm=args.gsd_cm, requirements=requirements_from_arguments(args),
            fence_margin_m=args.fence_margin_m,
            limits={"max_height_m": args.max_height_m, "max_blur_px": args.max_blur_px})
    except SurveyRefused as refusal:
        return {"passed": False, "errors": refusal.errors, "alternatives": refusal.alternatives,
                "flight_ready": False}
    write_bundle(bundle, args.output)
    review, chosen = bundle["review"], bundle["review"]["design"]
    return {"passed": True, "output": str(args.output), "mission_view": str(args.output / "mission.html"),
            "plan_sha256": review["plan_sha256"], "standoff_m": chosen["standoff_m"],
            "gsd_cm_coarsest": chosen["gsd_cm_coarsest"], "speed_m_s": chosen["speed_m_s"],
            "binding_speed_limit": chosen["binding_speed_limit"],
            "pass_heights_m": [face_pass["height_m"] for face_pass in review["faces"][0]["passes"]],
            "faces": review["totals"]["faces"], "passes": review["totals"]["passes"],
            "corner_stops": review["totals"]["corner_stops"],
            "headings_deg": review["yaw"]["headings_deg"],
            "estimated_minutes": review["totals"]["estimated_minutes"],
            "budget_minutes": review["totals"]["budget_minutes"],
            "min_lines_seeing_any_wall_point": review["checks"]["pointing"]["min_lines"],
            "closest_to_structure_m": review["totals"]["closest_to_structure_m"],
            "warnings": review["warnings"], "flight_ready": False}
