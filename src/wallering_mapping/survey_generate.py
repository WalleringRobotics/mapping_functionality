"""Generate a checked survey and its review bundle from optics, aircraft and target.

No vehicle IO: nothing here uploads, arms or reads a vehicle. The plan must pass
``survey-check`` unmodified and the independent coverage test before anything is written.
"""

import json
import math
import tempfile
from pathlib import Path

from . import __version__
from .survey_coverage import check_coverage
from .survey_design import TARGET_TYPES, design, read_aircraft
from .survey_plan import (DEFAULT_LIMITS, check_survey, estimate_minutes, load_plan, local_xy, read_camera,
                          read_profile, sha256_bytes)

MARGIN_ATTEMPTS = 4


class SurveyRefused(ValueError):
    """The request cannot be planned as asked; ``alternatives`` holds the nearest plans that can."""

    def __init__(self, errors, alternatives=None):
        super().__init__("; ".join(errors))
        self.errors, self.alternatives = list(errors), alternatives or {}


def _write_temporary(folder, data):
    path = Path(folder) / "survey.plan"
    path.write_bytes(data)
    return path


def plan_survey(*, boundary, home, camera, profile, aircraft, flight_time_budget_min, relief_m,
                gsd_cm=None, height_m=None, target="terrain", front=None, side=None, angle=None,
                fence_margin_m=20.0, limits=None, grid_m=1.0):
    """Design, lay out, write and check one survey; return the bundle or raise SurveyRefused.

    ``boundary`` is the field as (lat, lon) vertices, ``home`` is (lat, lon, AMSL metres);
    ``camera``, ``profile`` and ``aircraft`` are paths to the reviewed definitions.
    """
    from .survey_lines import field_polygon, plan_lines, to_latlon
    from .survey_write import build_plan, encode_plan
    limits = {**DEFAULT_LIMITS, **(limits or {})}
    camera_data = read_camera(camera)
    spec = camera_data["qgc_custom_camera"]
    capture = read_profile(profile, camera_data["stream"])
    vehicle = read_aircraft(aircraft)
    if len(home) != 3 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in home):
        raise ValueError("Home must be latitude, longitude and elevation above mean sea level")
    if not 5 <= fence_margin_m <= 200:
        raise ValueError("Fence margin must be in [5, 200] m")
    if vehicle["return_height_m"] > limits["max_height_m"]:
        raise SurveyRefused([f"The aircraft returns at {vehicle['return_height_m']} m, above the "
                             f"{limits['max_height_m']} m height limit; lower RTL_RETURN_ALT"])
    field = field_polygon(home, boundary)
    request = dict(relief_m=relief_m, gsd_cm=gsd_cm, height_m=height_m, target=target, front=front,
                   side=side, limits=limits)

    def attempt(shape, **changes):
        chosen = design(spec, capture, vehicle, **{**request, **changes})
        layout = plan_lines(shape, (0.0, 0.0), chosen, angle)
        return chosen, layout, encode_plan(build_plan(home, spec, vehicle, chosen, layout, fence_margin_m))

    def minutes(shape, **changes):
        """The checker's flight-time estimate for a variation of the request, or None if invalid."""
        try:
            data = attempt(shape, **changes)[2]
        except ValueError:
            return None
        with tempfile.TemporaryDirectory() as folder:
            loaded = load_plan(_write_temporary(folder, data))
        return estimate_minutes(loaded["home"], loaded["items"], limits, [], [])[0]

    chosen, layout, data = attempt(field)
    with tempfile.TemporaryDirectory() as folder:
        path = _write_temporary(folder, data)
        for grown in range(MARGIN_ATTEMPTS):
            coverage, raster = check_coverage(
                path, boundary, spec, capture, relief_m=chosen["requested"]["relief_m"],
                acceptance_radius_m=vehicle["acceptance_radius_m"],
                front=chosen["requested"]["front_overlap"], side=chosen["requested"]["side_overlap"],
                grid_m=grid_m)
            if coverage["passed"] or grown == MARGIN_ATTEMPTS - 1:
                break
            # The derived margins fell short for this shape: widen them and lay the lines again.
            chosen["across_extent_m"] += chosen["line_spacing_m"] / 2
            chosen["along_margin_m"] += chosen["frame_spacing_m"]
            layout = plan_lines(field, (0.0, 0.0), chosen, angle)
            data = encode_plan(build_plan(home, spec, vehicle, chosen, layout, fence_margin_m))
            path.write_bytes(data)
        coverage["margin_increases"] = grown
        check = check_survey(path, camera, profile, flight_time_budget_min, limits, gsd_cm)
    if not coverage["passed"]:
        raise SurveyRefused(["Coverage test failed: " + error for error in coverage["errors"]])
    if not check["passed"]:
        alternatives = {}
        estimated = check["flight"].get("estimated_minutes")
        if estimated is not None and estimated > flight_time_budget_min:
            alternatives = _alternatives(field, chosen, minutes, flight_time_budget_min, limits)
        raise SurveyRefused(check["errors"], alternatives)

    warnings = list(check["warnings"])
    for key, rate in (("climb_m_s", "climb"), ("descent_m_s", "descent")):
        if vehicle[key] < limits[key]:
            warnings.append(f"The aircraft's {rate} rate {vehicle[key]} m/s is below the {limits[key]} m/s the "
                            "flight-time estimate assumes; the estimate is optimistic")
    if vehicle["return_height_m"] > chosen["height_m"]:
        warnings.append(f"Return climbs to {vehicle['return_height_m']} m; the flight-time estimate does "
                        "not include that climb")
    if chosen["requested"]["relief_m"] == 0:
        warnings.append("Relief is stated as 0 m: overlap and blur hold only over ground level with take-off")
    return {"plan": data, "check": check, "coverage": coverage, "raster": raster,
            "review": _review(home, boundary, field, camera, profile, aircraft, camera_data, vehicle,
                              chosen, layout, check, coverage, warnings, data, fence_margin_m, to_latlon),
            "geojson": _geojson(home, boundary, chosen, layout, check, vehicle, to_latlon, json.loads(data))}


def _alternatives(field, chosen, minutes, budget, limits):
    """The two nearest plans that fit the time budget: a coarser GSD, or a smaller area."""
    from shapely.affinity import scale
    result = {}
    # Higher is not always quicker (the climb and descent grow), so step up to the first fit.
    low, high = chosen["height_m"], None
    step = 5.0
    while low + step <= limits["max_height_m"] + 1e-9:
        estimate = minutes(field, gsd_cm=None, height_m=low + step)
        if estimate is not None and estimate <= budget:
            high = low + step
            break
        low += step
    if high is not None:
        while high - low > 0.5:
            middle = (low + high) / 2
            estimate = minutes(field, gsd_cm=None, height_m=middle)
            low, high = (low, middle) if estimate is not None and estimate <= budget else (middle, high)
        height = math.ceil(high * 2) / 2
        result["coarser_gsd_for_this_area"] = {
            "height_m": height, "gsd_cm": height / chosen["height_m"] * chosen["gsd_cm"],
            "estimated_minutes": minutes(field, gsd_cm=None, height_m=height)}
    low, high = 0.0, 1.0
    while high - low > 0.02:
        middle = (low + high) / 2
        estimate = None if middle * middle * field.area < 100 else minutes(
            scale(field, middle, middle, origin="centroid"))
        low, high = (middle, high) if estimate is not None and estimate <= budget else (low, middle)
    if low > 0:
        result["largest_area_at_this_gsd"] = {
            "area_ha": low * low * field.area / 1e4, "fraction_of_field": low * low,
            "estimated_minutes": minutes(scale(field, low, low, origin="centroid")),
            "note": "The field shrunk about its centre; split it or redraw the boundary to this area"}
    return result


def _legs(home, check):
    by_seq = {item["mission_seq"]: item for item in check["mission_items"]}
    for number, leg in enumerate(check["legs"]):
        entry, exit_ = by_seq[leg["entry_seq"]], by_seq[leg["exit_seq"]]
        start, end = (local_xy(home, (item["lat"], item["lon"])) for item in (entry, exit_))
        yield number, leg, entry, exit_, start, end


def _review(home, boundary, field, camera, profile, aircraft, camera_data, vehicle, chosen, layout,
            check, coverage, warnings, data, fence_margin_m, to_latlon):
    legs, frames, radius = [], 0, vehicle["acceptance_radius_m"]
    for number, leg, entry, exit_, start, end in _legs(home, check):
        count = math.floor(max(0.0, leg["length_m"] - radius) / chosen["frame_spacing_m"]) + 1
        frames += count
        legs.append({"leg": number, "transect": leg["transect"],
                     "heading_deg": math.degrees(math.atan2(end[0] - start[0], end[1] - start[1])) % 360,
                     "length_m": leg["length_m"], "frames": count,
                     "entry": [entry["lat"], entry["lon"]], "exit": [exit_["lat"], exit_["lon"]],
                     "entry_seq": leg["entry_seq"], "exit_seq": leg["exit_seq"]})
    points = [local_xy(home, (item["lat"], item["lon"])) for item in check["mission_items"]
              if item.get("lat") is not None]
    flight = check["flight"]
    return {
        "schema_version": 1, "kind": "survey_plan_review", "planner_version": __version__,
        "flight_ready": False, "plan_sha256": sha256_bytes(data),
        "inputs": {
            "boundary": [list(vertex) for vertex in boundary], "home": list(home),
            "camera": camera_data.get("name"), "camera_sha256": sha256_bytes(Path(camera).read_bytes()),
            "profile_sha256": sha256_bytes(Path(profile).read_bytes()),
            "aircraft": vehicle["aircraft_id"], "aircraft_sha256": sha256_bytes(Path(aircraft).read_bytes()),
            "aircraft_values": {key: vehicle[key] for key in sorted(vehicle) if key not in {"schema"}},
            "flight_time_budget_min": flight["budget_minutes"], "fence_margin_m": fence_margin_m},
        "design": chosen,
        "layout": {"angle_deg": layout["angle"], "angle_selected": layout["angle_selected"],
                   "legs": len(legs), "crosshatch": chosen["crosshatch"]},
        "totals": {
            "field_area_ha": field.area / 1e4, "frames_on_legs": frames,
            "leg_length_m": sum(leg["length_m"] for leg in legs), "path_length_m": flight["path_length_m"],
            "estimated_minutes": flight["estimated_minutes"], "budget_minutes": flight["budget_minutes"],
            "max_distance_from_home_m": max(math.hypot(*point) for point in points)},
        "legs": legs,
        "checks": {"survey_check": {"passed": check["passed"], "file": "survey-check.json"},
                   "coverage": coverage},
        "warnings": warnings,
        "files": ["survey.plan", "survey-check.json", "review.json", "review.geojson", "coverage.png",
                  "review.kml"],
        "limitations": [
            "The plan flies these lines only if QGroundControl uploads it unedited; an edited Survey is "
            "regenerated by QGroundControl and must be checked again",
            "Coverage assumes a nadir camera, level flight and yaw along each leg (MIS_YAWMODE 0)",
            "RGB rolling-shutter readout is unmeasured (#14); skew is reported, not limited",
            "Flight time is the checker's estimate; wind, battery state and PX4 tuning are not modelled",
            "Line of sight, airspace and site approval remain with the operator"]}


def _geojson(home, boundary, chosen, layout, check, vehicle, to_latlon, plan):
    origin = home[:2]

    def lonlat(xy):
        lat, lon = to_latlon(origin, xy)
        return [round(lon, 9), round(lat, 9)]

    def feature(layer, kind, coordinates, **properties):
        return {"type": "Feature", "properties": {"layer": layer, **properties},
                "geometry": {"type": kind, "coordinates": coordinates}}

    def ring(vertices):
        return [[*vertices, vertices[0]]]

    features = [feature("boundary", "Polygon", ring([[lon, lat] for lat, lon in boundary])),
                feature("survey_polygon", "Polygon", ring([lonlat(xy) for xy in layout["polygon"]])),
                feature("home", "Point", [home[1], home[0]])]
    for polygon in plan["geoFence"]["polygons"]:
        features.append(feature("fence", "Polygon", ring([[lon, lat] for lat, lon in polygon["polygon"]])))
    path = [[home[1], home[0]]] + [[item["lon"], item["lat"]] for item in check["mission_items"]
                                   if item.get("lat") is not None and item["command"] == 16]
    features.append(feature("path", "LineString", path + [[home[1], home[0]]]))
    half_across, half_along = chosen["footprint_across_low_m"] / 2, chosen["footprint_along_low_m"] / 2
    radius, spacing = vehicle["acceptance_radius_m"], chosen["frame_spacing_m"]
    for transect in layout["transects"]:
        features.append(feature("turnarounds", "MultiLineString",
                                [[lonlat(transect[0]), lonlat(transect[1])],
                                 [lonlat(transect[2]), lonlat(transect[3])]]))
    for number, leg, entry, exit_, start, end in _legs(home, check):
        length = math.dist(start, end)
        along = ((end[0] - start[0]) / length, (end[1] - start[1]) / length)
        across = (along[1], -along[0])

        def at(distance, offset=0.0):
            return (start[0] + along[0] * distance + across[0] * offset,
                    start[1] + along[1] * distance + across[1] * offset)

        heading = math.degrees(math.atan2(along[0], along[1])) % 360
        features.append(feature("legs", "LineString", [lonlat(start), lonlat(end)], leg=number,
                                heading_deg=heading, length_m=length))
        count = math.floor(max(0.0, length - radius) / spacing) + 1
        features.append(feature("frame_centres", "MultiPoint",
                                [lonlat(at(n * spacing)) for n in range(count)], leg=number,
                                frame_spacing_m=spacing))
        middle = length / 2
        corners = [at(middle + dy * half_along, dx * half_across)
                   for dx, dy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        features.append(feature("sample_footprints", "Polygon", ring([lonlat(c) for c in corners]), leg=number))
    return {"type": "FeatureCollection", "features": features}


def coverage_png(raster, lines_needed):
    """The coverage grid as PNG bytes: red below the required views, green at or above, clear off-field."""
    import cv2
    import numpy as np
    image = np.zeros((*raster.shape, 4), dtype=np.uint8)  # BGRA
    short = (raster >= 0) & (raster < lines_needed)
    enough = raster >= lines_needed
    image[short] = (40, 40, 220, 220)
    extra = np.clip(raster - lines_needed, 0, 4)
    image[enough, 0] = 60
    image[enough, 1] = (130 + 25 * extra[enough]).astype(np.uint8)
    image[enough, 2] = 40
    image[enough, 3] = 170
    encoded, data = cv2.imencode(".png", image)
    if not encoded:
        raise RuntimeError("Could not encode the coverage image")
    return data.tobytes()


def review_kml(review, geojson):
    """The review layers and the coverage image for Google Earth."""
    height = review["design"]["height_m"]
    styles = {"boundary": ("ff00ffff", 3), "survey_polygon": ("ffffaa55", 1), "fence": ("ff0000ff", 2),
              "legs": ("ffffffff", 2), "path": ("99ffffff", 1)}

    def line(coordinates, altitude):
        text = " ".join(f"{lon:.9f},{lat:.9f},{altitude:.1f}" for lon, lat in coordinates)
        mode = "relativeToGround" if altitude else "clampToGround"
        return (f"<LineString><tessellate>1</tessellate><altitudeMode>{mode}</altitudeMode>"
                f"<coordinates>{text}</coordinates></LineString>")

    parts = ['<?xml version="1.0" encoding="UTF-8"?>', '<kml xmlns="http://www.opengis.net/kml/2.2">',
             "<Document>", f"<name>Survey {review['plan_sha256'][:12]}</name>"]
    for name, (colour, width) in styles.items():
        parts.append(f'<Style id="{name}"><LineStyle><color>{colour}</color><width>{width}</width>'
                     "</LineStyle></Style>")
    for item in geojson["features"]:
        layer, geometry = item["properties"]["layer"], item["geometry"]
        if layer in ("boundary", "survey_polygon", "fence"):
            body = line(geometry["coordinates"][0], 0.0)
        elif layer in ("legs", "path"):
            body = line(geometry["coordinates"], height)
        elif layer == "home":
            lon, lat = geometry["coordinates"]
            parts.append(f"<Placemark><name>take-off</name><Point><coordinates>{lon:.9f},{lat:.9f},0"
                         "</coordinates></Point></Placemark>")
            continue
        else:
            continue
        label = f"leg {item['properties']['leg']}" if layer == "legs" else layer.replace("_", " ")
        parts.append(f"<Placemark><name>{label}</name><styleUrl>#{layer}</styleUrl>{body}</Placemark>")
    grid = review["checks"]["coverage"]["grid"]
    parts.append("<GroundOverlay><name>coverage: lines seeing each ground point</name>"
                 "<Icon><href>coverage.png</href></Icon><LatLonBox>"
                 f"<north>{grid['north_east'][0]:.9f}</north><south>{grid['south_west'][0]:.9f}</south>"
                 f"<east>{grid['north_east'][1]:.9f}</east><west>{grid['south_west'][1]:.9f}</west>"
                 "</LatLonBox></GroundOverlay>")
    parts += ["</Document>", "</kml>", ""]
    return "\n".join(parts)


def write_bundle(bundle, output):
    """Publish the checked plan and its review in a new directory, refusing replacement."""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    (output / "survey.plan").write_bytes(bundle["plan"])
    for name, content in (("survey-check.json", bundle["check"]), ("review.json", bundle["review"]),
                          ("review.geojson", bundle["geojson"])):
        (output / name).write_text(json.dumps(content, indent=2, allow_nan=False) + "\n")
    (output / "coverage.png").write_bytes(coverage_png(bundle["raster"], bundle["coverage"]["lines_needed"]))
    (output / "review.kml").write_text(review_kml(bundle["review"], bundle["geojson"]))
    return output


def add_arguments(parser):
    parser.add_argument("--boundary", type=Path, required=True, help="Field outline: .kml, .geojson or .plan")
    parser.add_argument("--home", required=True, help="Take-off point as latitude,longitude in degrees")
    parser.add_argument("--home-amsl-m", type=float, required=True, help="Take-off elevation above mean sea level")
    parser.add_argument("--aircraft", type=Path, required=True,
                        help="Filled copy of configs/px4-multirotor.template.json")
    parser.add_argument("--camera", type=Path, default=Path("configs/qgc-oak-rgb-12mp.json"))
    parser.add_argument("--profile", type=Path, default=Path("configs/oakd-ros.yaml"))
    size = parser.add_mutually_exclusive_group(required=True)
    size.add_argument("--gsd-cm", type=float, help="Ground sample distance to hold over the lowest ground")
    size.add_argument("--height-m", type=float, help="Height above take-off instead of a GSD")
    parser.add_argument("--relief-m", type=float, required=True,
                        help="Terrain relief about take-off level, plus or minus metres; state 0 for level ground")
    parser.add_argument("--target", choices=sorted(TARGET_TYPES), default="terrain")
    parser.add_argument("--front", type=float, help="Forward overlap in percent; default from the target type")
    parser.add_argument("--side", type=float, help="Side overlap in percent; default from the target type")
    parser.add_argument("--angle", type=float, help="Compass bearing of the legs; default is the shortest flight")
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
        bundle = plan_survey(
            boundary=read_boundary(args.boundary), home=(latitude, longitude, args.home_amsl_m),
            camera=args.camera, profile=args.profile, aircraft=args.aircraft,
            flight_time_budget_min=args.flight_time_budget_min, relief_m=args.relief_m,
            gsd_cm=args.gsd_cm, height_m=args.height_m, target=args.target, front=args.front,
            side=args.side, angle=args.angle, fence_margin_m=args.fence_margin_m,
            limits={"max_height_m": args.max_height_m, "max_blur_px": args.max_blur_px})
    except SurveyRefused as refusal:
        return {"passed": False, "errors": refusal.errors, "alternatives": refusal.alternatives,
                "flight_ready": False}
    write_bundle(bundle, args.output)
    review = bundle["review"]
    return {"passed": True, "output": str(args.output), "plan_sha256": review["plan_sha256"],
            "height_m": review["design"]["height_m"], "gsd_cm": review["design"]["gsd_cm"],
            "speed_m_s": review["design"]["speed_m_s"],
            "binding_speed_limit": review["design"]["binding_speed_limit"],
            "line_spacing_m": review["design"]["line_spacing_m"], "angle_deg": review["layout"]["angle_deg"],
            "legs": review["layout"]["legs"], "frames_on_legs": review["totals"]["frames_on_legs"],
            "estimated_minutes": review["totals"]["estimated_minutes"],
            "budget_minutes": review["totals"]["budget_minutes"],
            "min_lines_seeing_any_point": review["checks"]["coverage"]["min_lines"],
            "warnings": review["warnings"], "flight_ready": False}
