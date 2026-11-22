"""Field boundaries, flight lines and the fence in a local metric frame (no vehicle IO).

Needs shapely, from the ``planner`` extra. Lines are this module's own: QGroundControl
uploads a loaded Survey's saved items unchanged and only regenerates them on an edit.
"""

import json
import math
from pathlib import Path
from xml.etree import ElementTree

from .survey_plan import EARTH_RADIUS_M, local_xy

MAX_SITE_M = 5000.0  # local_xy stays within 0.1 % over a few kilometres
MIN_LEG_M = 1.0


def to_latlon(origin, xy):
    """Inverse of ``survey_plan.local_xy``: east/north metres about origin to (lat, lon)."""
    return (origin[0] + math.degrees(xy[1] / EARTH_RADIUS_M),
            origin[1] + math.degrees(xy[0] / (EARTH_RADIUS_M * math.cos(math.radians(origin[0])))))


def _ring(points, order):
    ring = []
    for point in points:
        if len(point) < 2:
            raise ValueError("Boundary vertices need latitude and longitude")
        lat, lon = (float(point[1]), float(point[0])) if order == "lonlat" else (float(point[0]), float(point[1]))
        if not (-90 <= lat <= 90 and -180 <= lon <= 180):
            raise ValueError(f"Boundary vertex {lat}, {lon} is not a latitude and longitude")
        ring.append((lat, lon))
    if len(ring) > 1 and ring[0] == ring[-1]:
        ring.pop()
    if len(ring) < 3:
        raise ValueError("Boundary needs at least three vertices")
    return ring


def _geojson_polygons(node):
    kind = node.get("type")
    if kind == "FeatureCollection":
        return [p for feature in node.get("features", []) for p in _geojson_polygons(feature)]
    if kind == "Feature":
        return _geojson_polygons(node.get("geometry") or {})
    if kind == "Polygon":
        return [node["coordinates"]]
    if kind == "MultiPolygon":
        return list(node["coordinates"])
    return []


def read_boundary(path):
    """One field boundary as (lat, lon) vertices from KML, GeoJSON or a QGroundControl plan."""
    path = Path(path)
    suffix = path.suffix.lower()
    text = path.read_text()
    if suffix == ".kml":
        if "<!DOCTYPE" in text or "<!ENTITY" in text:
            raise ValueError("KML with a document type or entities is refused")
        polygons = [e for e in ElementTree.fromstring(text).iter() if e.tag.rpartition("}")[2] == "Polygon"]
        if len(polygons) != 1:
            raise ValueError(f"KML must hold exactly one Polygon; found {len(polygons)}")
        names = {e.tag.rpartition("}")[2]: e for e in polygons[0].iter()}
        if "innerBoundaryIs" in names:
            raise ValueError("Fields with holes are not supported")
        outer = names.get("outerBoundaryIs")
        coordinates = None if outer is None else next(
            (e for e in outer.iter() if e.tag.rpartition("}")[2] == "coordinates"), None)
        if coordinates is None or not (coordinates.text or "").strip():
            raise ValueError("KML Polygon has no outer boundary coordinates")
        return _ring([tuple(v.split(",")) for v in coordinates.text.split()], "lonlat")
    if suffix in {".geojson", ".json"}:
        polygons = _geojson_polygons(json.loads(text))
        if len(polygons) != 1:
            raise ValueError(f"GeoJSON must hold exactly one Polygon; found {len(polygons)}")
        if len(polygons[0]) != 1:
            raise ValueError("Fields with holes are not supported")
        return _ring(polygons[0][0], "lonlat")
    if suffix == ".plan":
        surveys = [item for item in json.loads(text).get("mission", {}).get("items", [])
                   if item.get("complexItemType") == "survey"]
        if len(surveys) != 1:
            raise ValueError(f"Plan must hold exactly one Survey to supply a boundary; found {len(surveys)}")
        return _ring(surveys[0].get("polygon") or [], "latlon")
    raise ValueError("Boundary must be a .kml, .geojson or .plan file")


def field_polygon(home, boundary):
    """The boundary as a valid shapely polygon in metres east/north of home."""
    from shapely.geometry import Polygon
    field = Polygon([local_xy(home, vertex) for vertex in boundary])
    if not field.is_valid:
        raise ValueError("Boundary crosses itself; redraw it as a simple outline")
    if field.area < 100:
        raise ValueError("Boundary encloses less than 100 square metres")
    far = max(math.hypot(x, y) for x, y in field.exterior.coords)
    if far > MAX_SITE_M:
        raise ValueError(f"Boundary reaches {far:.0f} m from the take-off point; the limit is {MAX_SITE_M:.0f} m")
    return field


def _axes(angle):
    """Unit vectors along a leg of compass bearing ``angle`` and across it (to the right)."""
    theta = math.radians(angle)
    return (math.sin(theta), math.cos(theta)), (math.cos(theta), -math.sin(theta))


def _grow(polygon, half_across, half_along):
    """The polygon swept by a box: every point within the box's half-sizes of the polygon."""
    from shapely.geometry import MultiPoint
    from shapely.ops import unary_union
    corners = [(sx * half_across, sy * half_along) for sx in (-1, 1) for sy in (-1, 1)]
    ring = list(polygon.exterior.coords)
    swept = [MultiPoint([(x + dx, y + dy) for x, y in edge for dx, dy in corners]).convex_hull
             for edge in zip(ring, ring[1:])]
    return unary_union([polygon, *swept])


def lay_lines(field, angle, spacing, reach, across_extent, along_margin):
    """Parallel lines that give the field's edge the same views as its interior.

    A line is flown wherever its footprint (``reach`` either side) touches the field, and
    runs ``along_margin`` past the field at both ends. Lines continue ``across_extent``
    beyond the field's sides. Returns the polygon the lines are chords of (east/north)
    and one (across, start, end) chord per line in the leg frame. A line that crosses a
    concave boundary twice spans the gap, as QGroundControl's does by default.
    """
    from shapely.geometry import LineString, Polygon, box
    along, across = _axes(angle)
    rotated = Polygon([(x * across[0] + y * across[1], x * along[0] + y * along[1])
                       for x, y in field.exterior.coords])
    low, south, high, north = rotated.bounds
    count = math.ceil((high - low + 2 * across_extent) / spacing - 1e-9) + 1
    positions = [(low + high) / 2 + (number - (count - 1) / 2) * spacing for number in range(count)]
    south, north = south - along_margin - 1, north + along_margin + 1
    grown = _grow(rotated, reach, along_margin).intersection(
        box(positions[0] - 0.01, south, positions[-1] + 0.01, north))
    grown = Polygon(grown.exterior).simplify(0.001)
    lines = []
    for position in positions:
        cut = grown.intersection(LineString([(position, south), (position, north)]))
        if cut.is_empty:
            continue
        ends = [c[1] for part in getattr(cut, "geoms", [cut]) for c in part.coords]
        if max(ends) - min(ends) >= MIN_LEG_M:
            lines.append((position, min(ends), max(ends)))
    if not lines:
        raise ValueError("No flight line fits the field")
    polygon = [(a * across[0] + t * along[0], a * across[1] + t * along[1])
               for a, t in list(grown.exterior.coords)[:-1]]
    return polygon, lines


def route(lines, angle, turnaround, start):
    """Back-and-forth transects [turnaround, entry, exit, turnaround], starting nearest ``start``."""
    along, across = _axes(angle)

    def point(position, distance):
        return (position * across[0] + distance * along[0], position * across[1] + distance * along[1])

    best = None
    for ordered in (lines, lines[::-1]):
        for first_up in (True, False):
            transects = []
            for number, (position, low, high) in enumerate(ordered):
                up = first_up == (number % 2 == 0)
                entry, exit_, step = (low, high, turnaround) if up else (high, low, -turnaround)
                transects.append([point(position, entry - step), point(position, entry),
                                  point(position, exit_), point(position, exit_ + step)])
            distance = math.dist(start, transects[0][0])
            if best is None or distance < best[0] - 1e-9:
                best = (distance, transects)
    return best[1]


def path_length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def plan_lines(field, home_xy, design, angle=None):
    """Lines for one design; the angle with the shortest flight unless one is given."""
    spacing, turnaround, speed = design["line_spacing_m"], design["turnaround_m"], design["speed_m_s"]
    shape = (spacing, design["footprint_across_low_m"] / 2, design["across_extent_m"],
             design["along_margin_m"])

    def build(candidate):
        polygon, lines = lay_lines(field, candidate, *shape)
        transects = route(lines, candidate, turnaround, home_xy)
        if design["crosshatch"]:
            _, crossing = lay_lines(field, candidate + 90, *shape)
            transects += route(crossing, candidate + 90, turnaround, transects[-1][-1])
        points = [home_xy] + [p for transect in transects for p in transect] + [home_xy]
        # Flight time in metres at survey speed: path plus the checker's two seconds per waypoint.
        return path_length(points) + 2.0 * speed * 4 * len(transects), polygon, transects

    if angle is not None:
        candidates = [float(angle) % 180]
    else:
        ring = list(field.exterior.coords)
        edges = {round(math.degrees(math.atan2(b[0] - a[0], b[1] - a[1])) % 180, 1) % 180
                 for a, b in zip(ring, ring[1:]) if math.dist(a, b) > 1}
        candidates = sorted(edges | {float(a) for a in range(0, 180, 5)})
    cost, polygon, transects, chosen = min((*build(c), c) for c in candidates)
    return {"angle": chosen, "polygon": polygon, "transects": transects, "cost_m": cost,
            "angle_selected": "given" if angle is not None else "shortest flight"}


def fence_polygon(points, margin):
    """Convex hull of every point, grown by a margin: it contains every straight path between them."""
    from shapely.geometry import MultiPoint
    hull = MultiPoint(points).convex_hull.buffer(margin, join_style="mitre", mitre_limit=2.0)
    return list(hull.exterior.coords)[:-1]
