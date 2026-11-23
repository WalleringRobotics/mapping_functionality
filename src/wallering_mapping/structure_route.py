"""Lay a fixed-yaw fly-by route round a structure and write it as a QGroundControl plan.

No vehicle IO. The route is plain waypoints, each with a yaw, because a QGroundControl
pattern cannot carry one. Round the structure, counter-clockwise from the corner nearest
the take-off point:

- **Face:** passes parallel to the wall at the stand-off distance, one per height, flown
  sideways back and forth with the nose square to the wall. Each pass is
  [lead-in, entry, exit, lead-out], like a survey transect.
- **Corner:** stops on an arc round the corner, the yaw stepped from one face's heading
  to the next and held on the corner, one stop per pass height. They give the images of
  one face views in common with the next. The aircraft is stationary at each stop.
"""

import math

from .structure_check import WAYPOINT, structure_faces
from .survey_lines import fence_polygon, to_latlon
from .survey_plan import PX4_AUTOPILOT
from .survey_write import FRAME_MISSION, FRAME_RELATIVE, RELATIVE, _simple


def _bearing(vector):
    return math.degrees(math.atan2(vector[0], vector[1])) % 360


def _rotate(vector, degrees):
    """Rotate an (east, north) vector counter-clockwise."""
    c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
    return (vector[0] * c - vector[1] * s, vector[0] * s + vector[1] * c)


def lay_route(home, outline, design):
    """Waypoints for the whole flight, in metres east/north of home and above the base.

    Each waypoint is a dict with ``xy``, ``height_m``, ``yaw_deg``, ``hold_s`` and what it
    is for (``role``, and ``face``/``pass`` or ``corner``/``station``/``stop``).
    """
    _, faces, _ = structure_faces(home, outline)
    standoff, lead, margin = design["standoff_m"], design["lead_m"], design["end_margin_m"]
    heights, settle = design["pass_heights_m"], design["settle_hold_s"]
    step_limit = design["requirements"]["max_yaw_step_deg"]

    def on_face(face, distance):
        return (face["start"][0] + face["along"][0] * distance + face["normal"][0] * standoff,
                face["start"][1] + face["along"][1] * distance + face["normal"][1] * standoff)

    # Start at the face whose lead-in is nearest the take-off point.
    first = min(range(len(faces)), key=lambda i: math.hypot(*on_face(faces[i], -margin - lead)))
    faces = faces[first:] + faces[:first]
    points, at_top = [], False

    def add(xy, height, yaw, role, **labels):
        new_yaw = not points or abs((yaw - points[-1]["yaw_deg"] + 180) % 360 - 180) > 0.01
        points.append({"xy": (round(xy[0], 3), round(xy[1], 3)), "height_m": height, "yaw_deg": round(yaw, 2),
                       "hold_s": settle if new_yaw else 0.0, "role": role, **labels})

    start = on_face(faces[0], -margin - lead)
    if design["safe_height_m"] > heights[0] + 1e-6:
        add(start, design["safe_height_m"], faces[0]["yaw_deg"], "approach", face=faces[0]["index"])
    for number, face in enumerate(faces):
        stations = [-margin - lead, -margin, face["length_m"] + margin, face["length_m"] + margin + lead]
        roles = ["lead_in", "entry", "exit", "lead_out"]
        levels = heights[::-1] if at_top else heights
        for sweep, height in enumerate(levels):
            order = stations if sweep % 2 == 0 else stations[::-1]
            for distance, role in zip(order, roles):
                add(on_face(face, distance), height, face["yaw_deg"], role, face=face["index"],
                    **{"pass": heights.index(height)})
        at_top = not at_top if len(levels) > 1 else at_top
        following = faces[(number + 1) % len(faces)]
        turn = (_bearing(following["along"]) - _bearing(face["along"])) % 360
        turn = turn - 360 if turn > 180 else turn  # compass bearings fall as the route turns left
        count = max(0, math.ceil(abs(turn) / step_limit - 1e-9) - 1)
        for station in range(1, count + 1):
            outward = _rotate(face["normal"], -turn * station / (count + 1))
            xy = (face["end"][0] + outward[0] * standoff, face["end"][1] + outward[1] * standoff)
            levels = heights[::-1] if at_top else heights
            for height in levels:
                add(xy, height, _bearing((-outward[0], -outward[1])), "stop", corner=face["index"],
                    station=station - 1, stop=heights.index(height))
                points[-1]["hold_s"] = design["stop_hold_s"]
            at_top = not at_top if len(levels) > 1 else at_top
    if design["safe_height_m"] > points[-1]["height_m"] + 1e-6:
        add(points[-1]["xy"], design["safe_height_m"], points[-1]["yaw_deg"], "climb_out")
    return points


def build_plan(home, aircraft, design, route, base_m, fence_margin_m):
    """The plan as a dictionary: speed, take-off, waypoints with yaw, return, inclusion fence."""
    origin, speed = home[:2], design["speed_m_s"]
    takeoff = round(base_m + design["safe_height_m"], 2)
    # Speed comes first so that the item after take-off requires actual flight.
    items = [_simple(0, 178, FRAME_MISSION, [1, speed, -1, 0, 0, 0, 0]),
             _simple(1, 22, FRAME_RELATIVE, [0, 0, 0, None, home[0], home[1], takeoff], takeoff)]
    for point in route:
        lat, lon = to_latlon(origin, point["xy"])
        height = round(base_m + point["height_m"], 2)
        point["seq"] = len(items)
        items.append(_simple(len(items), WAYPOINT, FRAME_RELATIVE,
                             [point["hold_s"], 0, 0, point["yaw_deg"], round(lat, 9), round(lon, 9), height], height))
    items.append(_simple(len(items), 20, FRAME_MISSION, [0, 0, 0, 0, 0, 0, 0]))

    def position(xy):
        lat, lon = to_latlon(origin, xy)
        return [round(lat, 9), round(lon, 9)]

    fence = [position(xy) for xy in fence_polygon([(0.0, 0.0)] + [p["xy"] for p in route], fence_margin_m)]
    return {"fileType": "Plan",
            "geoFence": {"circles": [], "polygons": [{"inclusion": True, "polygon": fence, "version": 1}],
                         "version": 2},
            "groundStation": "QGroundControl",
            "mission": {"cruiseSpeed": 15, "firmwareType": PX4_AUTOPILOT, "globalPlanAltitudeMode": RELATIVE,
                        "hoverSpeed": speed, "items": items, "plannedHomePosition": list(home),
                        "vehicleType": aircraft["vehicle_type"], "version": 2},
            "rallyPoints": {"points": [], "version": 2}, "version": 1}
