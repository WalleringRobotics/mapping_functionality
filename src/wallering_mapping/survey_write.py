"""Write a designed survey as the Plan file QGroundControl 5.1 saves (no vehicle IO).

Field names, versions and item order follow QGroundControl v5.1.5: SurveyComplexItem
version 5, TransectStyleComplexItem version 2 and CameraCalc version 2. No camera
trigger commands are written; the OAK free-runs and does not use them.
"""

import json
import math

from .survey_lines import fence_polygon, to_latlon
from .survey_plan import CAMERA_KEYS, PX4_AUTOPILOT

RELATIVE = 1  # QGroundControlQmlGlobal::AltitudeFrameRelative
FRAME_RELATIVE, FRAME_MISSION = 3, 2


def _simple(seq, command, frame, params, altitude=None):
    item = {"autoContinue": True, "command": command, "doJumpId": seq + 1, "frame": frame,
            "params": params, "type": "SimpleItem"}
    if altitude is not None:
        item.update(AMSLAltAboveTerrain=None, Altitude=altitude, AltitudeMode=RELATIVE)
    return item


def build_plan(home, spec, aircraft, design, layout, fence_margin_m):
    """The plan as a dictionary: speed, take-off, one Survey, return, and an inclusion fence."""
    origin, height, speed = home[:2], design["height_m"], design["speed_m_s"]

    def position(xy):
        lat, lon = to_latlon(origin, xy)
        return [round(lat, 9), round(lon, 9)]

    # Speed comes first so that the item after take-off requires actual flight.
    items = [_simple(0, 178, FRAME_MISSION, [1, speed, -1, 0, 0, 0, 0]),
             _simple(1, 22, FRAME_RELATIVE, [0, 0, 0, None, home[0], home[1], height], height)]
    first = len(items)
    visual, survey_items, frames = [], [], 0
    for transect in layout["transects"]:
        for xy in transect:
            point = position(xy)
            visual.append(point)
            survey_items.append(_simple(first + len(survey_items), 16, FRAME_RELATIVE,
                                        [0, 0, 0, None, point[0], point[1], height], height))
        frames += math.floor(math.dist(transect[1], transect[2]) / design["frame_spacing_m"]) + 1
    calc = {"AdjustedFootprintFrontal": design["footprint_along_m"] * (1 - design["saved_front_overlap"] / 100),
            "AdjustedFootprintSide": design["line_spacing_m"], "CameraName": "Custom Camera",
            "DistanceMode": RELATIVE, "DistanceToSurface": height, "FixedOrientation": True,
            "FrontalOverlap": design["saved_front_overlap"], "ImageDensity": design["gsd_cm"],
            "Landscape": spec["Landscape"], "MinTriggerInterval": spec.get("MinTriggerInterval", 0),
            "SideOverlap": design["saved_side_overlap"], "ValueSetIsDistance": True, "version": 2,
            **{key: spec[key] for key in CAMERA_KEYS}}
    transects = {"CameraCalc": calc, "CameraShots": frames, "CameraTriggerInTurnAround": False,
                 "HoverAndCapture": False, "Items": survey_items, "Refly90Degrees": design["crosshatch"],
                 "TurnAroundDistance": design["turnaround_m"], "VisualTransectPoints": visual, "version": 2}
    items.append({"TransectStyleComplexItem": transects, "angle": layout["angle"],
                  "complexItemType": "survey", "entryLocation": 0, "flyAlternateTransects": False,
                  "polygon": [position(xy) for xy in layout["polygon"]], "splitConcavePolygons": False,
                  "type": "ComplexItem", "version": 5})
    items.append(_simple(first + len(survey_items), 20, FRAME_MISSION, [0, 0, 0, 0, 0, 0, 0]))
    points = [(0.0, 0.0)] + [xy for transect in layout["transects"] for xy in transect]
    fence = [position(xy) for xy in fence_polygon(points, fence_margin_m)]
    return {"fileType": "Plan",
            "geoFence": {"circles": [], "polygons": [{"inclusion": True, "polygon": fence, "version": 1}],
                         "version": 2},
            "groundStation": "QGroundControl",
            "mission": {"cruiseSpeed": 15, "firmwareType": PX4_AUTOPILOT, "globalPlanAltitudeMode": RELATIVE,
                        "hoverSpeed": speed, "items": items, "plannedHomePosition": list(home),
                        "vehicleType": aircraft["vehicle_type"], "version": 2},
            "rallyPoints": {"points": [], "version": 2}, "version": 1}


def encode_plan(plan):
    """Bytes as QJsonDocument writes them: sorted keys, four-space indent. Same plan, same bytes."""
    return (json.dumps(plan, indent=4, sort_keys=True, allow_nan=False) + "\n").encode()
