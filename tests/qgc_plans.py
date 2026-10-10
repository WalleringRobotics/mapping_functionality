"""QGroundControl 5.1 Survey plans for tests, following its save code.

Field names, versions and item order mirror QGroundControl v5.1.5:
SurveyComplexItem::_saveCommon (version 5), TransectStyleComplexItem::_save (version 2),
CameraCalc::save (version 2) and _buildAndAppendMissionItems for a PX4 multirotor with
turnarounds and per-transect camera triggering. These are synthetic geometry, not a
plan saved by a running QGroundControl.
"""

import json
import math

from wallering_mapping.survey_plan import EARTH_RADIUS_M

HOME = (48.4, 12.9, 450.0)
CAMERA = {"SensorWidth": 6.29, "SensorHeight": 4.71, "ImageWidth": 4056, "ImageHeight": 3040,
          "FocalLength": 4.8, "Landscape": True, "FixedOrientation": True, "MinTriggerInterval": 0.5}


def to_latlon(east, north, origin=HOME):
    return [origin[0] + math.degrees(north / EARTH_RADIUS_M),
            origin[1] + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(origin[0]))))]


def simple(seq, command, frame, params, altitude=None):
    item = {"autoContinue": True, "command": command, "doJumpId": seq + 1, "frame": frame,
            "params": params, "type": "SimpleItem"}
    if altitude is not None:
        item.update(AMSLAltAboveTerrain=None, Altitude=altitude, AltitudeMode=1)
    return item


def survey_plan(height=40.0, speed=4.0, width=120.0, depth=90.0, turnaround=10.0, frontal=80,
                side=70, fence=True, vehicle_type=13, firmware=12, camera=None, hover=False,
                distance_mode=1, terrain_points=False, speed_item=True, end=20):
    camera = {**CAMERA, **(camera or {})}
    gsd_cm = height * camera["SensorWidth"] * 100 / (camera["ImageWidth"] * camera["FocalLength"])
    across, along = ((camera["ImageWidth"], camera["ImageHeight"]) if camera["Landscape"]
                     else (camera["ImageHeight"], camera["ImageWidth"]))
    spacing = across * gsd_cm / 100 * (100 - side) / 100
    trigger = along * gsd_cm / 100 * (100 - frontal) / 100
    south = 30.0
    lines = max(2, math.ceil(depth / spacing) + 1)
    frame = {1: 3, 2: 0, 3: 0, 4: 10}[distance_mode]
    altitude = height + (HOME[2] if frame == 0 else 0)
    items = [simple(0, 22, 3, [0, 0, 0, None, HOME[0], HOME[1], height], height)]
    if speed_item:
        items.append(simple(len(items), 178, 2, [1, speed, -1, 0, 0, 0, 0]))
    first = len(items)
    survey_items, visual = [], []

    def waypoint(point):
        survey_items.append(simple(first + len(survey_items), 16, frame,
                                   [0, 0, 0, None, point[0], point[1], altitude], altitude))

    def trigger_distance(distance):
        survey_items.append(simple(first + len(survey_items), 206, 2, [distance, 0, 1, 0, 0, 0, 0]))

    for number in range(lines):
        north = south + number * spacing
        west, east = (0.0, width) if number % 2 == 0 else (width, 0.0)
        step = turnaround if east > west else -turnaround
        points = [to_latlon(west - step, north), to_latlon(west, north),
                  to_latlon(east, north), to_latlon(east + step, north)]
        if turnaround <= 0:
            points = points[1:3]
        visual.extend(points)
        if turnaround > 0:
            waypoint(points[0])
        entry, exit_ = (points[1], points[2]) if turnaround > 0 else (points[0], points[1])
        waypoint(entry)
        trigger_distance(trigger)
        if terrain_points:
            waypoint(to_latlon(width / 2, north))
        waypoint(exit_)
        trigger_distance(0)
        if turnaround > 0:
            waypoint(points[3])
    calc = {"AdjustedFootprintFrontal": trigger, "AdjustedFootprintSide": spacing,
            "CameraName": "Custom Camera", "DistanceMode": distance_mode, "DistanceToSurface": height,
            "FrontalOverlap": frontal, "ImageDensity": gsd_cm, "SideOverlap": side,
            "ValueSetIsDistance": True, "version": 2, **camera}
    transect = {"CameraCalc": calc, "CameraShots": 0, "CameraTriggerInTurnAround": False,
                "HoverAndCapture": hover, "Items": survey_items, "Refly90Degrees": False,
                "TurnAroundDistance": turnaround, "VisualTransectPoints": visual, "version": 2}
    if distance_mode == 3:
        transect.update(TerrainAdjustTolerance=10, TerrainAdjustMaxClimbRate=0,
                        TerrainAdjustMaxDescentRate=0, TerrainFlightSpeed=speed)
    polygon = [to_latlon(0, south), to_latlon(width, south), to_latlon(width, south + depth),
               to_latlon(0, south + depth)]
    items.append({"TransectStyleComplexItem": transect, "angle": 90, "complexItemType": "survey",
                  "entryLocation": 0, "flyAlternateTransects": False, "polygon": polygon,
                  "splitConcavePolygons": False, "type": "ComplexItem", "version": 5})
    seq = first + len(survey_items)
    if end == 20:
        items.append(simple(seq, 20, 2, [0, 0, 0, 0, 0, 0, 0]))
    elif end == 21:
        items.append(simple(seq, 21, 3, [0, 0, 0, None, HOME[0], HOME[1], 0], 0))
    margin = 60.0
    fences = {"circles": [], "polygons": [], "version": 2}
    if fence:
        fences["polygons"].append({"inclusion": True, "version": 1, "polygon": [
            to_latlon(-margin, -margin), to_latlon(width + margin, -margin),
            to_latlon(width + margin, south + depth + margin), to_latlon(-margin, south + depth + margin)]})
    return {"fileType": "Plan", "geoFence": fences, "groundStation": "QGroundControl",
            "mission": {"cruiseSpeed": 15, "firmwareType": firmware, "globalPlanAltitudeMode": 1,
                        "hoverSpeed": speed, "items": items, "plannedHomePosition": list(HOME),
                        "vehicleType": vehicle_type, "version": 2},
            "rallyPoints": {"points": [], "version": 2}, "version": 1}


def write_plan(path, plan):
    # QJsonDocument::Indented output: sorted keys, four-space indent.
    path.write_text(json.dumps(plan, indent=4, sort_keys=True) + "\n")
    return path
