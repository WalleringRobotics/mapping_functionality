"""Offline, bounded PX4 multicopter calibration mission drafts (no vehicle IO)."""

import hashlib
import json
import math
from pathlib import Path


def add_arguments(parser):
    parser.add_argument("--home", required=True, help="Explicit latitude,longitude in degrees")
    parser.add_argument("--home-amsl-m", required=True, type=float)
    parser.add_argument("--alt", required=True, type=float, help="Height above home in metres")
    parser.add_argument("--speed", required=True, type=float, help="Groundspeed in m/s")
    parser.add_argument("--radius", required=True, type=float, help="Pattern half-width in metres")
    parser.add_argument("--site-radius", required=True, type=float, help="Clear site radius about home")
    parser.add_argument("--margin", required=True, type=float, help="Reserved lateral margin in metres")
    parser.add_argument("--max-alt", required=True, type=float, help="Site ceiling above home in metres")
    parser.add_argument("--altitude-step", type=float, default=3)
    parser.add_argument("--yaw-hold", type=float, default=4)
    parser.add_argument("--acceptance-radius", type=float, default=.3,
                        help="Waypoint acceptance radius in metres; must separate figure-eight endpoints")
    parser.add_argument("--output", required=True, type=Path)


def run(args):
    try:
        home = tuple(float(part) for part in args.home.split(","))
    except ValueError as error:
        raise ValueError("--home must be latitude,longitude") from error
    return generate_mission(home, args.home_amsl_m, args.alt, args.speed, args.radius,
                            args.site_radius, args.margin, args.max_alt, args.output,
                            args.altitude_step, args.yaw_hold, args.acceptance_radius)


def generate_mission(home, home_amsl_m, alt, speed, radius, site_radius, margin, max_alt,
                     output, altitude_step=3, yaw_hold=4, acceptance_radius=.3):
    """Write a deterministic QGC plan and hash-bound phase map, refusing replacement.

    Bounds are engineering limits for a draft, not airspace/site approval. The
    circular horizontal fence describes the supplied clear site only. Its upload,
    enforcement, vertical limits and tracking margin require operator verification.
    """
    from pyproj import Geod

    values = {"home_amsl_m": home_amsl_m, "alt": alt, "speed": speed, "radius": radius,
              "site_radius": site_radius, "margin": margin, "max_alt": max_alt,
              "altitude_step": altitude_step, "yaw_hold": yaw_hold,
              "acceptance_radius": acceptance_radius}
    if len(home) != 2 or not all(math.isfinite(v) for v in (*home, *values.values())):
        raise ValueError("All site and mission inputs must be finite; home is latitude,longitude")
    latitude, longitude = home
    if not -80 <= latitude <= 80 or not -180 <= longitude <= 180:
        raise ValueError("Home latitude must be within +/-80 and longitude within +/-180 degrees")
    bounds = {"home_amsl_m": (-500, 8000), "alt": (5, 100), "speed": (.5, 5),
              "radius": (5, 100), "site_radius": (10, 500), "margin": (5, 100),
              "max_alt": (5, 120), "altitude_step": (1, 10), "yaw_hold": (2, 30),
              "acceptance_radius": (.1, 2)}
    for name, (lower, upper) in bounds.items():
        if not lower <= values[name] <= upper:
            raise ValueError(f"{name} must be in [{lower}, {upper}]")
    if radius + margin > site_radius:
        raise ValueError("Pattern radius plus margin exceeds clear site radius")
    if margin < 3 * speed:
        raise ValueError("Margin must reserve at least three seconds of travel at requested speed")
    if alt + altitude_step > max_alt:
        raise ValueError("Altitude step exceeds the supplied site ceiling above home")
    figure = [(radius * math.sin(step * math.tau / 24),
               radius * .5 * math.sin(2 * step * math.tau / 24)) for step in range(25)]
    if 2 * acceptance_radius >= min(math.dist(a, b) for a, b in zip(figure, figure[1:])):
        raise ValueError("Waypoint acceptance regions overlap adjacent figure-eight endpoints")
    output = Path(output).resolve()
    sidecar = output.with_suffix(output.suffix + ".phases.json")
    if output.suffix != ".plan":
        raise ValueError("Output must end in .plan")
    if output.exists() or sidecar.exists():
        raise FileExistsError("Mission output or phase sidecar already exists")
    geod = Geod(ellps="WGS84")
    items, phases = [], []

    def add(command, phase, east=0, north=0, altitude=alt, yaw=None, hold=0, params=None):
        if math.hypot(east, north) > site_radius - margin + 1e-8:
            raise ValueError("Waypoint exceeds clear site with margin")
        if not 0 <= altitude <= max_alt:
            raise ValueError("Waypoint exceeds relative altitude bounds")
        lon, lat, _ = geod.fwd(longitude, latitude, math.degrees(math.atan2(east, north)),
                               math.hypot(east, north))
        navigation = params is None
        item = {"type": "SimpleItem", "autoContinue": True, "doJumpId": len(items) + 1,
                "command": command, "frame": 3 if navigation else 2,
                "params": params if params is not None else
                [hold, acceptance_radius if command == 16 else 0, 0, yaw,
                 round(lat, 9), round(lon, 9), altitude]}
        if navigation:
            item.update(Altitude=altitude, AltitudeMode=1, AMSLAltAboveTerrain=None)
        phases.append({"mission_seq": len(items), "do_jump_id": len(items) + 1,
                       "phase": phase, "command": command, "east_m": east, "north_m": north,
                       "relative_alt_m": altitude if navigation else None})
        items.append(item)

    # An immediate command after takeoff can overwrite its reached result before
    # PX4 publishes it. Apply speed first so the next item requires actual flight.
    add(178, "speed", params=[1, speed, -1, 0, 0, 0, 0])
    add(22, "takeoff")
    for yaw in (0, 90, 180, -90, 0, -90, 180, 90, 0):
        add(16, "yaw", yaw=yaw, hold=yaw_hold)
    for lap in range(2):
        for step in range(1, 25):
            theta = (1 if lap == 0 else -1) * step * math.tau / 24
            add(16, "figure_eight", radius * math.sin(theta),
                radius * .5 * math.sin(2 * theta))
    for east, north in ((radius, 0), (-radius, 0), (radius, 0), (0, 0),
                        (0, radius), (0, -radius), (0, radius), (0, 0)):
        add(16, "translation", east, north, hold=2)
    for height in (alt + altitude_step, alt, alt + altitude_step, alt):
        add(16, "altitude", altitude=height, hold=3)
    add(16, "return", hold=5)
    add(21, "land", altitude=0)
    plan = {"fileType": "Plan", "groundStation": "QGroundControl", "version": 1,
            "geoFence": {"version": 2, "polygons": [], "circles": [{"version": 1,
                         "inclusion": True, "circle": {"center": list(home), "radius": site_radius}}]},
            "rallyPoints": {"version": 2, "points": []},
            "mission": {"version": 2, "firmwareType": 12, "vehicleType": 2,
                        "globalPlanAltitudeMode": 1, "cruiseSpeed": speed, "hoverSpeed": speed,
                        "plannedHomePosition": [latitude, longitude, home_amsl_m], "items": items}}
    encoded = json.dumps(plan, indent=2, allow_nan=False) + "\n"
    report = {"schema_version": 1, "plan_sha256": hashlib.sha256(encoded.encode()).hexdigest(),
              "home": list(home), "parameters": values, "mission_items": phases,
              "progress_topic": "/mavros/mission/reached",
              "progress_semantics": "Reached is an endpoint event, not phase onset; verify uploaded numbering",
              "sequence_mapping_verified": False,
              "acceptance": {"qgroundcontrol": "pending", "px4_sitl": "pending",
                             "recorder_solver_sitl": "pending", "operator_flight_signoff": "pending"},
              "flight_ready": False,
              "limitations": ["Nominal waypoints only; tracking, yaw completion and braking unverified",
                              "Relative altitude is above home, not terrain clearance",
                              "Per-axis excitation and lever observability require measured solver evidence"]}
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x") as file:
        file.write(encoded)
    with sidecar.open("x") as file:
        json.dump(report, file, indent=2, allow_nan=False)
        file.write("\n")
    return {"plan": str(output), "phases": str(sidecar), "plan_sha256": report["plan_sha256"],
            "mission_items": len(items), "flight_ready": False, "acceptance": report["acceptance"]}
