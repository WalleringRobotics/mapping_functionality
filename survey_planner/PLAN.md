# Survey planner: plan for review

**Status:** the mission is defined in [Mission](#mission): one fixed camera facing
forward and about 30° down, two mission types (structure, ground), and the aircraft's
yaw planned as part of each. **Phase 1 as implemented on this branch
(`wr-map survey-plan`) assumes a nadir camera** and is wrong for this platform (see
[Phase 1 status](#phase-1-status)). The design below is for review before more code is
written. Implementing agents should also read [NOTES.md](NOTES.md).

Both files are review documents and are removed when the last phase merges.

## Mission

**The platform carries one camera, fixed to the airframe, facing forward and pitched
about 30° below the horizon by the mount. There is no gimbal. The only way to point
the camera is to yaw the aircraft, so the yaw is part of every plan.**

There are two missions (operator, 2026-11-22):

| | Mission 1: structure | Mission 2: ground and low vegetation |
|---|---|---|
| Target | A building or other structure | A field; mostly small vegetation |
| Yaw | **Must be planned.** Either the yaw sweeps to stay on the structure, or straight "fly-bys" with a fixed yaw | **Fixed for the whole grid.** No sweeping expected |
| Path | Around or along the structure at a stand-off distance, at one or more heights | Parallel lines over the field |

Rules for both:

1. **Yaw is a planned output, not a side effect.** Every leg carries the heading that
   puts the target in the frame, and the plan says how PX4 is commanded to hold it.
2. **Yaw must match the motion.** Images taken while the heading does not match are
   useless: the target is out of frame, or smeared. At the reviewed 1000 µs exposure
   and 4.79 mm lens (3093 px focal length), an unmatched yaw rate of 9.3 °/s moves the
   image 0.5 px during the exposure, which is the blur limit already used for speed.
3. **A plan is tailored to three inputs**, without entering values by hand in
   QGroundControl:

   | Input | What it is | Where it comes from |
   |---|---|---|
   | **Optics** | Sensor, focal length, **mounting (forward, fixed, measured pitch)**, orientation, frame rate, fixed exposure, shutter type | Camera definition and capture profile in `configs/` |
   | **Aircraft** | Vehicle type, speed limit, acceleration, **yaw rate and yaw acceleration limits**, acceptance radius, climb and descent rates, return height | A reviewed aircraft definition in `configs/` |
   | **Target** | Mission type, boundary or structure outline and height, GSD or stand-off distance, overlaps, relief, time budget | The request |

4. **The plan is checked for pointing before anyone loads it.** A plan is written only
   if, for every frame the camera would take on a leg, the target is inside the usable
   part of the frame at the planned position and yaw, the yaw rate and acceleration
   are within the aircraft's limits, and blur is within the limit. Coverage is counted
   from those frames.
5. **The review shows the pointing:** view direction and footprint along the path.

**Later, optional: follow mode.** The camera feed steers the yaw to keep a target in
frame during flight. That is closed-loop control on the vehicle, a separate component
with its own safety case. The planned-yaw missions must not depend on it.

**Then, as before:** a review page on satellite imagery and an MCP server.

### What the camera sees at 30° down

OAK RGB 12 MP (66.5° × 52.3° field of view), image width horizontal, 30 m above level
ground:

| Part of the frame | Angle below horizon | Ground distance ahead | Width seen | GSD across / along the ground |
|---|---|---|---|---|
| Bottom edge | 56.2° | 20 m | 43 m | 1.1 / 1.6 cm |
| Centre | 30.0° | 52 m | 79 m | 1.9 / 3.9 cm |
| Top edge | 3.8° | 448 m | — | useless |

For comparison, a nadir camera at 30 m gives 0.97 cm. Consequences:

- **Only the lower part of the frame is useful for ground.** Along-ground GSD passes
  twice the nadir value 29 m ahead and three times at 42 m. The plan takes a GSD limit
  and counts only the band of the frame inside it.
- **The footprint is ahead of the aircraft, not under it.** Nothing closer than 20 m
  ahead is seen at 30 m height. Lines must start that much earlier.
- **The view direction matters.** Images of the same ground taken looking opposite
  ways are 120° apart and will not match each other, least of all on vegetation.

### Mission 2: ground, fixed yaw (proposal)

- **One heading for the whole grid.** If the aircraft yawed along each leg, as PX4
  does by default and Phase 1 assumes, neighbouring lines would look in opposite
  directions and the side overlap between them would be useless. With a fixed heading
  every image looks the same way; the aircraft flies alternate lines backwards.
  *Alternative:* fly every line in the same direction and return empty, at about twice
  the flight time.
- **Rules that change from Phase 1:** footprint (a trapezoid ahead of the aircraft,
  limited to the usable band), line spacing (from the band's narrowest width), frame
  spacing (from the band's depth), line start and end (shifted by the look-ahead
  distance, different for forward and backward lines), blur (lower than nadir at the
  same speed, since the ground is further away), relief.
- **Rules that carry over:** boundary import, speed and turnaround, fence, time budget
  and alternatives, the independent coverage test (with the new footprint), bundle.
- **Optional second grid** with the opposite heading, for the sides of vegetation the
  first grid cannot see.

### Mission 1: structure (proposal)

Two patterns, both using the same pointing test:

- **Yaw sweep (orbit).** The aircraft flies around the structure at a stand-off
  distance, one loop per height, with the yaw held on the structure by a
  region-of-interest command. Overlap comes from the angular step between frames and
  the height step between loops.
- **Fly-bys, fixed yaw.** Straight passes parallel to each face, flown sideways with
  the heading fixed square to the face. Simpler to check and no yaw rate during
  imaging, but each face is a separate set of passes and corners are covered less
  well.

Recommended order: fly-bys first (they share the fixed-yaw mechanism with mission 2),
then the orbit.

At 30° down the camera looks at a wall obliquely from above: a pass must be higher
than the part of the wall it images, and the lowest part of a wall needs a larger
stand-off. Roofs are well covered.

### How PX4 is told the yaw

Checked in PX4 v1.17.0 source (the version `sitl.yml` builds) and QGroundControl
v5.1.5; **not yet flown in SIH**:

- **Fixed yaw:** a waypoint with a yaw value (`param4`) is flown at that heading; PX4
  uses the heading mode only when the waypoint's yaw is not set
  (`FlightTaskAuto.cpp`). QGroundControl's Survey item cannot carry a yaw, so these
  plans are simple waypoints, not a Survey item.
- **Yaw on a point:** with `MIS_MNT_YAW_CTL` 0 (default) on a multirotor, an active
  `MAV_CMD_DO_SET_ROI_LOCATION` makes PX4 recompute the heading towards that location
  continuously during the mission (`MissionBase::heading_sp_update`). It stops
  updating within the acceptance radius of the point.
- **Yaw square to the path:** `MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET` with a 90° yaw offset
  is what QGroundControl's own Structure Scan writes, and PX4 applies the offset to
  the vehicle heading.
- **Rate limits:** `MPC_YAWRAUTO_MAX` (default 60 °/s) and `MPC_YAWRAUTO_ACC`
  (default 20 °/s²). These go into the aircraft definition.
- **The heading-mode parameter is `MPC_YAW_MODE` in v1.17.0.** This repository's
  documents and camera definition call it `MIS_YAWMODE`, its older name.

### Consequences for what exists

- **`configs/qgc-oak-rgb-12mp.json` is wrong about the mounting.** It says nadir with
  yaw along each leg. It needs a reviewed correction with the measured pitch ("about
  30°" is not yet a measurement; a few degrees changes the footprint noticeably).
- **`wr-map survey-check` checks the wrong geometry for this platform**, and accepts
  only a Survey item. It needs to accept the waypoint plans above and check them with
  the oblique footprint. `capture --survey-check` and `survey-legs` depend on it.
- **Phase 1's design rules and coverage test use the nadir footprint.** They are
  reworked for mission 2, not thrown away.

### Open for the operator

1. Mission 2: fixed heading with alternate lines flown backwards (recommended), or
   all lines in the same direction at about twice the flight time?
2. Mission 1: fly-bys first, then the orbit, as recommended?
3. Is the nadir mounting in the camera definition simply out of date, or is there a
   second mount? If the camera can also be mounted nadir, Phase 1 stays valid for
   that mount.
4. Can the pitch be measured (for example from the rig calibration) rather than taken
   as 30°?

## Why in this repository

- The camera is already reviewed here.
- `wr-map survey-check` already defines what a consistent plan is.
- Everything downstream (`capture --survey-check`, `survey-legs`,
  `bag-import --survey-legs`) already consumes QGroundControl Survey plans.

## Decision this reverses

[`docs/survey-planning.md`](../docs/survey-planning.md) says *"This repository does
not draw grids."* QGroundControl remains the flight UI and the upload path, but stops
being the only planner. The generator writes the same Survey item QGroundControl 5.1
saves (`SurveyComplexItem` v5, `TransectStyleComplexItem` v2, `CameraCalc` v2, as
already mirrored in `tests/qgc_plans.py`), so the checker treats both routes the same
way. The guide is updated in the implementation commits.

### What QGroundControl does with a generated plan

Checked in QGroundControl v5.1.5 source:

- **On load and upload it uses the saved items as they are.**
  `TransectStyleComplexItem::appendMissionItems` appends the loaded mission items when
  there are any, and only builds its own otherwise. The vehicle therefore flies the
  lines this generator wrote.
- **Any edit to the Survey discards them.**
  `SurveyComplexItem::_rebuildTransectsPhase1WorkerSinglePolygon` clears the loaded
  items and QGroundControl regenerates the transects with its own algorithm.

Consequences:

- The generator does **not** port QGroundControl's transect algorithm. It uses its
  own simple line generator, so no QGroundControl code is copied.
- A plan edited in QGroundControl is a different plan. It must go through
  `wr-map survey-check` again, which the existing capture workflow already enforces by
  hash. The saved polygon, angle and spacing are kept consistent so that a regenerated
  plan is close to the original, but it is not guaranteed to be the same.

## Where it fits

```text
optics   (configs/: camera + capture profile) ─┐
aircraft (configs/: aircraft definition) ──────┤
target   (boundary, GSD, overlaps, relief, ────┼─▶ wr-map survey-plan ─▶ survey.plan + review bundle
          type, time budget)                   ┘          │
                                                          ├─▶ wr-map survey-check   (must pass)
                                                          └─▶ coverage test         (must pass)
                                                                     │
                              operator reviews plan and coverage ◀───┘
                                         │
                                         ▼
                 QGroundControl: open, inspect, upload  (operator action)
                                         │
                                         ▼
       wr-map capture --survey-check ─▶ survey-legs ─▶ bag-import --survey-legs
```

## Where it runs

Plans are made on the home workstation or on the laptop at the site. Both also run
QGroundControl and can have a link to the aircraft. The planner never runs on the
Jetson.

- **Install:** `pip install -e '.[planner,planner-ui]'` in a plain virtual
  environment. No ROS, Docker image or camera drivers are needed, so the planner
  modules import nothing from the capture side.
- **Same machine as QGroundControl:** the output directory is on the machine that
  uploads, so the hand-over is File → Open in QGroundControl. No file transfer, and
  the checked `survey.plan` is the file that gets uploaded.
- **On site, possibly without internet:** planning, both tests and the review page
  work offline. Only the satellite basemap needs a connection; without one the layers
  draw on a plain background. A plan made at home can be re-run on site with a
  corrected take-off point in seconds.
- **Aircraft values:** because QGroundControl on the same machine is connected to the
  aircraft, saving its parameter file (Phase 4) is one step, and it captures the
  values of the aircraft that will actually fly. Reading them live through
  QGroundControl's MAVLink forwarding is the later option; it would be read-only and
  whether forwarding carries parameter requests is to be verified.

## Aircraft definition

A new reviewed file, `configs/px4-multirotor.template.json`, in the style of
`configs/plane-limits.template.json`: null entries fail, and the operator keeps a
filled private copy per vehicle.

| Field | PX4 parameter it mirrors | Used for |
|---|---|---|
| `vehicle_type` | `MAV_TYPE` | Plan `vehicleType`; the checker requires a multirotor |
| `max_survey_speed_m_s` | at most `MPC_XY_CRUISE` | Speed rule |
| `accel_m_s2` | `MPC_ACC_HOR` | Turnaround length |
| `acceptance_radius_m` | `NAV_ACC_RAD` | Turnaround length; along-track edge margin |
| `climb_m_s`, `descent_m_s` | `MPC_Z_V_AUTO_UP`, `MPC_Z_V_AUTO_DN` | Compared with the checker's time-estimate rates |
| `return_height_m` | `RTL_RETURN_ALT` | Height limit and fence review |

Values are typed in from the vehicle in Phase 1. Phase 4 fills the same file from a
parameter file saved by QGroundControl.

## Planning rules

The OAK free-runs, so forward overlap comes from speed and frame rate, not from
camera-trigger distance. Formulas follow `survey_plan.py` exactly.

| # | Rule | Source of the limit |
|---|---|---|
| 1 | Height from target GSD: `h = GSD × ImageWidth × FocalLength / SensorWidth`, within 10–120 m | Camera definition; checker height limits |
| 2 | Terrain relief is a required input, ± metres about take-off level (0 must be stated, not assumed). Overlap and blur are evaluated at the lowest height above ground, `h − relief`; GSD at the highest, `h + relief` | Target |
| 3 | Line spacing = across-track footprint at `h − relief` × (1 − side overlap); Landscape puts image width across track | Camera definition |
| 4 | Speed = the lowest of the blur limit `0.5 px × GSD / exposure` and the forward-overlap limit `fps × along-track footprint × (1 − front overlap)`, both at `h − relief`, and the aircraft's survey speed limit. The result is rounded **down** to 0.1 m/s. The review names which limit binds | Capture profile; checker blur limit; aircraft definition |
| 5 | Explicit absolute groundspeed command before the survey; one speed for the whole mission, because the checker applies the highest commanded speed to blur | Checker requirement |
| 6 | Transect angle: automatic, choosing the shortest estimated flight time; can be overridden | Planner |
| 7 | Edge margin is derived, not chosen: the smallest margin with which the coverage test passes on the original boundary. Across track that is about one line spacing; along track about the acceptance radius. The saved Survey polygon is the buffered one | Coverage test |
| 8 | Turnaround on every leg: at least `v² / (2 × accel) + acceptance radius`, plus margin, so the aircraft is on the line and at speed at the leg entry | Aircraft definition |
| 9 | Start at the corner nearest the take-off point; end with Return | Planner |
| 10 | Inclusion geofence = the convex hull of home and every mission point, buffered by a margin. A convex fence contains every straight transit and the return path, which the checker does not test (it tests points only) | Checker requirement; planner |
| 11 | Height relative to take-off in v1; terrain following later | Checker |
| 12 | Target type sets defaults: **terrain** is a single grid; **building** adds the crossing grid (`Refly90Degrees`) and higher overlaps. The types match `configs/process-terrain.json` and `configs/process-building.json` | Target |
| 13 | Flight time is the checker's estimate and must fit the usable budget after reserve. If it does not, the planner reports the two nearest plans that do fit: the largest area at this GSD, and the finest GSD for this area | Checker estimate; planner |
| 14 | No camera-trigger commands are written. The OAK does not use them, and a warning that fires on every plan trains people to ignore warnings. QGroundControl adds them back if the Survey is edited there, which is harmless | Planner |
| 15 | Report the maximum distance from home; line-of-sight judgement stays with the operator | Review |

### Worked example: OAK RGB 12 MP at 30 m, level ground

| Quantity | Value |
|---|---|
| GSD | 0.97 cm/px |
| Footprint, across × along track | 39.3 m × 29.5 m |
| Line spacing at 70 % side overlap | 11.8 m |
| Speed allowed by 80 % forward overlap at 2 fps | 11.8 m/s |
| Speed allowed by 0.5 px blur at 1000 µs | 4.84 m/s → **4.8 m/s, binding** |
| Frame spacing at 4.8 m/s | 2.4 m, i.e. 92 % forward overlap |
| Distance to reach 4.8 m/s at 3 m/s² | 3.8 m; with a 2 m acceptance radius the turnaround is at least 5.8 m |

With ± 5 m of relief the same 11.8 m spacing would give only 64 % side overlap over
the high ground, and 4.8 m/s would give 0.6 px of blur. Rule 2 prevents both: spacing
becomes 9.8 m and speed 4.0 m/s.

For comparison, the hand-made plan that prompted this work used 25 m line spacing at
30 m: about 36 % side overlap, not enough for a reliable reconstruction.

### Sensitivities the review reports

These are not changed by the planner. They are shown because they limit the result
more than the planning choices do.

- **Exposure.** Blur binds only because exposure is fixed at 1000 µs. At 500 µs the
  limit at 30 m is 9.7 m/s, roughly twice the area per sortie. Changing it is a
  capture-profile decision with its own review.
- **Rolling shutter.** The RGB sensor is rolling shutter and its readout time is
  unmeasured (#14). The review reports skew as `speed × readout / GSD`: at 30 m and
  4.8 m/s that is 0.5 px per millisecond of readout, so 20 ms would be 10 px, far
  more than the 0.5 px exposure blur that sets the speed. It is reported only, until
  #14 gives a measured readout; then it becomes a limit.
- **Frame count.** 92 % forward overlap is more than processing needs. The review
  reports the frame count and the overlap that every second and third frame would
  give.

## Coverage test

`survey-check` confirms that a plan is internally consistent. It does not check that
the lines cover the field or are spaced as the plan claims, because QGroundControl
used to guarantee that. With our own line generator, a geometry bug would pass the
check. The coverage test closes that gap, and is written independently of the line
generator: it reads only the finished `.plan`.

- Sample the **original** boundary's interior and edge on a ground grid (1 m default).
- Place frames along each leg at the frame spacing, with the worst-case phase, and only
  between the leg entry and the exit minus the acceptance radius. That is the window
  `survey-legs` will keep.
- Project each frame's footprint at `h − relief`, oriented along its leg.
- For every sample, count the distinct lines and the frames that see it.
- **Pass:** every sample is seen from at least `floor(1 / (1 − side overlap))` lines
  and `floor(1 / (1 − front overlap))` frames, the same minimum the interior gets.
  At 70 % / 80 % that is 3 lines and 5 frames.

The same grid is the coverage layer on the review map, so what the operator sees is
what was tested.

## Outputs

One new directory per plan, like `survey-check`:

| File | Content |
|---|---|
| `survey.plan` | QGroundControl Plan, deterministic: the same inputs give the same bytes |
| `survey-check.json` | The unmodified `wr-map survey-check` report |
| `review.json` | Inputs with hashes, derived values, binding limits, sensitivities, coverage result, per-leg table (length, heading, entry/exit), frame count, path length, flight time vs budget, maximum distance from home, warnings |
| `review.geojson` | Layers for the map: boundary, buffered survey polygon, legs, turnarounds, frame centres, sample footprints, fence, home |
| `coverage.png` | The coverage grid as a georeferenced image; its bounds are in `review.json` |
| `review.kml` | The same layers and the coverage image, for Google Earth |

## Interfaces

All interfaces call one pure function, `plan_survey(request) -> PlanBundle`. It has no
network or vehicle IO.

1. **CLI**, the first deliverable:

   ```bash
   wr-map survey-plan --boundary field.kml --home 48.40,12.90 --home-amsl-m 450 \
     --aircraft /data/vehicles/hexa-1.json --gsd-cm 1.0 --front 80 --side 70 \
     --relief-m 2 --target terrain --flight-time-budget-min 15 \
     --output runs/plans/field-001
   ```

2. **Review page:** a local server bound to 127.0.0.1 (FastAPI) and one HTML page,
   with Leaflet vendored so there is no build step. Its centre is a satellite map:
   - **Draw the job on the imagery:** search or pan to the site, draw the boundary and
     the take-off point on the satellite image, or load a KML. Set the target beside
     the map; the plan regenerates on each change.
   - **See predicted performance on the same imagery:** a coverage heat map (images
     per ground point, with anything below the pass threshold in a warning colour),
     sample footprints to scale, legs with direction and numbers, turnarounds, fence
     and home.
   - a 2D fly-through: a time slider moving the aircraft along the path and dropping
     frame positions as they are captured
   - a metrics table with the binding limits, sensitivities, warnings and both test
     results
   - downloading the `.plan` and the `.kml`

   **Imagery source.** The first version uses one keyless satellite layer (Esri World
   Imagery; licence terms to be confirmed in Phase 2). Without a connection every
   layer still draws, on a plain background. The provider is a single setting, so
   keyed providers can be added later without changing the page:
   - Bing aerial, which is QGroundControl's default map (Bing, Hybrid) and so the
     closest match to what the operator sees there
   - Google Maps satellite, which Google allows only through its own Maps JavaScript
     API with a billed key

   The basemap is a backdrop only. Providers can differ by a few metres, so a boundary
   traced on one may look shifted on another; the coordinates in the plan are what the
   aircraft flies.

   Independently of the page, `review.kml` opens in Google Earth, which shows the plan
   on Google imagery with 3D terrain and needs no key.

3. **MCP server** (stdio, Python MCP SDK):

   | Tool | Purpose |
   |---|---|
   | `list_cameras`, `list_aircraft` | Available configurations |
   | `plan_survey` | Generate a plan; returns its id, summary and review URL |
   | `check_plan` | Run `survey-check` and the coverage test on any `.plan` |

   None of these tools upload, arm or write vehicle parameters. An agent such as
   Claude Code drives the planner through these tools and hands the person the review
   URL; the person approves on the review page.

4. **Built-in agent: deferred.** The trade-offs between GSD, area and time are closed
   form and rule 13 solves them deterministically. An outside agent already has the
   MCP tools. A built-in agent is reconsidered once the review page and MCP server
   have been used on real jobs.

## Aircraft values from QGroundControl

QGroundControl has no API for its plan editor; aircraft values belong to the vehicle.

- **Phase 1:** the aircraft definition above, filled in by hand.
- **Phase 4:** fill it from a parameter file saved from QGroundControl (Vehicle
  Setup → Parameters → Tools → Save to file). An existing `.plan` can supply the home
  point, fence and boundary.
- **Later:** read the same values live over MAVLink (read-only, via mavlink-router).

## Code layout

The core lives beside its checker in the main package, following
`calibration_mission.py`. There is no second package, CI job or version to drift.

```text
src/wallering_mapping/
  survey_design.py        rules 1–15: height, spacing, speed, margins, solver
  survey_lines.py         KML / GeoJSON / .plan boundary import, local frame, buffer, transects
  survey_coverage.py      coverage test; reads only a finished .plan
  survey_write.py         QGroundControl 5.1 Survey writer, review bundle, KML
  cli.py                  wr-map survey-plan
  planner/                optional: api.py, mcp_server.py, web/ (index.html, vendored Leaflet)
configs/
  px4-multirotor.template.json
```

- It reuses `wallering_mapping.survey_plan` (`read_camera`, `read_profile`,
  `check_survey`, `local_xy`, `DEFAULT_LIMITS`) rather than copying its formulas.
- `tests/qgc_plans.py` is generalised into the writer rather than duplicated.
- New optional dependencies in `pyproject.toml`, so the Jetson image is unchanged:
  `planner` (shapely, for buffering concave boundaries) and `planner-ui` (FastAPI,
  uvicorn, MCP SDK).

## Phases and commits

Each commit is small and passes tests and ruff.

**Phase 1, core (this PR after review)**
1. Aircraft definition template and reader
2. Design rules, with the worked example and the relief example as tests
3. Boundary import, local frame and buffer
4. Line generator: convex, concave and rotated fields (fields with holes refused); angle and start selection
5. Coverage test, with tests that a deliberately wrong spacing and a missing edge line fail it
6. QGroundControl Survey writer, with a round-trip test through `check_survey`
7. `wr-map survey-plan` and the review bundle, including KML
8. Extend `sitl.yml` to fly a generated plan
9. Update `docs/survey-planning.md`; remove this file and NOTES.md

Later phases follow, each proposed as its own PR:
- **Phase 2:** review page with the satellite map
- **Phase 3:** MCP server
- **Phase 4:** QGroundControl parameter-file import
- **Later:** built-in agent, terrain following from a DEM, battery and endurance model, splitting a field across sorties, live MAVLink read

## Acceptance

- [ ] Every generated plan passes `wr-map survey-check` with the default camera and profile, unmodified and without warnings other than relative height
- [ ] Every generated plan passes the coverage test on the original boundary
- [ ] Unit tests for geometry and limits; checker round-trip; deterministic output
- [ ] QGroundControl 5.1 opens a generated plan, shows the same transects and uploads the saved items unchanged; needs the operator's QGroundControl, which is not in CI
- [ ] PX4 SIH flies a generated plan to completion, and at every leg entry the simulated aircraft is within tolerance of the line and of the commanded speed
- [ ] Every generated plan carries a yaw for every leg, and passes a pointing test: target in frame for every frame on a leg, yaw rate within the aircraft's limits, blur within the limit
- [ ] PX4 SIH shows the simulated aircraft's heading within tolerance of the planned yaw throughout every leg
- [ ] `flight_ready` stays false; no interface uploads, arms or writes parameters

## Out of scope

Upload and arming, airspace or site approval, terrain following in v1, ArduPilot Plane
(which has its own handoff path), and camera-driven follow mode (see
[Mission](#mission)).

## Phase 1 status

**Phase 1 plans a nadir-camera survey with yaw along each leg. With the platform's
camera (forward, 30° down) the aircraft would image ground 20 m and more ahead of each
line, coarser than planned and looking in opposite directions on neighbouring lines.
Its coverage and overlap figures do not hold; do not fly its plans for mapping.** The
boundary import, speed and turnaround rules, fence, time-budget refusal, review bundle
and the independent test on the finished plan carry over to mission 2; the footprint,
line placement and plan format change.

Done, with tests, in `src/wallering_mapping/`:

| Module | Content |
|---|---|
| `survey_design.py` | Aircraft definition reader; rules for height, spacing, speed, turnaround and margins |
| `survey_lines.py` | KML / GeoJSON / `.plan` boundary import, line layout, angle and start selection, fence |
| `survey_coverage.py` | Coverage test; reads only a finished `.plan` |
| `survey_write.py` | QGroundControl 5.1 Survey writer |
| `survey_generate.py` | `plan_survey()`, time-budget alternatives, review bundle, `wr-map survey-plan` |

Differences from the plan above, found while building it:

- **Lines are not chords of a buffered boundary.** The first version clipped lines to
  a buffered polygon. It passed `survey-check` on every field and failed the coverage
  test on every field with a slanted or concave edge. A line is now flown wherever its
  footprint touches the field, and continues past the field's sides only as far as the
  edge needs: 0.33 of a line spacing at 70 % side overlap, not a full spacing. Along
  track the margin is the acceptance radius when blur binds, and up to half a
  footprint when forward overlap binds. The saved Survey polygon is the outline those
  lines are chords of.
- **Coverage counts lines, each with enough frames.** A line sees a ground point only
  if the required number of its frames contain the point. Every point needs the
  required number of such lines.
- **Saved side overlap.** QGroundControl derives spacing from the overlap at the
  nominal height, so with relief the saved overlap is the nominal-height value that
  gives the spacing needed over high ground. The requested overlap is in `review.json`.
- **`tests/qgc_plans.py` is unchanged.** It stays an independent builder of
  QGroundControl-shaped plans, and the coverage tests use it to show that lines clipped
  to the boundary fail at the edges.
- **The coarser-GSD alternative** is found by stepping the height up, because a higher
  flight is not always quicker.

Not done:

- [ ] **PX4 SIH flight of a generated plan** (commit 8). `deploy/verify-calibration-sitl.py`
  accepts only the calibration mission's simple items. Extending it to a Survey, and to
  measuring speed and cross-track error at each leg entry, could not be run on the
  development machine and is left for its own change, validated in the `sitl.yml`
  workflow.
- [ ] **QGroundControl opens a generated plan** and uploads the saved items unchanged.
  Needs the operator's QGroundControl.

## Decided in review, 2026-11-22

| Question | Decision |
|---|---|
| Placement | Core in `src/wallering_mapping/` as `wr-map survey-plan`; web and MCP as optional extras |
| Edge margin | Derived from the coverage test (rule 7), not a constant |
| Trigger commands | Omitted (rule 14) |
| Built-in agent | Deferred |
| Transect algorithm | Our own; QGroundControl uploads saved items as they are |
| Map imagery | One keyless satellite layer in the first version; Bing and Google as later keyed options |

## Questions answered in review, 2026-11-22

All five were agreed as proposed: the overlaps below, home elevation and relief as
required inputs, a 20 m fence margin, shapely as an optional dependency, and one PR
per phase.

1. **Default overlaps:** 80 % forward and 70 % side for terrain, as in the test
   fixtures? What for building: 85 % / 75 % with the crossing grid?
2. **Home elevation and relief:** both required inputs in v1, as proposed, or looked
   up from a DEM?
3. **Fence margin:** default 20 m around the convex hull?
4. **Shapely** as an optional dependency for buffering concave boundaries, or refuse
   concave fields in v1 and avoid the dependency?
5. **PR split:** one PR per phase, as proposed?
