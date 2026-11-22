# Survey planner: plan for review

**Status:** proposal for review. No code yet. Implementation commits follow on this
branch only after the plan is agreed. Implementing agents should also read
[NOTES.md](NOTES.md), which has the handoff context and verified format details.

## Goal

Generate a flyable photogrammetry survey for a specific aircraft and camera, without
entering values by hand in QGroundControl, and show it for review before anyone loads
it into QGroundControl.

- **Core, first:** field boundary plus target (GSD or height, overlaps) → a
  QGroundControl `.plan` that passes `wr-map survey-check` unmodified, plus a review
  of what will be flown.
- **Then:** a web review page, an MCP server so other agents can drive it, and a
  built-in agent that turns a brief into a checked plan.
- **Later, optional:** aircraft values read from QGroundControl, endurance/battery,
  terrain following, splitting a field across sorties.

## Why in this repository

- The camera is already reviewed here: `configs/qgc-oak-rgb-12mp.json` (sensor,
  focal length, orientation) and `configs/oakd-ros.yaml` (RGB 12 MP, 2 fps, fixed
  1000 µs exposure).
- `wr-map survey-check` already defines what an acceptable plan is. The generator's
  acceptance is simple: its output must pass that check unchanged.
- Everything downstream (`capture --survey-check`, `survey-legs`,
  `bag-import --survey-legs`) already consumes QGroundControl Survey plans.

## Decision this reverses

[`docs/survey-planning.md`](../docs/survey-planning.md) says *"This repository does
not draw grids."* QGroundControl remains the flight UI and the upload path, but stops
being the only planner. To keep both routes equivalent, the generator writes the same
Survey item QGroundControl 5.1 saves (`SurveyComplexItem` v5, `TransectStyleComplexItem`
v2, `CameraCalc` v2, as already mirrored in `tests/qgc_plans.py`). A generated plan
can therefore still be opened and edited in QGroundControl, and the checker treats
both routes the same way. The guide is updated in the implementation commits.

## Where it fits

```text
field boundary (KML / GeoJSON / .plan) ─┐
target: GSD or height, overlaps ────────┤
camera + capture profile (configs/) ────┼─▶ survey_planner ─▶ survey.plan + review bundle
aircraft limits (QGC parameter file) ───┘          │
                                                   └─▶ wr-map survey-check  (must pass)
                                                              │
                              operator reviews the plan ◀─────┘
                                         │
                                         ▼
                 QGroundControl: open, inspect, upload  (operator action)
                                         │
                                         ▼
       wr-map capture --survey-check ─▶ survey-legs ─▶ bag-import --survey-legs
```

## Planning rules

The OAK free-runs, so forward overlap comes from speed and frame rate, not from
camera-trigger distance. Formulas follow `survey_plan.py` exactly.

| # | Rule | Source of the limit |
|---|---|---|
| 1 | Height from target GSD: `h = GSD × ImageWidth × FocalLength / SensorWidth`, within 10–120 m | Camera definition; checker height limits |
| 2 | Line spacing = across-track footprint × (1 − side overlap); Landscape puts image width across track | Camera definition |
| 3 | Speed = the lowest of the blur limit `0.5 px × GSD / exposure`, the forward-overlap limit `fps × along-track footprint × (1 − front overlap)`, and the aircraft's survey speed limit. The review names which limit binds | Capture profile; checker blur limit; aircraft limits |
| 4 | Explicit absolute groundspeed command before the survey | Checker requirement |
| 5 | Transect angle: automatic, choosing the shortest estimated flight time; can be overridden | Planner |
| 6 | Edge margin: lay lines over the boundary buffered by a margin, so the field edge gets interior overlap. The saved Survey polygon is the buffered one; the original boundary stays in the review | Planner (default for review, below) |
| 7 | Turnaround on every leg, at least the distance to reach speed, `v² / (2 × MPC_ACC_HOR)`, plus margin; default 10 m | Aircraft limits; checker warning |
| 8 | Start at the corner nearest the take-off point; end with Return | Planner |
| 9 | Inclusion geofence around home and every mission point, with a margin | Checker requirement |
| 10 | Height relative to take-off in v1 (the checker warns that the ground must be level with take-off); terrain mode later | Checker |
| 11 | Estimated flight time within the usable budget after reserve; otherwise refuse and say what to change | Checker estimate |
| 12 | Report the maximum distance from home; line-of-sight judgement stays with the operator | Review |

### Worked example: OAK RGB 12 MP at 30 m

| Quantity | Value |
|---|---|
| GSD | 0.97 cm/px |
| Footprint, across × along track | 39.3 m × 29.5 m |
| Line spacing at 70 % side overlap | 11.8 m |
| Speed allowed by 80 % forward overlap at 2 fps | 11.8 m/s |
| Speed allowed by 0.5 px blur at 1000 µs | 4.8 m/s, **binding** |
| Frame spacing at 4.8 m/s | 2.4 m, i.e. 92 % forward overlap |
| Distance to reach 4.8 m/s at 3 m/s² | 3.9 m, inside a 10 m turnaround |

For comparison, the hand-made plan that prompted this work used 25 m line spacing at
30 m: about 36 % side overlap, not enough for a reliable reconstruction.

## Outputs

One new directory per plan, like `survey-check`:

| File | Content |
|---|---|
| `survey.plan` | QGroundControl Plan, deterministic: the same inputs give the same bytes |
| `survey-check.json` | The unmodified `wr-map survey-check` report |
| `review.json` | Inputs, derived values, binding limits, per-leg table (length, heading, entry/exit), frame count, path length, flight time vs budget, maximum distance from home, warnings |
| `review.geojson` | Layers for the map: boundary, buffered survey polygon, legs, turnarounds, frame centres, sample footprints, fence, home |

## Interfaces

All interfaces call one pure function, `plan_survey(request) -> PlanBundle`. It has no
network or vehicle IO.

1. **CLI**, the first deliverable:

   ```bash
   wr-survey plan --boundary field.kml --home 48.40,12.90 --home-amsl-m 450 \
     --gsd-cm 1.0 --front 80 --side 70 --budget-min 15 --output runs/plans/field-001
   ```

2. **Review page:** a local server (FastAPI) and one HTML page, with Leaflet vendored
   so there is no build step and nothing is fetched except map tiles. It provides:
   - drawing or loading the boundary and setting the target
   - a map of every `review.geojson` layer, with leg direction and numbers
   - a 2D fly-through: a time slider moving the aircraft along the path and dropping
     frame positions as they are captured
   - a metrics table with the binding limits, warnings and the check result
   - downloading the `.plan`

3. **MCP server** (stdio, Python MCP SDK):

   | Tool | Purpose |
   |---|---|
   | `list_cameras`, `list_aircraft` | Available configurations |
   | `plan_survey` | Generate a plan; returns its id, summary and review URL |
   | `check_plan` | Run `survey-check` on any `.plan` |
   | `design_survey` | Hand a brief to the built-in agent |

   None of these tools upload, arm or write vehicle parameters.

4. **Built-in agent:** a Claude API tool-use loop over the same tools. It turns a brief
   ("map the block north of the barn at 1 cm, 15 minutes of battery") into
   parameters, iterates when the check fails, explains trade-offs, and finishes with
   a review link. The person approves the plan on the review page. Model and API key
   come from the environment.

## Aircraft values from QGroundControl

QGroundControl has no API for its plan editor; aircraft values belong to the vehicle.

- **v1:** import a parameter file saved from QGroundControl (Vehicle Setup →
  Parameters → Tools → Save to file). The values used are `MPC_XY_CRUISE`,
  `MPC_XY_VEL_MAX`, `MPC_ACC_HOR`, `MPC_Z_V_AUTO_UP`, `MPC_Z_V_AUTO_DN`,
  `MIS_TAKEOFF_ALT`, `NAV_ACC_RAD` and `RTL_RETURN_ALT`; names are checked against the
  vehicle's firmware. An existing `.plan` can supply the home point, fence and
  boundary.
- **Later:** read the same values live over MAVLink (read-only, via mavlink-router).

## Folder layout

```text
survey_planner/
  PLAN.md                 this document
  README.md
  pyproject.toml          package wr-survey-planner; depends on wallering-mapping
  src/wr_survey_planner/
    boundary.py           KML / GeoJSON / .plan import, local metric frame, buffer
    coverage.py           transects, angle and start selection
    speed.py              blur, frame-rate and aircraft speed limits
    qgc_survey.py         QGroundControl 5.1 Survey writer
    vehicle_params.py     QGroundControl parameter-file import
    review.py             review.json / review.geojson
    service.py            plan_survey()
    cli.py
    api.py  mcp_server.py  agent.py
    web/                  index.html, vendored Leaflet
  tests/
```

It reuses `wallering_mapping.survey_plan` (`read_camera`, `read_profile`,
`check_survey`, `DEFAULT_LIMITS`) rather than copying its formulas. Transect
generation follows QGroundControl's Survey algorithm, so reopening a plan in
QGroundControl shows the same lines. QGroundControl is dual-licensed Apache-2.0 /
GPLv3; porting under this repository's Apache-2.0 licence is to be confirmed in
review.

## Phases and commits

Each commit is small and passes tests and ruff.

**Phase 1, core (this PR after review)**
1. Package skeleton and CI job
2. Boundary import and local frame
3. Camera and speed rules, with the worked example as a test
4. Transects: convex, concave and rotated fields (fields with holes refused); angle and start selection
5. QGroundControl Survey writer, with a round-trip test through `check_survey`
6. CLI and review bundle
7. Update `docs/survey-planning.md`

Later phases follow, each proposed as its own PR:
- **Phase 2:** review page
- **Phase 3:** MCP server
- **Phase 4:** built-in agent
- **Phase 5:** QGroundControl parameter-file import
- **Later:** terrain following from a DEM, battery and endurance model, splitting a field across sorties, live MAVLink read

## Acceptance

- [ ] Every generated plan passes `wr-map survey-check` with the default camera and profile, unmodified
- [ ] Unit tests for geometry and limits; checker round-trip; deterministic output
- [ ] QGroundControl 5.1 opens a generated plan and shows the same transects; needs the operator's QGroundControl, which is not in CI
- [ ] PX4 SIH flies a generated plan to completion, extending the existing `sitl.yml` workflow
- [ ] `flight_ready` stays false; no interface uploads, arms or writes parameters

## Out of scope

Upload and arming, airspace or site approval, terrain following in v1, ArduPilot Plane
(which has its own handoff path), and corridor or structure scans.

## Questions for the reviewer

1. **Placement:** a self-contained `survey_planner/` folder, as proposed. Or the core
   generator in `src/wallering_mapping/` as `wr-map survey-plan`, next to its checker,
   with only the web, MCP and agent layers in the folder?
2. **Default overlaps:** 80 % forward and 70 % side, as in the test fixtures?
3. **Default edge margin:** half a line spacing, one line spacing, or none? A larger
   margin costs flight time on small fields.
4. **Trigger commands:** QGroundControl's Survey writes per-transect camera-trigger
   commands, which the checker flags as unused by the OAK. Mirror QGroundControl
   exactly and accept the warning, or omit them?
5. **Home elevation:** a required input, like `calibrate-mission`, or looked up from a
   DEM?
6. **Fence margin:** default 20 m?
7. **Agent:** Claude API with the key from the environment; default model?
8. **PR split:** one PR per phase, as proposed?
