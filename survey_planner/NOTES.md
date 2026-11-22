# Handoff notes for the implementing agent

Context gathered while writing [PLAN.md](PLAN.md), so a fresh session does not have to
rediscover it. Read PLAN.md first, then these notes. Everything here was checked on
2026-11-22 unless marked unverified.

## What the operator asked for

Summarised from the operator's own request:

- **No hand-entry in QGroundControl.** Planning a survey there by hand is manual and
  will be repeated. It prompted this work: a 30 m Survey in Manual camera mode with
  25 m spacing and 25 m trigger distance, far too little overlap.
- **The tool:** an HTML front end with a Python backend.
  - An **MCP connection**, so an outside agent can say "these are my settings" and
    get a plan back.
  - Its **own agent**, which works out what is needed and produces a viable survey
    plan.
- **Reviewable output.** After the agent dialogue the operator sees the plan as a 2D
  overview or fly-through, with the relevant details: distances, flying speeds,
  headings, time of flight and other estimates. Endurance and battery are optional
  extras.
- **Core priority:** a flyable trajectory for the specific aircraft, and a sensible
  photogrammetry survey. Loading the result in QGroundControl matters; the
  aircraft's details should come from QGroundControl or the vehicle where possible.
- **Workflow:** small, explainable commits on `feat/survey-planner`, PR #33. Follow
  the repository's [AGENTS.md](../AGENTS.md): feature branch, push, and record
  validation results in the PR.

## Facts about this repository that shape the design

- `wr-map survey-check` (`src/wallering_mapping/survey_plan.py`) is the acceptance
  gate. It refuses:
  - anything but a QGroundControl **Survey** ComplexItem
  - camera values that differ from `configs/qgc-oak-rgb-12mp.json` by more than 0.5 %
  - a missing explicit groundspeed (`DO_CHANGE_SPEED`: param1 = 1, param3 = -1,
    param4 = 0) before the survey
  - a missing inclusion geofence around home and every mission point
  - blur over 0.5 px, or forward overlap at the profile frame rate below the planned
    overlap
  - a flight time over budget
  - items that do not match `VisualTransectPoints` in order

  It warns on camera-trigger commands, because the OAK free-runs.
- `tests/qgc_plans.py` already writes QGroundControl 5.1.5-shaped Survey plans:
  - `SurveyComplexItem` v5, `TransectStyleComplexItem` v2, `CameraCalc` v2
  - item order: turnaround → entry → trigger-distance → exit → trigger 0 → turnaround
  - output style: sorted keys, 4-space indent

  It only draws axis-aligned rectangles. Generalise this rather than starting over.
- Capture profile (`configs/oakd-ros.yaml`): RGB 12 MP, `i_fps: 2.0`, manual exposure
  `r_exposure: 1000` µs. Forward overlap therefore comes from speed ÷ fps, not from
  the trigger distance.
- The OAK focal length is **4.8 mm** (calibrated, `configs/`). An earlier estimate of
  4.6 mm from the 69° HFOV spec is wrong; do not use it.
- `src/wallering_mapping/calibration_mission.py` is an existing offline QGroundControl
  plan generator (for calibration flights). Reuse its conventions: explicit
  `--home` and `--home-amsl-m`, no vehicle IO, hashed output.
- `.github/workflows/sitl.yml` already generates a mission and flies it in PX4 SIH.
  Extend it for the "SIH flies a generated plan" acceptance item.
- The repository is Apache-2.0. CI runs ruff (`E4, E7, E9, F`, line length 100) and
  pytest on Python 3.10 and 3.12.

## QGroundControl plan loading (from QGroundControl source, master `2af411c`, 2026-11-18)

- **Mission** (`MissionController::_loadJsonMissionFileV2`):
  - required: `plannedHomePosition` (array, altitude required), `items`,
    `firmwareType`
  - optional: `vehicleType`, `cruiseSpeed`, `hoverSpeed`, `globalPlanAltitudeMode`
  - `firmwareType` is `MAV_AUTOPILOT`: **PX4 = 12, ArduPilot = 3**
- **SimpleItem** (`MissionItem::load`):
  - required: `type`, `frame`, `command`, `params` (exactly 7; params 1–4 a number or
    null), `autoContinue`
  - `doJumpId` is optional, and is used to resolve `DO_JUMP` targets; keep it unique
- **Altitude keys** (`SimpleMissionItem::load`): if any of `AltitudeMode`, `Altitude`
  or `AMSLAltAboveTerrain` is present, all three must be. Without them, QGroundControl
  derives relative or absolute from the MAV_FRAME.
- **`AltitudeFrame` enum** (`QGroundControlQmlGlobal.h`, also used for CameraCalc
  `DistanceMode`):

  | Value | Mode |
  |---|---|
  | 0 | Mixed |
  | 1 | Relative |
  | 2 | Absolute |
  | 3 | CalcAboveTerrain |
  | 4 | Terrain |
  | 5 | None |

  Relative is **1**, not 0.
- **Unverified, check next:** whether QGroundControl, when it loads a Survey, uploads
  the saved `TransectStyleComplexItem.Items` as-is or regenerates transects from the
  polygon. Read `TransectStyleComplexItem.cc` and `SurveyComplexItem.cc` at the
  **v5.1.5** tag, which the checker targets. This decides how closely the transect
  algorithm must match QGroundControl's.

## Existing tools reviewed and not adopted

- **HOT DroneTM `drone-flightplan`** (`hotosm/drone-tm`,
  `src/backend/packages/drone-flightplan`, v1.0.0, commit `ac59c92`, GPL-3.0). It has
  a QGroundControl exporter, GSD/overlap waylines and DEM terrain following. Read, not
  run. Pitfalls in its exporter, worth avoiding here too:
  - firmware constants swapped (`px4` writes `firmwareType` 3)
  - every item gets `doJumpId` 1
  - `AltitudeMode` 0, which is Mixed in QGroundControl
  - the QGROUNDCONTROL camera is a DJI Mini 4 Pro placeholder
  - pins `gdal==3.10.3`
  - plain waypoints only, so `survey-check` would refuse its plans
- **MAVLink MCP servers** (`ardupilot-mavlink-mcp`, `mavlink-mcp`, ion-g-ion
  `MAVLinkMCP`) do live vehicle control, not survey planning, and are out of scope.
  Any vehicle IO stays out of this tool; upload and arming remain operator actions.

## Environment notes from the first session

- In the hosted sandbox, running third-party code cloned from the internet was
  refused, so that code was only read. Installing ordinary PyPI dependencies for this
  repository's own code was not attempted.
- `gh pr create` fails there because GraphQL is blocked. The REST call
  `gh api repos/WalleringRobotics/mapping_functionality/pulls ...` works.
- QGroundControl and PX4 SITL are not installed in that sandbox. The "QGroundControl
  opens it" acceptance needs the operator's machine.

## Still open

The eight questions at the end of PLAN.md are unanswered as of this commit. Check
PR #33 for the operator's answers before starting Phase 1.
