# Survey planning with QGroundControl

The default workflow below is OAK/PX4. The explicit ArduPilot Plane path uses
`--handoff` and `--aircraft-limits`; see [the wing contract](survey-wing.md) for
its conservative supported subset and remaining acceptance.

A survey plan comes from one of two routes. `wr-map survey-plan` generates one from
the camera, the aircraft and the target; or you draw one by hand in QGroundControl's
Survey pattern. QGroundControl remains the flight UI and the only upload path. Either
way this repository checks the saved plan against the OAK capture profile, binds the
checked plan to the recording, verifies the mission PX4 actually holds, and selects
only the images flown on survey legs.

```text
wr-map survey-plan ──────────────────────────────────────▶ checked/ (also review files)
QGroundControl Survey ──save .plan──▶ wr-map survey-check ──▶ checked/ (survey.plan, survey-check.json)
                                                                │
QGroundControl uploads to PX4 ◀──────────────────────────────────┘ same .plan
                                                                │
wr-map capture --survey-check checked/ ──▶ sealed session (plan, mission list, reached events)
                                                                │
wr-map survey-legs SESSION ──▶ survey-legs.json ──▶ wr-map bag-import --survey-legs
```

## Generate the plan

This runs on the workstation or the site laptop, beside QGroundControl, never on the
Jetson. Install it once with `pip install -e '.[planner]'`; it needs no ROS or camera
drivers and works offline.

1. Copy [`configs/px4-multirotor.template.json`](../configs/px4-multirotor.template.json)
   and fill it in for the vehicle. Null entries are refused.

   | Field | PX4 parameter | Used for |
   |---|---|---|
   | `vehicle_type` | `MAV_TYPE` (2, 13, 14 or 15) | The plan's vehicle type |
   | `max_survey_speed_m_s` | at most `MPC_XY_CRUISE` | Survey speed |
   | `accel_m_s2` | `MPC_ACC_HOR` | Turnaround length |
   | `acceptance_radius_m` | `NAV_ACC_RAD` | Turnaround length; where frames stop before a leg exit |
   | `climb_m_s`, `descent_m_s` | `MPC_Z_V_AUTO_UP`, `MPC_Z_V_AUTO_DN` | Warned about if slower than the time estimate assumes |
   | `return_height_m` | `RTL_RETURN_ALT` | Refused above the height limit |

2. Save the field outline as KML or GeoJSON (one polygon, no holes), or point at an
   existing `.plan` with one Survey.

3. Generate:

   ```bash
   wr-map survey-plan --boundary field.kml --home 48.40,12.90 --home-amsl-m 450 \
     --aircraft /data/vehicles/hexa-1.json --gsd-cm 1.0 --relief-m 2 \
     --flight-time-budget-min 15 --output runs/plans/field-001
   ```

   - `--gsd-cm` or `--height-m`: exactly one. A GSD is held over the lowest ground.
   - `--relief-m`: how far the ground rises or falls from take-off level. It must be
     stated; `0` means level. Overlap and blur are held over the highest ground.
   - `--target terrain` (default: 80 % forward, 70 % side, one grid) or
     `--target building` (85 % / 75 %, with the crossing grid). `--front` and `--side`
     override the overlaps.
   - `--angle` fixes the leg bearing; without it the shortest flight is chosen.
   - `--fence-margin-m` (default 20) grows the inclusion fence around every mission point.

The rules it applies:

- **Speed** is the lowest of the motion-blur limit, the forward-overlap limit at the
  profile frame rate, and the aircraft limit, rounded down to 0.1 m/s. The OAK free-runs,
  so forward overlap comes from speed, not from a trigger distance. At 30 m the blur
  limit binds at 4.8 m/s, which gives about 92 % forward overlap.
- **Lines** are spaced for the side overlap over the highest ground. A line is flown
  wherever its footprint touches the field and runs past the field at both ends, so the
  edges get the same number of views as the interior.
- **Turnarounds** are long enough to reach speed, plus the acceptance radius, plus 2 m.
- **No camera-trigger commands** are written. QGroundControl adds them back if the
  Survey is edited there; the check then warns, which is harmless.
- **The fence** is the convex hull of the take-off point and every mission point, grown
  by the margin, so it contains every straight transit and the return path.

A plan is written only if it passes both tests:

- `survey-check`, exactly as in section 3, with no warning other than relative height.
- **The coverage test.** `survey-check` confirms a plan is consistent; it does not
  confirm that the lines cover the field. The coverage test reads only the finished plan
  and counts, for every square metre of the original boundary, the lines that see it
  with enough frames. Frames are counted from each leg entry to the exit less the
  acceptance radius, at the lowest height above ground. At 70 % / 80 % every point must
  be seen from 3 lines with 5 frames each.

If the flight does not fit the time budget, nothing is written. The command reports
the two nearest plans that do fit: a coarser GSD for the same field, and the largest
area at the requested GSD.

The output directory holds:

| File | Content |
|---|---|
| `survey.plan` | The plan to open and upload in QGroundControl. The same inputs give the same bytes |
| `survey-check.json` | The unmodified check report; `wr-map capture --survey-check` accepts this directory |
| `review.json` | Inputs with hashes, every derived value and the limit that binds, per-leg table, frame count, flight time against budget, maximum distance from home, warnings |
| `review.geojson` | Map layers: boundary, survey polygon, fence, path, legs, turnarounds, frame centres, sample footprints, home |
| `coverage.png` | Lines seeing each ground point: green at or above the requirement, red below |
| `review.kml` | The layers and the coverage image; open it in Google Earth to see the plan on imagery |

`review.json` also reports what limits the result more than the planning does: the
speed a halved exposure would allow, rolling-shutter skew per millisecond of readout
(unmeasured, #14), and the forward overlap if every second or third frame were used.

Open `survey.plan` in QGroundControl, inspect it, and upload it **without editing the
Survey**. QGroundControl 5.1 uploads a loaded Survey's saved items unchanged, but
regenerates the lines with its own algorithm on any edit; an edited plan is a
different plan and must go through section 3 again. Then continue at section 4.

Sections 1 to 3 describe the by-hand route.

## 1. Enter the camera once in QGroundControl

In a Survey item, choose **Camera → Custom Camera** and enter the values from
[`configs/qgc-oak-rgb-12mp.json`](../configs/qgc-oak-rgb-12mp.json). Save them as a
Survey preset so every plan uses the same camera.

| Field | Value |
|---|---|
| Sensor width | 6.29 mm |
| Sensor height | 4.71 mm |
| Image width | 4056 px |
| Image height | 3040 px |
| Focal length | 4.8 mm |
| Orientation | Landscape |

These come from DepthAI's 12 MP intrinsics (focal length 3093.2 px) and the IMX378's
1.55 µm pixels. QGroundControl keeps sensor size to 0.01 mm and focal length to
0.1 mm; the rounded values change GSD by 0.06 %. The 12 MP intrinsics are derived
from the factory 1080p calibration and remain unmeasured at 12 MP (#14).

"Landscape" means the image width lies across the flight track. That holds only
when the OAK is mounted nadir with its image width across the vehicle, and PX4
yaws along each leg (`MIS_YAWMODE` 0, the default).

## 2. Plan the survey

1. Import the field boundary with **Pattern → Survey → KML/SHP**, or draw it.
2. In **Mission Start**, set an explicit **flight speed**. Without it PX4 flies at its
   own cruise speed, which the check cannot see, so the check refuses the plan.
3. Set altitude, overlaps and a turnaround distance. Turnarounds let the vehicle
   reach speed before each leg. Do not enable hover-and-capture: the OAK runs at
   a fixed frame rate.
4. Draw an inclusion **geofence** around the field, take-off point and turnarounds.
5. Save the plan. The `.plan` file is what gets checked and uploaded.

For planning, GSD is about 0.32 mm per metre of height: about 1 cm at 30 m and
2 cm at 60 m. With 70 % side overlap, covering about 10 ha within a 20-minute
sortie needs roughly 60 m and 5 m/s. At 30 m a sortie covers about 3–4 ha.

## 3. Check the plan

```bash
wr-map survey-check field.plan --flight-time-budget-min 15 --output runs/plans/field-001
```

`--flight-time-budget-min` is the usable flight time after the battery reserve.
Defaults: `--camera configs/qgc-oak-rgb-12mp.json`, `--profile configs/oakd-ros.yaml`,
`--max-height-m 120`, `--max-blur-px 0.5`. Add `--target-gsd-cm` to enforce a GSD.

The check refuses a plan when any of these fail:

| Check | Rule |
|---|---|
| Format | QGroundControl Plan 1, mission 2, PX4 firmware, multirotor vehicle |
| Patterns | Survey only; corridor and structure scans are refused |
| Camera | Custom Camera values match the camera definition within 0.5 %, same orientation |
| Height | 10–120 m; relative or terrain altitude, not AMSL; every item below the limit |
| Speed | An explicit absolute groundspeed before horizontal travel; unchanged throttle |
| Overlap | Forward overlap at the profile frame rate is at least the planned overlap |
| Blur | Speed × fixed exposure ≤ 0.5 px |
| Consistency | QGC's saved line spacing agrees with the camera values |
| Flight time | Each segment uses its active commanded speed; estimate stays within the budget |
| Geofence | An inclusion fence contains home and every mission point |
| Transects | Every PX4 item matches the saved transects in order |

Warnings flag PX4 camera-trigger commands, which the free-running OAK does not use,
missing turnarounds, and heights relative to take-off. The output directory holds
an exact copy of the plan as `survey.plan` and the report as `survey-check.json`,
including the zero-based PX4 index, role and transect of every mission item.

A passing check is not airspace, site or operator approval, and `flight_ready`
stays false.

Speed commands with `-1` retain the previous speed. Default-speed resets,
relative speed changes and airspeed/climb-speed commands are not modelled by the
PX4 survey estimate and are refused. The maximum commanded speed remains the
conservative bound used for camera blur and overlap checks.
Unmodelled execution commands such as jumps, unlimited/counted loiters and delays,
and items requiring manual continuation, are refused rather than omitted from the
time budget.

## 4. Record

Upload the same `.plan` from QGroundControl, then record with MAVROS running:

```bash
wr-map capture --survey-check runs/plans/field-001 --announce \
  --output /mnt/nvme/mapping/field-001 --duration 900 --warmup 60
```

- `--survey-check` refuses a failed or modified check and copies the plan and report
  into the session. At readiness the recorder downloads the vehicle mission once
  through `/mavros/mission/pull`, which is read-only. It records the latched
  `/mavros/mission/waypoints` list with `/mavros/mission/reached`. A failed download is
  written to `mission-pull.json` and never blocks the recording.
- `--announce` sends "recording started" and "recording stopped" as MAVLink status
  text and plays a short tune on the vehicle buzzer. The outcome goes to
  `announce-*.json`. Announcements never stop or fail a recording. The stop notice
  is attempted for timed and operator-stopped recordings. Timed runs wait for the
  stop announcer after the minimum two-second postroll, with a ten-second deadline.
  Operator stops attempt it after launch exits and before sealing; keep MAVROS
  separately owned so the link remains available then. Missing subscribers or a
  stopped link are recorded in the announcement report, not reported as delivery.

The status text reaches QGroundControl only if PX4 forwards it from the companion
link to the ground-station link (`MAV_n_FORWARD` on both MAVLink instances). The
buzzer tune does not depend on forwarding. Changing those parameters is a vehicle
configuration change for the operator.

## 5. Extract survey legs and import their images

```bash
wr-map survey-legs /mnt/nvme/mapping/field-001 --output runs/field-001-legs
wr-map bag-import /mnt/nvme/mapping/field-001 --output runs/field-001-images \
  --survey-legs runs/field-001-legs/survey-legs.json
```

`survey-legs` compares the recorded vehicle mission with the checked plan: item count,
command, altitude frame, position within 2×10⁻⁷ degrees and altitude within 5 cm.
It uses the list in force when the mission first progressed, plus every later list.
An earlier list superseded by the operator's upload is ignored. Each leg's window
runs from the reached event of its entry waypoint to that of its exit waypoint.

A leg is qualified only when the numbering is verified and the event stream is
clean. A restarted or resumed mission, unknown indices or nonmonotonic clocks
disqualify every leg. Legs that were never finished, such as after an early
landing, are listed as errors, while completed legs stay qualified.

`bag-import --survey-legs` imports only frames whose ROS header time falls inside a
qualified window. It refuses a legs report from another recording. Bag ordinals
still number every source frame, and the manifest's `survey_selection` records
imported and skipped counts per stream.

## Limitations

- Reached events mark arrival within PX4's acceptance radius, received by MAVROS.
  They bound legs to within a fraction of a second, not exposure time.
- Survey-leg selection does not qualify camera geolocation; timing and rig
  calibration remain separate.
- **Generated plans are for a nadir camera with yaw along each leg.** The platform's
  camera is fixed, facing forward and about 30° down, so the ground it sees is ahead
  of the aircraft and the heading has to be planned. `wr-map survey-plan` does not do
  this yet; its coverage and overlap figures do not hold for that camera, so do not
  fly its plans for mapping. The requirement and the proposed design are in
  [`survey_planner/PLAN.md`](../survey_planner/PLAN.md#mission).
- Generated plans have not yet been opened in the operator's QGroundControl or flown
  in PX4 SIH. Coverage assumes a nadir camera, level flight, yaw along each leg and
  constant speed on the legs; it does not model wind, attitude or tracking error.
- The coverage test and the edge margins are for one camera height per plan. Terrain
  following is not generated.
- Only QGroundControl's Survey pattern is parsed. Plans are tested against MAVSDK's
  QGroundControl-saved sample and synthetic plans that follow QGroundControl 5.1.5's
  save code. A plan saved by the operator's own QGroundControl and a PX4 SIH run of
  it remain to be checked.
- The flight-time estimate ignores wind, battery state and PX4 tuning.
- RGB rolling-shutter readout is unmeasured (#14); the blur limit covers exposure only.
