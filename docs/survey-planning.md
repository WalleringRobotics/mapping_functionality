# Survey planning with QGroundControl

The default workflow below is OAK/PX4. The explicit ArduPilot Plane path uses
`--handoff` and `--aircraft-limits`; see [the wing contract](survey-wing.md) for
its conservative supported subset and remaining acceptance.

QGroundControl's Survey pattern is the planner and the flight UI. This repository
does not draw grids. It checks the saved plan against the OAK capture profile, binds
the checked plan to the recording, verifies the mission PX4 actually holds, and
selects only the images flown on survey legs.

```text
QGroundControl Survey ──save .plan──▶ wr-map survey-check ──▶ checked/ (survey.plan, survey-check.json)
                                                                │
QGroundControl uploads to PX4 ◀──────────────────────────────────┘ same .plan
                                                                │
wr-map capture --survey-check checked/ ──▶ sealed session (plan, mission list, reached events)
                                                                │
wr-map survey-legs SESSION ──▶ survey-legs.json ──▶ wr-map bag-import --survey-legs
```

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
2 cm at 60 m. The table assumes the camera above mounted nadir and landscape, 70 %
side and 80 % forward overlap and a flat field. Survey time counts legs only; turns
and transit add roughly 10–20 %.

| Height above ground | GSD | Footprint | Line spacing | Minimum fps at 5 m/s | 10 ha at 3 m/s | 10 ha at 5 m/s |
|---|---|---|---|---|---|---|
| 30 m | 9.7 mm | 39 × 30 m | 11.8 m | 0.85 | 47 min | 28 min |
| 45 m | 14.5 mm | 59 × 44 m | 17.7 m | 0.57 | 31 min | 19 min |
| 60 m | 19.4 mm | 79 × 59 m | 23.6 m | 0.42 | 24 min | 14 min |

Covering about 10 ha within a 20-minute sortie needs roughly 60 m and 5 m/s, which
gives about 2 cm GSD; at 30 m a sortie covers about 3–4 ha. The profile's 2 fps is
enough at every height. At its 1 ms exposure, 5 m/s blurs 0.52 px at 30 m, just over
the check's 0.5 px default; 4.8 m/s passes. Rolling-shutter skew is not checked: a
30 ms readout, assumed until #14 measures it, smears about 8 px at 60 m and 5 m/s,
and ODM's rolling-shutter correction needs the measured readout time. Expected map
accuracy for each configuration is in
[RTK accuracy](rtk-accuracy.md#expected-accuracy-by-configuration).

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
- Only QGroundControl's Survey pattern is parsed. Plans are tested against MAVSDK's
  QGroundControl-saved sample and synthetic plans that follow QGroundControl 5.1.5's
  save code. A plan saved by the operator's own QGroundControl and a PX4 SIH run of
  it remain to be checked.
- The flight-time estimate ignores wind, battery state and PX4 tuning.
- RGB rolling-shutter readout is unmeasured (#14); the blur limit covers exposure only.

## Open-source tools evaluated

Evaluated on 2026-10-10, when QGroundControl was chosen as the planner. This
repository is Apache-2.0, so AGPL and GPL tools run as separate programs or
services and are never imported or vendored.

**Mission planning**

| Tool | Licence | Fit | Verdict |
|---|---|---|---|
| [QGroundControl](https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/plan_view/pattern_survey.html) 5.1.5 | Apache-2.0 / GPL-3.0 | PX4's reference ground station. Survey, corridor and structure scans; custom camera by sensor size, resolution and focal length; KML and SHP polygon import; terrain following; geofence. Writes a documented JSON `.plan` | **Planner and flight UI** |
| [ArduDeck](https://ardudeck.com/) beta 1.1 | GPL-3.0 | New cross-platform station with GSD-first survey planning, camera presets and a German surveying partner. PX4 support is labelled experimental | Watch; not for a first flight |
| ArduPilot Mission Planner | GPL-3.0 | Mature survey grid, but Windows-first and built around ArduPilot | Not for a PX4 vehicle |
| [Drone Tasking Manager](https://github.com/hotosm/drone-tm) `drone-flightplan` 1.0.0 | AGPL-3.0 | Python flight-plan generator from a GeoJSON area with terrain following. Exports QGC `.plan` as plain waypoints. Its QGC camera values are hardcoded DJI placeholders marked FIXME | Only if plans must be generated in scripts; contribute a custom-camera input upstream first |
| [Fields2Cover](https://github.com/Fields2Cover/Fields2Cover) | BSD-3-Clause | Field coverage-path library for ground vehicles: headlands, swaths, turns | Not needed: a multicopter has no turning-radius or headland constraint |

**Vehicle link and operator feedback**

| Tool | Licence | Use here |
|---|---|---|
| MAVROS `gcs_url` | GPL-3.0, LGPL-3.0 or BSD | MAVROS already owns the PX4 serial link. Its built-in GCS bridge can relay MAVLink to QGC over the Orin's network, so no second serial owner is needed |
| [mavlink-router](https://github.com/mavlink-router/mavlink-router) | Apache-2.0 | Alternative if several programs need the link and the MAVROS bridge is not enough |
| MAVROS `sys_status` status text and `play_tune` | GPL-3.0, LGPL-3.0 or BSD | Used by `capture --announce` for QGC messages and the vehicle buzzer |
| MAVROS `cam_imu_sync` | GPL-3.0, LGPL-3.0 or BSD | Records PX4's camera-trigger events, for distance triggering later. In the current image |
| [MAVSDK](https://github.com/mavlink/MAVSDK) `MissionRaw.import_qgroundcontrol_mission` | BSD-3-Clause | Parses QGC `.plan` files, including surveys, into mission items. `survey-check` is tested against its QGroundControl-saved sample; not needed for upload, which QGC does |

**Corrections and ground truth**

| Tool or service | Licence or terms | Use here |
|---|---|---|
| SAPOS Bayern HEPS | Paid registration with LDBV | 1–2 cm horizontal and 2–3 cm vertical in official ETRS89. The central caster offers **VRS mounts only** (`VRS_3_4G`, `VRS_3_2G`), and VRS requires the rover's NMEA GGA position. The repository's NTRIP bridge does not send GGA, and its fixed-base coordinate check does not fit a virtual station. Using SAPOS needs a VRS mode in the bridge |
| [RTKBase](https://github.com/Stefal/rtkbase) | AGPL-3.0 | Web-managed base station on a single-board computer with a second ZED-F9P. Serves a single-base NTRIP mount the current bridge already supports, logs raw data for PPK of the base, and works without mobile coverage at the field. Its coordinates must be tied to ETRS89, for example by a SAPOS-corrected occupation |
| RTKLIB `str2str` | BSD-2-Clause | Command-line NTRIP client and server, a fallback for relaying corrections |
| QGIS and QField | GPL-2.0+ | Draw or import field boundaries as KML or SHP for QGC, and manage GCP and checkpoint coordinates |

**Processing and review**

| Tool | Licence | Use here |
|---|---|---|
| [WebODM](https://github.com/WebODM/WebODM) and NodeODM | AGPL-3.0 | Browser UI for ODM tasks, results, measurements and the built-in GCP interface. Point it at a NodeODM running the same pinned ODM 3.6.2 image. Keep this repository's audited CLI as the path of record and use WebODM for viewing and GCP tagging |
| [PyODM](https://github.com/OpenDroneMap/PyODM) | BSD-3-Clause | Python client for NodeODM, if this repository should submit tasks to a shared NodeODM server |
| GCP Editor Pro | Fair Source 0.9, not open source | Faster GCP tagging, but its licence limits use; prefer WebODM's built-in interface |
| [Lichtblick](https://github.com/lichtblick-suite/lichtblick) | MPL-2.0 | Open-source continuation of Foxglove Studio for browsing MCAP recordings: images, IMU, GNSS and mission progress on one timeline |
| [PlotJuggler](https://github.com/facontidavide/PlotJuggler) | MPL-2.0 | Time-series plots from MCAP and PX4 ULog, useful for calibration and TIMESYNC review |
| [PX4 Flight Review](https://github.com/PX4/flight_review) | BSD-3-Clause | Standard ULog analysis, including the airframe vibration check |
| QGIS and CloudCompare | GPL | Inspect orthophotos, DSMs and point clouds |

**Later option: distance triggering.** PX4 can trigger the camera by distance and
publish trigger events, which `cam_imu_sync` already records. Capturing only the
frames the plan needs would cut the recorded volume, about 4.4 GiB per minute with
the default profile, by a large factor. It needs a trigger path into the OAK driver.
