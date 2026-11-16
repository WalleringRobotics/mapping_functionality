# Technical review and phased testing plan, 2026-11-16

Reviewed `main` at `9af2625` on the x86-64 processing workstation, not on the Orin.
This document states what is outstanding and proposes a phased testing plan. It
changes no code, acceptance gate or qualification status. Proposed pass criteria
repeat existing issue acceptance unless marked **proposed**.

## 1. Verdict

The software foundation is broad and its tests pass, but **nothing physical beyond
static bench recording has been demonstrated**. No recording has been made while
the rig moved, no reconstruction has been run on our own imagery, no calibration
has been solved from real data, no RTK fix has been observed and no control points
have been surveyed. Thirteen issues are open: nine wait on measured evidence
rather than code, three need the survey wing's acquisition code, and one is the
tracker.

Three things matter most for planning:

1. **The only route to a georeferenced terrain product today is ground control
   points.** The RTK direct-georeferencing commands are not connected to the ROS
   recording stack (finding F1). Plan the first map around GCPs.
2. **The first end-to-end run is the largest unknown.** Recording, import, ODM and
   checkpoint reporting have each been tested in isolation, never chained on real
   moving imagery. Do that on the ground before any flight (Phase 3).
3. **Two platforms are now in scope**: the PX4 multirotor with the OAK-D, and an
   ArduPilot survey wing with a GigE camera. Only the first can record anything.
   The plan below takes the multirotor to a first map and keeps the wing separate.

## 2. How this review was done

| Check | Result |
|---|---|
| Full test suite, ROS Humble container on this workstation, PyCOLMAP 3.12.6 wheel | **492 passed**, none skipped, 78 s |
| Ruff (CI selection) and `bash -n` on `deploy/*.sh` | Pass |
| Offline CLI smoke: `simulate`, `validate`, `process --prepare-only` | Pass |
| Refusals: terrain without control, engine run on synthetic images, `sync` without telemetry | Refuse as documented, exit 2 |
| `survey-check` on the QGC fixture plan | Fails the plan with five specific errors, as intended |
| Code trace of recording to import to georeferencing to terrain product | Findings F1, F4, F8 |
| CI history, open issues, open and merged pull requests, branches | Section 5 |

Not done here: no hardware was attached, and no bench evidence exists on this
machine (`runs/` is ignored and lives on the Orin), so every measured figure below
is quoted from `docs/hardware-acceptance.md`, not re-verified. The repository image
and the processing environment could not be set up on this workstation as shipped
(finding F10), so the tests ran in a disposable `ros:humble` container with the
pinned Python packages, and **no ODM or OpenSfM engine run was made here**. Logs are
under ignored `runs/review-20261116/`.

## 3. What is established

| Area | Evidence | Limit |
|---|---|---|
| Software | 492 tests; CI green on `main` for Python 3.10/3.12, arm64 image, x86 OpenSfM and ODM on public sample photos | Fixtures and public photos only |
| Bench recording, default profile | 1200 s soak: valid, capture-ready, coverage-complete; RGB 2399 and mono 2400 frames; no image gaps | Static rig; stored October audit |
| OAK IMU | 199.85 Hz, 0.07 % nominal count deficit, 4 gaps, longest 15 ms | Hardware continuity unknown (#18) |
| PX4 link | TELEM2 at 921600 baud; 100 Hz raw IMU on request in a 60 s run | Timing gate, below |
| PX4 SIH | Calibration mission and Hold-then-Land abort pass in CI on PX4 v1.17.0 | Calibration mission only; no recorder in the loop |
| Survey planning | `survey-check`, plan binding to recordings, survey-leg extraction and import filter | Never used on a real QGC plan for a real site |

## 4. Findings

Ranked by how directly each blocks a first accurate map.

### Structural gaps that need code or a decision

**F1. RTK direct georeferencing cannot be reached from a ROS recording.**
`image-accuracy` needs the alignment written by `sync` and reads `telemetry.jsonl`
from a legacy SDK session. `sync` refuses imported bags
(`association.py`: "ROS image import has no qualified clock bridge"), and
`bag-import` writes no telemetry journal. `sync-bag` produces diagnostic poses from
`/mavros/local_position/pose` only and its outputs are explicitly not geolocation
inputs. The capture command in `docs/rtk-accuracy.md`
(`wr-map capture --config ...json --telemetry-config ...`) is rejected by the
current CLI. So `ntrip`, `image-accuracy`, `georeference` and the image half of
`map-accuracy` serve only `legacy-capture`, which the recording stack replaced.
Needed: a bag-native path from sealed GNSS, TIMESYNC and correction evidence to
per-image geolocation, with the same gates.

**F2. Corrections cannot reach the receiver yet.** The NTRIP bridge supports a
fixed single-base v2 mount only; it sends no rover GGA and has no VRS mode. The
fitted receiver is on DroneCAN, where PX4 forwards RTCM only with a persistent
parameter change. `/mavros/gpsstatus/gps1/rtk` has never published. No caster or
base is chosen.

**F3. The PX4 timing gate almost never qualifies.** In the 20-minute soak 26 of
11,999 TIMESYNC samples qualified (best streak 526 against 501 required), although
the residual p95 was 1.1 ms. The 20 fps candidate qualified none. With F1 fixed,
almost every image would still be unqualified. Either find the round-trip outliers
or decide whether isolated outliers may be rejected instead of resetting the streak.
That is an acceptance-rule change and needs the operator's approval.

**F4. No rolling-shutter handling.** The RGB sensor is rolling shutter and its
readout time is unmeasured (#14). `odm.py` passes no rolling-shutter options, and
`survey-check` bounds blur from exposure only. The global-shutter mono stream is
the available cross-check.

**F5. The 12 MP RGB intrinsics are extrapolated and then held fixed.** The factory
calibration is at 1920×1080; the 12 MP values are scaled from it. The terrain
recipe runs ODM with `--use-fixed-camera-params`, so any error in the scaling goes
straight into the product. Checkpoints will expose it; a target calibration (#14)
or a comparison run with refined intrinsics would resolve it.

**F6. Nothing produces GCP image observations.** `process --gcp` needs pixel
observations per image in the documented format. Tagging is left to an external
tool, and none is named or tested against `bag-import` image names.

**F7. Processing scale is untested.** `process-terrain.json` selects every 2 fps
frame, so a 20-minute flight is about 2400 lossless 12 MP images. This workstation
has 15 GiB of RAM and the pinned ODM image is CPU-only, so its GPU is unused. No
ODM run on our own imagery has been timed or sized. A 20-minute recording is also
about 99 GiB to move off the Orin.

**F8. The service path is unexercised and thinner than documented.**
`docs/hardware-acceptance.md` and `docs/hardware-commands.md` say
`deploy/record-session.sh` runs the hardware checks before recording. It does not;
it calls `record-rosbag.sh`, whose preflight covers storage, the mount and duplicate
owners. It also passes no `--survey-check`, `--announce` or `--px4-imu-rate`, so a
service-started flight recording is not bound to its plan and gives no operator
feedback. The unit has never been installed or stopped on the vehicle.

**F9. Platform scope.** #27, #28 and #30 describe a second airframe, autopilot and
camera. The repository has offline contract checks for it and no acquisition path.
Its issues should not gate the multirotor phases, and the multirotor's open
calibration issues do not transfer to it.

### Workstation and repository

**F10. Workstation setup needs two packages the guides omit.** On stock Ubuntu
24.04 with `docker.io`, `deploy/setup-postprocessing.sh` fails at `python3 -m venv`
(needs `python3.12-venv`), and `deploy/build-image.sh` fails because `docker buildx`
is absent. The README lists Docker and Python as the only prerequisites.

**F11. Bench evidence exists in one place.** Every audit the acceptance record
cites is under ignored `runs/` on the Orin. The documents record hashes, not
copies. Back the directory up before the next hardware session.

**F12. Review process is not enforced.** `main` has no branch protection, so the
required-check process in `AGENTS.md` is a convention. The `mavros-ros2` CI job
installs from live `packages.ros.org` and failed once on `main` with a DNS error
(run 38069275475); the rerun passed. The PX4 SIH workflow runs only on pull
requests touching its paths, so it has not run since 2026-10-10.

**F13. Stale material.** PR #25 is still open; its blockers B12 and B13 are fixed
and it predates the wing. The README status paragraph describes the superseded
100 Hz profile, the architecture diagram still shows COLMAP as the building engine,
and milestone M0 quotes 333 tests. Fifteen merged branches remain on the remote.

### Physical evidence never produced

Moving recording; reconstruction of own imagery; real IMU-to-IMU and camera-to-IMU
solves; measured lever arms and antenna ARP; RTK-fixed position; surveyed GCPs and
checkpoints; optical exposure timing and RGB readout; 20-minute soak of the 20 fps
candidate; airframe mount, vibration and power; QGC load of a generated plan.

## 5. Open issues

| Issue | Software state | Still needed | Phase |
|---|---|---|---|
| #7 PX4 100 Hz | Per-session request and restore, audited | Matched 60 s before/after windows with qualified timing | 1 |
| #13 20 fps profile | Candidate profile, resource logging | 20-minute soak, RGB rate within 1 %, thermal and storage figures | 1 |
| #18 IMU continuity | QoS and DDS fixes, deficit and gap diagnostics | Accepted 20-minute runs at both profiles | 1 |
| #8 Guided recording | 110 s mode, phase file, lifecycle tests | One real props-off session, sealed and audited | 2 |
| #9 IMU-to-IMU solver | Synthetic recovery and refusal tests | Independent segments agree on a real session | 2 |
| #10 Camera-to-IMU solver | Synthetic and degeneracy tests | Left/right agreement, mounting comparison | 2 |
| #12 Drift check | `validate --rig-calibration` | Real session passes against its own result | 2, then every recording |
| #16 Flight calibration mission | Generator; SIH mission and abort pass | QGC load/save, recorder and solvers in simulation, then flight | 4, 5 |
| #14 Optical timing and targets | Offline event audit, procedure | LED rig, target views, readout measurement | 6 |
| #15 Umbrella | Tracking only | Closes with the above | — |
| #27, #28, #30 Survey wing | Handoff, Plane plan and event checks offline | GigE recorder, raw GNSS, Plane SITL, V2 modes | 7 |

## 6. Phased testing plan

Each phase ends at a gate the operator reviews. Phases 0 and 4 need no vehicle
hardware. Only one process may own the OAK and the PX4 link at a time.

### Phase 0: workstation and repository (now, this machine)

1. Install `python3.12-venv` and `docker-buildx`; run `deploy/setup-postprocessing.sh`
   and both `wr-map doctor --mode process` backends.
2. Run `deploy/check-opensfm.py --execute`, then ODM on the public sample set as CI
   does. Record wall time, peak memory and disk use on this workstation.
3. Copy the Orin's `runs/` evidence here (F11). Re-run `wr-map validate` on
   `default-soak-003` and compare with the stored audit hash.
4. Import the static bench bag and run `process --prepare-only`; time the import and
   measure PNG size per frame.
5. Close PR #25 as superseded; correct the documents in F8, F10 and F13; decide on
   branch protection (F12).

**Gate:** workstation processes the sample to products; stored soak audit reproduces;
import throughput and disk per minute of recording are known.

### Phase 1: Orin static bench re-qualification

1. Rebuild the image from current `main`; run the full suite in it.
2. 20-minute soak at the default profile and at the 20 fps candidate, with resource
   logging (#13, #18).
3. Matched 60 s PX4 windows before and after the 100 Hz request (#7). Log every
   TIMESYNC sample that breaks a streak with CPU, USB and serial context (F3).
4. Install the systemd unit; start, stop with `systemctl stop`, and cut power once
   after seal. Confirm each session's state and audit (F8).
5. Fill storage to the 5 GiB guard and confirm an orderly stop.

**Gate:** both soaks pass audit with rates within 1 % and no interior gaps; the
service seals `complete`; the timing gate either qualifies most of a 20-minute
window or the operator has ruled on F3.

### Phase 2: bench motion and calibration, props off

1. `wr-map calibrate record`, 110 s, textured scene at least 2 m away (#8).
2. `calibrate solve` and `calibrate camera`; review segment and left/right
   agreement (#9, #10). Repeat the recording once and compare the two results.
3. Tape-measure body origin to left optical centre, RGB centre and antenna ARP with
   per-axis uncertainty; complete the rig file; `calibration-check --require-complete`.
4. `validate --rig-calibration` on both sessions (#12).

**Gate:** a reviewed rig file with sigmas; two sessions agree within their sigmas.

### Phase 3: ground-moving rehearsal, first end-to-end map

No flight risk. This is the first test of the whole chain and should find most
integration faults.

1. Survey 5–8 GCPs and 3–5 separate checkpoints on a textured site; record CRS,
   vertical datum and uncertainty. Choose and test the tagging tool (F6).
2. Carry or drive the rig at 1–3 m/s for 5–10 minutes, camera towards the ground,
   at the intended flight profile. Validate on the Orin before leaving.
3. Transfer, `bag-import`, then `process` with `--gcp` for RGB and for left mono
   separately (F4, F5). Record wall time and peak memory (F7).
4. `map-accuracy` on the withheld checkpoints. Run the drift check (#12).

**Gate:** a georeferenced orthomosaic and DSM with checkpoint RMSE reported for
both streams; no recorder loss; processing fits the workstation. **Proposed**
target for this rehearsal: horizontal RMSE within three GSD.

### Phase 4: planning and simulator rehearsal

1. Draw the Phase 5 site in QGC Survey with the custom camera from
   `configs/qgc-oak-rgb-12mp.json`; pass `survey-check`; reload the saved plan in QGC.
2. Fly that survey plan in PX4 SIH. The workflow currently flies only the
   calibration mission; extend it.
3. Load the generated calibration mission in QGC and save it back (#16).
4. Feed SIH mission progress through `survey-legs` and `bag-import --survey-legs`.

**Gate:** the exact plan to be flown passes the check and completes in SIH with an
abort; survey-leg windows are produced from simulated progress.

### Phase 5: airframe integration and first flight

1. Mount the rig; record mass, centre of gravity, cabling and isolation. Hover with
   recording running; check the PX4 vibration log and image sharpness.
2. Start and stop recording through the service with `--announce` and the bound
   plan (needs the F8 changes). Confirm the operator sees state in QGC.
3. Fly the Phase 4 plan at conservative height and speed over the Phase 3 site, with
   manual takeover ready and the operator's site approval.
4. Validate, transfer, process with GCPs, report checkpoints, run the drift check.

**Gate:** first flight map with stated checkpoint RMSE; recording valid and bound to
its plan; achieved overlap matches the plan.

### Phase 6: RTK direct georeferencing and optical timing

Software first: the bag-native geolocation path (F1), the correction route (F2) and
the timing-gate outcome (F3). Then:

1. RTK fixed on the ground with corrections forwarded and recorded.
2. Refly the Phase 5 site. Process without GCPs and judge against the same
   checkpoints.
3. Optical timing and readout measurement (#14) only if step 2 misses its target.

**Gate:** checkpoint RMSE without GCPs, compared with the Phase 5 result.

### Phase 7: survey wing, separate track

GigE acquisition adapter, raw GNSS and event ingestion, Plane SITL with the
recorder, and the V2 precision and coverage modes (#27, #28, #30). It needs its
own bench and calibration phases; none of Phases 1–6 qualifies it.

### Software work by phase

| Work | Finding | Needed before |
|---|---|---|
| Fix setup guides; correct stale documents; close PR #25 | F8, F10, F13 | Phase 0 gate |
| Service passes plan binding, announce and PX4 rate; documented preflight | F8 | Phase 5 |
| Name and test a GCP tagging tool against imported image names | F6 | Phase 3 |
| Terrain recipe: frame decimation guidance, rolling-shutter option, optional intrinsics refinement | F4, F5, F7 | Phase 3 |
| SIH job flies a survey plan with recorder input | #16 | Phase 4 |
| Bag-native GNSS geolocation; NTRIP VRS or own base; timing-gate decision | F1, F2, F3 | Phase 6 |

## 7. Decisions for the operator

1. **First-map platform.** Confirm the PX4 multirotor with the OAK-D carries the
   plan to a first map, with the wing as a later track.
2. **GCP-first.** Confirm the first accurate map uses GCPs and RTK follows.
3. **Accuracy target** for the first deliverable, horizontal and vertical.
4. **Timing gate** (F3): keep the 501-consecutive rule or reject isolated outliers.
5. **Correction source** (F2): network VRS, which needs bridge work, or an own base.
6. **Flight capture profile:** 12 MP lossless at 2 fps, or a smaller stream that
   matches the factory-calibrated resolution (F5, F7).
7. **Survey equipment** for GCPs and checkpoints, and the first test site.
8. **Branch protection** on `main` (F12).
