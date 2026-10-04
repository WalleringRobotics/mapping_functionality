# Status report and plan: first setup, flight and processing

Status as of 2026-10-04, taken from `main` at `51a3d32`, draft PR #24 and the
bench evidence under ignored `runs/`. This document is a plan for review. It
does not change any acceptance gate, and it does not qualify any result.

## 1. Where the project stands

### What works

| Area | State | Evidence |
|---|---|---|
| Runtime | Reproducible Docker image (ROS Humble, OAK driver 2.12.2, MAVROS 2.14.0, rosbag2/MCAP, PyCOLMAP 3.12.6) | 333 tests passed in the image; CI green on `main` |
| Bench recording | ROS launch owns OAK, MAVROS and rosbag2; sealed sessions with SHA256 seal, audit and acquisition window | 60 s full-stack runs are `valid`, `capture_ready` and `coverage_complete` |
| 20-minute soak, default profile | `default-soak-003` recorded 1200 s, sealed and passed the audit | Soak audit below |
| 20 fps candidate | 60 s run: stereo exactly 20 Hz with no gaps, OAK IMU 199.9 Hz, PX4 IMU 100 Hz | `candidate-short-003`; not qualified, see blockers |
| OAK IMU | Native 200 Hz gyro on true timestamps (PR #22); BNO086 firmware 3.9.9 | `docs/hardware-acceptance.md` |
| PX4 link | TELEM2 at 921600 baud, MAVROS TIMESYNC, 100 Hz IMU by per-session request with restore | Issue #7 |
| Calibration software | Rig file (#6), IMU-to-IMU solver (#9), camera-to-IMU solver (#10), application in association and accuracy (#11, closed), drift checks (#12), guided recording (#8), mission draft generator (#16) | Merged PRs #17, #19 to #21, #23; synthetic tests only |
| RTK software | NTRIP v2 bridge into MAVROS `gps_rtk`, RTK-fixed image gate, per-image budgets, checkpoint map report | PR #3; never run against a real caster or RTK receiver |
| Offline processing | One pinned ODM 3.6.2 image for OpenSfM and terrain on x86-64; GDAL/PDAL product checks | Draft PR #24; CI passes on upstream sample photos |
| PX4 SITL | Upstream PX4 SIH builds in CI and runs the generated mission | PR #24 `upstream-px4` job. **It has failed on all six pushes**: the mission and abort completion checks are not met |

### What has never happened

- No recording has been made while the rig moved, on foot or in the air.
- No reconstruction has been run on our own imagery. The static bench bag was only imported and prepared.
- No calibration has been solved from real data. Every rig transform, lever arm and time offset is still unmeasured.
- No RTK-fixed position has been observed. The fitted receiver is a Holybro H-RTK F9P Helical (u-blox ZED-F9P, dual-band, DroneCAN). Without corrections it reported a 3D fix with about 3.5 m horizontal and 5.1 m vertical uncertainty on the bench.
- No GCPs or checkpoints have been surveyed.

### Soak audited in this review

`default-soak-003` (2 fps 12 MP RGB, 2 fps 800p stereo, OAK IMU 200 Hz, PX4 IMU
50 Hz) was recorded and sealed by Codex but not audited. This review ran
`wr-map validate` on it. The report is kept in ignored
`runs/hardware-continuation-20261004/default-soak-003-audit-claude.json`.

| Check | Result |
|---|---|
| Verdict | `valid`, `capture_ready` and `coverage_complete` are true; `survey_ready` is false |
| Acquisition window | 1200.0 s, PX4 connected throughout |
| Cameras | RGB 2399 of 2400 frames, left and right 2400 of 2400, no gaps |
| OAK IMU | 199.85 Hz; 177 of 240,002 nominal samples missing (0.07 %); 4 gaps, longest 15 ms; 218 repeated samples |
| PX4 IMU | 50.0 Hz raw and filtered, no gaps; pose 30 Hz |
| MAVLink link | 24.0 kB/s received; 18 inferred sequence gaps and 18 reorders in 488,230 messages |
| TIMESYNC | **26 of 11,999 samples qualified.** Longest good streak 526 against 501 required. RTT median 3.1 ms, p95 5.6 ms, max 21.8 ms. Offset residual median 0.23 ms, p95 1.1 ms |
| GNSS | Fused and raw fixes present and valid; RTK status topic absent |
| Storage | 74.6 MiB/s mean write rate, 81.3 MiB/s peak; about 99 GiB in total |
| Thermal and load | Peak 56.7 °C against a 70 °C lowest passive trip; CPU busy 37–70 %; at least 2.3 GiB RAM available |

This soak is strong evidence for M1 at the default profile. Two caveats remain.
The 0.07 % IMU deficit is a nominal-count estimate, not proven hardware loss (#18).
The timing gate barely qualified once in 20 minutes, which is blocker B6.

### Open housekeeping

- **Draft PR #24** holds the x86 ODM/OpenSfM workflow and PX4 fixes. All CI jobs pass except `upstream-px4`. That job failed on every push from `0f54d11` to `b8e48cc` with `SITL mission/abort did not satisfy completion checks`.
- **A MAVROS container** named `wr-hardware-mavros-20261004` was still running at review time. Codex probably started it. Stop it before the next hardware session so one owner holds the link.
- **Stale worktrees** remain under `/tmp/mapping-*` and `.claude/worktrees/`. The `fix/imu-sample-loss` worktree has uncommitted changes that look superseded by the QoS and DDS work merged in #23. Confirm, then remove.

## 2. Blockers to a first flight and first map

Ranked by how directly each one stops the first useful map. "Agent" means software
work an agent can do unattended. "Operator" means it needs hands on hardware or a
decision.

| # | Blocker | Blocks | Owner | Notes |
|---|---|---|---|---|
| B1 | **Airframe integration is undocumented.** Mount, power, cabling, vibration isolation and centre of gravity of the OAK and Orin on the vehicle are not recorded in this repository. | Flight | Operator | Needs a photo, mass, a hover test and a vibration log before any mapping flight |
| B2 | **No ground-control points or checkpoints.** Without them no absolute accuracy can be claimed. | Accuracy | Operator | Needs targets and a survey-grade GNSS rover or total station |
| B3 | **RTCM corrections cannot reach the receiver yet.** The receiver is RTK-capable, but it is on DroneCAN. PX4 forwards MAVLink RTCM to a DroneCAN receiver only when `UAVCAN_PUB_RTCM=1`, which needs a reboot. The MAVROS `gps_rtk` status topic has never published. | RTK | Operator approval, then agent | Writing a persistent PX4 parameter needs the operator's approval. Verified in PX4 v1.17 source: `src/drivers/uavcan/sensors/gnss.cpp` maps DroneCAN RTK fixed to fix type 6 |
| B4 | **No correction source is chosen.** No NTRIP caster, mount point or surveyed base exists in configuration. | RTK | Operator decision | The repository's NTRIP bridge supports a **single-base mount only**. VRS or network mounts that need the rover's GGA position are not supported. A national CORS single-station mount gives datum-tied corrections; a self-surveyed base does not |
| B5 | **Rig calibration is unmeasured.** No guided session (#8), no real solves (#9, #10), no tape-measured lever arms. | Direct georeferencing; survey gate | Operator, then agent | About one bench hour plus solver review |
| B6 | **PX4 timing almost never meets the qualification gate.** The gate needs 501 consecutive TIMESYNC samples, about 50 s, each with RTT under 10 ms and residual at most 2 ms. The 20-minute soak qualified 26 of 11,999 samples. The 20 fps candidate qualified none. | Every capture's timing qualification, so every image stays unqualified for geolocation | Agent, then operator decision | The offset itself looks good: residual p95 is 1.1 ms. Rare RTT outliers reset the streak. Options are to find the outlier source or to change the gate so isolated outliers are rejected rather than resetting it |
| B7 | **RGB is rolling shutter and its readout time is unmeasured** (#14). Motion skews the image during readout. | Map quality when moving | Agent and operator | Fly slowly at first. Process the global-shutter left mono stream as a comparison. Use ODM rolling-shutter correction with a stated readout time |
| B8 | **Data volume.** The default profile writes 74.6 MiB/s on average, about 4.4 GiB per minute, because it stores lossless 12 MP frames. | Endurance; transfer to workstation | Agent | The 481 GiB free holds about 110 minutes. Moving 99 GiB per 20 minutes to the workstation needs a plan. A 4K ISP-scaled candidate exists in `runs/` but is not in `configs/` |
| B9 | **IMU continuity is not proven** (#18). Loss at 200 Hz fell from 2.6 % to a 0.07 % nominal deficit with 4 short gaps in the 20-minute soak. | Calibration solvers, mildly | Agent | Probably good enough for the solvers, which exclude gaps. ROS headers cannot prove hardware continuity |
| B10 | **No operator status feedback in flight.** Only the CLI reports status. Recording start and stop on the vehicle are not exercised through the systemd service. | Field operation | Agent | A start-on-arm or RC-switch hook and an LED are enough for a first flight |
| B11 | **x86 workstation not provisioned** for real data. PR #24 is proven in CI only. | Processing | Operator | Docker plus the pinned ODM image; about 300 GiB per flight of scratch space |
| B12 | **The PX4 SIH mission check fails in CI** on every push of PR #24. No simulated mission has completed end to end. | Simulator acceptance of any mission (#16); confidence before flight | Agent | Debug from the uploaded `px4-sih-evidence` artifact |

Not blockers for a first flight: absolute optical timing (#14), the in-flight
calibration mission (#16) and the 20 fps default (#13). They matter for direct
georeferencing at centimetre level, which comes later.

## 3. Achievable accuracy

These are **engineering estimates**, not measured results. Assumptions: nadir 12 MP
RGB, 30 m above ground (about 1.0 cm GSD), 3 m/s, 1 ms exposure, 80/70 % overlap,
ODM processing, accuracy stated as checkpoint RMSE. Values scale roughly with
height above ground.

| Configuration | Horizontal | Vertical | What it needs |
|---|---|---|---|
| Today: standalone GNSS, no GCPs | 1.5–5 m absolute | 3–8 m absolute | Nothing new. Relative accuracy inside the model is about 2–5 cm, but scale can be off by 1–2 % |
| SBAS or code DGPS, no GCPs | 0.5–1.5 m | 1–3 m | DGPS corrections to the existing receiver. Little benefit for mapping; the repository's image gate also rejects DGPS (fix type 4) |
| GCPs plus checkpoints, any GNSS | 2–4 cm | 3–6 cm | 5–8 surveyed targets per site. Independent of timing calibration. **Lowest-risk route to an accurate first map** |
| RTK fixed via NTRIP, uncalibrated timing and lever arms | 5–15 cm | 8–20 cm | The fitted F9P, a PX4 parameter change and a caster. The timing uncertainty of 10–30 ms at 3 m/s dominates |
| RTK fixed via NTRIP, calibrated timing (≤ 2 ms) and lever arms (±1 cm), no GCPs | 2–4 cm | 4–8 cm | B3 to B6 and #14 closed. Keep at least one checkpoint to detect datum and vertical bias |
| RTK plus GCPs | 1.5–3 cm | 2–4 cm | Everything above. Best achievable with this camera; limited by 1 cm GSD and rolling shutter |

Budget behind the calibrated RTK row, per camera centre (1σ): receiver 1–2 cm
horizontal and 2–3 cm vertical, plus 1 ppm of base distance; timing 2 ms × 3 m/s
= 0.6 cm; lever arm 1 cm; 0.5° attitude over a 0.3 m lever = 0.3 cm. Base
coordinates matter most of all. A base that was only surveyed in by averaging can be
1–2 m off in absolute terms while every result looks consistent. A network CORS
mount, or a base tied to one, avoids this.

**PPK** would give the same accuracy as RTK without a live correction link, but
it needs the rover's raw observations. PX4 logs those only from a serial u-blox
receiver (`GPS_DUMP_COMM` in the serial GPS driver). The fitted receiver is on
DroneCAN, so PPK would need a separate raw-data connection to the F9P. Plan on
live RTK unless the site has no reliable network.

**Recommendation.** Make the first map accurate with GCPs, not with RTK. That
removes timing and lever-arm calibration from the critical path. Because the RTK
receiver is already fitted, bring RTK up in parallel and record it on every
recording from Phase 2 onward. Judge direct georeferencing against the same
checkpoints.

## 4. Plan

Each phase ends at a gate the operator reviews before the next starts. Phases 1A
and 1B can run in parallel. Only one agent or session may own the OAK at a time.

### Phase 0: consolidate (agent, about half a day)

1. Finish PR #24: fix the failing PX4 SIH mission check, or split the SIH job into its own PR so the processing workflow can merge. Address review, then merge.
2. Record the `default-soak-003` audit result in `docs/hardware-acceptance.md` and update #13 and #18.
3. Stop the stray MAVROS container. Remove stale `/tmp` and `.claude` worktrees after confirming nothing unique is in them.
4. Prepare the flight capture profile choice: 12 MP lossless at 2 fps against the 4K scaled candidate, using measured data rate and RGB rate accuracy.
5. Propose a revised TIMESYNC gate with evidence from the soak (B6). It changes an acceptance rule, so it needs the operator's approval.

**Gate:** `main` contains the processing workflow; the soak result is documented.

### Phase 1A: bench calibration (operator with agent, about one day)

1. Run the guided 110-second calibration recording on the rigid rig, props off (#8).
2. Solve IMU-to-IMU and camera-to-IMU from that session (#9, #10). Review segment agreement.
3. Tape-measure lever arms from body origin to the left optical centre, the RGB centre and the GNSS antenna ARP, with per-axis uncertainty. Enter them in the rig file.
4. Re-run the PX4 100 Hz before/after windows at matched duration (#7). Log every TIMESYNC sample that breaks a streak and find its cause, such as CPU load, USB or serial interrupts, or MAVROS scheduling (B6).

**Gate:** a reviewed rig calibration file with sigmas; TIMESYNC qualifies for most of a 20-minute window under the chosen flight profile, or the operator has approved a revised gate.

### Phase 1B: workstation and site (operator, about one day)

1. Provision the x86-64 workstation with Docker and the pinned ODM image (`deploy/setup-postprocessing.sh`). Run `wr-map doctor`.
2. Choose a test site with texture and open sky. Lay out 5–8 GCPs and 3–5 separate checkpoints and survey them, recording CRS, vertical datum and uncertainty.
3. Choose the correction source (decision 2 in section 5). After approval, set `UAVCAN_PUB_RTCM=1`, reboot, run `wr-map ntrip` and confirm RTK fixed on the ground with the fitted F9P (B3, B4).
4. Record the receiver's uncertainty convention and timestamp mode in the private GNSS profile, as `docs/rtk-accuracy.md` requires.

**Gate:** workstation processes the CI sample; surveyed points exist for the site; RTK fixed is observed with corrections forwarded through MAVROS.

### Phase 2: ground-moving rehearsal (operator with agent, half a day)

Carry or drive the rig over the surveyed site at 1–3 m/s, at a few metres height,
for 5–10 minutes. This is the first moving recording, and it carries no flight risk.

1. Record with the flight profile; validate and import.
2. Process RGB and left mono separately through ODM with GCPs. Report checkpoint residuals.
3. Run drift validation (#12) against the Phase 1A calibration.

**Gate:** a georeferenced product with checkpoint RMSE reported; no recorder loss.

### Phase 3: integration and first flight (operator; agent for software)

1. Mount on the airframe (B1). Hover test, props on, recording running: check vibration in the PX4 log and image sharpness.
2. Exercise recording start and stop through the service with an operator signal (B10).
3. Verify the survey mission in QGC and PX4 SIH. Fly it at 30 m, 3 m/s, with manual takeover ready and the operator's site approval.
4. Validate, transfer, process with GCPs, report checkpoints.

**Gate:** first flight map with stated checkpoint accuracy, expected 2–4 cm horizontally.

### Phase 4: RTK direct georeferencing (later)

1. Fly the same site with RTK fixed throughout. Process once without GCPs, then compare with the checkpoints.
2. Continue with optical timing (#14) and the in-flight calibration mission (#16) only if direct georeferencing misses its target.

## 5. Decisions needed from the operator

1. **RTK parameter.** The receiver is answered: an H-RTK F9P Helical is fitted. May an agent set `UAVCAN_PUB_RTCM=1` on the Pixhawk 6X? It is persistent and needs a reboot.
2. **Corrections.** Which NTRIP caster and single-base mount point, or a surveyed base of our own?
3. **Accuracy target.** What horizontal and vertical accuracy does the first deliverable need? That decides whether GCPs alone are enough.
4. **Airframe.** Which vehicle carries the rig, and what is its payload margin?
5. **Site.** Where is the first test site, and what flight approvals apply?
6. **Flight profile.** Accept 12 MP lossless at 2 fps (about 4.4 GiB/min) for the first flight, or prefer the smaller 4K scaled stream?
7. **Timing gate.** Should the 501-consecutive-sample gate stay as it is, or may it reject isolated outliers? The soak shows the gate, not the offset quality, is what fails.
