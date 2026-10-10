# Camera upgrades and delivery roadmap

## Acceptance milestones

Status as of 2026-10-04; evidence is in [hardware acceptance](hardware-acceptance.md).
The [first flight plan](first-flight-plan.md) lists the blockers, accuracy estimates
and phased steps toward the first flight and map.

| Milestone | Evidence required | Current state |
|---|---|---|
| M0: software foundation | IO fixtures, corruption/failure tests, command plans | Done; 168 tests run in the repository image (CI and Orin) |
| M1: Orin/OAK bench capture | Inventory; reproducible runtime; 20-minute capture; stop/fault checks | ROS/MCAP stack and Docker image verified on 60 s runs with PX4, Ctrl+C and `docker stop`; 20-minute soak, service path and fresh-Orin preparation pending |
| M2: static building reconstruction | Connected sparse model, dense result, scale/control and held-out checks | Bench bag imports and prepares (`--prepare-only`); no reconstruction executed |
| M3: terrain product | GCP/CRS-verified ODM product and independent accuracy report | Calibrated runner implemented; real survey acceptance pending |
| M4: moving platform capture | Blur, vibration, exposure timing, power and throughput acceptance | Not demonstrated; blocked on timing and camera-to-body calibration |
| M5: upgraded synchronized rig | Trigger/PTP/GNSS event evidence, calibration and metric rig solve | Planned |

## Upgrade decision matrix

| Candidate | Benefit | Integration requirements / limitation |
|---|---|---|
| Current OAK RGB | Colour/detail, simplest start | Rolling shutter on common SKU; daylight/slow motion calibration trial |
| Current OAK mono | Global shutter on standard OAK-D | Lower resolution; useful independent geometry comparison |
| Global-shutter RGB OAK variant | Existing DepthAI ecosystem | Verify exact sensor/SKU, exposure synchronization and lens calibration |
| GigE Vision global-shutter camera | Industrial lenses, trigger/PTP options | New SDK/GenICam adapter, packet/chunk metadata, network bandwidth, power |
| USB3 Vision global-shutter camera | Direct host link and industrial triggering | Vendor SDK or GenICam adapter, trigger wiring, per-image calibration |
| CSI/GMSL synchronized cameras | Compact integrated multicamera rig | Carrier, drivers, trigger distribution, ISP and timestamp-domain validation |

No product purchase is prescribed until required range/GSD, lens, motion, FPS, SWaP
and environmental protection are known. The camera interface should deliver an image,
native sequence, exposure start/mid/end evidence, sensor clock/domain, settings,
per-image pixel calibration and model/serial identity into the same session writer.

PTP aligns clocks; it does not inherently trigger all shutters. Hardware triggering
aligns exposure events; it does not automatically establish absolute UTC or calibrate
rolling-row timing. Require both a characterized exposure event and a traceable clock
mapping when integrating multiple cameras and navigation sensors.

## Prioritized engineering backlog

### Repository review and issue coverage, 2026-10-04

The calibration PRs add useful software but do not establish a survey-qualified rig.
Review found and corrected unsupported ROS-to-SDK clock conversion, invented zero
factory covariance, acceptance of inconsistent stereo estimates, incomplete frame
and timestamp checks, and a motion pattern that confused camera clock offset with
mounting rotation. Rig completeness now refuses contradictory measured transform
paths. Original recordings and the pre-existing `.claude/` worktrees are preserved.

| Issue | Implemented or reviewed | Acceptance still needed |
|---|---|---|
| [#7](https://github.com/WalleringRobotics/mapping_functionality/issues/7), PX4 100 Hz | Audited per-session requests for HIGHRES_IMU and ATTITUDE_QUATERNION, ACK logging and default-interval restoration | Measured sustained rates, link continuity, pose rate and TIMESYNC comparison |
| [#8](https://github.com/WalleringRobotics/mapping_functionality/issues/8), guided recording | 110-second sealed calibration mode, 20 Hz mono profile, observed prompt timestamps and ROS lifecycle tests | Operator-performed motion session with accepted rates and successful calibration |
| [#9](https://github.com/WalleringRobotics/mapping_functionality/issues/9), IMU solver | Synthetic offset/rotation/bias/rate-mismatch tests, frame and timestamp refusal, gap exclusion, residual and held-out gates | Real moving-rig segment repeatability; no absolute optical timing claim |
| [#10](https://github.com/WalleringRobotics/mapping_functionality/issues/10), camera solver | Supported command, synthetic scene/tracking tests, stereo consistency required, timing ambiguity refusal | Real left/right agreement and mounting check under representative motion |
| [#11](https://github.com/WalleringRobotics/mapping_functionality/issues/11), calibration application | Legacy and sealed ROS diagnostic camera poses, explicit evidenced ROS timestamp bridges, calibrated antenna lever, hash propagation and unknown-uncertainty preservation | Physical clock-bridge evidence, optical latency, receiver reference and full geolocation acceptance |
| [#12](https://github.com/WalleringRobotics/mapping_functionality/issues/12), drift | Read-only validate checks with configurable sigma thresholds and explicit insufficient-motion result | A real motion recording compared with its independently reviewed calibration |
| [#13](https://github.com/WalleringRobotics/mapping_functionality/issues/13), 20 fps | Lossless 1080p RGB/800p mono candidate, resource logging and window rate/gap audit | Passing 20-minute soak before changing the default; thermal and storage evidence |
| [#14](https://github.com/WalleringRobotics/mapping_functionality/issues/14), optical timing/targets | Repeatable instrumented procedure and evidence template | LED/trigger/target equipment, independent measurements and held-out validation |
| [#16](https://github.com/WalleringRobotics/mapping_functionality/issues/16), flight trajectory | Bounded offline QGC Plan generator, hashed item map and sealed mission endpoint extraction | QGC loading, PX4 SITL plus synthetic recorder/solver pipeline, then operator-approved flight |
| [#18](https://github.com/WalleringRobotics/mapping_functionality/issues/18), IMU drops | Deep telemetry QoS, larger DDS shared memory, per-window count deficit and gap diagnostics | Loss-free 20-minute evidence at both default and candidate profiles |

[#15](https://github.com/WalleringRobotics/mapping_functionality/issues/15) remains the
umbrella tracker. Do not close measured-acceptance issues on synthetic tests alone.
Nominal rate deficits, inferred timestamp gaps and hardware sequence loss are
different quantities; standard ROS headers still cannot prove hardware continuity.
QGroundControl and PX4 SITL were not installed in the inspected host environment.

### Remaining qualification priorities

1. **Timing and camera-to-body calibration.** Qualify OAK exposure-to-PX4 time
   offset and the camera-IMU/camera-body rotation and lever arm, with uncertainty.
   Use the calibration recording and solver commands to produce reviewed results,
   then verify the optical timing and processing clock bridge. Recordings stay
   `survey_ready=false` until the required physical evidence is supplied.
2. Field-duration qualification: a 20-minute soak through the image with intended
   workloads and thermal state; exercise the systemd service path and storage mount.
3. RTK: qualify corrected rover geolocation and NTRIP forwarding against the actual
   receiver/base; `/mavros/gpsstatus/gps1/rtk` is requested but not yet published.
   Native receiver correction-age evidence and survey event marks need integration.
4. First end-to-end map: take a recording through `bag-import`, building/terrain
   processing and independent checkpoints on a workstation with CUDA COLMAP/ODM.
5. Operator feedback: a physical status display/LED and controlled start/stop
   input, plus field QA thumbnails; CLI status already exists.
6. Controlled recovery for interrupted sessions; never mutate originals.
7. Calibrated stereo/rig reconstruction with validated exposure pairing. Do not
   assume nearest timestamps alone are sufficient.
8. Qualify COLMAP metric alignment, ODM GCP/geo ingestion and checkpoint breakdowns
   on a real survey; extend raster/CRS checks and geodetic QA.
9. Large-dataset matching, scene masks and repeat-survey comparisons after small
   surveys consistently meet their defined accuracy objectives.

## Configuration changes needing requalification

Camera/lens/focus, mounting stiffness, shutter mode/resolution, FPS, compression,
USB cable/hub, storage device/filesystem, power supply, thermal setup, DepthAI, JetPack,
and concurrent workloads can affect data quality or continuity. Retain versioned
profiles and calibration identities; do not infer equivalence from a successful launch.
