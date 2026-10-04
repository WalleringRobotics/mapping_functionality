# Camera upgrades and delivery roadmap

## Acceptance milestones

Status as of 2026-10-04; evidence is in [hardware acceptance](hardware-acceptance.md).

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

1. **Timing and camera-to-body calibration.** Qualify OAK exposure-to-PX4 time
   offset and the camera-IMU/camera-body rotation and lever arm, with uncertainty.
   Provide a repeatable calibration mode that records a dedicated session and
   produces a versioned calibration result the processing chain consumes. Recordings
   stay `survey_ready=false` until this exists.
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
