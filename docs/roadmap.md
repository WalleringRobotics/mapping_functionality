# Camera upgrades and delivery roadmap

## Acceptance milestones

Status as of 2026-10-04; evidence is in [hardware acceptance](hardware-acceptance.md).

| Milestone | Evidence required | Current state |
|---|---|---|
| M0: software foundation | IO fixtures, corruption/failure tests, command plans | Done; 333 tests passed in the repository image at the merged handoff; continuation acceptance checks below |
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
| [#7](https://github.com/WalleringRobotics/mapping_functionality/issues/7), PX4 100 Hz | Audited per-session requests/ACKs/default restoration; 60-second raw IMU at 100.004 Hz and pose at 30 Hz | Before/after timing and source-continuity nondegradation: prior run had zero qualified TIMESYNC samples in both windows, higher post-request RTT/residual percentiles and unresolved sequence reordering |
| [#8](https://github.com/WalleringRobotics/mapping_functionality/issues/8), guided recording | 110-second sealed calibration mode, 20 Hz mono profile, observed prompt timestamps and ROS lifecycle tests | Actual guided bench calibration session, sealed complete with phase file, valid audit and requested mono/OAK/PX4 rates; moving it also supplies #9/#10/#12 evidence |
| [#9](https://github.com/WalleringRobotics/mapping_functionality/issues/9), IMU solver | Synthetic offset/rotation/bias/rate-mismatch tests, frame and timestamp refusal, gap exclusion, residual and held-out gates | Real moving-rig segment repeatability; no absolute optical timing claim |
| [#10](https://github.com/WalleringRobotics/mapping_functionality/issues/10), camera solver | Supported command, synthetic scene/tracking tests, stereo consistency required, timing ambiguity refusal | Real left/right agreement and mounting check under representative motion |
| [#11](https://github.com/WalleringRobotics/mapping_functionality/issues/11), calibration application | **Closed after software acceptance:** identity, hand-computed offset/rotation/lever, explicit missing-link reasons and simulated camera-centre shift; hash/source/sigma propagation checked | Physical bridge, optical latency, receiver reference and map accuracy remain separate qualification work; diagnostic outputs retain `survey_ready=false` |
| [#12](https://github.com/WalleringRobotics/mapping_functionality/issues/12), drift | Read-only validate checks with configurable sigma thresholds and explicit insufficient-motion result | A real motion recording compared with its independently reviewed calibration |
| [#13](https://github.com/WalleringRobotics/mapping_functionality/issues/13), 20 fps | Lossless 1080p RGB/800p mono candidate, resource logging and window rate/gap audit | Passing 20-minute soak before changing the default; thermal and storage evidence |
| [#14](https://github.com/WalleringRobotics/mapping_functionality/issues/14), optical timing/targets | Repeatable instrumented procedure and evidence template | LED/trigger/target equipment, independent measurements and held-out validation |
| [#16](https://github.com/WalleringRobotics/mapping_functionality/issues/16), flight trajectory | Bounded offline QGC Plan generator, hashed item map and sealed mission endpoint extraction | QGC loading, PX4 SITL plus synthetic recorder/solver pipeline, then operator-approved flight |
| [#18](https://github.com/WalleringRobotics/mapping_functionality/issues/18), IMU drops | Deep telemetry QoS, larger DDS shared memory, per-window count deficit and gap diagnostics | Loss-free 20-minute evidence at both default and candidate profiles |

[#15](https://github.com/WalleringRobotics/mapping_functionality/issues/15) remains the
umbrella tracker. The issue-by-issue acceptance review distinguishes #11's explicit
software-only criteria from the physical and simulator acceptance of the other ten
issues. Do not close measured-acceptance issues on synthetic tests alone.
Nominal rate deficits, inferred timestamp gaps and hardware sequence loss are
different quantities; standard ROS headers still cannot prove hardware continuity.
QGroundControl and PX4 SITL were not installed in the inspected host environment.

The continuation reran 90 association, ROS-bag association, GNSS accuracy, rig,
georeference, mission and flight-phase tests in the pinned ROS/PyCOLMAP image:
all passed. This covers #11's identity and hand-computed fixture criteria, including
the combined +30 ms bridge shift and `[8.2, 2, 2.9]` m camera centre, an ECEF lever
shift and georeference hash propagation. The portable environment lacks PyCOLMAP;
its collection failure is not counted as validation. Issue #11's original scope
also proposed using gyro-offset sigma as camera latency and mounting sigma as
vehicle attitude uncertainty; these are different quantities, so the implemented
consumers preserve independent profile bounds and block unsupported clock paths.

### Physical evidence to supply

The operator will capture moving survey imagery and GCP/checkpoint observations
later. Their absence remains explicit; current static bench data can exercise
import/export and failure diagnostics but cannot qualify a survey reconstruction.

1. **Rigid-rig motion, #8–#10/#12:** remove props and rotate the final OAK/PX4
   assembly through the guided 110-second sequence in front of sharp, well-lit
   texture at least 2 m away. Record its actual device identity and manual body-to-
   left-optical-centre position/orientation with per-axis uncertainties. Preserve
   both cameras, factory calibration, both IMUs, phase timestamps and seal. Review
   independent-segment timing/rotation agreement, left/right camera agreement and
   mounting agreement; validate the real session against its own accepted result.
2. **Loaded recording, #7/#13/#18:** retain before/after 60-second PX4 timing, pose
   and MAVLink source windows, then 20-minute captures at both the default and the
   recommended candidate profile. Record runtime USB speed, storage write rate,
   free space, CPU/RAM, temperature and camera/IMU rates/gaps. Resolve source-count
   uncertainty rather than treating the nominal count deficit as proven loss.
3. **Optical timing, #14:** use an independently timestamped LED plus measured
   optical edge and clock bridge, with at least 100 identified usable events per
   camera and a separate held-out sequence. Measure frame timestamp meaning,
   latency/jitter and RGB row readout under final exposure/load settings; see the
   [instrumented procedure](optical-target-acceptance.md).
4. **Target and metric geometry, #14/map acceptance:** use a measured rigid flat
   checkerboard/AprilGrid, at least 30 varied sharp views per camera and independent
   held-out views; preserve target dimensions, corner observations and uncertainty.
   Supply independently surveyed GCPs and separate checkpoints, CRS/vertical datum
   and uncertainty for the building/terrain dataset. Static bench images do not
   supply baseline, survey coverage or checkpoint truth.
5. **GNSS reference:** measure body-to-antenna ARP in FLU with covariance, verify
   PX4 FRD lever parameters and the receiver's output reference; retain correction
   age/status, base coordinates/datum and independent RTK checks. Relative IMU
   timing does not qualify exposure geolocation.
6. **Flight, #16:** first preserve QGC load/save/reload evidence and real PX4 SITL
   mission/abort results with the exact plan hash. Only an operator-approved site,
   reviewed plan and manual takeover can supply the later real-flight comparison.
   Mission endpoint labels alone do not qualify solver motion windows or lever-arm
   observability.

### Reuse of reconstruction engines

Use OpenSfM and OpenDroneMap for feature matching, reconstruction and terrain
products. Repository work should concentrate on the recording and export contracts
those engines need: sharp overlapping images with translation/parallax, calibrated
camera models, explicit image names and masks, defensible camera positions or GCPs,
CRS and separate checkpoints, input hashes and honest failure/accuracy reports.
Keep timing, measured rig transforms and immutable raw navigation evidence beside
the images; neither a completed engine run nor a software fixture qualifies these
physical inputs.

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
