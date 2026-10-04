# Surveyed base, MAVROS rover and accuracy reporting

The corrected rover position is used in both reconstruction paths: camera geolocation
for terrain, and a metric similarity alignment of the building model and its PLY
products. Every selected photograph retains its own navigation uncertainty budget.
The final map report uses withheld surveyed checkpoints; camera-position uncertainty
alone cannot establish the accuracy of a reconstructed surface.

## Correction and observation path

Use the existing companion connection from
[drone_autonomy_platform](https://github.com/Darainer/drone_autonomy_platform), with
the [MAVROS timing setup](mavlink-integration.md). MAVROS remains the only owner of
the PX4 serial connection and TIMESYNC exchange.

| Step | Component | Evidence retained |
|---|---|---|
| Base | Your surveyed, fixed antenna reference point (ARP), base receiver and caster mount | Survey coordinates, covariance, datum/epoch and station ID |
| Correction transport | `wr-map ntrip` → existing MAVROS `gps_rtk` input → PX4 → rover receiver | CRC-checked RTCM, receipt times, station/ARP verification and forwarding decisions |
| Rover observations | MAVROS `gps_status` raw GNSS topic | Fix type, receiver uncertainty, satellite count, original source header and CDR |
| Camera association | OAK exposure clock + qualified PX4/MAVROS timing and orientation | Exposure-to-navigation interpolation and estimated timing budget |
| Reconstruction | ODM geolocation or COLMAP metric alignment | Sealed inputs, products and navigation-control residuals |
| Acceptance | Independent withheld checkpoints | Bias, scatter, reference uncertainty, shared-base term and horizontal/vertical error |

NTRIP transports corrections; it does not establish an accuracy class. The image
gate requires **RTK fixed (`fix_type=6`) at both interpolation endpoints**. DGPS
(`4`), RTK float (`5`), a nearby base, or a small HDOP alone does not establish
centimetre accuracy. Wrong integer ambiguities and multipath can survive a fixed
flag and small receiver-reported uncertainty; field checks remain necessary.

## Configure the real installation

Install the processing `terrain` extra on the reporting workstation for geodesy.
On the Orin, use the existing ROS 2 Humble environment with `rclpy`, `mavros_msgs`,
`geometry_msgs`, `sensor_msgs` and `rosidl_runtime_py` importable in the recorder's
Python environment. Source ROS before starting the commands. A normal venv may
need `--system-site-packages` to see the installed ROS modules.

Enable the deployed MAVROS `gps_status` and `gps_rtk` plugins and verify MAVLink 2
and the required raw GNSS messages on PX4. This package does not change PX4
parameters, start a second MAVLink connection or issue flight commands. Inspect
the running graph; the supplied topic names are examples, not discovery results:

```bash
source /opt/ros/humble/setup.bash
ros2 node list
ros2 topic list -t
ros2 topic info /mavros/gpsstatus/gps1/raw -v
ros2 topic echo /mavros/gpsstatus/gps1/raw --once
ros2 topic info /mavros/gps_rtk/send_rtcm -v
```

Copy `configs/mavros-rtk-survey.json` into a private deployment directory and set
the actual topics and time-plugin node. `gps_raw` and `rtcm` are required by this
profile. `gps_rtk` status is recorded when present but is optional: PX4/receiver
combinations do not all emit it. Avoid two correction publishers on the same input.
Stop the platform's DepthAI ROS camera driver before the direct OAK recorder
claims USB. Keep the existing MAVROS connection running.

Copy these templates into that same private directory:

| Template | Required installation evidence |
|---|---|
| `ntrip-base.template.json` | Exact fixed mount URL, station ID, surveyed ARP ECEF metres and coordinate tolerance |
| `gnss-accuracy.template.json` | Base covariance/datum, receiver/firmware semantics, verified timestamp convention, rig calibration and motion/latency bounds |
| `map-accuracy.template.json` | Product CRS/height datum, withheld-point reference sigmas, independence, control IDs and shared-base treatment |

Nulls and `unknown` values are intentional. No surveyed coordinates, receiver
confidence model, antenna convention or centimetre uncertainty is fabricated.
Evidence strings should identify saved survey/calibration/bench records and their
version or checksum. Keep credentials and real locations outside this repository.

### Surveyed base and NTRIP

Configure the base receiver with your surveyed **ARP** coordinates, rather than
treating a survey-in average as independent survey truth. Identify the antenna
model, antenna height, measurement point, WGS84 realization/epoch and survey
uncertainty. Transform external survey coordinates correctly before populating
the WGS84 profile. Include remaining datum/epoch uncertainty in the base budget.

The NTRIP adapter supports a fixed single-base **NTRIP v2 HTTP(S)** mount. It does
not send rover GGA, implement VRS selection or support legacy `ICY` NTRIP v1.
RTCM 1005/1006 ARP coordinates and station ID must match the configured survey
before any corrections are forwarded. Configure the caster to repeat 1005/1006
during capture, including its warmup. RTCM 1006 antenna height is recorded; it is
not subtracted from coordinates that already describe the ARP.

Set both credential environment variables, or neither for an anonymous caster.
Authenticated mounts require HTTPS; redirects and credentials in URLs are rejected.
Obtain secrets through your private environment/secret mechanism. Do not put them
in shell history or a checked-in JSON file.

```bash
# Terminal 1: corrections through the existing MAVROS connection.
wr-map ntrip --config /data/private/ntrip-base.json \
  --output /mnt/nvme/corrections/survey-001

# Terminal 2: immutable image and telemetry capture.
wr-map capture --config configs/oakd-mavros-survey.json \
  --telemetry-config /data/private/mavros-rtk-survey.json \
  --output /mnt/nvme/mapping/survey-001 --duration 600
wr-map validate /mnt/nvme/mapping/survey-001
```

Use unused output directories. Start corrections and check the rover's fixed
solution before capture. The camera profile has a 60-second warmup, excluded from
the saved-image duration; telemetry includes warmup evidence. A correction stall,
bad CRC, station change or coordinate mismatch fails the bridge rather than
silently switching bases. Restart explicitly into a new journal after diagnosing
the fault. Stop the bridge with Ctrl+C after capture finishes.

The bridge seals `corrections.rtcm3`, `events.jsonl` and `report.json`. Large valid
RTCM frames are forwarded as ordered chunks of at most 720 bytes because the
reviewed MAVROS plugin rejects larger ROS payloads. A DDS subscriber or a published
chunk proves transport activity, not that the rover applied the corrections.

### Receiver uncertainty and correction age

`GPSRAW.h_acc` and `v_acc` are millimetre uncertainty fields, but MAVLink does not
specify a universal confidence interpretation. Record the actual receiver/firmware
documentation and validate it against control occupations. Supported interpretations
are horizontal axis 1σ, horizontal RMS or radial 95%; vertical axis 1σ or absolute
95%. The calculation converts those explicitly into covariance. `eph`/`epv` are
DOP values, not metre errors. `GPSRTK.accuracy` is receiver-specific and is not
silently converted to metres or sigma.

The reviewed MAVROS primary `GPS_RAW_INT` handler sets `dgps_age` to
`UINT32_MAX`: **native receiver-applied correction age is unavailable on that
path**. Fresh received observation corrections and verified base coordinates are
reported separately. Repeated station descriptors alone do not refresh observation
correction age. The default image policy requires verified receiver age, so it
reports unqualified images if that evidence is unavailable.

For a validated deployment using this primary topic, an operator can explicitly
set `policy.require_receiver_correction_age` to `false`. Transport freshness and
RTK-fixed gates still apply; every image retains the missing receiver-age limitation,
and `fully_documented_receiver_age` remains false. This is a conditional transport
policy, not a claim that forwarded corrections were applied. Keep
`receiver.correction_age_field_verified` false unless the deployed message path
really carries the native receiver age. A future native receiver/status adapter
can provide stronger evidence; this implementation does not invent it.

### Verify the GNSS time convention

The GPSRAW header needs its own deployed-firmware check. The reviewed PX4
`GPS_RAW_INT` source emits UTC when valid and boot time otherwise. The reviewed
MAVROS `gps_status` plugin applies its FCU clock conversion to either value.
Applying a boot-to-ROS offset to an already Unix timestamp can add a second epoch;
ROS's signed 32-bit seconds field can then wrap negative. Such original headers
are preserved in the recording.

Choose the verified convention in `receiver.gpsraw_time_mode`:

| Mode | Recorded header interpretation |
|---|---|
| `unknown` | No qualified GNSS association |
| `px4_boot_via_mavros` | MAVROS already converted PX4 boot time into ROS time |
| `unix_via_mavros` | Undo the MAVROS boot offset on a UTC source, including signed-seconds wrapping |

Retain firmware/plugin revisions and bench evidence in
`timestamp_validation_evidence`. The UTC mode uses qualified TIMESYNC evidence at
the observation's receipt; source time is then mapped through the recorded ROS
clock bridge. Receipt time is never substituted for measurement time. Changing
firmware, GPS message/plugin or time mode requires requalification. See the
[clock audit](mavlink-integration.md) for exposure alignment and convergence gates.

## Per-image position budgets

The receiver's reported position may refer to ARP or antenna phase centre (APC).
Declare that reference and measure `antenna_to_camera_flu_m` from the same point
to the selected camera optical centre, in body forward/left/up axes. Confirm that
the recorded body orientation is in the base-tangent ENU frame; an unverified
heading/local-frame convention cannot rotate the lever arm reliably. Each camera
stream needs its own rig profile. Fused vehicle `NavSatFix` is not a substitute for
the raw receiver solution plus a calibrated antenna-to-camera transform.

Prefer `image-accuracy --rig-calibration FILE` with the same file passed to
`sync --rig-calibration` (a different or missing alignment calibration is refused).
The calibration then supplies `antenna_to_camera_flu_m` and
`lever_covariance_flu_m2` (camera minus `gnss_antenna_arp`, for the aligned stream),
`calibration_evidence` (its sha256), `lever_from_receiver_reference_verified`
(true only when the receiver `position_reference` is `ARP`, the calibrated antenna
point) and `camera_latency_bound_ms` (the OAK→PX4 offset 1σ, counted once inside the
alignment budget). Those profile fields are ignored, and any non-default value is
listed in `warnings`. `attitude_sigma_rad` (vehicle attitude error) and the other
motion bounds still come from the profile. Every `calibration-check` blocking item
becomes an image reason (`Rig calibration incomplete: ...`), so an `unset` link or
unknown sigma prevents qualification.

Supply symmetric positive semidefinite 3×3 covariances in m² for the base in ENU
and the lever arm in FLU, and an attitude axis 1σ in radians. The small-angle model
requires attitude sigma ≤0.1 rad. State whether the receiver uncertainty already
includes base uncertainty so it is not added twice. The model assumes the declared
component independence; known cross-correlation needs a more complete model.

Position is interpolated between fresh source-time GNSS samples **in ECEF**, with
no extrapolation. Both endpoints must be fixed and have valid uncertainty.
Endpoint covariance is blended linearly as a conservative envelope for unknown
temporal correlation, rather than halving variance by assuming independent noise.
The lever arm is rotated using the body orientation at the exposure.

Bound the receiver reference point's speed and acceleration, rig angular rate and
uncompensated receiver/camera latency. Include rotational antenna acceleration in
the acceleration bound. The deterministic allowance is:

```text
(speed_bound + angular_rate_bound × lever_length) × timing_interval
  + 0.5 × acceleration_bound × interval_before × interval_after
```

The timing interval includes the image association budget, GNSS clock-read bracket
and declared latency bounds. The acceleration term bounds linear interpolation
curvature. These are conditional on measured clock/motion bounds, not guaranteed
hardware timestamp accuracy. Latitude/longitude E7 and millimetre height
quantization are also budgeted. An accepted mismatch between RTCM ARP coordinates
and the surveyed base is retained as a deterministic position allowance.

The report separates rover, shared-base, lever and attitude covariance. Horizontal
95% is a Gaussian enclosing-ellipse radius; vertical 95% is 1.96σ. Motion,
quantization and coordinate-discrepancy allowances are then added conservatively.
These conditional **camera-position** figures exclude camera-ray uncertainty,
scene depth, bundle adjustment, dense reconstruction and non-Gaussian ambiguity/
multipath failures. Shared base error never decreases by √number-of-images.

On the workstation:

```bash
wr-map sync /data/sessions/survey-001 \
  --output /data/alignment/survey-001 --stream rgb --min-fraction 0.9

# Match the chosen processing recipe's selection settings. The bundled recipes
# use RGB, interval 0.5 s, min sharpness 0 and reject gaps.
wr-map export /data/sessions/survey-001 \
  --output /data/projects/survey-001 --stream rgb --interval 0.5
wr-map image-accuracy /data/sessions/survey-001 \
  --alignment /data/alignment/survey-001 \
  --profile /data/private/gnss-accuracy.json \
  --project /data/projects/survey-001 --output /data/image-accuracy/survey-001
```

Inspect `report.html`, `images.csv` and `images.jsonl` for every photograph's
qualification, reasons, missing evidence, correction status, source samples and
budget. Outputs and source/profile hashes are sealed in `report.json`. A partial
report is retained when evidence gates fail. A metric two-dimensional projected
CRS is required for reconstruction geolocation; choose a suitable local projection
and verify its scale/datum relationship. Heights remain **WGS84 ellipsoidal**.
No geoid/orthometric conversion or survey epoch propagation is performed here.

## Use the rover positions in reconstruction

### Terrain

With at least three noncollinear qualified camera positions, the report writes
`camera-geo.txt` for the selected names. It deliberately supplies four columns:
camera orientation and statistical prior weights are not fabricated. Preparation
renames the geolocation entries to match the calibrated undistorted working PNGs.

```bash
wr-map process /data/sessions/survey-001 --config /data/private/process-terrain-rtk.json \
  --output /data/runs/terrain-001 \
  --geo /data/image-accuracy/survey-001/camera-geo.txt \
  --vertical-datum 'WGS84 ellipsoidal height' --execute
```

Copy `configs/process-terrain.json` for that recipe. If desired, add
`gps_accuracy_m` using a conservative reviewed prior budget; the runner passes it
as ODM's `--gps-accuracy`. ODM 3.6.2 otherwise defaults to 3 m. This parameter
controls a reconstruction prior weight, not a measured map accuracy claim. Keep
the same image selection settings as the export. If some images lack qualified
positions, report those omissions and inspect registration; the file contains
only qualified entries. Real GCPs can also be supplied, with consistent datums.

### Buildings/objects

Run the existing building workflow in its original COLMAP world. After inspecting
its chosen sparse component and completing dense/mesh stages, align it using the
qualified camera centres. Read the actual paths from `workflow.json`/`report.json`;
the following path placeholders must be replaced:

```bash
wr-map georeference /data/projects/survey-001 \
  --model /data/runs/building-001/PATH-TO-SELECTED-MODEL \
  --image-accuracy /data/image-accuracy/survey-001 \
  --artifact /data/runs/building-001/PATH-TO-fused.ply \
  --artifact /data/runs/building-001/PATH-TO-mesh.ply \
  --output /data/georeferenced/building-001 --max-residual-m 0.15
```

At least five registered qualified camera positions with noncollinear geometry
are required. The weighted similarity estimates scale, rotation and translation;
it fails on excessive prior residuals rather than silently discarding outliers.
This uses RTK navigation as alignment control, not visual-inertial bundle adjustment.
Camera-prior residuals are not independent checkpoint accuracy.

The transformed COLMAP model and scalar-vertex ASCII/binary PLY products use
metres in projected axes with a nearby stored origin. Add
`report.coordinate_origin_xyz_m` **once** to obtain absolute projected XYZ.
PLY XYZ becomes float64, normals rotate, and colours/faces are retained. Original
models/products remain untouched. The original workflow pose CSV still describes
the original COLMAP world; read poses from the transformed model for metric use.
Supply every PLY map product when linking the final accuracy report to a building
workflow. Do not use sparse camera controls as withheld checks or claim that the
alignment measured individual ground-point error.

## Total map accuracy breakdown

Survey withheld checkpoints spread over the footprint, heights and relevant
surface classes. Do not use them in GCP fitting, camera alignment or an adjustment
to improve the reported errors. Record all fitted control IDs in the map profile;
workflow GCP IDs and georeference camera-control names are also checked when
linked. Independence beyond matching IDs needs documented survey evidence.

Measure checkpoint coordinates in the final product externally, retaining target
identification, pixel/surface selection method and reference survey records. The
tool accepts those measurements; it does not automatically identify targets in
an orthomosaic or point cloud. CSV columns are:

```csv
id,role,surface_class,reference_x_m,reference_y_m,reference_z_m,model_x_m,model_y_m,model_z_m,reference_sigma_x_m,reference_sigma_y_m,reference_sigma_z_m
```

Use `role=check`, unique IDs, positive reference axis 1σ values in metres, and a
common CRS and vertical datum. `surface_class` is optional. At least three
noncollinear points are needed to compute a report. The template requires 30 for
its acceptance gate; smaller reports explicitly fail that gate. This follows the
2024 ASPRS minimum as a starting policy, but the program does **not** assess full
standards compliance or generate certification language.

If checks use the same base as the map, common base bias cancels in their observed
differences. Set `shared_base_with_map=true`, supply its covariance in **map XYZ
axes**, and exclude it from per-checkpoint sigmas. The report retains that term
separately; more images/checkpoints do not reduce it. With independent external
control, set `shared_base_with_map=false` and leave shared covariance null; those
checks already test absolute map/base bias and adding it again would double-count.

For local-origin building output set `model_coordinates=local_origin`; the linked
georeference supplies its sealed origin. Reference XYZ remains absolute. For
absolute terrain coordinates use `absolute` with a null origin. The tool applies
an origin once and does not fit any transformation using the checks.

```bash
wr-map map-accuracy /data/quality/terrain-checks.csv \
  --profile /data/private/map-accuracy.json --output /data/map-accuracy/terrain-001 \
  --image-accuracy /data/image-accuracy/survey-001 --workflow /data/runs/terrain-001

wr-map map-accuracy /data/quality/building-checks.csv \
  --profile /data/private/map-accuracy-building.json --output /data/map-accuracy/building-001 \
  --image-accuracy /data/image-accuracy/survey-001 --workflow /data/runs/building-001 \
  --georeference /data/georeferenced/building-001
```

The linked workflow must be complete and its stage/product hashes must match.
Unscaled building workflows cannot produce a linked metric assessment. The report
breaks down XYZ bias, centred scatter, horizontal/vertical/3D RMSE, empirical p95
and maximum error, surface classes, checkpoint coverage, reference uncertainty,
shared-base covariance and the image-navigation summary. The reference-aware RSS
envelope combines observed residual RMS, reference sigma RMS and separately
shared base uncertainty. It is a conservative engineering budget, **not a
confidence interval or a guaranteed map-wide bound**. Empirical percentiles describe
this checkpoint sample; scene regions between checkpoints remain untested.

## Commissioning evidence before field claims

On a static control point, compare raw GNSS coordinates, height reference and
receiver uncertainty to independent surveyed truth across repeated occupations
and ambiguity resets. Measure timing/latency with an independent event reference,
including camera exposure, GNSS source time and PX4 attitude. Calibrate the lever
arm and heading/frame conventions for the actual mounting. Test moving trajectories
with known bounds and crossing geometry; straight camera paths cannot fully
constrain the metric similarity.

On a disposable bench dataset, interrupt corrections, force float/no-fix, inject
stale timing and verify reports fail visibly. Verify repeated base descriptors and
MAVROS fragmentation reach the actual PX4/receiver. Run both full engines on real
imagery, inspect outputs and measure distributed withheld checkpoints, including
independent control that can reveal base bias. Retain receiver/base/camera/PX4/
MAVROS revisions and calibration records with each accepted deployment profile.

Software tests cover synthetic evidence, geodesy, covariance, source clocks,
interpolation, protocol CRC/fragmentation, actual COLMAP binary IO and sealed
workflow integration. CI additionally exercises actual ROS Humble services, DDS,
GPS message CDR and a local NTRIP stream. These tests do not establish hardware
timing, receiver accuracy or successful real-survey reconstruction.

## Reviewed primary references

- [MAVROS GPS status and time conversion](https://github.com/mavlink/mavros/tree/5c68b905ab30de6ce630822dc46c33467e8f23ea/mavros/src/plugins)
  and [message definitions](https://github.com/mavlink/mavros/tree/5c68b905ab30de6ce630822dc46c33467e8f23ea/mavros_msgs/msg): raw units, primary correction-age sentinel, timestamp conversion and RTCM forwarding.
- [PX4 GPS_RAW_INT timestamp source](https://github.com/PX4/PX4-Autopilot/blob/b12d17f63241e7c875a0394e7b52064ca3a34946/src/modules/mavlink/streams/GPS_RAW_INT.hpp)
  and [PX4 RTK integration](https://docs.px4.io/main/en/gps_compass/rtk_gps): deployed receiver and UTC/boot convention need verification.
- [MAVLink GPS_RAW_INT](https://mavlink.io/en/messages/common.html#GPS_RAW_INT): fields and protocol units; no universal h_acc/v_acc confidence model is specified.
- [RTKLIB RTCM decoder reference](https://github.com/tomojitakasu/RTKLIB/blob/master/src/rtcm3.c): station/ARP field layout; this repo's decoder is an original minimal implementation, not copied RTKLIB code.
- [ODM 3.6.2 configuration](https://github.com/OpenDroneMap/ODM/blob/v3.6.2/opendm/config.py)
  and [camera geolocation format](https://docs.opendronemap.org/geo/): explicit prior weighting and original/derived name mapping.
- [NGS single-base real-time GNSS guidelines](https://www.ngs.noaa.gov/web/news/NOAA_Releases_RealTime_Guidelines.shtml)
  and [antenna calibration](https://www.ngs.noaa.gov/ANTCAL/): survey practice, ARP/APC and independent occupations.
- [ASPRS 2024 accuracy-standard changes](https://old.asprs.org/archives/asprs-approves-edition-2-version-2-of-the-asprs-positional-accuracy-standards-for-digital-geospatial-data-2024.html): checkpoint uncertainty and minimum counts; full compliance is outside this report's scope.
