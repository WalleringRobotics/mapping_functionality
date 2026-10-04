# Recording and derived dataset contracts

New recordings use the [ROS2/MCAP session contract](rosbag-recording.md#inspect-and-replay).
The following v1 image/journal layout describes legacy SDK sessions and offline
image imports. For `source=rosbag2`, calibration comes from CameraInfo, sequence
numbers are import ordinals, IMU/PX4 remain in the original MCAP, and both
`device_ns` / `host_synced_ns` compatibility fields contain original ROS header
time. The import manifest records these exceptions explicitly; they must not be
interpreted as hardware counters or measured clock bridges. Per-frame exposure,
ISO, focus and white balance are requested driver settings, not device readbacks.

## Image dataset v1

Each session is a new directory; never resume or overwrite it.

| Path | Meaning |
|---|---|
| `manifest.json` | Schema/software version, config, device, lifecycle, counts, hashes, host details |
| `calibration.json` | Complete original EEPROM JSON payload |
| `frames.jsonl` | One indexed row per saved image |
| `imu.jsonl` | Independent raw accelerometer/gyroscope rows; may be empty |
| `clock.jsonl` | Bracketed SDK steady-to-Python monotonic observations plus receipt wall provenance |
| `telemetry.jsonl` | Optional independent MAVROS messages and ROS-to-monotonic clock evidence |
| `events.jsonl` | Gap and stop/failure records |
| `images/rgb/000000000123.jpg` | RGB JPEG, or PNG in the lossless profile |
| `images/left/000000000123.png` | Native mono image |
| `images/right/000000000123.png` | Native mono image |

Filenames contain sequence numbers, which are per camera. Do not pair sensors just
because sequence numbers match. Timestamps have integer nanosecond **units**;
Python's DepthAI `timedelta` representation is microsecond-resolution. Units do not
imply nanosecond measurement precision.

## Frame fields

| Field | Meaning |
|---|---|
| `stream`, `sequence`, `path` | Stable camera name, native sequence, relative image path |
| `width`, `height`, `bytes`, `sha256` | Stored dimensions and image integrity |
| `device_ns` | OAK monotonic `getTimestampDevice(MIDDLE)` |
| `host_synced_ns` | DepthAI host steady-clock `getTimestamp(MIDDLE)` |
| `received_monotonic_ns`, `received_utc_ns` | Host dequeue times, including transport/scheduling delay |
| `exposure_us`, `iso`, `lens_position`, `white_balance_k` | Returned frame settings; availability varies by sensor |
| `camera.K` | Pixel intrinsics from frame transformation metadata |
| `camera.distortion`, `camera.model` | Source distortion parameters and explicit model |
| `camera.source_width`, `camera.source_height` | Source geometry reported by SDK |
| `camera.processing` | Provenance of requested full-FOV/unrectified output |

MIDDLE is an SDK convention; physical latency and rolling readout need measurement.
A placeholder optional setting is not a measured zero. Verify fixed geometry/focus
with target images, especially on fixed-focus modules returning a placeholder lens
position. Export refuses changing camera metadata or lens position.

## IMU

`sensor` is `accelerometer` or `gyroscope`; `xyz` units are m/s² or rad/s. `report`
is `RAW`, `frame` is `sensor_native`. Each has its own sequence/timestamps. Repeated
reports in a batch are deduplicated. Requested and actual rates may differ [R4](references.md).
There is no interpolation, gravity removal or orientation estimation. This evidence
is retained for later VIO and is not an automatic camera-pose source.

## Lifecycle and audit

- `recording`: manifest written before acquisition; interrupted sessions stay here.
- `complete`: accepted work drained and final counts written.
- `failed`: detected capture/storage/stream failure; reason retained.

`validate` returns 0 for structurally valid complete data, 2 for invalid/incomplete
data. Sequence gaps are warnings to be assessed against survey overlap. Export
requires explicit `--allow-gaps` for a selected stream with gaps. Integrity validation
does not assert that imagery can reconstruct or meet an accuracy target.

Completed manifests seal the four camera journals and optional telemetry journal with SHA-256, protecting timestamps
and settings against unnoticed transfer corruption.

Validation checks images, hashes, counts, decode dimensions, pixel intrinsics,
monotonic clocks, sequences, actual rates and nearest timestamps. It reports unindexed
images and invalid JSON tails. Hashes detect corruption, not capture authenticity.

Preserve failed originals. Recovery should operate on a copy, identify the last
complete indexed image, document exclusions and create a derived dataset with
provenance. Automated recovery is future work; the exporter rejects failed sources.

## Export

`images/` contains byte-identical selected images. `project.json` contains source
hashes, image hashes, selection parameters and camera model/parameters. `selection.jsonl`
records every decision, timestamp, Laplacian sharpness and dark/bright pixel fractions.
`source-calibration.json` and `validation.json` retain evidence. Status is `building`,
`ready` or `failed`; reconstruction results go in a separate directory.

Lens conversion supports perspective→`FULL_OPENCV` and four-coefficient
fisheye→`OPENCV_FISHEYE`. Nonzero thin-prism/tilt terms are rejected, not discarded.
Export requires one resolution, focus and calibration group. New camera adapters
must honor the clock, calibration and integrity contract or version the schema.

## Operational and processing manifests

Capture `status.json` is an atomic, mutable heartbeat separate from sealed sensor
journals. It updates every five seconds and at shutdown. It is useful for monitoring,
not a replacement for final manifest/journal validation. The manifest records software
versions, repository revision/dirty state, architecture, kernel and Jetson release.

Offline `workflow.json` records immutable source/config/control fingerprints and
numbered stage attempts. Each completed attempt seals nonempty artifacts with length
and SHA-256. `report.json` summarizes successful preparation/execution and product paths;
`workflow.json` is authoritative after failures. `stages/terrain-input-NNN/preparation.json`
records undistortion, original-to-derived filenames, applied camera model, control CRS
and operator-stated height reference. Engine `run.json` files retain command arrays,
versions, completion/failure and products. None of these manifests certifies metric accuracy.

## Optional MAVROS side contract v1

`manifest.telemetry` identifies `adapter=mavros_ros2`, schema version, complete
configuration, read-only parameter snapshot, frame/height conventions and final
capture summary. Existing sessions without this field remain supported. This
contract is independent of the camera dataset schema.

Message rows contain `record_type=message`, `role`, absolute `topic`, `ros_type`,
per-role `ordinal`, original `source_stamp_ros_ns`, `fields`, `cdr_base64`,
`received_monotonic_ns`, `received_utc_ns` and `receipt_clock`. Nonfinite sensor
values are explicit JSON markers such as `{"nonfinite":"nan"}`; the CDR keeps
original bits. These are serialized **ROS** messages, not raw MAVLink packets.
TIMESYNC rows additionally contain reproducible `sync_quality` evidence.

Periodic `record_type=clock` observations map `target=ros_system` to
`reference=python_monotonic`. SDK observations in `clock.jsonl` use
`target=depthai_steady`. Each bridge includes integer `target_ns`, midpoint
`reference_ns` and `bracket_ns` spanning the clock read. Receipt brackets are
retained independently. Source/receipt clocks must never be interchanged.
Legacy standalone clock snapshots remain readable; the new `sync` command
requires bracketed bridges from a telemetry-enabled capture.

MAVROS local body pose is ENU with FLU body axes; ROS IMU is FLU; OAK raw IMU
is sensor-native. MAVROS global NavSatFix is vehicle position with WGS84
ellipsoidal height. ROS nanosecond units do not imply nanosecond PX4 source
precision: local position commonly uses millisecond boot timestamps.

`sync` writes a fresh derived directory containing `associations.jsonl`,
`body-poses.csv` and sealed `report.json` metadata. Per-image decisions retain
original filenames, exposure clocks, body-pose interpolation evidence, optional
nearest IMU/GNSS fields and estimated temporal budgets. Unassociated images are
retained with reasons. There is no camera mounting transform or automatic pose
fusion. See [integration and timing acceptance](mavlink-integration.md).

## Corrected GNSS and derived accuracy

The RTK telemetry profile adds `gps_raw`, optional `gps_rtk` and `rtcm` roles to
the same immutable message journal. Raw integer coordinates, altitude fields,
uncertainty fields, fix type and correction-age sentinel are preserved. GPSRAW
original headers can contain valid negative signed seconds after an incorrectly
reapplied UTC offset; they are recorded without receipt-time substitution.
The explicit deployed timestamp convention is verified during offline reporting.

`ntrip` seals a separate raw RTCM and forwarding journal. The capture's RTCM
subscription independently records the published correction stream. Base station
descriptors and observation freshness are distinct from receiver-applied age.

`image-accuracy` creates sealed JSONL/CSV/HTML reports. Each image retains exact or
bracketing source GNSS samples, ECEF interpolation weight, camera lever transform,
covariance components, deterministic allowances and qualification reasons. An
optional selected-image project restricts names for `camera-geo.txt`. Source
capture, alignment, profile and derived output hashes identify the inputs used.

`georeference` creates a transformed COLMAP model and PLY copies, navigation-control
residuals, source/output hashes and a projected XYZ origin. Add that origin once
to the stored local metre coordinates. `map-accuracy` seals independent checkpoint
errors and the reference/shared-base breakdown, optionally linked to those products.
The latter does not fit a transformation on the checks. See the
[accuracy contract and operator guide](rtk-accuracy.md).
