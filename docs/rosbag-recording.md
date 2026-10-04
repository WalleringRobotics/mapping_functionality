# ROS2 recording and offline processing

The acquisition stack is the official Luxonis `depthai_ros_driver`, MAVROS and
`ros2 bag record` with the MCAP storage plugin. [ROS launch](../deploy/record.launch.py)
owns the driver and recorder processes, their exits and shutdown timeouts. The shell
prepares the session, runs launch in the foreground, then seals the stopped files.
The Python launch description configures standard ROS processes; it does not
subscribe to or write sensor messages.

The commissioned runtime is ROS Humble on ARM64, driver 2.12.2 with DepthAI C++
2.31.1, MAVROS 2.14.0 and rosbag2/MCAP 0.15.14 (bench, 2026-10-04). The repository
[image](../docker/Dockerfile) pins driver 2.12.2, MAVROS 2.14.0 and rosbag2/MCAP
0.15.16 from the 2026-08-07 ROS snapshot, plus `mavros_extras` for the GPS status
topics; build it as in [Jetson setup](jetson-setup.md#fresh-orin).
`deploy/run-ros.sh` uses it whenever ROS is not sourced. Native installations need
`ros-humble-depthai-ros-driver`, `ros-humble-mavros` and
`ros-humble-rosbag2-storage-mcap` in a sourced Humble environment. Python DepthAI
3.10.0 remains the separate firmware/diagnostic SDK, not the live ROS driver.

## Record

From this checkout, install `pip install -e '.[bags]'`. Give the OAK one owner,
verify the UART owner and use a new directory under an existing parent:

```bash
# Existing MAVROS connector: reuse it in DDS domain 1.
.venv/bin/wr-map capture --output runs/session-001 --duration 120 --warmup 60

# If this session owns MAVROS instead, select the real adapter by its stable path.
export WR_MAPPING_SERIAL_DEVICE=/dev/serial/by-id/your-adapter
.venv/bin/wr-map capture --output runs/session-002 --duration 120 --warmup 60 \
  --start-mavros --fcu-url /dev/ttyUSB0:921600

# Camera-only commissioning; explicitly excludes PX4 acceptance.
.venv/bin/wr-map capture --output runs/camera-001 --duration 60 --camera-only
```

`capture` and `record` invoke the same standard recorder. `--duration 0` records
until terminal Ctrl+C, `docker stop`, or `systemctl stop`. `--device-id` selects a specific OAK. Native ROS uses the
supplied FCU URL directly; the Docker wrapper maps `WR_MAPPING_SERIAL_DEVICE` to
`/dev/ttyUSB0`. It does not change PX4 baud, stream rates or other parameters.
Container recordings outside the checkout require `WR_MAPPING_ROOT` set to the
existing output parent. Use `WR_MAPPING_MOUNT` plus `--require-mount` for a required
storage mount. The service uses these checks by default. Container file ownership follows the image/runtime user mapping; preserve
appropriate ownership when transferring.

The default [driver profile](../configs/oakd-ros.yaml) requests RGB 4056×3040 and
left/right 1280×800 at 2 fps, fixed exposure, RGB focus and white balance, with
raw gyroscope at 200 Hz and raw accelerometer at 250 Hz with
`LINEAR_INTERPOLATE_ACCEL`: each `/oak/imu/data` message is a native gyroscope
sample on its own timestamp, with the accelerometer interpolated onto it. The BNO086
offers gyro 25/33/50/100/200/400 Hz and accel 15/31/62/125/250/500 Hz (requests round
up), with no common rate. The former 100/100 Hz `COPY` profile stamped messages on the
accelerometer's native ~128 Hz grid and copied the latest gyro sample in, so gyro
timestamps were off by up to one accelerometer period and samples repeated; 400 Hz
lost samples heavily. Some loss remains at 200 Hz ([#18](https://github.com/WalleringRobotics/mapping_functionality/issues/18));
check the audit. See the [OAK-D hardware reference](oak-d-hardware.md). Driver 2.12.2 does not declare the
batch-size requests with rotation disabled; use the saved parameter dump to
establish what was actually accepted. Orientation is not enabled.

Raw images are recorded directly, without JPEG/PNG encoding or video compression
in the acquisition path. MCAP uses indexed uncompressed chunks, a 100 MiB rosbag
cache (rosbag2 double-buffers it), approximately 1 GiB file splits and a 5 GiB
free-space reserve. At these dimensions, expect approximately 78 MB/s, 4.4 GiB/min
and 260 GiB/hour. Benchmark the actual disk and budget the run length accordingly.

One-shot startup checks verify camera-info/IMU arrival and PX4 connection,
saves effective parameters, and refuses a second `/oak` or MAVROS owner. An owner
using a different node name/domain still needs to be stopped manually. It checks
process exits through ROS launch events and disk reserve through a launch timer. Offline validation detects
required-stream stalls; process health alone does not establish stream health.
Warmup runs with rosbag already recording, retaining pre-roll for discovery.
The requested capture interval starts afterward. At its timed end a launch timer
retains two seconds of post-roll, then emits the standard launch shutdown event.
ROS launch signals its processes and rosbag2 flushes its own cache and finalizes
MCAP. There is no shell PID polling, process-group termination loop or custom
signal escalation. The recorder gets 30 seconds to exit before launch escalates.
Unowned MAVROS remains running. A manual interruption shuts down immediately;
it has no guaranteed post-roll, so inspect its coverage audit.
`acquisition-start-ns.txt` / `acquisition-end-ns.txt` seal the host realtime
interval; coverage checks exclude pre-roll and shutdown drain. These times are
not calibrated physical exposure timestamps. Failed/preflight runs are retained;
`state=complete` means clean finalization, not survey acceptance. Checksums and
`ros2 bag info` run only after launch returns; content validation is the separate
offline command below. The container init and systemd deliver stop signals to the
foreground group. The shell merely survives that signal to finish sealing.

## Inspect and replay

```bash
.venv/bin/wr-map status runs/session-001
.venv/bin/wr-map validate runs/session-001 --report runs/session-001-audit.json
bash deploy/run-ros.sh ros2 bag info "$PWD/runs/session-001/bag"
# Replay on an isolated DDS domain, away from the live vehicle connector.
ROS_DOMAIN_ID=71 bash deploy/run-ros.sh ros2 bag play "$PWD/runs/session-001/bag"
```

The session contains `bag/*.mcap`, rosbag metadata, requested/effective driver and
MAVROS time parameters, vendor calibration, topic inventory, versions, logs and
`SHA256SUMS`. Keep the entire directory together. Validation checks seals, decodes
CDR, compares counts with metadata, validates image geometry and calibration,
checks timestamps/rates, required streams, start/end coverage and PX4 connection, and reports the
existing 501-sample/10 ms RTT/2 ms offset-residual timing gate. Missing optional
GNSS/RTK topics are explicit. PX4 connection is checked throughout the sealed
acquisition interval, including the last state known at its start; disconnected
pre/post-roll samples remain counted in the report. `valid` means storage/content validation passed;
`capture_ready` also requires image coverage at both bag boundaries. An early
stream stop can leave a readable bag that fails full-interval coverage. Reports go outside the immutable source session.

Image and IMU messages have no original hardware sequence counters. Validation
reports source sequence loss as unknown, never zero. `Imu` has one timestamp for
the combined accelerometer/gyro values; separate sample times are unavailable.
ROS header time is the driver's mapped measurement time; MCAP receipt time is
separate. Neither proves physical exposure-to-PX4 synchronization. Factory IMU
extrinsics and the physical camera-to-vehicle mounting still need qualification.

## Offline image preparation

```bash
.venv/bin/wr-map bag-import runs/session-001 --output runs/session-001-images
.venv/bin/wr-map validate runs/session-001-images
.venv/bin/wr-map process runs/session-001-images --config configs/process-building.json \
  --output runs/session-001-building --prepare-only
```

Import checks the original bag, writes lossless PNGs and CameraInfo calibration,
and preserves original ROS header/receipt timestamps and source-seal provenance.
Image sequence fields are import ordinals. The compatibility `device_ns` and
`host_synced_ns` fields contain ROS header timestamps, not OAK uptime/steady time;
the manifest explicitly records that distinction. Exposure/focus settings are
requested parameters, not per-frame hardware readbacks. IMU and PX4 remain in
the original MCAP. The existing `sync` command refuses these derived image sets
because they lack its measured clock-bridge contract. Camera geolocation must not
be inferred from matching timestamp units alone.

COLMAP/ODM image workflows operate on the imported dataset. The original bag
remains the authoritative source for future ROS playback, timing or inertial
processing. A stationary bench capture tests acquisition and image preparation,
not a usable multi-view reconstruction or mapping accuracy.

## Service and diagnostics

`deploy/record-session.sh` and `deploy/wr-mapping.service` now invoke this same
ROS recorder. Configure [the environment file](../deploy/wallering-mapping.env.example)
for a native sourced ROS installation and the required NVMe mount. The service
user needs storage, USB and UART permissions (`plugdev`, `dialout`). Reuse MAVROS
by default; opt into `WR_MAPPING_START_MAVROS=true` only with exclusive UART
ownership. The service is supplied for installation after commissioning; the
bench command does not enable an unattended boot service.

The former direct Python writer is available only as `legacy-capture` for
reproducing diagnostics. JSON capture profiles, `doctor --mode capture` and
`hardware-check --mode capture` refer to that legacy SDK path. They do not certify
the ROS stack. Use a bounded ROS capture followed by `validate` for its startup
and throughput check. Existing SDK-format datasets and offline workflows remain
readable. Historical hardware reports retain their original tool/version context.


## Guided calibration and the 20 fps candidate

`wr-map calibrate-record --output runs/calibration-001` records a fixed 110-second
motion sequence after readiness and 60 seconds of warmup. It uses
`configs/oakd-ros-calibration.yaml`: lossless mono 800P at 20 Hz, RGB 1080P at
2 Hz, and native OAK gyro 100 Hz / accelerometer 125 Hz with acceleration
interpolated onto gyro times. The ordinary default remains the historical
12 MP / 2 Hz profile until the 20-minute qualification below passes.

The mode requests both PX4 IMU streams at 100 Hz using an **existing MAVROS
owner**; start MAVROS separately. `--px4-imu-rate 0` preserves existing streams,
and `--camera-only` skips PX4 entirely for camera diagnostics. These options do
not satisfy combined sensor calibration acceptance. The per-session request and
reset logs are described in [MAVLink integration](mavlink-integration.md#temporary-px4-imu-rate-requests).

Remove propellers, support the rigidly mounted OAK/PX4 assembly, provide cable
strain relief, and keep cameras facing a textured scene at least 2 m away.
Follow the terminal prompts: still 10 s, then three 20 s cycles, each containing
one smooth roll sweep through ±30° (6 s), one pitch sweep (7 s) and one yaw sweep
(7 s); finish with free multi-axis motion (30 s) and still 10 s. Cycling axes
keeps independent solver windows observable while retaining three sweeps per
axis. `session.json` marks `kind: calibration` and
`calibration-phases.json` records the actual host realtime of each prompt. These
are operator hints, not measured rotations or solver ground truth. Normal seal
and audit rules apply; an interrupted calibration can preserve a clean bag but
fails the full-phase calibration audit. All present provenance files, including
factory calibration, phase logs and rate logs, must appear in `SHA256SUMS`.

`configs/oakd-ros-20fps-candidate.yaml` uses the same cameras with OAK gyro 200 Hz
and accelerometer 250 Hz. The deliberate RGB choice is **lossless 1080P at 2 Hz**:
2×1280×800 mono at 20 Hz plus 1920×1080×3 RGB at 2 Hz is about 53.4 MB/s,
2.99 GiB/min and 179 GiB/hour before MCAP/telemetry overhead. Keep at least
65 GiB free for a 20-minute soak plus reserve and pre/post-roll; the measured
budget, not this pixel calculation, determines actual available run length.
Lossy RGB has not been adopted. Compared with 12 MP RGB, 1080P sacrifices
spatial detail; the actual CameraInfo intrinsics determine GSD = height/focal
length in pixels. Along-track overlap depends on footprint, speed and frame
interval (overlap ≈ 1 − speed/(fps × footprint length)). At survey speed the
2 Hz RGB stream may have less overlap than the 20 Hz mono pair. Choose altitude,
speed and output stream against the required GSD before qualifying a survey.
Offline `bag-import` retains each stream's independent requested rate in the
derived manifest; derived validation uses that rate for gap warnings.

Recorder subscriptions now use the copied `rosbag-qos.yaml` with deep small-message
queues, and participants launched by the recorder use the copied
`fastdds-profile.xml` with a 64 MiB shared-memory segment. These mitigate queue
pressure from large frames; BEST_EFFORT and upstream losses remain possible.
Every image/IMU audit now reports `in_window`: observed samples, nominal expected
count, count deficit, inferred missing periods in gaps, measured rate and repeated
gyro values. Count deficits are estimates affected by boundary phase and sensor
clock drift; repeated stationary gyro values are not proof of loss. Hardware
sequence loss remains unknown. Empty windows report a full requested count
deficit; duplicate/backward IMU stamps fail validation. PX4 IMU below 95% of an
explicitly requested rate fails validation.

For qualification, record both the historical default and candidate for at least
1200 seconds after warmup. Archive `tegrastats` during each run and use
`wr-map validate SESSION --report runs/NEW-audit.json`. Inspect each in-window
camera and IMU rate (within 1%), no interior gaps, USB SUPER in the driver log,
mean/peak bag write load, RAM/CPU and maximum temperature/thermal margin. Check
PX4 pose rate, MAVLink loss and TIMESYNC RTT/residual against a baseline. A clean
seal or an audit with IMU gap warnings is not a passing soak. Physical motion
acceptance requires an operator; automated stationary capture cannot verify it.
