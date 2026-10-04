# ROS2 recording and offline processing

The acquisition stack is the official Luxonis `depthai_ros_driver`, MAVROS and
`ros2 bag record` with the MCAP storage plugin. The repository's shell supervisor
owns startup, preflight and shutdown; it does not receive or serialize sensor
messages. Python is used only for CLI dispatch and offline checks/import.

The commissioned runtime is ROS Humble on ARM64, driver 2.12.2 with DepthAI C++
2.31.1, MAVROS 2.14.0 and rosbag2/MCAP 0.15.14. The existing
`drone_autonomy_platform:orin` image supplies these packages on this Jetson.
`deploy/run-ros.sh` bypasses its application entrypoint. Native installations need
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
until SIGINT/SIGTERM. `--device-id` selects a specific OAK. Native ROS uses the
supplied FCU URL directly; the Docker wrapper maps `WR_MAPPING_SERIAL_DEVICE` to
`/dev/ttyUSB0`. It does not change PX4 baud, stream rates or other parameters.
Container recordings outside the checkout require `WR_MAPPING_ROOT` set to the
existing output parent. Use `WR_MAPPING_MOUNT` plus `--require-mount` for a required
storage mount. The service uses these checks by default. Containers currently
write as their image user (root); preserve or adjust ownership when transferring.

The default [driver profile](../configs/oakd-ros.yaml) requests RGB 4056×3040 and
left/right 1280×800 at 2 fps, fixed exposure, RGB focus and white balance, with
100 Hz raw accelerometer and gyroscope requests. The BNO's supported sensor rates
and COPY synchronization can produce irregular combined IMU timestamps. The
measured ROS IMU message rate is approximately 100 Hz; it is not proof of two
independent lossless 100 Hz sensor streams. Driver 2.12.2 does not declare the
batch-size requests with rotation disabled; use the saved parameter dump to
establish what was actually accepted. Orientation is not enabled.

Raw images are recorded directly, without JPEG/PNG encoding or video compression
in the acquisition path. MCAP uses indexed uncompressed chunks, a 100 MiB rosbag
cache (rosbag2 double-buffers it), approximately 1 GiB file splits and a 5 GiB
free-space reserve. At these dimensions, expect approximately 78 MB/s, 4.4 GiB/min
and 260 GiB/hour. Benchmark the actual disk and budget the run length accordingly.

The supervisor checks camera-info/IMU arrival and PX4 connection before capture,
saves effective parameters, and refuses a second `/oak` or MAVROS owner. An owner
using a different node name/domain still needs to be stopped manually. It checks
owned process health and disk reserve during capture. Offline validation detects
required-stream stalls; process health alone does not establish stream health.
SIGINT drains rosbag before publishers stop. Failed/preflight runs are retained;
`state=complete` means clean finalization, not survey acceptance.

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
GNSS/RTK topics are explicit. `valid` means storage/content validation passed;
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
