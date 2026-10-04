# Orin hardware checks and bench results

See the [command guide](hardware-commands.md) for the complete reproducible command
set and passive serial diagnostics, and the
[repository skill](../skills/jetson-mapping-checks/SKILL.md) for future agent sessions.

**Update 2026-10-04:** BNO086 firmware is now 3.9.9. The default stack uses the
official Luxonis ROS driver, MAVROS and standard rosbag2/MCAP. The final bench
recording passes integrity and requested-window coverage at 2 fps and a 100 Hz
IMU request. Survey readiness still requires physical timing/rig calibration and
missing GNSS/RTK evidence. OAK runtime USB is SUPER (5 Gbit/s); TELEM2 remains
921600 baud with flow control off.

## ROS launch ownership verified: 2026-10-04

`deploy/record.launch.py` now owns the official OAK driver, optional MAVROS and
standard `ros2 bag record`. Launch events handle exits and shutdown; the shell
has no PID polling or signal escalation. A launch timer retains two seconds of
post-roll for timed recordings. `seal-rosbag.sh` runs after launch returns.
Docker init and the service deliver SIGINT to the foreground group; finalization
can finish before container/service teardown.

The new full-stack run requested 60 seconds and measured **60.005 seconds** in
the acquisition interval. The complete bag, including startup/pre/post-roll,
contains **175 RGB, 174 left and 174 right images** at approximately 2 Hz and
**8,700 OAK IMU messages at 99.77 Hz**. No interior image timestamp gaps occurred;
all three camera streams cover both interval boundaries. OAK, MAVROS and rosbag2
exited cleanly. The external audit reports `valid=true`, `capture_ready=true`,
`coverage_complete=true`, `survey_ready=false`.

PX4 was connected at interval start and throughout acquisition. One disconnected
startup state remains in the pre-roll and is reported separately. Timing reached
805 consecutive good samples, with 305 of 911 samples qualified by the unchanged
501-sample gate. This does not qualify the entire interval or physical exposure
timing. The missing GNSS/RTK topics and calibration limitations still apply.

A second, unbounded camera-only run was stopped with normal `docker stop` after
17.210 seconds of acquisition. ROS launch and rosbag2 again exited cleanly; the
sealed bag passed integrity and coverage checks. Its worst image tail gap was
0.451 seconds. Manual stops have no guaranteed post-roll, so this result is not a
replacement for checking each interrupted recording.

Evidence remains under ignored `runs/rosbag-commission-20261004-001/launch-001`
and `launch-stop-001`, with external audits (`launch-001-audit-v2.json` is current).
The first audit retained the old all-bag PX4 connection gate and rejected the
expected disconnected startup sample; the current audit uses the sealed interval
and retains pre/post-roll state counts. Coverage also refuses to shorten the
requested interval when a bag ends early.

Verification: **140 portable tests passed**, with processing tests still awaiting
the PyCOLMAP source build and two ROS modules skipped on the host. In the ROS
container, **six integration tests passed**, including timed completion, Ctrl+C,
driver/recorder failures, failed readiness and MAVROS DDS/parameter/CDR behavior.
Ruff, shell syntax and whitespace checks passed. Both hardware sessions released
their devices; no recording service was installed or enabled.

## Recording-window investigation before ROS launch: 2026-10-04

Checkpoint `549d23d` introduced the ROS stack. An independent mono-only rosbag2
recorder then received two late left-camera frames absent from the main recorder.
The first comparison run is retained as failed because editing its executing
shell script caused a shutdown syntax error; subsequent trials used frozen copies.
The then-tested shell implementation retained pre-roll with rosbag already running,
started the requested interval after warmup, kept two seconds of post-roll, stopped
owned publishers, let queued messages drain, then closed rosbag. It has since been
replaced by standard ROS launch process ownership. Host realtime interval bounds are sealed;
they do not establish physical shutter timing.

The final 60-second request measured a **60.362-second acquisition interval** inside
a 65.568-second bag. It saved **131 RGB, 131 left and 131 right images**, including
pre/post-roll, at 2.000 Hz, plus **6,536 OAK IMU messages at 99.79 Hz**. All image
headers are monotonic with no interior interval over 1.5 frame periods. Both ends
of the acquisition interval are covered. An independent rosbag2 instance captured
121 frames from each mono camera within that interval, and **every one of those
ROS header timestamps is present in the main bag**. Original hardware sequence
continuity remains unknown; these are ROS message comparisons.

The audit reports `valid=true`, `capture_ready=true`, `coverage_complete=true`,
`survey_ready=false`. Fresh PX4 timing reached 640 consecutive good observations;
140 of 651 samples passed the unchanged 501-sample qualification gate. That covers
the later part of this short run, not its entire interval. RTT median/p95/max were
2.835/4.035/12.716 ms and offset-residual median/p95/max were 0.269/0.631/2.076 ms.
Configured fused GNSS and `gpsstatus` raw/RTK topics remain absent. IMU intervals
are irregular (maximum 31.978 ms); no lossless hardware-sample claim is made.

Evidence: ignored `runs/rosbag-commission-20261004-001/window-001`, its external
audit, `mono-reference-003` and `mono-comparison-003.json`. Earlier partial-coverage
bags remain preserved. Actual offline processing imported all 707 images from the
first two-minute bag and prepared 120 selected RGB images for the building workflow.
No reconstruction or accuracy claim is made from stationary bench images.

Verification: 137 portable tests passed and one ROS-only test module skipped;
`test_processing.py` was excluded because the pinned ARM64 `pycolmap` dependency
was not yet installed. The then-current supervisor tests covered SIGINT/SIGTERM, publisher-before-
recorder shutdown, sealed completion and acquisition-window handling. Ruff, shell
syntax checks and the actual recording/import/preparation paths passed. All owned
recording containers exited, releasing the camera and UART.

Next: field-duration soak under intended workloads, physical timing/mounting
calibration, and the missing GNSS/RTK integration. Retain the existing timing gates.

## Initial ROS2/MCAP commissioning: 2026-10-04

See [the reproducible recording procedure](rosbag-recording.md). Runtime: Humble,
`depthai_ros_driver` 2.12.2 / C++ DepthAI 2.31.1, MAVROS 2.14.0 and rosbag2/MCAP
0.15.14, all ARM64 from the existing platform image. The ROS driver is separate
from the host Python DepthAI 3.10.0 used for firmware commissioning.

The default profile requests 2 fps RGB/left/right, 100 Hz raw accelerometer and
gyroscope, COPY synchronization, manual image settings and uncompressed indexed
MCAP. A 60-second warmup preceded a requested 120-second recording; saved receipt
time spans 119.725 seconds. The bag is approximately 8.7 GiB across nine MCAP files.

| Stream | Saved messages | Observed rate / dimensions |
|---|---:|---|
| RGB | 239 | 1.9997 Hz, 4056×3040 |
| Left mono | 237 | 2.0000 Hz, 1280×800 |
| Right mono | 231 | 2.0000 Hz, 1280×800 |
| OAK combined IMU | 11,902 | 99.4057 Hz |
| PX4 raw IMU | 5,985 | 50.0003 Hz |
| PX4 local pose | 3,591 | 29.9994 Hz |
| Raw GNSS fix | 598 | 598 valid status values |
| TIMESYNC | 1,196 | 0 qualified observations |

All three streams start within 0.362 seconds of the bag start, but the last left
image precedes the bag end by 1.700 seconds and the last right image by 4.687
seconds (RGB: 0.425 seconds). The cause of this truncated mono tail remains
unresolved; it must not be attributed to initial discovery. All saved image
streams have monotonic headers and no interior interval over 1.5 frame periods.
The revised audit separates readable/valid storage from full capture coverage:
this bag is `valid=true`, `capture_ready=false`, `survey_ready=false`. Effective parameters, constant image/CameraInfo geometry,
all CDR payloads, metadata counts and file checksums passed the offline audit.
No writer-overflow message occurred. The IMU average rate is near 100 Hz, but its
combined timestamps are irregular (maximum interval 94.864 ms); hardware sequence
loss is **unknown**, because standard ROS Image/Imu messages lack those counters.
This is not evidence that the earlier accelerometer source gaps are fixed.

PX4 remains connected. Its fresh timing window reached only 143 consecutive good
samples, below the unchanged requirement of 501. RTT median/p95/max were
3.795/6.095/26.172 ms; offset residual median/p95/max were 0.437/1.179/9.323 ms.
The configured fused-global GNSS topic and both `gpsstatus` raw/RTK topics had no
messages. Valid raw fixes do not replace RTK evidence or qualify geolocation.
The official driver also reports absent IMU extrinsics and publishes a placeholder
zero transform; physical mounting calibration remains required.

An additional camera-only, unbounded recording was stopped with SIGTERM. rosbag2
finalized successfully and the supervisor released its camera owner. Finalization
includes whole-file hashing, which can take substantial time for uncompressed bags;
wait for complete state before power removal. The service allows hashing to finish.

Private evidence is under ignored `runs/rosbag-commission-20261004-001/`: the
`final-001` recording and external audit, preceding 60-second trial, preserved
preflight failure, shutdown trial, and offline/replay/test evidence. The source
checkout changed during commissioning; each bag saves the requested/effective
profiles and package versions. Subsequent runs also archive the supervisor scripts.

Offline import produced all 707 lossless PNG images with CameraInfo calibration
and original timestamps. The corrected external audit is `final-001-audit-v2.json`;
the imported dataset carries that same coverage result. Standard rosbag2 replayed
the recorded IMU in an isolated network/domain. At 10× playback it reported queue
starvation delays; this confirms readability, not real-time replay performance.

The verified follow-up above addresses the recording-window issue. A field-duration
soak, loaded PX4 timing, missing GNSS/RTK plugins/topics and physical mounting/timing
qualification remain separate tasks. Static bench images are not a reconstruction
or survey-accuracy test.

## 100 Hz IMU trial: 2026-10-04

Reduced `imu_hz` from 200 to 100 in `configs/oakd-survey.json` and requested a
60-second capture. RGB/left/right remained at 2 FPS, with `queue_frames=12` and
the existing IMU batching settings. The recorder again aborted early with
`Writer backlog exceeded queue_frames`; this rate change alone is not a fix.

It saved one image from each camera and 42 samples from each IMU sensor over
approximately 0.41 seconds. The gyroscope delivered approximately 100 Hz with
zero sequence gaps. The accelerometer delivered approximately 99 Hz while its
sequence counter advanced at approximately 126 Hz, leaving 11 missing sequences.
This short observation is consistent with the BNO086's differing rate steps;
it does not establish sustained performance. Firmware readback was still 3.9.9
and USB was `SUPER`.

This historical trial left the SDK profile at 100 Hz. The subsequent ROS2 trial
above replaces its live writer; increasing the custom writer queue is no longer
the chosen acquisition fix. Original accelerometer sample continuity remains
unresolved. Validation thresholds are unchanged.

The failed capture, logs, validation report and measured rates are preserved
under ignored `runs/oak-imu-100hz-20261004-001/`. Validation correctly returns a
failure for the incomplete session and reports the accelerometer gaps.

## OAK IMU firmware upgrade: 2026-10-04

Used the pinned DepthAI 3.10.0 SDK's bundled firmware through Luxonis's explicit
IMU update API, with no running camera/IMU pipeline. The SDK reported completion
at 100%; a fresh device connection then read back 3.9.9. Camera calibration JSON
was identical before and after the update, and runtime USB remained `SUPER`.
The [commissioning procedure](hardware-commands.md#oak-bno-imu-firmware-commissioning)
and `deploy/update-oak-imu.py` reproduce the version checks, backup, flash and
readback. Startup checks do not flash firmware.

| Check | Result |
|---|---|
| Firmware | BNO086 3.2.13 → 3.9.9; matches the pinned SDK baseline after reconnect |
| Calibration | Before/after EEPROM calibration JSON matches |
| Default `imu=auto` startup | Host, dependency and storage checks pass; the requested 60-second camera probe aborts early with `Writer backlog exceeded queue_frames` |
| Preserved recorder reproduction | A requested five-second capture aborts with the same queue overflow; 29 accelerometer and 29 gyroscope samples were saved, with eight accelerometer sequence gaps |
| Isolated IMU, 10 seconds | At a requested 200 Hz, 2,000 samples from each sensor; observed rates approximately 200 Hz. Gyroscope: zero sequence gaps. Accelerometer: 518 sequence gaps |

The isolated IMU test used batch threshold/max reports 20/20 and no cameras or
dataset writer. Its accelerometer gaps therefore also occur without the recorder
queue overflow. Luxonis documents different
[BNO08X accelerometer and gyroscope rate steps](https://docs.luxonis.com/hardware/platform/sensors/imu/bno08x)
and an upstream [multi-report packet loss issue](https://github.com/luxonis/depthai-core/issues/585).
The observed pattern is consistent with that issue, but its cause has not been
proved for this SDK build. Preserve gap detection and resolve report delivery
before claiming lossless IMU capture. The application, capture rates, queue limits
and acceptance gates were unchanged during this commissioning.

Private evidence is in `runs/oak-imu-upgrade-20261004-001/`: `flash/` contains the
firmware journal and before/after readbacks/calibration; `startup-60s.json` retains
the failed integrated check; `capture-before-batching/` retains the failed short
capture; `imu-probe.json` and `imu-probe-samples.jsonl` retain the isolated test.
The firmware updater and hardware checks have 28 passing focused tests. Ruff,
Python compilation, shell syntax and diff whitespace checks pass. This firmware
repair does not qualify camera/IMU/PX4 survey capture.

## Five-second collection: 2026-10-04

Collected RGB/left/right plus PX4 telemetry after a 90-second timing warmup.
The explicit `oakd-mavros-camera-only.json` profile disables the incompatible
OAK IMU; PX4 IMU telemetry remains enabled. The preserved session is marked
**failed**, and all three `sync` commands correctly refuse association. No timing
threshold, PX4 setting, stream rate or firmware was changed to obtain acceptance.

| Check | Finding |
|---|---|
| Image integrity | 10 images per camera, approximately 2 Hz, zero sequence gaps; all images decode and their checksums match. All six saved journal hashes match. |
| Camera settings | RGB 4000×3000, mono 1280×800; 1000 µs and ISO 400; RGB focus 135 and white balance 5000 K. USB 3 `SUPER`. |
| Camera timestamp offsets | Left/right median 0.017 ms, max 0.018 ms. RGB/left median 15.952 ms, max 16.463 ms. These are reported timestamp differences, not measured physical exposure synchronization. |
| Host time | NTP synchronized before/after capture; reported last offset +0.526 ms, root distance 22.819 ms. This does not certify submillisecond UTC accuracy. |
| Clock continuity | No detected clock resets/jumps at the configured 5 ms gate. Maximum adjacent clock-step residual: SDK/monotonic 0.104 ms; ROS/monotonic 0.641 ms. |
| MAVROS startup | Connected; MAVLINK mode, convergence window 500, max RTT 10 ms. The isolated startup probe reached 501 consecutive acceptable samples. |
| Timing during camera capture | Across 954 saved samples including warmup: RTT median 3.374 ms, p95 7.474 ms, max 28.250 ms; offset residual median 0.563 ms, p95 1.698 ms, max 9.266 ms. Seventeen observations reset the qualification streak. Maximum streak 256/501; zero qualified samples. |
| Image interval | Its individual TIMESYNC samples stayed below RTT 10 ms and residual 2 ms, but the preceding required qualification window was absent. All 30 images therefore lack qualified timing evidence. |
| PX4 data rates during image interval | IMU and attitude approximately 50 Hz, pose 30 Hz, TIMESYNC 10 Hz, state/time reference 1 Hz. No GNSS messages on the configured global topic. |
| GNSS follow-up | Separate five-second read-only subscription received 25 `GPS_RAW_INT` packets and 25 raw `NavSatFix` messages: 3D fix, 15–18 satellites. No `GLOBAL_POSITION_INT` or global `NavSatFix`. Receiver-reported horizontal accuracy 3.504–3.623 m; vertical 5.098–5.132 m. No RTK-fixed evidence. |
| OAK IMU | Startup fails: BNO firmware 3.2.13 differs from pinned DepthAI's 3.9.9 baseline. No OAK IMU samples in this diagnostic. |
| Host resources | Jetson/L4T/dependency checks pass; active thermal zones 49.5–52.0 °C; 2.74 GiB available RAM before capture; approximately 778 GiB free on the checked filesystem; write/fsync/read passes. |
| Power/CUDA | Host confirms MAXN_SUPER. CUDA allocation/device-memory readback passes, compute capability 8.7. Container could not run `nvpmodel`; the host query supplies that evidence. |

The raw-fix follow-up demonstrates a working GNSS receiver. The configured
`/mavros/global_position/global` path still supplies no mapping GNSS data; the
cause of the absent fused/global stream has not been established. Timing spikes
also need investigation under camera load; this run does not identify their cause.

Local evidence is under ignored `runs/smoke-20261004-001/`: `capture/` (28.0 MB),
`hardware.json`, `validation-detailed.json`, `measurements.json`, `gnss.json`,
host-clock snapshots, logs, and a findings/next-steps report. Device identities and
coordinates stay in those private artifacts, which remain excluded from Git.

### Next steps

1. Firmware commissioning is now complete (see above). Resolve the recorder queue
   overflow and raw accelerometer sequence gaps, then repeat default `imu=auto`
   camera/IMU startup and sustained capture checks.
2. Diagnose TIMESYNC spikes with camera load active. Compare unloaded/loaded UART
   RTT, CPU scheduling and USB contention. Keep the current 10 ms RTT, 2 ms
   residual, 501-sample qualification and 5 ms association limits for acceptance.
   A longer warmup alone does not guarantee that these gates will pass.
3. Inspect PX4 estimator/global-position validity and `GLOBAL_POSITION_INT`
   streaming. Decide explicitly whether mapping needs the raw receiver fix or
   fused global position, then verify that topic's timestamp and reference frame.
   Do not silently substitute one for the other.
4. For RTK acceptance, provision the missing MAVROS extras and verify correction
   delivery, fixed status, receiver uncertainty and the commissioned base/rig.
5. Repeat full camera/OAK-IMU/PX4 capture, integrity checks and association for all
   cameras, followed by the loaded soak and physical timing/calibration tests.
   Resolve the earlier storage-service and reconstruction-engine gaps before
   deploying those parts of the PR stack.

Validation before publication: 117 host tests pass, with the ROS test skipped on
the host and passing separately in the platform image. The processing test module
is excluded locally because pinned pycolmap is unavailable; workstation CI retains
that coverage. Ruff and shell syntax checks pass. The GNSS script and ROS/container
command wrapper were exercised on this hardware. The sample's failed acceptance
is retained; passing software tests do not override it.

After collection, the temporary MAVROS connector and camera process exited. Host
descriptor inspection found no UART or USB-device owners, with no denied process
inspections. No temporary containers remained. The hardware can be shut down
normally; no shutdown command was issued by the checks.

## Bench review: 2026-10-03

Reviewed the complete stack through `0ff3ff1` (PRs #1–#3) on the physical Orin.
The platform reference is `../drone_autonomy_platform` at `c9f542a` and its
existing `drone_autonomy_platform:orin` image. The image's entrypoint differs from
the checkout and attempts a model export, so the mapping check wrapper explicitly
sources ROS/its overlay instead of invoking that entrypoint.

| Area | Measured result | Acceptance |
|---|---|---|
| Host | Orin Nano Super, aarch64, Ubuntu 22.04.5, Python 3.10.12, L4T 36.5.0, MAXN_SUPER | Host baseline passes |
| CUDA | CUDA 12.6 installed; driver identifies Orin, compute capability 8.7; allocation and device-memory readback pass | Basic CUDA access passes; reconstruction untested |
| Temperature | Active CPU/GPU/SoC zones around 50–53 °C; inactive CV zones return EAGAIN | Startup handles inactive zones and checks active hot/critical trips |
| OAK USB | Enumerates as `03e7:2485` at 480 Mbit/s before boot; DepthAI negotiates `SUPER` | Runtime USB 3 passes |
| OAK sensors | IMX378 RGB, two OV9282 mono cameras, BNO086 IMU; factory RGB focus 135 | Identity and EEPROM/pixel calibration access pass |
| Camera recording | RGB output 4000×3000, mono 1280×800; 1000 µs, ISO 400, RGB focus 135 / white balance 5000 K; 60-second probe saved 120 frames per stream at 2 FPS without gaps or setting changes | Camera-only startup passes with an explicit diagnostic `imu=off` profile |
| OAK IMU | Installed 3.2.13; DepthAI 3.10.0 bundles 3.9.9 and rejects the installed firmware; raw IMU produces no samples | **Default `imu=auto` capture fails**; firmware commissioning required |
| Storage | Checkout is on NVMe; approximately 778 GiB free at inspection; `/mnt/nvme/mapping` does not exist | Configure the service's real storage root/mount before deployment |
| ROS runtime | Host has no `/opt/ros`; existing platform image has Humble, MAVROS 2.14.0, GeographicLib EGM96 data | Real ROS parameter-service/DDS/CDR and startup-subscriber tests pass in that image |
| MAVROS graph | `/mavros/state`, `/mavros/timesync_status`, `/mavros/time`; MAVLINK mode, convergence window 500, max RTT 10 ms | Supplied time-node defaults corrected to `/mavros/time` |
| PX4 serial | UAV-DEV/CP210x `/dev/ttyUSB0` opens in the platform container with dialout access; host user lacks dialout | No heartbeat; passive 5-second listens at 921600/57600/115200/460800 and a further 20-second retry at 115200 received **zero bytes** |
| RTK | Platform image lacks `ros-humble-mavros-extras`; no GPSRAW/GPSRTK/RTCM topics appeared | Install extras in the platform image; receiver/corrections cannot be validated until PX4 telemetry works |
| Building processing | COLMAP CLI absent; PyPI has no pinned pycolmap 3.12.6 wheel for this ARM64/Python combination | Processing check fails; use a qualified source build or a processing workstation |
| Terrain processing | Docker available; configured `opendronemap/odm:3.6.2` image absent | Processing check fails; image must match the processing host's native architecture |

The original failed capture is retained locally under `runs/orin-bench-first`.
The successful camera report is `runs/orin-camera-only-60s.json`; the platform's
expected failure report is `runs/orin-platform-startup-v2.json`. Hardware reports
under `runs/` are ignored by Git and may contain device IDs.
The BNO firmware was **not flashed**, and PX4 parameters were **not changed**.
No complete camera/IMU/PX4/RTK survey has passed acceptance on this setup.

Validation of this change: 95 tests pass on the host, with the ROS test skipped
there; that test also passes separately inside the platform image, including the
new live startup subscriber. Ruff and shell syntax checks pass. The
`tests/test_processing.py` module could not run because pinned pycolmap is
unavailable on this interpreter. Its workstation CI coverage remains required.

The observed pre-boot USB speed agrees with the
[Luxonis USB deployment guide](https://docs.luxonis.com/hardware/platform/deploy/usb-deployment-guide).
The host release is covered by [NVIDIA's R36.5 baseline](https://developer.nvidia.com/embedded/jetson-linux-r365).
Firmware version queries use the [DepthAI device API](https://docs.luxonis.com/software-v3/depthai/api/cpp).

## Run startup checks

Use the same account, interpreter, device access, ROS overlay and DDS domain as
the recorder. Unlike `doctor`, this command opens the OAK and records a disposable
short session, then checks decoding, checksums, settings and continuity. Stop any
other camera owner first. It never opens the UART, starts MAVROS, forwards NTRIP,
flashes firmware, changes power settings or changes flight-controller parameters.

```bash
mkdir -p runs
bash deploy/check-hardware.sh --require-jetson \
  --config configs/oakd-survey.json --output-root runs \
  --report runs/startup.json
```

Exit `0` means all requested checks passed; `2` means setup is not ready. Reports
contain `pass`, `fail`, `warn`, and `skip` entries plus remedies and provenance.
An omitted telemetry configuration explicitly skips PX4 checks. Reports never
overwrite existing files. The output directory must already exist. Temporary
images are removed after the probe, including on a native-probe timeout.

The camera interval defaults to five seconds after the configured warmup (at
least three frame periods for slower profiles). `--camera-seconds 60` lengthens
it. The entire camera child has a deadline of warmup + interval + 45 seconds.
`--min-memory-gib` defaults to 1 GiB available RAM; capture disk reserve comes from
the profile. Memory/disk thresholds are deployment settings, not throughput guarantees.

For a deployment disk, add `--require-mount /mnt/nvme` and use an output directory
on that mount. This rejects missing mounts and symlinks onto another filesystem.
An NVMe root filesystem can be selected explicitly with `--require-mount /`;
the launcher never substitutes it for a missing configured mount.

### Reuse the existing platform's ROS environment

The wrapper uses the locally built platform image, the checkout's `.venv`, DDS
domain 1 by default, host networking/IPC, and USB access. It mounts only this
checkout as writable application storage. Store output/reports under `runs/` or
another directory in the checkout. `/tmp` inside the container is disposable.
`WR_PLATFORM_IMAGE` selects a rebuilt platform image; `ROS_DOMAIN_ID` overrides
the platform domain. Provision the pinned `.venv` first as described in
[Jetson setup](jetson-setup.md).

```bash
bash deploy/check-platform-hardware.sh --require-jetson \
  --config configs/oakd-mavros-survey.json --output-root runs \
  --telemetry-config configs/mavros-survey.json \
  --report runs/platform-startup.json
```

Start the platform's existing MAVROS connector separately. Ensure a single UART
owner. The mapping startup probe reads time-plugin parameters and subscribes to
telemetry; it requires current connected state, live required topics and a fresh
qualified TIMESYNC window. `--telemetry-seconds` defaults to 75 seconds; slow or
noisy links may need longer. Capture still collects its own independent timing
window after startup. A startup pass never substitutes for `wr-map sync`.

For corrected GNSS, select `configs/mavros-rtk-survey.json`. This also requires
RTK-fixed GPSRAW with usable `h_acc`/`v_acc`, CRC-valid observation corrections and
recent receipt of required topics. Add `--gnss-profile PRIVATE.json` to require a
completed commissioning profile, matching RTCM station/base ARP coordinates,
fresh observation corrections and the configured receiver correction-age policy.
The checked-in GNSS template is deliberately incomplete and will fail this check.
No caster credentials are read or transmitted by startup checks.

The platform image needs `ros-humble-mavros-extras` in addition to its current
MAVROS packages for `gps_status` and `gps_rtk`. Rebuild that environment and verify
the actual plugin/topic names. Merely installing message definitions does not
create GPS/RTK publishers. Configure the existing correction source separately.

For the silent PX4 link, check TELEM2 TX/RX/GND wiring and the platform's
`docs/architecture/px4_setup.md`: `MAV_1_CONFIG=TELEM 2`, `MAV_1_MODE=Onboard`,
`SER_TEL2_BAUD=921600`. Confirm these against the actual firmware in QGroundControl.
Host serial access requires dialout membership; the recorder itself only needs
ROS access. A serial adapter's presence does not establish an FCU connection.

### TELEM2 follow-up audit

A further check on 2026-10-03 used the adapter's stable `/dev/serial/by-id` path.
USB enumeration identified a UAV-DEV USB2Serial Rev. 1.2, VID:PID `10c4:ea60`,
with the `cp210x` driver. This differs from the FT231X in the linked current
[Holybro converter specification](https://holybro.com/collections/gps-accessories/products/uart-to-usb-converter).
The individual adapter identity is kept in ignored local reports.

No MAVROS/router/platform container was running. ModemManager was active but
reported no modems. Inspection of host process descriptors by device number
found no UART owner before the active comparison test. The new wrapper also
refused a second invocation while the diagnostic held the port.

| Probe | Observation |
|---|---|
| 921600, receive-only, 10 seconds | Zero bytes |
| 921600, RTS/CTS, requested 60 seconds | CTS deasserted at open; 22 accepted heartbeat writes, then write timeout; zero received bytes |
| 921600, flow control off, 30 seconds | 30 accepted heartbeat writes; no write error; zero received bytes |

Accepted writes may only represent queued data. PX4 receipt and bidirectional
communication were not established. No PX4 heartbeat, IMU, attitude, GNSS or
TIMESYNC reply was received. Stream configuration and clock offset/RTT remain
unknown. `MAV_1_CONFIG`, `MAV_1_MODE`, `SER_TEL2_BAUD`, `MAV_1_RATE`,
`MAV_1_RADIO_CTL` and `MAV_1_FLOW_CTRL` could not be read back.

The requested integration is MAV_1/TELEM2. MAV_0/TELEM1 is the SiK radio and
MAV_2 is Ethernet. No PX4 parameters, stream rates, routing, firmware or deployed
integration settings were changed. Confirm the controller-side configuration
and cable pin mapping, then repeat the
[active audit](hardware-commands.md#telem2-parameter-stream-and-timing-audit).

Follow-up on 2026-10-04: the same USB identity remained present, and the ownership
guard found no owner. A fresh 10-second 921600-baud audit with host flow control
off accepted ten heartbeat writes and received zero bytes, without a write error.
At that point the controller-side configuration and physical signal path were
still unverified.

#### Successful retest after controller configuration/reboot

Later on 2026-10-04, a 60.011-second audit at 921600 baud with host flow control
off received 1,320,241 bytes from PX4 system/component 1/1. It collected all
required diagnostic evidence. The actual parameter readback was:

| Parameter | Observed value |
|---|---|
| `MAV_1_CONFIG` | 102 (TELEM2) |
| `MAV_1_MODE` | 2 (Onboard) |
| `SER_TEL2_BAUD` | 921600 |
| `MAV_1_RATE` | 0 |
| `MAV_1_RADIO_CTL` | 0 |
| `MAV_1_FLOW_CTRL` | 0 (Force off) |

Observed receiver rates were HEARTBEAT 1 Hz, ATTITUDE 100 Hz,
ATTITUDE_QUATERNION 50 Hz, HIGHRES_IMU 50 Hz, GPS_RAW_INT 5 Hz,
LOCAL_POSITION_NED 30 Hz and ODOMETRY 30 Hz. Sixty host heartbeat writes and
sixty PX4 heartbeats were recorded. The six parameter replies and 592 matching
TIMESYNC replies independently establish bidirectional protocol communication;
heartbeats themselves have no acknowledgement.

TIMESYNC round-trip time was median 2.805 ms, p95 5.204 ms and maximum 8.385 ms.
The median local-monotonic-minus-PX4 offset was +19525.232287 seconds, with
0.856 ms standard deviation. These clocks have different origins; that offset
is not a UTC error. The statistics are unfiltered serial-probe measurements,
not a MAVROS convergence or survey timing qualification. GNSS message reception
does not establish a valid fix or RTK accuracy.

This successful sample supersedes the earlier silent-link finding. It does not
resolve the separate OAK IMU firmware and reconstruction-engine blockers. The
full report, including the stable adapter identity, is retained under ignored
`runs/telem2-after-px4-config-20261004-001.json`. The diagnostic changed no PX4
parameters or stream rates.

### Optional processing host checks

```bash
bash deploy/check-hardware.sh --mode building --output-root runs
bash deploy/check-hardware.sh --mode terrain --output-root runs
```

Building checks exercise the CUDA driver and device memory, require COLMAP 3.12.x
advertising CUDA support and import pinned pycolmap. Terrain checks inspect the
local ODM image, require Linux/native architecture, and run its version command.
Neither command downloads an engine or runs a reconstruction. The same native
architecture guard also runs before normal ODM execution.

## Service integration

`deploy/record-session.sh` runs the checks before creating the survey session. A
failure prevents recording and emits the JSON report to the service log. If the
output root exists, it also preserves a uniquely named `*-hardware.json` beside
the future session. Firmware, missing hardware and absent telemetry fail closed.

Copy/customize `deploy/wallering-mapping.env.example` as `/etc/wallering-mapping.env`
when installing the service. Set the real output root/mount, optional camera ID,
ROS setup/overlay, DDS domain, telemetry config and private GNSS profile there.
If changing the mount, also update `RequiresMountsFor` in the systemd unit. A host
service needs a host ROS installation; alternatively deploy in the established
platform runtime. The provided Docker wrapper is an on-demand checker.

## Remaining physical acceptance

After resolving IMU firmware, loaded PX4 timing, GNSS/RTK delivery and storage, repeat the
default startup checks and a complete synchronized capture. Then perform the
20-minute loaded soak, disconnect/service-stop checks, measured exposure/rolling
shutter timing, mounting/lever-arm calibration, receiver timestamp/uncertainty
validation and independent survey checkpoints described in
[Jetson commissioning](jetson-setup.md), [MAVROS timing](mavlink-integration.md)
and [postprocessing](postprocessing.md). These are measured acceptance tasks;
startup cannot establish them from device presence or an RTK-fixed flag.
