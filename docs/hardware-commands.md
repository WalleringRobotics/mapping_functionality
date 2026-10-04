# Hardware command guide

These are the reproducible commands used for the Orin/OAK/PX4 review. Run them
from the repository root and choose a new report/session name for each run.
[Measured results](hardware-acceptance.md) describe what passed and what is blocked.
The [repository skill](../skills/jetson-mapping-checks/SKILL.md) guides future agent
sessions through these workflows.

## Python environment

```bash
sudo apt-get install python3-venv libusb-1.0-0 usbutils
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install --upgrade pip 'setuptools>=68,<80' wheel
.venv/bin/python -m pip install -e '.[oak,terrain,diagnostics,dev]'
mkdir -p runs
```

The `diagnostics` extra supplies pinned pyserial/pymavlink for passive serial
checks. ROS comes from the host deployment or existing platform image, not pip.
The processing extra, `.[processing]`, requires pycolmap 3.12.6; this bench had no
compatible ARM64 wheel. Install it on a supported processing host or qualify a
source build. The repository's Python CLI also works as
`.venv/bin/python -m wallering_mapping.cli`.

## Host and device inventory

```bash
uname -a
cat /etc/os-release /etc/nv_tegra_release
tr -d '\000' </proc/device-tree/model
nvpmodel -q
timedatectl status
timedatectl timesync-status
free -h
df -h .
findmnt -T .
lsusb
lsusb -t
id
ls -l /dev/serial/by-id/ /dev/ttyUSB0
docker image ls
docker ps
.venv/bin/wr-map inspect
```

Run these in the actual host/device-access context. A sandbox can hide USB/UART
devices and supplementary groups. `inspect` opens the OAK and now includes IMU
firmware versions. It needs exclusive camera ownership. Pre-boot USB speed is
not the negotiated speed after DepthAI opens the camera.

## OAK BNO IMU firmware commissioning

Firmware commissioning is an explicit maintenance step. Stop the camera owner,
keep the OAK powered and connected throughout the update, and use the pinned
`.venv` SDK. The separate updater calls Luxonis's
[`startIMUFirmwareUpdate` / `getIMUFirmwareUpdateStatus` API](https://docs.luxonis.com/software-v3/depthai/api/cpp)
without starting an IMU pipeline. It targets only BNO085/BNO086 IMU firmware;
startup checks remain read-only with respect to firmware.

Read the device ID, installed version and SDK's embedded version with `inspect`.
For the commissioned BNO086, the supported upgrade was 3.2.13 to the DepthAI
3.10.0 bundle's 3.9.9. Substitute the inspected ID below; keep it in private
reports. Omit `--apply` for inspection only, using a different output directory.

```bash
.venv/bin/wr-map inspect
.venv/bin/python deploy/update-oak-imu.py --device-id DEVICE_ID \
  --from-version 3.2.13 --to-version 3.9.9 \
  --output runs/oak-imu-upgrade-001 --apply

bash deploy/check-hardware.sh --require-jetson --device-id DEVICE_ID \
  --config configs/oakd-survey.json --camera-seconds 60 \
  --output-root runs --report runs/oak-imu-startup-001.json
```

The updater checks the reviewed device and versions, saves calibration before
flashing, journals progress, and requires both SDK completion at 100% and firmware
readback after reconnect. It also compares calibration before/after. A camera
already at the target is not reflashed. Evidence directories are never replaced.
Do not interrupt an active flash or run it under a process timeout. If the SDK
reports failure, retain its evidence and diagnose before attempting another write.
The follow-up capture must report both accelerometer and gyroscope data with
`imu=auto`; firmware readback alone does not establish working sensor streams.

## Startup and processing checks

```bash
bash deploy/check-hardware.sh --require-jetson \
  --config configs/oakd-survey.json --output-root runs --report runs/startup-001.json

# Isolate RGB/stereo explicitly; this does not qualify IMU or PX4 capture.
bash deploy/check-hardware.sh --require-jetson \
  --config configs/oakd-camera-only.json --camera-seconds 60 \
  --output-root runs --report runs/camera-only-001.json

bash deploy/check-platform-hardware.sh --require-jetson \
  --config configs/oakd-mavros-survey.json --output-root runs \
  --telemetry-config configs/mavros-survey.json --report runs/platform-001.json

# Existing corrected-GNSS telemetry and a completed private commissioning profile.
bash deploy/check-platform-hardware.sh --require-jetson \
  --config configs/oakd-mavros-survey.json --output-root runs \
  --telemetry-config configs/mavros-rtk-survey.json \
  --gnss-profile runs/private-gnss-profile.json --report runs/rtk-001.json

bash deploy/check-hardware.sh --mode building --output-root runs
bash deploy/check-hardware.sh --mode terrain --output-root runs
```

These probes return 0 on requested readiness, 2 on failure. They store temporary
images on the chosen filesystem and remove them after validation. The platform
wrapper uses `drone_autonomy_platform:orin`, the checkout's venv, and DDS domain 1;
override `WR_PLATFORM_IMAGE` or `ROS_DOMAIN_ID` for a different deployment. It
subscribes to an existing MAVROS connector. Reports must be inside the checkout to
survive that temporary container. See `hardware-check --help` for memory, duration,
camera-ID and required-mount options. For a dedicated disk, use
`--output-root /mnt/nvme/mapping --require-mount /mnt/nvme` on the host.

For just the CUDA driver/device-memory probe used during the review:

```bash
.venv/bin/python -m wallering_mapping.hardware_probe cuda '{}'
```

This internal probe prints `WR_HARDWARE_RESULT=...`; inspect its `ok` field. Use
`hardware-check --mode building` when a readiness exit code is needed.

## Passive serial checks

Stop the current UART owner first. These commands open the serial device in
exclusive mode and transmit no bytes. Opening/configuring the port changes host
termios and may toggle the adapter's modem-control lines. They do not configure
the flight controller. Use the platform wrapper when the host user lacks dialout.

```bash
# Native host, with serial permissions:
bash deploy/check-serial.sh --device /dev/ttyUSB0 --baud 115200 --seconds 20 \
  --report runs/serial-115200-001.json

# Same test with the existing platform image's device access:
bash deploy/check-platform-serial.sh --baud 115200 --seconds 20 \
  --report runs/platform-serial-115200-001.json

# Sequential scan: five seconds per baud, twenty seconds total.
bash deploy/check-platform-serial.sh --baud 921600 57600 115200 460800 --seconds 5 \
  --report runs/serial-scan-001.json
```

`WR_SERIAL_DEVICE` selects the host device for the container wrapper, including a
stable `/dev/serial/by-id/...` path. It resolves the device, derives its group ID,
and maps it as `/dev/wr-serial`. The wrapper takes `--baud`, `--seconds`, and
`--report`; use the environment variable to choose a different device.

The report distinguishes `no_bytes`, `bytes_without_valid_mavlink`,
`mavlink_without_heartbeat`, and `heartbeat_received`. Exit 0 requires a heartbeat
at one tested rate; exit 2 means no heartbeat or a diagnostic error. `BAD_DATA`
counts are parser events, not a dropped-packet measurement. A silent link at
several rates requires checking wiring/port assignment; a detected heartbeat is
only evidence of MAVLink reception.

## TELEM2 parameter, stream and timing audit

This connection uses MAVLink instance 1 over TELEM2 and a USB-to-UART adapter.
The expected PX4 values supplied for this setup are:

| Parameter | Expected value |
|---|---|
| `MAV_1_CONFIG` | TELEM2 |
| `MAV_1_MODE` | Onboard |
| `SER_TEL2_BAUD` | 921600; verify readback |
| `MAV_1_RATE` | 0 |
| `MAV_1_RADIO_CTL` | 0 |
| `MAV_1_FLOW_CTRL` | Verify wiring/readback; 0 for the initial flow-control-off test |

MAV_0/TELEM1 serves the SiK radio; MAV_2 is Ethernet and does not configure this
UART. Expected settings are not evidence of the controller's actual configuration.
Six wires alone do not establish the adapter revision or correct RTS/CTS wiring.

Identify the intended adapter and check services before opening it:

```bash
ls -l /dev/serial/by-id/
lsusb
systemctl list-units --all --type=service --no-pager
docker ps
# Replace the placeholder with the selected adapter's exact stable path.
export WR_SERIAL_DEVICE=/dev/serial/by-id/REPLACE_WITH_ADAPTER_ID

bash deploy/check-platform-px4-link.sh --baud 921600 --flow-control off \
  --seconds 60 --report runs/telem2-link-001.json
```

Use `--flow-control rtscts` only for an explicit hardware-flow-control test or
when the wiring and PX4 configuration require it. The wrapper inspects all host
process descriptors by device number, including alternate container paths, and
refuses to proceed if the port is occupied or inspection is incomplete. The
inspection container has read/inspection capabilities; the serial probe runs as
the invoking user with those capabilities removed. It takes an exclusive serial
lock and `TIOCEXCL`, but these do not evict an owner that opened the port earlier.
Keep one managed UART owner and route other consumers through that owner. Do not
run this probe alongside MAVROS, QGroundControl or a MAVLink router on the UART.

The probe sends onboard heartbeats, named parameter read requests and TIMESYNC;
it does not set parameters, request new stream rates, arm, reboot or change modes.
It reports PX4 heartbeats, IMU/attitude/GNSS presence, per-message receiver rates,
and unfiltered TIMESYNC offset/round-trip statistics. Host writes can remain
queued behind CTS. Heartbeats have no acknowledgement; a matching parameter or
TIMESYNC reply provides separate evidence of bidirectional communication.
Exit 0 means the requested evidence was collected, not flight readiness or a
match to the expected configuration. Compare parameter readback before changing
the integration; use the MAVROS startup probe to assess its timing convergence.

Device identities and complete reports belong under ignored `runs/`. Do not add
individual adapter serial numbers or private access details to these examples.

### If TELEM2 remains silent

1. Connect the flight controller's own USB port to a computer with working
   QGroundControl. This provides an independent path to inspect PX4 startup and
   the actual UART configuration. In QGroundControl's MAVLink Console, collect:

   ```text
   ver all
   mavlink status
   param show MAV_1_CONFIG
   param show MAV_1_MODE
   param show SER_TEL2_BAUD
   param show MAV_1_RATE
   param show MAV_1_RADIO_CTL
   param show MAV_1_FLOW_CTRL
   param show UXRCE_DDS_CFG
   ```

   Save output privately before changing settings. Match the reported UART to
   the board's TELEM2 mapping; runtime instance numbering alone is insufficient.
   No running UART instance points toward startup/configuration. Nonzero PX4 TX
   with zero Orin RX points toward the serial signal path or host settings.
   The [PX4 module reference](https://docs.px4.io/main/en/modules/modules_communication#mavlink)
   documents MAVLink status inspection.

2. Compare readback with the expected table above. For the initial test use
   flow control off on both ends, save configuration changes and reboot PX4.
   Confirm that another driver, including uXRCE-DDS, has not claimed TELEM2.
   `UXRCE_DDS_CFG` depends on firmware version; record a missing parameter instead
   of guessing. PX4 documents [serial port conflicts and reboot requirements](https://docs.px4.io/v1.16/en/peripherals/serial_configuration)
   and [uXRCE-DDS port assignment](https://docs.px4.io/v1.16/en/middleware/uxrce_dds).

3. Confirm the physical USB adapter identity: unplug only its USB connection,
   compare `lsusb` and `ls -l /dev/serial/by-id/`, reconnect, and select the device
   that disappeared and returned. Do not apply an FT231X pinout to an adapter
   identified as CP210x without verifying its actual model.

4. With power disconnected, compare both connector pinouts: PX4 TX to adapter
   RX, PX4 RX to adapter TX, and common ground. Check the power wire too: the
   [PX4 companion wiring diagram](https://docs.px4.io/main/en/companion_computer/pixhawk_companion)
   leaves the TELEM2 +5 V connection to the adapter unused. Connector shape and
   wire count do not prove the mapping.

5. If configuration and wiring are confirmed but reception remains absent,
   isolate the adapter from PX4 and perform a TX-to-RX loopback using the verified
   adapter pinout and flow control off. An exact byte echo checks the Orin/USB/
   adapter path; it does not validate TELEM2. A known-good cable or adapter is an
   alternative isolation test. Repeat the MAVLink audit after each physical change.

The user identified the controller as a **Pixhawk 6X**. PX4's
[6X serial mapping](https://docs.px4.io/v1.16/en/flight_controller/pixhawk6x#serial-port-mapping)
maps TELEM2 to UART5, `/dev/ttyS4` on PX4. Inspect that device in `mavlink status`;
this path belongs to the controller, not the Orin.

For the standard Holybro baseboard, the
[TELEM pinout](https://docs.holybro.com/autopilot/pixhawk-baseboards/pixhawk-baseboard-v2-ports)
is below. Use the manufacturer's connector-orientation diagram to identify pin 1.

| TELEM2 pin | Signal on Pixhawk | Adapter connection for flow-control-off test |
|---|---|---|
| 1 | +5 V | Leave unconnected; use the controller's normal power input |
| 2 | TX, 3.3 V logic | Adapter RX |
| 3 | RX, 3.3 V logic | Adapter TX |
| 4 | CTS | Unused when PX4 flow control is forced off |
| 5 | RTS | Unused when adapter flow control is off |
| 6 | Ground | Adapter ground |

Verify the adapter's pin labels independently. Its six-pin connector need not
have the same ordering as the flight controller. Make wiring changes with power
disconnected, then restore normal power and repeat the audit.

## QGroundControl on the Orin

Checked on 2026-10-03: this host is aarch64 Ubuntu 22.04.5. QGroundControl's
[stable installation guide](https://docs.qgroundcontrol.com/Stable_V5.1/en/qgc-user-guide/getting_started/download_and_install.html)
offers a Linux aarch64 AppImage, but states that its current AppImages require
Ubuntu 24.04 or 26.04. Ubuntu 22.04 requires a source build. An x86_64 AppImage
does not match the Orin's architecture. QGroundControl has not been installed or
GUI-tested during this review.

For installation on this host, follow the
[official source-build guide](https://docs.qgroundcontrol.com/Stable_V5.1/en/qgc-dev-guide/getting_started/)
with ARM64 dependencies compatible with Ubuntu 22.04. Check the chosen release's
Qt requirements before building; this repository does not yet provide a tested
QGroundControl build recipe.

To diagnose the silent TELEM2 link, connect a data-capable USB cable directly
between the Orin and the flight controller's USB port. Inspect USB enumeration
before opening QGroundControl. These commands only inspect the host:

```bash
uname -m
cat /etc/os-release
id -nG
lsusb
ls -l /dev/serial/by-id/
journalctl -k -b --since '-5 min' --no-pager | rg 'usb|ttyACM|ttyUSB'
```

For native serial access, the official QGroundControl guide uses:

```bash
sudo usermod -aG dialout "$(id -un)"
# Log out and log back in, then check:
id -nG
```

The group change above has not been applied by this review. Stop any process
using the selected serial port before opening it in QGroundControl. Once
connected, inspect vehicle messages and the current TELEM2 port assignment,
MAVLink instance and baud parameters. The sibling platform's intended settings
are in `docs/architecture/px4_setup.md`; first establish the controller's actual
configuration and save its parameters before making changes.

Zero UART bytes at several baud rates do not establish that the controller
entered another state when the already-powered Orin was connected. If a USB-UART
adapter is wired to TELEM2, inspect the wiring with power disconnected: the
[PX4 companion wiring diagram](https://docs.px4.io/main/en/companion_computer/pixhawk_companion)
leaves TELEM2's +5 V pin unconnected to the adapter and connects crossed TX/RX
with a common ground. Check the actual board's pinout. A direct USB connection
helps separate controller startup/configuration from the TELEM2 adapter path.

## MAVROS connection and graph

Use the established platform connector where possible. For a bounded bench test
when no other UART owner is running, the following runs a 115200-baud
MAVROS launch. It uses normal MAVROS link traffic; unlike the passive probe, it
transmits protocol messages. Timeout exit 124 is expected after 30 seconds.

```bash
serial_group="$(stat -c %g /dev/ttyUSB0)"
docker run --rm --network host --ipc host --entrypoint /bin/bash \
  --device /dev/ttyUSB0 --group-add "$serial_group" --env ROS_DOMAIN_ID=1 \
  drone_autonomy_platform:orin -c '
    source /opt/ros/humble/setup.bash
    exec timeout --signal=INT --kill-after=5s 30s \
      ros2 launch mavros px4.launch fcu_url:=/dev/ttyUSB0:115200
  '
```

Inside the same ROS environment/domain as the running connector:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1
ros2 node list --no-daemon
ros2 topic list -t --no-daemon
ros2 param get /mavros/time timesync_mode
ros2 param get /mavros/time convergence_window
ros2 param get /mavros/time max_rtt_sample
timeout 10s ros2 topic echo /mavros/state --once
timeout 10s ros2 topic hz /mavros/timesync_status
timeout 10s ros2 topic hz /mavros/local_position/pose
dpkg-query -W ros-humble-mavros ros-humble-mavros-extras
ls /usr/share/GeographicLib/geoids/egm96-5.pgm
```

For graph inspection without an aircraft, replace the `fcu_url` above with
`udp://:14580@127.0.0.1:14581` and omit `--device`/`--group-add`. A graph alone
does not prove receipt of FCU messages. The existing sibling repository documents
the TELEM2 port assignment in `docs/architecture/px4_setup.md`.

## Capture, validation and service commands

For a five-second camera/PX4 diagnostic using the existing MAVROS connector:

```bash
mkdir runs/smoke-001
bash deploy/run-platform-command.sh .venv/bin/wr-map capture \
  --config configs/oakd-mavros-camera-only.json \
  --telemetry-config configs/mavros-survey.json \
  --output runs/smoke-001/capture --duration 5
.venv/bin/wr-map validate runs/smoke-001/capture \
  --report runs/smoke-001/validation.json
for stream in rgb left right; do
  .venv/bin/wr-map sync runs/smoke-001/capture \
    --stream "$stream" --output "runs/smoke-001/sync-$stream"
done
```

This diagnostic profile explicitly disables the **OAK IMU** and retains PX4 IMU
telemetry. It allows 90 seconds of warmup before the five-second image interval;
telemetry and clock observations are saved throughout warmup. The normal timing
thresholds remain unchanged. A longer warmup does not guarantee qualification:
check the capture, validation and all three association exit codes/reports.
Use `oakd-mavros-survey.json` after OAK IMU commissioning for full sensor capture.
Keep reports outside the immutable `capture` directory. `run-platform-command.sh`
reuses ROS Humble and USB access; it does not start a UART owner.

Check NTP on the host with `timedatectl` before capture. MAVROS offset/RTT and
camera-to-host clock bridges assess relative timing; they do not establish
absolute UTC accuracy or measured exposure/rolling-shutter latency. Inspect GNSS
fix validity separately from message presence and pose association.

Validation now reports received topic rates (including warmup), RTT and offset
residual distributions, the longest good timing streak, and missing configured
topics. An intact image set can still belong to a failed timing session; preserve
that session and its failure status.

If the configured GNSS topic is empty, collect receiver evidence through the
existing MAVROS router without opening the serial adapter:

```bash
bash deploy/run-platform-command.sh .venv/bin/python deploy/check-mavros-gnss.py \
  --seconds 5 --output runs/smoke-001/gnss.json
```

This listens to `/uas1/mavlink_source`, `/mavros/global_position/raw/fix` and
`/mavros/global_position/global`. Override `--source-topic` if the existing router
uses a different name. The report includes raw GPS packets and coordinates, so
keep it private. Exit 2 means no valid global fix was observed or decoding failed;
inspect raw receiver fixes separately. A valid raw fix does not establish a fused
global position or RTK accuracy. This command publishes nothing.

```bash
.venv/bin/wr-map capture --config configs/oakd-camera-only.json \
  --output runs/camera-bench-001 --duration 60
.venv/bin/wr-map status runs/camera-bench-001
.venv/bin/wr-map validate runs/camera-bench-001 --report runs/camera-validation-001.json

# After IMU and MAVROS acceptance, in a ROS-enabled environment:
.venv/bin/wr-map capture --config configs/oakd-mavros-survey.json \
  --telemetry-config configs/mavros-survey.json --output runs/survey-001 --duration 60
.venv/bin/wr-map sync runs/survey-001 --output runs/alignment-001
```

The `record` alias also works. The saved interval follows the configured warmup.
For service deployment, customize `deploy/wallering-mapping.env.example` and
follow [Jetson setup](jetson-setup.md#optional-service). The launcher command is
`bash deploy/record-session.sh`; it checks hardware before creating a session.
Installed-service commands are:

```bash
sudo systemctl start wr-mapping
journalctl -u wr-mapping -f
sudo systemctl stop wr-mapping
```

## Tests and command discovery

```bash
.venv/bin/python -m ruff check src tests
bash -n deploy/check-hardware.sh deploy/check-platform-hardware.sh \
  deploy/check-serial.sh deploy/check-platform-serial.sh \
  deploy/check-platform-px4-link.sh deploy/record-session.sh
.venv/bin/python -m pytest -q
```

When pycolmap is unavailable, use `pytest -q --ignore=tests/test_processing.py`
and state that exclusion in the results. To reproduce the ROS test using the
existing platform image without attached devices:

```bash
mapping_repo="$(pwd)"
docker run --rm --network none --entrypoint /bin/bash \
  --mount "type=bind,src=$mapping_repo,dst=$mapping_repo" --workdir "$mapping_repo" \
  drone_autonomy_platform:orin -c '
    source /opt/ros/humble/setup.bash
    export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
    .venv/bin/python -m pytest -q tests/test_mavros_ros2.py
  '
```

Use `.venv/bin/wr-map --help` for all commands and `COMMAND --help` for arguments.
The complete CLI covers:

| Task | Commands |
|---|---|
| Readiness and diagnostics | `inspect`, `doctor`, `hardware-check`, `serial-check` |
| Capture and integrity | `capture` / `record`, `status`, `validate`, `simulate` |
| Timing and corrected GNSS | `sync`, `ntrip`, `image-accuracy` |
| Processing | `export`, `process`, `reconstruct`, `dense` |
| Independent accuracy | `accuracy`, `map-accuracy` |

The [MAVROS guide](mavlink-integration.md) and [processing guide](postprocessing.md)
describe the telemetry, correction-forwarding and reconstruction workflows. Use
real private survey evidence for RTK/accuracy commands; checked-in templates are
not commissioned profiles.
