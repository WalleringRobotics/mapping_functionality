# Jetson setup and commissioning

The commissioned host is Orin Nano Super, L4T 36.5.0 / Ubuntu 22.04.5 / Python
3.10.12. OAK uses USB 3; PX4 TELEM2 uses a USB-UART adapter at 921600 baud.
Keep the working JetPack installation. See [hardware acceptance](hardware-acceptance.md)
for measured results and outstanding timing/GNSS limits.

## Runtime and storage

Follow the [ROS recording guide](rosbag-recording.md) for native Humble packages
or reuse of the existing `drone_autonomy_platform:orin` image. The live recorder
is rosbag2/MCAP; the host Python environment is for CLI dispatch and offline import:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e '.[bags]'
```

Firmware/SDK diagnostics additionally use `.[oak,diagnostics]`; they must run with
no active ROS camera owner. The bench BNO086 was upgraded from 3.2.13 to 3.9.9 on
2026-10-04, with firmware and calibration verified after reconnect. Firmware
updates remain an explicit commissioning operation, never an automatic startup
step; see [the procedure](hardware-commands.md#oak-bno-imu-firmware-commissioning).

Install USB permissions for the capture user:

```bash
sudo install -m 0644 deploy/80-luxonis.rules /etc/udev/rules.d/80-luxonis.rules
sudo usermod -aG plugdev,dialout "$USER"
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Log out/in and reconnect. OAK re-enumerates during boot, so the rule covers both
bootloader and runtime. Verify `SUPER` in `oak.log`, not just the idle USB device.
Use a secured direct USB 3 cable. Mount NVMe explicitly and grant the capture user
write access to the output directory. Budget approximately 4.4 GiB/minute for the
uncompressed 2 fps profile plus space for offline PNG imports and exports.

```bash
export WR_MAPPING_ROOT=/mnt/nvme/mapping
export WR_MAPPING_MOUNT=/mnt/nvme
wr-map capture --output /mnt/nvme/mapping/bench-001 --duration 120 --warmup 60 \
  --require-mount /mnt/nvme
wr-map validate /mnt/nvme/mapping/bench-001 --report /mnt/nvme/mapping/bench-001-audit.json
```

This reuses an existing MAVROS connector. See the recording guide for explicit
session ownership with `--start-mavros`, stable serial selection, or `--camera-only`.
Preflight failures and failed recordings are preserved in the output directory.
Do not overwrite or resume a recording.

## Commissioning

1. Record device identity, firmware, runtime versions, effective driver parameters
   and calibration privately; different OAK variants require separate profiles.
2. Record a printed target at the working distance. Inspect fixed exposure/focus
   and calibration at full resolution; the default focus is not a survey guarantee.
3. Run a short recording and validate geometry, rate, timestamps and required
   topics. ROS message headers cannot establish hardware sequence continuity.
4. Run at least a 20-minute soak with intended onboard workloads and temperature.
   Monitor disk growth, `tegrastats`, throttling, stream continuity and timing.
   The two-minute bench result does not replace this field-duration qualification.
5. Test Ctrl+C and service stop using disposable runs; verify finalized MCAP and
   checksums. Test USB loss on the bench and require a failed audit. Preserve all
   fault evidence. Do not fill a disk or alter a real survey to exercise failures.
6. Capture a moving multi-view scene with independent targets before flight.
   Qualify physical timestamp latency, mounting extrinsics, camera calibration,
   GNSS/RTK and independent map errors separately.

The existing PX4 timing thresholds remain unchanged. A stable recorder, valid
GNSS fix and similar timestamp units do not establish geolocation accuracy.

## Optional service

The service expects native ROS at `/opt/ros/humble/setup.bash`, the checkout at
`/opt/wallering/mapping_functionality`, and a dedicated `wr-mapping` user with
`plugdev`, `dialout` and output-storage access. Install the environment file with
actual paths and UART ownership. Do not enable unattended boot until commissioned.

```bash
sudo install -m 0644 deploy/wallering-mapping.env.example /etc/wallering-mapping.env
# Edit the installed environment file for this deployment before starting.
sudo install -m 0644 deploy/wr-mapping.service /etc/systemd/system/wr-mapping.service
sudo systemctl daemon-reload
sudo systemctl start wr-mapping
journalctl -u wr-mapping -f
sudo systemctl stop wr-mapping
```

The launcher uses the same ROS preflight/recorder as the CLI, requires the storage
mount and creates a unique session. `KillMode=control-group` and `KillSignal=SIGINT`
deliver stop to the foreground ROS launch job. ROS launch handles child shutdown
and rosbag2 flushes its own cache. The shell then seals the stopped files; the unit
permits this checksum step to finish even for long recordings. Timed recordings
include two seconds of post-roll; a manual/service stop is immediate and its
end coverage must be checked by validation.
Wait for `state=complete` before removing power; sealing large bags takes time.
Automatic restart is disabled. Check final state and validation before removing
power. The interactive Docker fallback requires daemon access; the supplied service
assumes native ROS and does not silently grant Docker privileges.

## Transfer

Copy the entire finalized directory with `rsync -a --partial`, then validate the
copy. Keep two copies before freeing source storage. Preserve hardware identity,
firmware evidence and mounting/field notes privately alongside the session. Import
images offline with `bag-import` before using the image reconstruction workflows.
