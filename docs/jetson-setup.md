# Jetson setup and commissioning

## Baseline

Orin **Nano**, JetPack 6 / Ubuntu 22.04 / Python 3.10, USB OAK-D, USB 3 and mounted
NVMe. The 2026-10-03 bench identified Orin Nano Super, L4T 36.5.0, Ubuntu 22.04.5
and Python 3.10.12. See [hardware acceptance](hardware-acceptance.md) for measured
results, current setup failures, and repeatable startup probes. Do not upgrade a working
JetPack deployment just for this recorder. USB DepthAI does not use Jetson CSI/Argus.

For capture alongside the existing PX4 connector, use the optional
[MAVROS integration guide](mavlink-integration.md), including the Humble-compatible
venv, actual topic/node names, camera ownership and longer timing warmup.

DepthAI 3.10.0, NumPy 1.26.4 and headless OpenCV 4.11.0.86 are pinned. Use a venv to
avoid replacing JetPack packages. If pip cannot find the ARM64 wheel for the Python/
glibc combination, resolve that combination rather than silently changing API versions.

```bash
sudo apt-get update
sudo apt-get install python3-venv python3-dev libusb-1.0-0 usbutils git
git clone --branch feat/oak-photogrammetry-foundation https://github.com/WalleringRobotics/mapping_functionality.git
cd mapping_functionality
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip 'setuptools>=68,<80' wheel
pip install -e '.[oak]'
```

USB permissions for a normal capture user:

```bash
sudo install -m 0644 deploy/80-luxonis.rules /etc/udev/rules.d/80-luxonis.rules
sudo usermod -aG plugdev "$USER"
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Log out/in and reconnect the camera. Run `lsusb -t` and `wr-map inspect`; the recorder
requires `SUPER`/`SUPER_PLUS`. The device re-enumerates at boot, so permissions must
cover bootloader and runtime. Prefer a direct port and secured cable. Separately mount
NVMe and grant the capture account write access. Avoid root-filesystem fallback.

Before recording, run:

```bash
wr-map doctor --mode capture --config configs/oakd-survey.json \
  --output-root /mnt/nvme/mapping --probe-device
```

Without `--probe-device`, doctor checks dependencies/storage but does not open the
camera. For startup acceptance, run the stronger bounded capture/validation probe:

```bash
bash deploy/check-hardware.sh --require-jetson --config configs/oakd-survey.json \
  --output-root /mnt/nvme/mapping --require-mount /mnt/nvme
```

This checks the actual write/fsync path, host resources, binary imports, USB 3,
BNO firmware baseline, settings readback, calibrated frames and enabled IMU streams.
It records temporary images and removes them after validation. `imu=auto` enables
a present IMU and fails if it cannot work; it never silently falls back to no IMU.
The present BNO086 reports unsupported firmware 3.2.13 versus the SDK baseline
3.9.9. Commission that firmware separately before enabling IMU recording.

During a recording use `wr-map status SESSION` from another terminal; a
heartbeat older than 15 seconds is flagged stale. The heartbeat includes counts,
writer backlog, free storage, memory availability and host load. A stale heartbeat
is evidence to investigate, not a command to restart or overwrite a session.

## Commissioning

1. **Identity:** save `inspect` output outside the public repo. Confirm sensor names,
   actual dimensions, autofocus and IMU. Lite/W/Pro variants differ. Select present streams.
2. **Focus/exposure:** record a printed target at working distance for 10 seconds. Inspect
   at 100%, tune fixed lens position/exposure/ISO/white balance, and preserve the profile.
   Factory focus is the AF-module fallback, not guaranteed optimal survey focus.
3. **Short session:** record 60 seconds, validate all requested streams, actual settings,
   dimensions, monotonic times and absence of gaps. Check both IMU sensors if enabled.
4. **Soak:** record at least 20 minutes at intended FPS, texture and temperature. Use
   `tegrastats` to watch CPU/RAM, temperature and throttling. Measure disk growth and
   actual rates. Repeat with other intended onboard workloads active.
5. **Faults:** test Ctrl+C, then service stop. Disconnect USB on a bench test and verify
   failed status/nonzero exit. Exercise the reserve by setting it higher than free space,
   without filling a disk. Abruptly terminate only a disposable run and verify it fails
   validation. Preserve failed data for diagnosis.
6. **Geometry:** walk a small loop around a textured static object, then a building with
   targets. Check registered views, folded/duplicated surfaces and independent metric
   error before attempting a flight survey.

Initial engineering acceptance targets: no unexplained image/IMU gaps, no throttling,
no backlog failure, disk headroom, and complete status after clean stop. Timing and
accuracy require additional checks in [acquisition](acquisition.md) and
[postprocessing](postprocessing.md).

If overloaded, reduce FPS first, then test RGB alone. Turn off IMU only when that
evidence is not needed. Increasing buffers cannot fix sustained overload. PNG is
lossless relative to the ISP image, **not raw Bayer**; it may be expensive on the Nano.

## Optional service

`deploy/wr-mapping.service` assumes a checkout/venv at
`/opt/wallering/mapping_functionality`, an NVMe mount at `/mnt/nvme`, and a dedicated
`wr-mapping` account in `plugdev`. Create the account, install the checkout/venv there
and grant it write access to `/mnt/nvme/mapping` before installing the service.
Enable unattended startup only after manual commissioning.

```bash
sudo install -m 0644 deploy/wr-mapping.service /etc/systemd/system/wr-mapping.service
sudo systemctl daemon-reload
sudo systemctl start wr-mapping
journalctl -u wr-mapping -f
sudo systemctl stop wr-mapping
```

The launcher runs hardware checks before recording, requires the configured mount,
and uses a unique session name. It saves a separate startup report beside the session
and prevents capture if a required check fails. Configure deployment paths and optional
ROS settings using `deploy/wallering-mapping.env.example`; the unit reads
`/etc/wallering-mapping.env`. Update the unit's `RequiresMountsFor` if changing storage.
SIGTERM reaches the
recorder through `exec`. Automatic restart is disabled so recurring power/USB faults
cannot silently split a survey. Tune the stop timeout against measured flush time.
Check final status before removing power. A physical recording-status/start-stop
interface is future work.

## Transfer

After clean stop, copy the entire session (e.g. `rsync -a --partial`) and validate on
the destination. Keep two copies before freeing source storage. Save survey field
notes plus repository commit, `pip freeze`, JetPack version, camera ID and mounting
configuration alongside the private dataset.
