# Jetson setup and commissioning

## Proposed baseline

Orin **Nano**, JetPack 6 / Ubuntu 22.04 / Python 3.10, USB OAK-D, USB 3 and mounted
NVMe. The installed JetPack is unconfirmed: first record `uname -a`,
`/etc/nv_tegra_release`, Python version and available memory. Do not upgrade a working
JetPack deployment just for this recorder. USB DepthAI does not use Jetson CSI/Argus.

DepthAI 3.10.0, NumPy 1.26.4 and headless OpenCV 4.11.0.86 are pinned. Use a venv to
avoid replacing JetPack packages. If pip cannot find the ARM64 wheel for the Python/
glibc combination, resolve that combination rather than silently changing API versions.

```bash
sudo apt-get update
sudo apt-get install python3-venv python3-dev libusb-1.0-0 usbutils git
git clone https://github.com/WalleringRobotics/mapping_functionality.git
cd mapping_functionality
python3 -m venv .venv
source .venv/bin/activate
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

The launcher checks the mount and uses a unique session name. SIGTERM reaches the
recorder through `exec`. Automatic restart is disabled so recurring power/USB faults
cannot silently split a survey. Tune the stop timeout against measured flush time.
Check final status before removing power. A physical recording-status/start-stop
interface is future work.

## Transfer

After clean stop, copy the entire session (e.g. `rsync -a --partial`) and validate on
the destination. Keep two copies before freeing source storage. Save survey field
notes plus repository commit, `pip freeze`, JetPack version, camera ID and mounting
configuration alongside the private dataset.

