# Jetson setup and commissioning

The commissioned host is Orin Nano Super, L4T 36.5.0 / Ubuntu 22.04.5 / Python
3.10.12. OAK uses USB 3; PX4 TELEM2 uses a USB-UART adapter at 921600 baud.
Keep the working JetPack installation. See [hardware acceptance](hardware-acceptance.md)
for measured results and outstanding timing/GNSS limits.

## Fresh Orin

A new Orin needs this checkout, a prepared host and one image build. The
[image](../docker/Dockerfile) holds ROS Humble, the OAK driver, MAVROS, rosbag2/MCAP,
the pinned Python dependencies and a source-built PyCOLMAP; the host holds only what
must be on the host. Wrappers mount the checkout into the image, so code changes
need no rebuild.

1. Flash **Jetson Linux (L4T) R36.5.0**, Ubuntu 22.04, with NVIDIA's SDK Manager or
   SD card image; take the matching JetPack from
   [NVIDIA's R36.5 release](https://developer.nvidia.com/embedded/jetson-linux-r365).
   Create the operator account.
2. Clone and prepare the host (reports first with `--check`, changes nothing):

   ```bash
   git clone https://github.com/WalleringRobotics/mapping_functionality.git
   cd mapping_functionality
   sudo deploy/prepare-orin.sh --check
   sudo deploy/prepare-orin.sh
   ```

   Log out and in so new groups apply, and replug the OAK.
3. Mount the recording storage (see [Storage](#storage)).
4. Build the image, then confirm the runtime:

   ```bash
   deploy/build-image.sh
   bash deploy/run-ros.sh bash -c 'for p in depthai_ros_driver mavros rosbag2_storage_mcap; do ros2 pkg prefix "$p"; done'
   bash deploy/run-platform-command.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
   ```

   The first build compiles COLMAP: allow about 30 GiB of Docker storage and over
   an hour on an Orin Nano. `BUILD_JOBS` defaults to 2 to fit 8 GB of RAM; pass
   `--build-arg BUILD_JOBS=4` on larger modules. The tag is `wallering-mapping:humble`;
   `WR_MAPPING_IMAGE` selects another for the build and every wrapper.

Other L4T releases are refused until requalified (`--l4t` overrides the check).
`prepare-orin.sh` is idempotent; rerun `--check` to audit an existing host.

### Host dependencies

Everything below is provided by the host, not the image. `prepare-orin.sh` applies
the rows marked *applied* and reports the rest.

| Dependency | Why | Script |
|---|---|---|
| L4T 36.5.0, Ubuntu 22.04, aarch64 | Commissioned release; kernel USB/serial drivers | verified |
| `ca-certificates curl git gnupg usbutils` | Clone, Docker repository key, `lsusb` checks | applied |
| Docker Engine + buildx plugin | Build and run the image; BuildKit features | applied (Docker CE repo, or `docker-buildx` beside JetPack's `docker.io`) |
| `/etc/udev/rules.d/80-luxonis.rules` | OAK (VID `03e7`) access for `plugdev`, bootloader and runtime | applied |
| Operator in `plugdev`, `dialout`, `docker` | Camera, PX4 UART, Docker daemon (`docker` is root-equivalent) | applied |
| ModemManager disabled | It probes new `ttyUSB` devices, including the PX4 adapter | applied |
| `brltty` removed | It claims CP210x (`10c4:ea60`) adapters such as the bench TELEM2 cable | applied |
| NTP enabled | Host wall clock for session metadata; does not synchronize shutters | applied |
| NVMe mounted at `/mnt/nvme`, in `/etc/fstab` | Recording storage (~4.4 GiB/min) | reported only |
| Power mode (MAXN_SUPER on Orin Nano Super) | Commissioned throughput | reported only |

The image needs no NVIDIA container runtime: recording and PyCOLMAP are CPU-only.
CUDA probes (`hardware-check --mode building`) and dense COLMAP remain host or
workstation concerns. Image provenance is in `/opt/wallering/manifest/` inside the
image, and each recording stores the image ID and that manifest.

The ROS packages come from the signed 2026-08-07 snapshot (key
`4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA`, vendored in `docker/`, expires
2027-06-01; refresh it from `keyserver.ubuntu.com` before then). Ubuntu archive
packages follow `jammy-updates`. MAVROS GeographicLib datasets download from
SourceForge at build time; their checksums are recorded in the manifest.

## Storage

Partition, format and mount the NVMe deliberately; no script does this. Add it to
`/etc/fstab` and grant the operator (or service user) write access to
`/mnt/nvme/mapping`. Budget approximately 4.4 GiB/minute for the
uncompressed 2 fps profile plus space for offline PNG imports and exports.

## Runtime

Firmware/SDK diagnostics use Python DepthAI 3.10.0, which is also in the image;
they must run with no active ROS camera owner. The bench BNO086 was upgraded from
3.2.13 to 3.9.9 on 2026-10-04, with firmware and calibration verified after
reconnect. Firmware updates remain an explicit commissioning operation, never an
automatic startup step; see [the procedure](hardware-commands.md#oak-bno-imu-firmware-commissioning).

OAK re-enumerates during boot, so the udev rule covers both bootloader and runtime.
Verify `SUPER` in `oak.log`, not just the idle USB device. Use a secured direct
USB 3 cable. `wr-map capture` dispatches to the image through `deploy/run-ros.sh`;
a host venv (`python3 -m venv .venv && .venv/bin/pip install -e '.[bags]'`) is only
needed for running the CLI outside Docker.

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

The service expects the checkout at `/opt/wallering/mapping_functionality` and a
dedicated `wr-mapping` user with `plugdev`, `dialout` and output-storage access.
Without native ROS it records through the image, which also requires `docker`
membership (root-equivalent); grant it deliberately. Install the environment file with
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
power.

## Transfer

Copy the entire finalized directory with `rsync -a --partial`, then validate the
copy. Keep two copies before freeing source storage. Preserve hardware identity,
firmware evidence and mounting/field notes privately alongside the session. Import
images offline with `bag-import` before using the image reconstruction workflows.
