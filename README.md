# Wallering Robotics — photogrammetric mapping

**Two operating modes: record on Jetson Orin Nano + Luxonis OAK-D, then process offline on a workstation.**

The processing mode supports **building/object geometry** with COLMAP and
**terrain maps** with OpenDroneMap. Original photographs, calibration, inertial
measurements and timing evidence remain available for future camera and algorithm upgrades.

**Status:** OAK BNO086 firmware has been upgraded to 3.9.9. Acquisition now uses
the official Luxonis ROS driver, MAVROS and **rosbag2 with MCAP**. A physical bench
recording passed integrity and requested-window coverage at 2 fps with a 100 Hz
IMU request (99.79 Hz observed). Field-duration stability, GNSS/RTK and physical
timing/rig calibration still need qualification; see the
[measured results](docs/hardware-acceptance.md).

## Mode 1 — onboard ROS2 recording

See [the recording guide](docs/rosbag-recording.md) for the commissioned ROS Humble
runtime, USB/UART ownership, service setup and timestamp limitations.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e '.[bags]'
# Reuse the existing MAVROS connector; add --start-mavros only to own that link.
wr-map capture --output runs/bench-001 --duration 120 --warmup 60
wr-map status runs/bench-001
wr-map validate runs/bench-001 --report runs/bench-001-audit.json
```

`capture` / `record` launches vendor ROS nodes and standard `ros2 bag record`.
ROS launch owns driver/recorder startup and shutdown; the shell prepares the
session and seals it afterward. The default stores full-resolution raw
RGB and two mono streams at 2 fps plus IMU/PX4, calibration and timing messages.
A new directory is required; `--duration 0` runs until Ctrl+C. For camera-only
commissioning use `--camera-only`. Storage demand is about 4.4 GiB/minute.

The immutable session contains MCAP files, parameters, calibration, logs and
checksums. ROS headers do not carry hardware sequence counters, so source sample
loss remains unknown. A completed bag is not a claim of survey accuracy.

Import images **offline**, after recording, to use the existing image workflows:

```bash
wr-map bag-import runs/bench-001 --output runs/bench-001-images
wr-map process runs/bench-001-images --config configs/process-building.json \
  --output runs/bench-001-building --prepare-only
```

The import writes lossless PNGs and preserves ROS timestamps and CameraInfo
calibration; original IMU/PX4 messages remain in MCAP. Legacy SDK datasets remain
supported. The former Python acquisition command is explicitly `legacy-capture`.

## Mode 2 — offline processing

Copy the complete ROS session to the workstation and run `bag-import` into a separate image dataset. The examples below use that derived dataset. Install Python extras and the
external engines using [postprocessing setup](docs/postprocessing.md).

```bash
pip install -e '.[bags,processing,terrain]'
wr-map doctor --mode process --backend building --output-root /data/runs

# Review the plan; no output files are created.
wr-map process /data/sessions/bench-001 --config configs/process-building.json \
  --output /data/runs/bench-001

# Export → sparse model → quality gate → dense cloud → Poisson mesh.
wr-map process /data/sessions/bench-001 --config configs/process-building.json \
  --output /data/runs/bench-001 --execute

# Retry safely using unchanged inputs/config; completed stages are hash-verified.
wr-map process /data/sessions/bench-001 --config configs/process-building.json \
  --output /data/runs/bench-001 --execute --resume
```

For terrain, supply surveyed GCP observations or actual camera geolocation plus
an explicit height reference. The tool does not invent GPS coordinates.

```bash
docker pull opendronemap/odm:3.6.2
wr-map process /data/sessions/site-001 --config configs/process-terrain.json \
  --output /data/runs/site-001 --gcp /data/control/site-001.txt \
  --vertical-datum EGM2008 --execute
```

| Product recipe | Outputs | Coordinate/accuracy limits |
|---|---|---|
| Building/object | Binary sparse model, camera poses CSV, registration report, coloured PLY, Poisson PLY mesh | Arbitrary scale/origin until externally aligned; mesh is untextured |
| Terrain | GeoTIFF orthomosaic and DSM, georeferenced LAZ, textured mesh; optional DTM | Requires real reference data; pixel resolution is not measured accuracy |

`--prepare-only` produces verified selected inputs without invoking an engine.
`workflow.json` records stage attempts and hashes; `report.json` lists product paths,
quality and provenance. Failed outputs/logs are retained. Parameters are immutable
within a run: change a recipe or control file by starting a new run directory.

The lower-level `export`, `reconstruct`, `dense` and `accuracy` commands remain
available. Recorded IMU and stereo images are preserved but are not yet consumed
as rig/VIO constraints. Optional MAVROS GNSS acquisition is implemented; automatic
camera geolocation, building scale alignment and live SLAM remain future work.

## Development without hardware

```bash
pip install -e '.[dev,bags,processing,terrain]'
wr-map simulate --output /tmp/mapping-fixture --frames 12
wr-map validate /tmp/mapping-fixture
wr-map process /tmp/mapping-fixture --config configs/process-building.json \
  --output /tmp/mapping-run --prepare-only
pytest -q
ruff check src tests
```

Synthetic IO images are not a physical 3D scene; engine execution rejects them.
CI tests Python 3.10 and 3.12, including installed CLI preparation and resume.
An additional Humble job exercises real ROS parameter services, DDS and CDR with
simulated PX4 publishers. OAK/PX4 hardware remains outside automated coverage.

## Design and field guides

- [Architecture and decisions](docs/architecture.md)
- [Dataset and timing contract](docs/dataset-format.md)
- [PX4/MAVROS integration, time synchronization and bench acceptance](docs/mavlink-integration.md)
- [Jetson setup and commissioning](docs/jetson-setup.md)
- [Capture geometry and field procedure](docs/acquisition.md)
- [Postprocessing recipes, control formats and recovery](docs/postprocessing.md)
- [Camera upgrades and remaining qualification](docs/roadmap.md)
- [Primary references and confidence](docs/references.md)

License: Apache-2.0. Keep datasets, serial numbers, locations and credentials outside
this public source repository.
