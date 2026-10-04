# Wallering Robotics — photogrammetric mapping

**Two operating modes: record on Jetson Orin Nano + Luxonis OAK-D, then process offline on a workstation.**

The processing mode supports **building/object geometry** with COLMAP and
**terrain maps** with OpenDroneMap. Original photographs, calibration, inertial
measurements and timing evidence remain available for future camera and algorithm upgrades.

**Status:** software implemented and tested without attached camera hardware.
DepthAI 3.10.0, COLMAP 3.12.x and ODM 3.6.2 are the supported baselines. Hardware
throughput and real-survey accuracy require [commissioning](docs/jetson-setup.md#commissioning).
Automated tests exercise storage, geometry conversion, real COLMAP model IO and
controlled engine orchestration; they do not establish field accuracy or a successful
full external-engine reconstruction on this development machine.

## Mode 1 — onboard capture

Follow [Jetson setup](docs/jetson-setup.md) for USB permissions, NVMe and service deployment.

```bash
git clone --branch feat/rtk-accuracy-reporting \
  https://github.com/WalleringRobotics/mapping_functionality.git
cd mapping_functionality
python3 -m venv .venv
source .venv/bin/activate
pip install -e '.[oak]'
wr-map doctor --mode capture --config configs/oakd-survey.json \
  --output-root /mnt/nvme/mapping --probe-device
wr-map capture --config configs/oakd-survey.json \
  --output /mnt/nvme/mapping/bench-001 --duration 60
wr-map status /mnt/nvme/mapping/bench-001
wr-map validate /mnt/nvme/mapping/bench-001
```

Use a new directory for every capture. Omit `--duration` to stop with Ctrl+C or
SIGTERM. `record` remains an alias for `capture`. The default requests RGB and two
mono cameras at 2 fps with fixed exposure/focus, JPEG RGB and lossless mono PNG.
The lossless recipe requests 1 fps and PNG throughout. Both need bench qualification.

The recorder saves factory calibration, full-FOV images, raw IMU when available,
per-stream timestamps, sequence numbers and checksums. Bounded buffering, disk
reserve and stream/IMU watchdogs fail explicitly. Live `status.json` updates every
five seconds; `wr-map status` flags stale recording heartbeats. Clean completion
means accepted data was drained and sealed, not that a survey meets accuracy targets.

For the existing **PX4/MAVROS companion connector**, follow the
[integration and timing guide](docs/mavlink-integration.md). Keep MAVROS connected
and release the OAK from the ROS camera driver before direct capture:

```bash
wr-map capture --config configs/oakd-mavros-survey.json \
  --telemetry-config configs/mavros-survey.json \
  --output /mnt/nvme/mapping/px4-bench-001 --duration 60
wr-map sync /mnt/nvme/mapping/px4-bench-001 \
  --output /mnt/nvme/alignment/px4-bench-001 --min-fraction 0.9
```

Configure the actual ROS node/topic names first. The supplied profile has a
60-second warmup before the saved duration. Telemetry retains original headers
and serialized ROS messages. Exposure-to-body-pose association uses measured
clock bridges and qualified TIMESYNC evidence; it rejects stale timing and pose
extrapolation. The derived body poses require calibrated camera mounting before
use as camera priors. Physical timestamp accuracy still needs bench measurement.

For the moving RTK/DGPS receiver and your nearby surveyed base, follow the
[RTK and accuracy guide](docs/rtk-accuracy.md). The `ntrip` command verifies the
fixed base's station ID and ARP coordinates before forwarding corrections through
MAVROS. The recorder retains raw GNSS uncertainty and correction evidence.
`image-accuracy` reports each image's conditional position budget and produces
qualified camera geolocation; `map-accuracy` reports independent checkpoint errors,
reference uncertainty and shared base bias. Receiver semantics, time convention
and antenna-to-camera calibration must be supplied from the actual installation.

## Mode 2 — offline processing

Copy the complete session to the workstation. Install Python extras and the
external engines using [postprocessing setup](docs/postprocessing.md).

```bash
pip install -e '.[processing,terrain]'
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
as rig/VIO constraints. Qualified MAVROS RTK camera geolocation is available for
terrain; `georeference` aligns building models and PLY products to metric projected
coordinates. Navigation alignment does not establish surface accuracy. Live SLAM
and tightly coupled visual-inertial/rig bundle adjustment remain future work.

## Development without hardware

```bash
pip install -e '.[dev,processing,terrain]'
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
- [Surveyed-base NTRIP, per-image budgets and total map accuracy](docs/rtk-accuracy.md)
- [Jetson setup and commissioning](docs/jetson-setup.md)
- [Capture geometry and field procedure](docs/acquisition.md)
- [Postprocessing recipes, control formats and recovery](docs/postprocessing.md)
- [Camera upgrades and remaining qualification](docs/roadmap.md)
- [Primary references and confidence](docs/references.md)

License: Apache-2.0. Keep datasets, serial numbers, locations and credentials outside
this public source repository.
