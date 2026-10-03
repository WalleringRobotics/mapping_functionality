# Wallering Robotics — photogrammetric mapping

**Capture on Jetson Orin Nano + Luxonis OAK-D. Reconstruct offline on a workstation.**

A capture-first foundation for building surveys and terrain mapping, with a portable
dataset contract for later global-shutter/GigE upgrades.

**Status:** implemented and tested without camera hardware. The OAK adapter targets
DepthAI **3.10.0**. Jetson throughput, the actual OAK model and reconstruction accuracy
still require [commissioning](docs/jetson-setup.md#commissioning). Synthetic tests
exercise IO, not SfM geometry. No centimetre-accuracy claim is made.

## Implemented scope

| Stage | Implementation |
|---|---|
| Inspect | Device ID, sensors, focus capability, IMU and USB speed |
| Record | Full-FOV RGB + left/right images, optional raw IMU, calibration and timestamps |
| Storage | Bounded writer, hashes, atomic image publication, disk reserve, failure status |
| Validate | Integrity, image decoding, counts, sequences, time continuity and camera offsets |
| Export | Original image selection, quality metrics, calibrated single-camera COLMAP project |
| Reconstruct | Reviewable/executable COLMAP 3.12 sparse and dense pipelines, logs and provenance |
| Accuracy | Independent checkpoint bias, RMSE, p95 and maximum error |
| Terrain products | Documented ODM workflow; automated ODM integration is future work |

ROS2, online SLAM, GNSS/flight-controller recording, rig-constrained reconstruction,
hardware triggers and automatic georeferencing are future work. The initial COLMAP
pipeline does **not** consume the recorded IMU.

## First capture

After the USB permissions and NVMe setup in [Jetson setup](docs/jetson-setup.md):

```bash
git clone https://github.com/WalleringRobotics/mapping_functionality.git
cd mapping_functionality
python3 -m venv .venv
source .venv/bin/activate
pip install -e '.[oak]'
wr-map inspect
wr-map record --config configs/oakd-survey.json \
  --output /mnt/nvme/mapping/bench-001 --duration 60
wr-map validate /mnt/nvme/mapping/bench-001
```

Use a **new directory** each time. Without `--duration`, record until Ctrl+C/SIGTERM.
The default requests 2 fps per camera, RGB JPEG quality 97 and lossless mono PNG.
`configs/oakd-lossless.json` requests 1 fps with RGB PNG. These are bench starting
profiles; measure throughput and tune exposure/focus before a moving capture.

## Workstation postprocessing

```bash
# Transfer the entire session, including calibration and journals, then validate.
wr-map validate /data/sessions/bench-001
wr-map export /data/sessions/bench-001 --output /data/projects/bench-001 \
  --stream rgb --interval 1

# Without --execute, print the plan without creating output files.
wr-map reconstruct /data/projects/bench-001 --output /data/runs/bench-001
wr-map reconstruct /data/projects/bench-001 --output /data/runs/bench-001 --execute

# Review sparse components and explicitly choose one for dense reconstruction.
wr-map dense /data/projects/bench-001 --model /data/runs/bench-001/sparse/0 \
  --output /data/runs/bench-001-dense --execute
```

The runner supports COLMAP **3.12.x** (baseline 3.12.6), not arbitrary newer CLI versions.
Sparse extraction/matching can use `--cpu`; dense PatchMatch in this baseline needs
CUDA. Outputs initially have arbitrary frame and scale. See
[postprocessing](docs/postprocessing.md) for installation, GCPs and terrain products.

## Development without hardware

```bash
pip install -e '.[dev]'
wr-map simulate --output /tmp/mapping-fixture --frames 12
wr-map validate /tmp/mapping-fixture
wr-map export /tmp/mapping-fixture --output /tmp/mapping-export
wr-map reconstruct /tmp/mapping-export --output /tmp/mapping-plan
pytest -q
ruff check src tests
```

Synthetic images are deliberately not a physical 3D scene; executing their SfM plan
is rejected. CI runs camera-independent tests on Python 3.10 and 3.12.

## Design documents

- [Architecture and decisions](docs/architecture.md)
- [Dataset and timing contract](docs/dataset-format.md)
- [Jetson setup and commissioning](docs/jetson-setup.md)
- [Capture geometry and field procedure](docs/acquisition.md)
- [Postprocessing and accuracy](docs/postprocessing.md)
- [Camera upgrades and roadmap](docs/roadmap.md)
- [Primary references and confidence](docs/references.md)

License: Apache-2.0. Keep datasets, serial numbers, survey locations and credentials
outside this public source repository.
