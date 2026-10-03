# Primary references and confidence

Reviewed 2026-10-03. The implementation was checked against the installed DepthAI
3.10.0 Python API signatures without attached hardware. Links may track newer docs;
the pinned source release is the API baseline.

| ID | Primary source | Used for |
|---|---|---|
| R1 | [Luxonis OAK-D](https://docs.luxonis.com/hardware/products/OAK-D) | Standard RGB/mono sensor and shutter specifications; variants must be inspected |
| R2 | [Luxonis shutter types](https://docs.luxonis.com/hardware/platform/sensors/shutter-type) | Rolling versus global exposure/readout |
| R3 | [DepthAI Camera](https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/camera/) and [3.10.0 release](https://github.com/luxonis/depthai-core/releases/tag/v3.10.0) | Camera builder, output requests and pinned release |
| R4 | [DepthAI IMU](https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/imu) | IMU reports, requested/actual rate distinction and sensor-native frames |
| R5 | [DepthAI device](https://docs.luxonis.com/software-v3/depthai/depthai-components/device) | Device/host timestamp relationship |
| R6 | [NVIDIA Orin Nano encoding](https://docs.nvidia.com/jetson/archives/r36.4/DeveloperGuide/SD/Multimedia/SoftwareEncodeInOrinNano.html) | Orin Nano lacks NVENC; do not assume Jetson video encode offload |
| R7 | [COLMAP 3.12 CLI](https://colmap.github.io/legacy/3.12/cli.html) and [release 3.12.6](https://github.com/colmap/colmap/releases/tag/3.12.6) | Sparse/dense CLI sequence and reproducible runner baseline |
| R8 | [ODM GCPs](https://docs.opendronemap.org/gcp/) | Ground-control input conventions |
| R9 | [ODM image geolocation](https://docs.opendronemap.org/geo/) | Per-image coordinate file semantics |
| R10 | [ODM arguments](https://docs.opendronemap.org/arguments/) | Terrain products, rolling-shutter settings and processing choices |
| R11 | [Luxonis image quality](https://docs.luxonis.com/hardware/platform/sensors/image-quality) | Exposure/image-quality trade-offs |
| R12 | [COLMAP camera models](https://colmap.github.io/cameras.html) | Explicit pinhole/distortion model selection |

Engineering formulas for GSD, blur, timing displacement and stereo uncertainty are
first-order pinhole derivations, not vendor accuracy specifications. Sensor sizes/FOV
used in examples are illustrative standard-model values, not verified measurements
of the user's OAK. Requested rates, disk budgets, overlap, resource margins and
registration thresholds are commissioning assumptions to test.

Confidence: **high** that a capture-first/archive-first architecture preserves useful
reprocessing options; **moderate** in this hardware adapter until target testing;
**unknown** for sustained Nano throughput and **unestablished** for centimetre-level
mapping accuracy. Passing software tests does not change the last two statements.


## Pinned terrain implementation references

- [ODM v3.6.2 camera override conversion](https://github.com/OpenDroneMap/ODM/blob/v3.6.2/opendm/camera.py): supported camera fields and exact camera-ID handling.
- [ODM v3.6.2 image identity](https://github.com/OpenDroneMap/ODM/blob/v3.6.2/opendm/photo.py): no-EXIF defaults and `camera_id()`.
- [ODM v3.6.2 ingestion](https://github.com/OpenDroneMap/ODM/blob/v3.6.2/stages/dataset.py): image/mask naming and ingestion metadata.
- [ODM v3.6.2 OpenSfM stage](https://github.com/OpenDroneMap/ODM/blob/v3.6.2/opendm/osfm.py): calibration overrides and fixed-camera configuration.
- [OpenSfM geometry conventions](https://opensfm.org/docs/geometry.html): normalized image coordinates and camera models.

These contracts were inspected against the pinned source. Full Docker reconstruction,
real sensor throughput and independent metric accuracy were not measured in the
hardware-free development environment; the commissioning procedures remain required.
