# Offline processing: building and terrain recipes

ROS recordings must first be checked and imported using `wr-map bag-import`, as
shown in the [recording guide](rosbag-recording.md#offline-image-preparation).
All image-dataset paths below refer to that derived output or a legacy SDK session.
Keep the original MCAP alongside it for inertial and telemetry processing.

## Workstation setup

Use the **x86-64 Linux workstation** for offline reconstruction. Capture stays on
the Orin; copy the complete stopped dataset to the workstation. The standard path
uses one official **OpenDroneMap 3.6.2 Docker image**, including its pinned OpenSfM
engine. No ROS, separate OpenSfM build, PyCOLMAP or CUDA installation is required.
The small host virtual environment handles import, calibration conversion and
auditing; reconstruction runs inside that image.

```bash
bash deploy/setup-postprocessing.sh
source .venv-postprocess/bin/activate
wr-map doctor --mode process --backend opensfm --output-root /data/runs
wr-map doctor --mode process --backend terrain --output-root /data/runs
```

The setup script creates the environment and pulls the pinned official image.
Docker must already be installed and accessible to the operator. The runner resolves
the locally installed tag to its immutable image ID, verifies the engine's version,
and records image ID/digests. A recipe can pin
`opendronemap/odm@sha256:<64 hexadecimal characters>` instead. Mutable `latest` tags
are rejected. Containers use the operator's UID/GID, a private run directory and no
network. Rootless Docker UID mappings and SELinux mounts may need local deployment
adjustments. Docker access is not configured by this package.

`doctor` checks dependencies and writable/free storage; it does not verify the
Docker daemon/image, benchmark resource use or qualify a survey. Version/container
checks also run when executing the relevant engine. Allow ample disk space:
immutable originals, exports, derived PNGs/masks, engine inputs, databases, depth
maps and failed attempts are retained. No automatic pruning occurs.

### Optional COLMAP alternative

Existing COLMAP recipes remain supported. Install `.[processing]` in the host
environment and a CUDA-enabled **COLMAP 3.12.6** workstation build using the
[official release/source instructions](https://github.com/colmap/colmap/releases/tag/3.12.6).
Check `colmap -h` and `colmap feature_extractor -h`. The runner gates CLI execution
to 3.12.x. The pinned `pycolmap==3.12.6` package reads models and reports poses; it does
not install the `colmap` executable. `cpu: true` changes sparse SIFT extraction and
matching only. Dense PatchMatch still needs CUDA; for CPU-only sparse processing,
set `dense: false` and `mesh: false` in a copied recipe.

```bash
wr-map doctor --mode process --backend building --output-root /data/runs
```

## PyCOLMAP on Jetson ARM64

PyPI's 3.12.6 release has no Linux ARM64 wheel. The [image](../docker/Dockerfile)
builds it from the pinned source commit (`4d5b60e1`) using the
[official source layout](https://github.com/colmap/colmap/tree/3.12.6/python) and
installs it, so `deploy/run-platform-command.sh` already has `pycolmap`. To use the
same wheel in a host venv, export it from the build stage:

```bash
docker buildx build -f docker/Dockerfile --target pycolmap-wheel \
  --output type=local,dest=runs/pycolmap-wheel .
.venv/bin/python -m pip install --no-deps runs/pycolmap-wheel/pycolmap-3.12.6-*.whl
.venv/bin/python -c 'import pycolmap; print(pycolmap.__version__, pycolmap.has_cuda)'
```

PyCOLMAP 3.12.6's bindings require Ceres 2.1 or newer, but Ubuntu 22.04 ships 2.0, so
the build stage compiles a checksum-pinned CPU Ceres 2.2.0 first. It repairs the wheel
with auditwheel, so it bundles Ceres and its other native libraries; the host needs no compiler, devel
packages or `LD_LIBRARY_PATH`. This CPU build supports model IO and quality
assessment. It does not install a host COLMAP CLI or qualify CUDA dense
reconstruction. Reduce `--build-arg BUILD_JOBS=1` if compiler memory pressure is high.

## Ingest and select photographs

Transfer the entire stopped session, including calibration and journals; retain a
second copy. `wr-map validate SESSION` audits all recorded streams. Incomplete or
corrupted sessions are rejected. `process` repeats validation before planning or
running. It exports one stream at a time using the recipe's `stream`,
`interval_seconds`, `min_sharpness` and `allow_gaps` fields.

Selection uses device time. `selection.jsonl` records each decision, Laplacian
sharpness, dark and bright fractions. Sharpness defaults to zero because a universal
threshold would reject low-texture walls or favor unstable foliage. Inspect the
scores and coverage before raising it. Selected GCP photographs must survive the
selection interval and quality filter. Decrease the interval to retain them.

Images are initially copied byte-for-byte. Export preserves actual pixel calibration
and rejects changing dimensions/focus or unsupported lens terms. Perspective models
map to COLMAP `FULL_OPENCV`; fisheye to `OPENCV_FISHEYE`. RGB and mono are separate
reconstructions; recording stereo does not automatically supply metric scale.

## Run and resume

```bash
# Plan only; validates the source but creates no output files.
wr-map process /data/sessions/building-001 --config configs/process-opensfm.json \
  --output /data/runs/building-001

# Optional input preparation before a long engine run.
wr-map process /data/sessions/building-001 --config configs/process-opensfm.json \
  --output /data/runs/building-001 --prepare-only

# Continue the prepared run.
wr-map process /data/sessions/building-001 --config configs/process-opensfm.json \
  --output /data/runs/building-001 --execute --resume
```

A new run must use an unused directory outside its source. Use `--resume` explicitly
for an existing run. Source manifest, recipe, control-file hashes and height reference
must match. Completed stages are hash-checked before reuse; a modified artifact is
an error. Failed/interrupted stages retry in a fresh numbered directory, preserving
all earlier evidence. Concurrent processing of the same run is locked out. A killed
or powered-off worker may leave a `running` attempt; resume marks it interrupted and
creates a new attempt. Check that any surviving engine process has stopped first.

SIGINT/SIGTERM cancel owned subprocess groups. ODM cleanup targets only the unique
container created by that attempt. Interrupted source capture recovery is different:
failed capture sessions remain ineligible for export, and automated salvage is future work.

`workflow.json` contains stage history, commands/results, checksums and provenance.
`report.json` is written after successful preparation/execution and lists product
paths and quality. After a failed retry, consult `workflow.json` as the current
status; an older report can describe an earlier successful state. Engine logs live
inside each attempt directory. Capture and workflow paths can contain spaces.

Changing configuration, including choosing a different sparse `model_index`, requires
a new run. For manual reuse of an inspected existing model, use the lower-level
`dense` command. No existing model/database is silently overwritten.

## OpenSfM building/object recipe

`configs/process-opensfm.json` delegates feature detection, matching, tracks,
reconstruction, bundle adjustment, sparse export and dense stereo to the upstream
OpenSfM bundled in ODM. Its source revision is
[`c5328439465e6ace011f39077d1077d7b1cdd65d`](https://github.com/OpenDroneMap/ODM/blob/v3.6.2/SuperBuild/cmake/External-OpenSfM.cmake).
The runner checks source-interface hashes and records native-library hashes plus
the immutable Docker image ID before running. `backend: "opensfm"` selects this
engine; the terrain recipe selects ODM. Older recipes without a backend field
retain their original COLMAP/ODM behavior.

Preparation creates a native `dataset/` containing images, masks, fixed camera
overrides, EXIF overrides and `config.yaml`. Rational/fisheye calibration is
converted through the same OpenCV geometry used for ODM, preserving original
dimensions and intrinsic matrix. Derived PNGs have no inherited EXIF GPS. Selection
timestamps and source calibration remain in the preparation audit; device times
are not represented as invented EXIF epochs. Native image masks exclude unusable
undistortion boundaries. The prepared dataset also works with the pinned
[upstream CLI](https://github.com/OpenDroneMap/OpenSfM/tree/c5328439465e6ace011f39077d1077d7b1cdd65d/bin).

The engine uses SIFT/FLANN visual matching, fixed intrinsics and no GPS/GCP priors.
`matcher: "exhaustive"` pairs all images; `"sequential"` uses upstream matching by
filename with eight neighbors. Sequential selection does not guarantee loop or
cross-track closure. `max_concurrency` bounds upstream processes; `max_image_size`
bounds feature/undistorted-image resolution. Native depthmaps use up to 640 pixels
width. Set `dense: false` for sparse-only processing; keep `mesh: false` for this
backend. Use the ODM terrain recipe for georeferenced rasters and textured meshes.

After upstream reconstruction, the runner verifies the applied calibration,
camera/point coordinates, selected-image membership and the same default 90%
registration/100-point quality gates. Multiple components require an explicit
`model_index`; the complete upstream reconstruction is preserved before staging
the selected component. These gates run before dense stereo. Empty dense clouds
are failures, even when upstream exits successfully.

Products are native `reconstruction.json` (including camera poses), `tracks.csv`,
`camera_models.json`, sparse `reconstruction.ply`, `quality.json` and, when enabled,
`undistorted/depthmaps/merged.ply`. Upstream rotation vectors/translations describe
camera-from-world poses; world axes, origin and scale are arbitrary. OpenSfM may
write a default geographic-origin file internally; it is not measured geolocation.
This workflow makes no metric scale, RTK or centimetre-accuracy claim. Existing
`georeference` operates on COLMAP models; it does not yet align native OpenSfM
products. Use qualified control through ODM for the georeferenced terrain path.

### Reproducible upstream sample check

Before processing a field dataset, run the three-photo Berlin example already
bundled inside the same official image:

```bash
python deploy/check-opensfm.py --output runs/opensfm-berlin-check
python deploy/check-opensfm.py --output runs/opensfm-berlin-check --execute --resume
```

The first command prepares inputs; the second reconstructs them. Each stage is
sealed and retryable through the same workflow journal. Image and reference-camera
hashes are pinned to the upstream source revision. The harness converts the
example's perspective camera to the equivalent pixel-intrinsic/radial model and
passes that through the real production adapter. It supplies no reference poses,
points or GPS to the new reconstruction. Calibration comes from upstream's example
reconstruction, **not a measured OAK calibration**. Passing this check establishes
software interoperability only; it does not qualify capture hardware or survey
accuracy. Use `--sparse-only` on both commands for a sparse-only test.

## COLMAP building/object alternative

`configs/process-building.json` runs export, feature extraction, exhaustive matching,
incremental mapping, sparse quality assessment, dense fusion, and Poisson meshing.
COLMAP fixes lens distortion/principal point and may refine focal length. Compare
factory calibration against measured calibration before making accuracy claims.

Default quality gates require one sparse component, at least 90% registered selected
images, and 100 sparse points. These are engineering starting gates, not survey
standards. The report includes unregistered images, median/p95 reprojection errors
and median track length. Multiple components require explicit `model_index`; do not
silently accept an arbitrary fragment. Large surveys may need a different matching
strategy; sequential matching alone does not guarantee loop/cross-track closure.

Outputs include COLMAP binary model files, `quality.json`, `camera-poses.csv`,
`dense/fused.ply`, and optional `mesh.ply`. Pose CSV contains camera centers in COLMAP
world coordinates and **camera-from-world** quaternion `w,x,y,z` plus translation.
World units/axes are arbitrary until independently aligned. The Poisson mesh is
untextured and can fill gaps; retain the measured cloud and inspect invented surfaces.

Single-camera SfM cannot determine scale, rotation or translation from pixels alone.
Apply measured control/scale alignment separately before evaluating metric errors.
Neither IMU nor the OAK stereo baseline is used as a metric constraint in this version.
The implemented `georeference` command uses qualified, exposure-aligned RTK camera
centres to align a selected COLMAP model and its PLY products. Follow the
[RTK workflow](rtk-accuracy.md#buildingsobjects) for calibration, geometry gates,
product selection and the stored local-origin convention.

## Ordinary geotagged photographs with native ODM

For conventional drone photographs that already contain camera and GPS EXIF,
use the [official ODM Docker workflow](https://github.com/OpenDroneMap/ODM/blob/v3.6.2/README.md#quickstart).
Copy the original JPEGs/TIFFs into `/data/odm/site-001/images/`, keeping their EXIF
and a separate original backup. On the x86-64 processing workstation:

```bash
docker run --rm --network none --user "$(id -u):$(id -g)" --env HOME=/tmp \
  --mount type=bind,src=/data/odm,dst=/datasets \
  opendronemap/odm:3.6.2 --project-path /datasets --max-concurrency 4 \
  --dsm --orthophoto-resolution 2 --dem-resolution 5 site-001
```

ODM reads the original EXIF, estimates camera calibration and writes its normal
orthophoto, elevation, point-cloud and model directories under
`/data/odm/site-001/`. Resolutions are centimetres per pixel; choose them to suit the
photographs. Inspect the actual CRS and height reference, and use measured control
and independent checkpoints when accuracy matters. Ordinary GPS EXIF is a camera
position prior, not centimetre-accuracy evidence. Native ODM manages its own stage
outputs; this command does not create this repository's sealed workflow journal.

The calibrated OAK workflow below starts from a verified imported recording and
requires explicit geolocation/control because its derived PNGs carry no GPS EXIF.

## Calibrated OAK terrain recipe and control inputs

`configs/process-terrain.json` requires **GCP observations or camera geolocation**,
plus `--vertical-datum`. Inputs use original selected image names and original image
pixel coordinates. `project.json` and `frames.jsonl` provide the name mapping.
Annotate points externally in those original images; an annotation GUI is not bundled.

Native GCP format (illustrative values, not real survey observations):

```text
EPSG:32632
500000 6000000 100 320 240 rgb_000000000.jpg target-01
```

The first non-comment line declares an explicit projected/geographic CRS. Each row
is `X Y Z pixel_x pixel_y image_name label`; names/labels must not contain whitespace.
This recipe requires at least **five noncollinear ground points**, each observed in
**three selected images**. Repeated point labels must have identical ground XYZ.
Distribute targets over the footprint and heights. Keep withheld checkpoints out
of this file. Input syntax/layout checks do not establish reference accuracy.

Alternatively use an ODM geolocation file:

```text
EPSG:32632
rgb_000000000.jpg 500000 6000000 110
```

Rows contain `image X Y Z`, optionally followed by
`yaw pitch roll horizontal_sigma vertical_sigma` (all five optional values together).
Use ODM's axis/angle conventions [R9](references.md), positive stated uncertainties,
and actual calibrated camera positions at exposure time. At least three noncollinear
XY camera positions are required. GNSS receipt time is not camera exposure time.
The optional MAVROS recorder acquires raw receiver evidence; `image-accuracy`
generates this file from qualified RTK positions and calibrated lever arms. See
[RTK geolocation](rtk-accuracy.md#terrain). The optional recipe `gps_accuracy_m`
explicitly sets ODM's prior weight; it does not declare measured map accuracy.

```bash
wr-map process /data/sessions/terrain-001 --config configs/process-terrain.json \
  --output /data/runs/terrain-001 --gcp /data/control/terrain-001.txt \
  --vertical-datum EGM2008 --prepare-only
wr-map process /data/sessions/terrain-001 --config configs/process-terrain.json \
  --output /data/runs/terrain-001 --gcp /data/control/terrain-001.txt \
  --vertical-datum EGM2008 --execute --resume
```

Use `--geo FILE` instead of or alongside `--gcp FILE`. Supply consistent horizontal
and vertical references. The height label is recorded, **not transformed**: do not
mix ellipsoidal and orthometric heights. Confirm actual output GeoTIFF CRS, bounds,
height convention and nodata in GIS before delivery. Automated checks establish
decoding, valid masks, finite coordinates, common horizontal CRS and overlapping
extents; independent raster/geodetic accuracy still requires measured checkpoints.

### Calibration handling

OAK rational/fisheye coefficients cannot all be represented by ODM's Brown model.
Preparation therefore undistorts a **working copy** with OpenCV, keeping the original
intrinsic matrix and dimensions. It writes lossless derived PNGs, usable-area masks,
an exact ODM/OpenSfM camera override and transformed GCP pixel observations. Source
images remain unchanged. GCPs outside usable undistorted pixels are rejected.

OpenSfM normalized intrinsics use `max(width,height)` and center
`((width-1)/2,(height-1)/2)`. Derived images have no EXIF; the pinned ODM camera ID
includes its documented-in-source placeholder focal ratio. The override supplies
actual calibration. The runner first ingests images and verifies that ID, then
checks applied camera parameters and sparse connectivity before dense products.
It fixes camera parameters and uses visual FLANN matching, without fabricated GPS.

This conversion is pinned to ODM 3.6.2 source behavior. Upgrading ODM needs renewed
camera-ID, parameter and GCP checks. Calibration unit tests cover both rational and
fisheye rays; real imagery must still establish reconstruction quality.

### Products and acceptance

The runner requires nonempty orthomosaic GeoTIFF, DSM GeoTIFF and georeferenced LAZ;
mesh is enabled by default and includes available texture/material files. `dtm: true`
adds DTM generation/verification. Request it only with defensible ground classification
and visible ground. Dense vegetation does not yield measured bare earth by configuration.

Before accepting completion, GDAL decodes the rasters and PDAL streams the LAZ
inside the same ODM image. `product-validation.json` records pixel spacing, valid
data, point count, bounds and the actual CRS. Empty/corrupt data, incompatible
coordinate systems, non-metric axes and disjoint extents fail the run. These are
file and coordinate-consistency checks; supplied height labels do not perform a
vertical datum conversion or establish independent accuracy.

`orthophoto_cm` and `dem_cm` specify output pixel spacing in centimetres. They do not
assert that imagery supports that detail or that absolute accuracy matches it. Inspect
seams, duplicate structures, moving foliage, water, edge extrapolation and nodata.
Do not guess rolling-shutter readout from exposure duration; no correction is enabled
without characterized sensor timing.

## Independent accuracy assessment

After external building alignment or terrain georeferencing, measure withheld points
in the output and prepare a CSV in a common **metre-based frame**:

```csv
id,role,reference_x_m,reference_y_m,reference_z_m,model_x_m,model_y_m,model_z_m
check-01,check,0,0,0,0.003,0.004,0.002
check-02,check,10,0,0,10.004,0.003,0.004
check-03,check,0,10,1,0.003,10.002,1.004
```

```bash
wr-map accuracy /data/quality/checkpoints.csv
```

The tool reports XYZ bias, per-axis/horizontal/3D RMSE, 3D p95 and maximum. It does
not fit alignment or verify the supplied observations' independence. State reference
uncertainty, coverage, outliers/exclusions and repeat-survey behavior. A successful
process exit, low reprojection error or fine output grid does not establish centimetre
accuracy. Hardware capture and a full real-survey reconstruction remain commissioning
gates for both product recipes.

For reference-aware reporting, use `wr-map map-accuracy` with the
[map profile and extended checkpoint CSV](rtk-accuracy.md#total-map-accuracy-breakdown).
It reports reference sigmas, same-base correlation, vertical errors, surface-class
statistics and coverage alongside the observed comparison. Link the completed
workflow and image report; for building products also link the sealed georeference.
The original `accuracy` command remains a lightweight residual-only calculation.
