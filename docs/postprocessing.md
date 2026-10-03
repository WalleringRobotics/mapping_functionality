# Postprocessing design and operating procedure

## Product paths

| Goal | Initial workflow | Result | Main constraint |
|---|---|---|---|
| Building/object geometry | Export RGB → COLMAP SfM → MVS | Camera poses, sparse model, dense coloured PLY | Scale/CRS absent unless constrained separately |
| Rolling-shutter comparison | Separate left-mono export → same SfM process | Independent geometry comparison | Lower texture resolution, no RGB colour |
| Terrain/orthomosaic | Export RGB → ODM/WebODM with GCPs/geolocation | Point cloud, mesh, orthomosaic, DSM; classified DTM if justified | Coordinate/datum and ground-visibility checks |
| Later high-accuracy rig | Triggered cameras + calibrated rig + time/GNSS constraints | Metric, georeferenced multi-sensor mapping | Future implementation; not enabled by recording two cameras alone |

The implementation automates the first two paths up to dense point cloud. ODM,
georeferencing, textured mesh generation and rig/VIO integration are designed here
but remain external/manual steps. Recorded IMU and stereo images are preserved;
standard single-camera COLMAP does not consume them as constraints.

## 1. Ingest and inspect

Copy the entire session and run `wr-map validate`. Do not process an incomplete
session as if it were intact. Keep originals on a second medium. Review sequence
gaps, frame periods, timing offsets, illumination, blur, exposure and focus. A
nearest timestamp match does not prove frame synchronization.

Record the survey ID, capture commit/config, device calibration hash, source manifest
hash, targets/CRS, software builds, and output parameters. The tools store many of
these automatically; field measurements and deployment versions remain operator inputs.

## 2. Select keyframes

```bash
wr-map export /data/sessions/site-001 --output /data/projects/site-001 \
  --stream rgb --interval 0.5
```

Interval selection uses sensor time and keeps original pixels. Sharpness is the
variance of the full-resolution grayscale Laplacian. It is scene/resolution dependent,
so `--min-sharpness` defaults to 0: inspect scores before choosing a threshold.
Textureless walls can score low even in sharp images; grass can score high despite
poor stable correspondences. Dark/bright fractions are diagnostics, not universal
rejection thresholds. Check rejected-image distribution so quality filtering does
not create coverage holes. Dynamic objects and water masks are an external step
for the initial workflow.

The exporter initializes COLMAP intrinsics from actual frame metadata. Perspective
coefficients are mapped to `FULL_OPENCV`; fisheye to `OPENCV_FISHEYE`. It refuses
unsupported lens terms or changing focus/geometry. The runner initially fixes lens
distortion/principal point and allows focal length refinement. Treat this as a
controlled baseline: compare against a separately calibrated lens and selected
bundle-adjustment refinements on independent checkpoints.

## 3. Sparse reconstruction

Install a CUDA-enabled **COLMAP 3.12.6** workstation build using its official
[release/source instructions](https://github.com/colmap/colmap/releases/tag/3.12.6).
Verify `colmap -h` and `colmap feature_extractor -h`. The repository deliberately
gates execution to the 3.12 family; a future major-version migration needs CLI and
dataset validation. A system package might be older or CPU-only, and pip's `pycolmap`
does not by itself provide the `colmap` CLI used here.

```bash
wr-map reconstruct /data/projects/site-001 --output /data/runs/site-001
wr-map reconstruct /data/projects/site-001 --output /data/runs/site-001 --execute
```

The stages are feature extraction, matching and incremental mapping [R7](references.md).
The plan contains explicit camera parameters and argument arrays; execution writes
per-stage logs and `run.json`. Existing output directories are refused. Failed
outputs are kept for diagnosis; retry in a new directory rather than silently
reusing a partially changed database.

Exhaustive matching is the default for small surveys, including building loop
closure. `--matcher sequential` is cheaper for long ordered sequences but does not
guarantee the start/end or cross-track links. For larger missions, add image retrieval,
spatially guided pairs or hierarchical SfM; do not infer full connectivity from a
successful process exit.

Inspect every `sparse/N` component with COLMAP GUI or `model_analyzer --path ...`.
Report registered/selected image fraction, disconnected components, point tracks,
reprojection-error distribution, camera path and obvious geometry failures. A
suggested first-trial target is >95% registered useful images in one component,
but scene coverage and withheld checks take precedence over this heuristic.

## 4. Dense geometry

```bash
wr-map dense /data/projects/site-001 --model /data/runs/site-001/sparse/0 \
  --output /data/runs/site-001-dense --max-image-size 2000 --execute
```

Explicit model selection prevents silently processing only an arbitrary fragment.
Undistortion, PatchMatch and geometric fusion produce `dense/fused.ply`. This baseline
requires CUDA for PatchMatch. Start at 2000 pixels on the long edge; increase only
after checking VRAM/runtime and useful detail. An RTX 3060-class 12 GB workstation
is a reasonable trial platform, but image count, size and scene complexity determine
resources. No runtime estimate has been benchmarked here.

Optional meshing uses COLMAP's Poisson/Delaunay tools or a separately versioned
OpenMVS/MeshLab workflow. Meshing can invent surfaces over gaps; retain the measured
point cloud, masks and confidence information. Texture/mesh export is not currently
wrapped by `wr-map`.

## 5. Scale, georeferencing and coordinate conventions

Single-camera SfM has an arbitrary similarity transform: scale, rotation and
translation are unobservable from pixels alone. The OAK stereo baseline is **not**
used by this runner, so recording left/right does not automatically provide metric
scale. For the initial object trial, fit scale/alignment with measured control
targets/scale bars, then evaluate different withheld targets.

For terrain products, use spatially distributed surveyed GCPs, and preserve reference
coordinate system, axis order and vertical datum. Do not mix ellipsoidal heights with
orthometric heights without a documented transformation. Ordinary GNSS priors can
help initialization but do not establish centimetre accuracy. RTK/PPK requires event
timing, antenna-to-camera lever arm, attitude and uncertainty modelling.

For future direct georeferencing, record raw GNSS and event marks plus device↔GNSS
clock calibration. At time `t`, camera centre is antenna/body position plus the
rotated lever arm. Delay, rotation and lever-arm uncertainty can dominate the RTK
receiver's quoted position precision. No such telemetry adapter exists in v0.1.

## 6. ODM / WebODM terrain path

Use the exported `images` directory as an ODM dataset. It contains no invented GPS
EXIF, camera identity or focal-length tags. `project.json` is **not an ODM calibration
file**. Initially, use WebODM to define camera/calibration overrides where supported
and annotate GCP observations; or build/verify an explicit ODM/OpenSfM camera override.
Do not assume ODM auto-detects a trustworthy lens model from these OAK image files.

Keep the export immutable: copy its selected images into a separate ODM working
directory, e.g. `/data/odm/site-001/images`. Add `gcp_list.txt` in the dataset root
using [ODM's exact format](https://docs.opendronemap.org/gcp/): projection header,
ground XYZ, observed image pixel XY, image filename and point label. Keep checkpoints
out of this fitted-control file [R8]. Use local projected metres where appropriate.

For command-line ODM, choose a tested release image and pin its immutable digest.
The following is a **template**; replace the digest with the one actually installed:

```bash
docker run --rm -v /data/odm:/datasets \
  opendronemap/odm@sha256:REPLACE_WITH_VERIFIED_DIGEST \
  --project-path /datasets --dsm --orthophoto-resolution 1 site-001
```

ODM output resolution in this option is cm/pixel; requesting 1 cm pixels does not
mean 1 cm absolute accuracy. Add `--dtm` only with defensible ground classification
and visible ground. Preserve canopy/no-data regions rather than promising bare-earth
terrain under dense vegetation. `--geo` can accept image positions [R9], but derive
them from calibrated exposure times and real telemetry, not host receipt timestamps.

ODM has rolling-shutter correction/readout options [R10]. Do not guess the OAK
readout time or substitute exposure duration: measure it for the sensor mode or obtain
vendor evidence before enabling a numeric override. This remains an experiment,
not a guarantee that dynamic distortion can always be corrected.

Deliverable targets: GeoTIFF orthomosaic and DSM, optional justified DTM, georeferenced
point cloud, mesh/texture when required, explicit horizontal/vertical CRS, and a quality
report. Inspect seams, leaning structures, moving foliage, water and extrapolated areas.

## 7. Accuracy report

Define the requirement first: e.g. local dimensional error, horizontal map RMSE,
vertical RMSE, 3D p95, or absolute position. These are different from GSD and reprojection
error. A proposed 1 cm objective must specify which metric, scene/range and valid area.

Create a CSV after the model has been aligned using **separate control data**:

```csv
id,role,reference_x_m,reference_y_m,reference_z_m,model_x_m,model_y_m,model_z_m
check-01,check,0,0,0,0.003,0.004,0.002
check-02,check,10,0,0,10.004,0.003,0.004
check-03,check,0,10,1,0.003,10.002,1.004
```

These rows are illustrative only. Use actual surveyed coordinates spanning the
footprint and heights, with enough independent points to assess spatial bias.

```bash
wr-map accuracy /data/quality/site-001-checkpoints.csv
```

The tool reports mean XYZ bias, per-axis RMSE, horizontal/3D RMSE, 3D p95 and max.
It does not align the two sets or verify the truth/independence of supplied coordinates.
Do not use withheld checkpoints to tune the same model repeatedly and then call
them independent. Report reference-measurement uncertainty, point count, coverage,
outliers/exclusions and repeat-survey consistency.

## Acceptance and next iteration

First demonstrate an end-to-end static object/building result, then repeat it.
Investigate accuracy loss with controlled changes: slower motion, shorter exposure,
fixed calibrated focus, mono comparison, larger convergent baseline, improved control.
Upgrade hardware after identifying the limiting mechanism. The current confidence
is high in the workflow structure, moderate in expected OAK/Jetson integration until
bench tests, and unestablished for any metric accuracy target.

