# Rig calibration file

The rig calibration relates the OAK cameras, the OAK IMU and the GNSS antenna to the
PX4 body, and the OAK clock to the PX4 clock, each with an uncertainty. Start from a
hand measurement; the calibration solvers ([#15](https://github.com/WalleringRobotics/mapping_functionality/issues/15))
later replace entries with `estimated` values. The file is device-specific: keep the
filled copy with private rig records (for example `runs/rig-calibration/rig.json`),
not in the repository.

```bash
cp configs/rig-calibration.template.json runs/rig-calibration/rig.json
# edit, then check against any recording from the same OAK:
.venv/bin/wr-map calibration-check runs/rig-calibration/rig.json --session runs/<session>
```

`calibration-check` validates the file, composes every camera (RGB and right from the
OAK factory extrinsics in the session) into the body frame, reports the antenna-to-camera
lever arm in the form the accuracy reports use, and lists what still blocks a complete
calibration. `--require-complete` exits 2 until nothing is blocking.

## Frames and conventions

| Frame | Meaning |
|---|---|
| `base_link` | PX4 body as MAVROS publishes it: **FLU** (x forward, y left, z up). Origin is PX4's body origin, the point PX4's `EKF2_*_POS_*` offsets are relative to. If `EKF2_IMU_POS_*` are zero, that is the flight controller IMU. |
| `oak_left_camera_optical_frame` | Left mono camera, optical convention: x right, y down, z out of the lens. Origin at the lens optical centre. |
| `oak_imu_frame` | OAK IMU chip axes. Not visible from outside; leave it to the solvers. |
| `gnss_antenna_arp` | GNSS antenna reference point (ARP, from the antenna datasheet, usually the bottom mount centre). |

A transform `parent -> child` maps child coordinates into the parent:
`p_parent = R p_child + t`. `translation_m` is **where the child origin is, measured in
the parent frame**. Rotations may be entered as `rotation_xyzw` (quaternion) or
`rotation_rpy_deg` = [roll, pitch, yaw] in degrees with `R = Rz(yaw) Ry(pitch) Rx(roll)`
about the parent axes (the ROS convention). Uncertainties are 1-sigma: rotation in
radians about the parent axes (at most 0.2 rad), translation in metres along them.
Unknown sigmas (`null`) keep the calibration incomplete rather than being treated as zero.

Time offsets: add `offset_ns` to `clock` timestamps to express them on `reference`.

## What to measure by hand

Fill these three and leave the IMU links `unset`:

1. **`base_link -> oak_left_camera_optical_frame`** (`source: manual_measurement`).
   - Translation: from the body origin to the left lens centre, along body forward, left
     and up. "Left" is from the camera's own view: the outer lens on your left when
     standing behind the OAK looking where it looks (on your *right* when facing its
     front); the factory data places the right camera 7.5 cm along image-right. A tape or ruler with `translation_sigma_m` 0.005–0.01 is a reasonable
     start; use 0.02 if the origin itself is uncertain.
   - Rotation: describe where the optical axes point in the body frame. Worked examples:

     | Mounting | `rotation_rpy_deg` | Check |
     |---|---|---|
     | Facing forward, image upright | `[-90, 0, -90]` | lens axis (z) → body +x; image right (x) → body −y |
     | Looking straight down, image top toward the vehicle front | `[180, 0, -90]` | lens axis (z) → body −z; image right (x) → body −y |

     Add any measured tilt from there and set `rotation_sigma_rad` to match how well
     you can sight it (0.03–0.05 rad ≈ 2–3° for a careful by-eye estimate).
2. **`base_link -> gnss_antenna_arp`** (`source: manual_measurement`): translation only
   (keep the rotation `[0, 0, 0]` with zero sigma; antenna orientation is not used).
   Cross-check with PX4: `EKF2_GPS_POS_X/Y/Z` are the same vector in **FRD**, so they
   should equal `[x, -y, -z]` of this entry.
3. **Time offsets**: without a measurement, enter `offset_ns: 0` with an honest bound
   as `sigma_ns` (for example 10 ms = `10000000`) and `source: manual_measurement`; the
   IMU-to-IMU solver replaces the OAK-to-PX4 entry with a measured value.

Record how and when you measured in `evidence` and `date`, and the OAK device ID
(the `*_calibration.json` name in any session) in `hardware.oak_device_id`;
`calibration-check --session` refuses a session from a different OAK.

## Solved links

`base_link -> oak_imu_frame` and `oak_imu_frame -> oak_left_camera_optical_frame` are
written by the solvers as `estimated`. When both exist, they take precedence over the
hand-measured camera link, and `calibration-check` reports their disagreement with it
(`direct_vs_imu_chain`, including a 3-sigma consistency flag): a large disagreement means
a measurement or sign error on one side.

The OAK-D (BW1098OBC) factory calibration has no IMU extrinsics, and the OAK ROS
driver's `/tf_static` publishes `oak_imu_frame` only as an identity under a separate
`oak_oak` parent, so no recorded data relates the IMU to the cameras. Only the
camera-to-IMU solver can supply that link.

## IMU-to-IMU solver

```bash
.venv/bin/wr-map calibrate-solve runs/<calibration-session> \
  --calibration runs/rig-calibration/rig.json --output runs/rig-calibration/solve-001
```

Both IMUs are on one rigid body, so their angular rates differ only by the mounting
rotation, constant biases and the clock offset. The solver interpolates the OAK gyro
(`/oak/imu/data`) onto PX4 stamps (`/mavros/imu/data_raw`) for each candidate offset,
fits the rotation in closed form (Kabsch on bias-centred rates), and keeps the offset
with the lowest residual (2 ms grid, 0.1 ms refinement, parabolic interpolation). It
writes a *new* calibration (`base_link -> oak_imu_frame` and the `oak_ros_stamp ->
px4_ros_stamp` offset, both `estimated`) and a report; the input file is not modified.

- **Uncertainty:** standard error across four independent segments (floors 0.1 ms,
  0.002 rad). The final quarter is held out: a fit of the earlier three quarters must
  agree with it within 4·σ_segment·√(1 + 1/n), or the solve is refused and nothing is
  written. The accepted result is the full-data fit.
- **Integrity:** the session must be `complete` and its `SHA256SUMS` seal must verify
  before any data is read; the seal's hash is recorded as evidence. Device identity is
  checked before solving, and the output directory appears only once complete.
- **Refusals:** a principal rotation axis below 0.3 rad/s RMS (rotate about every axis),
  fewer than two well-excited segments, an offset at the search limit, or a reflection
  fitting far better than a rotation (an IMU axis/handedness convention error).
- **Gaps:** OAK gaps over 50 ms are skipped, not interpolated, and counted in
  `skipped_for_oak_gaps`.
- **Translation** is not observable from gyros. If the file has a hand-measured camera
  position, the IMU position is taken from it with a 0.03 m housing bound added;
  otherwise its uncertainty stays unknown.
- **Meaning:** the offset relates OAK and PX4 *timestamps*. It includes any difference in
  sensor filtering delay and does not measure exposure timing (#14).

## Camera-to-IMU solver

`wallering_mapping.calibration_camera.solve_camera_imu(session)` estimates
`oak_imu_frame -> oak_left_camera_optical_frame` and the `oak_camera_exposure -> oak_imu`
offset ([#10](https://github.com/WalleringRobotics/mapping_functionality/issues/10)) without
a target. It returns a report whose `entries` drop into this file as `estimated`.

Method: corners are tracked (pyramidal LK, forward-backward checked) over `stride` frames
(default 3) on the global-shutter left mono stream; each pair's rotation comes from the
essential matrix (USAC with local optimisation, on points undistorted with the session's
factory intrinsics), or from a rotation-only fit when parallax is negligible. The gyro is
integrated over the same intervals, bias-corrected (start value from still windows, then
refined jointly with the rotation). The rotation is the Kabsch fit `r_imu = R r_camera` on
rotation vectors; the time offset is scanned (default ±50 ms, 1 ms then 0.1 ms grid with a
parabolic refinement) for the minimum residual. Sigmas come from a moving-block bootstrap
over frame pairs, floored at 0.5 mrad and 0.1 ms for effects the bootstrap cannot see
(intrinsics error, IMU sampling). The right camera is solved independently and mapped
through the factory extrinsics as a cross-check (`cross_check`, with a 3-sigma flag).
On synthetic data (20 fps, 100 Hz gyro with bias and noise, 0.4 px noise, 10 % outlier
points) it recovers the rotation to ~0.01° and the offset to < 0.1 ms.

Assumptions and limits:

- Translation is not observable from rotation alone: `translation_m` is written as zero
  with `translation_sigma_m` 0.03 m, a bound for an IMU inside the OAK housing (both lenses
  are within a few centimetres of it).
- The offset is relative (exposure stamp to IMU stamp on the OAK); absolute exposure
  timing needs an optical event ([#14](https://github.com/WalleringRobotics/mapping_functionality/issues/14)).
- Mono global-shutter cameras only; the RGB rolling-shutter readout is not modelled.
  Factory intrinsics and distortion are trusted, not re-estimated.
- It refuses, with a `ValueError`, recordings whose weakest rotation direction has an RMS
  rate below 0.15 rad/s (static, single-axis or pure translation), cameras below 10 fps,
  textureless scenes, fewer than 100 usable frame pairs, and an offset at the scan limit.

What the calibration recording ([#8](https://github.com/WalleringRobotics/mapping_functionality/issues/8))
should provide: mono left and right at 20 fps (≥ 10 fps) with `/oak/imu/data` at 100 Hz;
a few seconds still at the start; then 30–60 s of hand-held rotation about all three axes
(roughly ±20–30° at 0.5–1 Hz, with changes of speed so the offset is observable), in front
of a textured, well-lit scene a few metres away; short exposure to avoid motion blur.
