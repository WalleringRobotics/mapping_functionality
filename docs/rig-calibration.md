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
