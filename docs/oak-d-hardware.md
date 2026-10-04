# OAK-D hardware reference

What the bench OAK-D is, what it natively does, and what we measured. Each fact is
marked by where it comes from: **[doc]** vendor documentation, **[src]** driver/SDK
source, **[cal]** the device's own factory calibration, **[bench]** our recordings
(evidence under ignored `runs/`). Numbered references are listed under [Sources](#sources).
Device serials and private calibration stay in `runs/`, not here.

## Device

| Item | Value | From |
|---|---|---|
| Product / board | OAK-D, board `BW1098OBC`, revision `R0M0E0`, calibration format version 5 | [cal] |
| Processor | Myriad X VPU (RVC2). USB ID `03e7:2485` before boot, 480 Mbit/s until DepthAI boots it | [bench] |
| USB | Negotiates `SUPER` (5 Gbit/s) at runtime on the Orin; Luxonis lists USB 2/3 "up to 10 Gbps" | [bench], [doc 1] |
| Power | 2.5–3 W base plus camera streaming | [doc 1] |
| Operating temperature | −20 °C to 50 °C when fully using the VPU | [doc 1] |

## Cameras

| Socket | Role | Sensor | Resolution | Shutter | Focus | FOV (D/H/V) | From |
|---|---|---|---|---|---|---|---|
| 0 (CAM_A) | RGB | Sony IMX378, 1/2.3" | 4056×3040 | **Rolling** | Auto (we fix lens position 135) | 81° / 69° / 55° | [doc 1], [cal] |
| 1 (CAM_B) | Left mono | OmniVision OV9282, 1/4" | 1280×800 | **Global** | Fixed | 81° / 72° / 49° | [doc 1], [cal] |
| 2 (CAM_C) | Right mono | OmniVision OV9282, 1/4" | 1280×800 | **Global** | Fixed | 81° / 72° / 49° | [doc 1], [cal] |

- **Left/right naming** is from the camera's own view. Standing behind the OAK looking where
  it looks, left is on your left. **[cal]**: the right camera sits 7.47 cm along the left
  camera's image-right axis.
- **Stereo baseline:** 7.47 cm measured, 7.5 cm nominal (`specTranslation`) **[cal]**. Luxonis's
  product page shows "75cm", evidently a typo for 7.5 cm **[doc 1]**. Ideal depth range ~0.8–12 m **[doc 1]**.
- **RGB to right camera:** 3.70 cm, nominal 3.75 cm: the RGB camera is midway **[cal]**.
- **Factory intrinsics:** the mono cameras are calibrated at 1280×800 (fx ≈ 855–859 px). The RGB
  camera is calibrated at **1920×1080** (fx ≈ 1464 px), with 14 distortion coefficients each and
  factory RGB lens position 135 **[cal]**. When we capture 4056×3040, DepthAI derives the 12 MP
  intrinsics from that 1080p calibration, scaling and shifting them (fx 1464.2 → 3093.2,
  cy 547.9 → 1536.6). The 12 MP intrinsics are therefore derived, not directly calibrated;
  verify them with a target (#14) **[bench]**.
- **Frame timestamps:** our profile sets `i_add_exposure_offset: true`, `i_exposure_offset: 1`.
  In DepthAI, `CameraExposureOffset` START=0, MIDDLE=1, END=2, so frames are stamped at
  **mid-exposure** **[src 6, 7]**. The driver default is no offset **[src 7]**. Mid-exposure
  is a convention: physical latency and RGB rolling-row readout are unmeasured (#14).

## IMU (CEVA BNO086)

| Item | Value | From |
|---|---|---|
| Model | BNO086, 9-axis (accelerometer, gyroscope, magnetometer) | [doc 1], [bench] |
| Firmware | Was 3.2.13; upgraded to **3.9.9** on 2026-10-04 to match DepthAI 3.10.0's baseline. Calibration JSON unchanged by the update | [bench] (`docs/hardware-acceptance.md`) |
| Factory IMU extrinsics | **None.** `imuExtrinsics.toCameraSocket = -1`, empty rotation | [cal] |
| Driver TF for the IMU | `oak_oak → oak_imu_frame` identity. `oak_oak` is not connected to the `oak` camera tree, and the driver warns "IMU extrinsics are not set" | [bench] |
| Batching (profile) | `i_batch_report_threshold: 20`, `i_max_batch_reports: 20`, `i_max_q_size: 50`: reports arrive in bursts | [bench], [doc 3] |

### Native report rates

Requested rates **round up** to the next supported point on RVC2 with BNO08X **[doc 2, 3]**:

| Sensor | Stable request points | Maximum exposed |
|---|---|---|
| Accelerometer (raw/calibrated) | 15 / 31 / 62 / 125 / 250 / 500 Hz | 512 Hz |
| Gyroscope raw | 25 / 33 / 50 / 100 / 200 / 400 Hz | up to 1000 Hz; above 400 Hz has occasional jitter **[doc 3]** |
| Gyroscope calibrated/uncalibrated | — | 100 Hz |

- **No rate is common** to both sensors.
- **The accelerometer's "125 Hz" point runs at 128.15 Hz** (7.803 ms period) on our device **[bench]**.
- Luxonis's own high-rate examples use accelerometer raw 480 Hz and gyroscope raw 400 Hz **[doc 3]**.

### Stream semantics

- `GYROSCOPE_RAW` / `ACCELEROMETER_RAW` are in the **sensor-native frame**, with **no
  extrinsics or affine calibration** applied. `*_CALIBRATED` streams are aligned to the
  DepthAI IMU frame and corrected **[src 6]**. The profile records raw streams.
- **ROS sync methods** (driver 2.12.2, `depthai_bridge/ImuConverter`) **[src 4, 5]**:
  - `COPY`: each message is stamped with the **accelerometer** report time
    (`tstamp = accel.getTimestamp()`), and the current gyro report is copied in.
  - `LINEAR_INTERPOLATE_ACCEL`: calls `interpolate(accelHist, gyroHist, …)`. The accelerometer
    is interpolated, and each message is stamped with the **gyroscope** report's time.
    Reports are deduplicated by sensor sequence number.
  - `LINEAR_INTERPOLATE_GYRO`: the reverse, with messages on accelerometer times.
  - Luxonis's parameter page summarises the interpolation methods ambiguously; the source is
    authoritative **[doc 8]**.

### What we measured (camera-only 20 s runs, `runs/imu-rate-20261004-001`) [bench]

| Profile | Message rate | Interval median (p1–p99) | Repeated gyro samples | Gaps > 2× |
|---|---|---|---|---|
| gyro 100, accel 100→125, `COPY` (old default) | 99.95 Hz | 7.93 ms (7.4–16.4) | 93 | 226 (pattern) |
| **gyro 200, accel 250, `LINEAR_INTERPOLATE_ACCEL` (default)** | 194.7 Hz | 4.99 ms (4.5–5.6) | 2 | 33 (max 45 ms) |
| gyro 400, accel 500, `LINEAR_INTERPOLATE_ACCEL` | 345.7 Hz | 2.51 ms (p99 30) | 1 | 84 (max 70 ms) |

- **Old default:** messages sat on the 128 Hz accelerometer grid with one slot in five empty,
  giving 7.8 / 16 ms intervals averaging 100 Hz. In a 148 s bag, 1230 of 14,388 messages
  repeated the previous gyro sample. Gyro timestamps were off by up to one accelerometer
  period, which would bias OAK-to-PX4 timing calibration.
- **New default:** native gyro samples on their own timestamps.
- **Sample loss grows with rate:** 0.27% at 100 Hz, 2.6% at 200 Hz, 13.6% at 400 Hz. Investigated in #18.

## Design decisions and constraints

The capture design follows what this device natively supports and how it was
factory-calibrated. Where a choice departs from that, it needs its own calibration.

| Decision | Basis | Status |
|---|---|---|
| Use the factory calibration; capture at the **calibrated resolutions**: mono 1280×800, RGB **1920×1080** | Factory intrinsics/extrinsics exist only at those sizes [cal]. 12 MP intrinsics are derived from the 1080p calibration (scaled and shifted), not measured [bench] | Mono already native. RGB at 1080p is the target for the 20 fps profile (#13); the 2 fps default is still 12 MP |
| Fix RGB focus at the **factory lens position 135** | The RGB calibration was made at lens position 135 [cal]. Autofocus or another position changes the intrinsics | In the profile (`r_set_man_focus: true`, `r_focus: 135`) |
| Prefer the **global-shutter mono pair** for geometry and calibration | OV9282 is global shutter; IMX378 is rolling [doc 1]; rolling readout is unmeasured (#14) | Camera-to-IMU calibration uses left mono (#10) |
| IMU: **gyro 200 Hz native**, `LINEAR_INTERPOLATE_ACCEL`, accel 250 Hz | Gyro supported points [doc 2]; true gyro stamps in this sync mode [src 4]; regular 5 ms intervals measured [bench] | Default profile |
| No IMU-to-camera prior from the device | No factory IMU extrinsics [cal]; driver TF identity, disconnected [bench] | Solved by #10; hand-measured camera link meanwhile (`docs/rig-calibration.md`) |
| Frame time = **mid-exposure** with fixed exposure | `i_exposure_offset: 1` = MIDDLE [src 6, 7]; fixed 1000 µs exposure in the profile | Physical latency unmeasured (#14) |
| 20 fps target | Mono 2 × 800P at 20 fps ≈ 41 MB/s; RGB 1080p at 20 fps ≈ 124 MB/s; 12 MP at 20 fps (≈ 740 MB/s) is not feasible raw | Profile qualification in #13 |

The 1080p RGB stream is a 16:9 output, while the sensor is 4:3 (4056×3040). It
therefore covers a narrower vertical field than 12 MP. The factory calibration describes
exactly that 1080p output, so using it avoids the derived 12 MP intrinsics. The ISP mode
behind it should be confirmed on the bench (#13).

## Bandwidth

Raw ROS images as recorded **[bench]**:

| Stream | Frame | 2 fps | 20 fps |
|---|---|---|---|
| RGB 4056×3040 `bgr8` | 37.0 MB | 74 MB/s | 740 MB/s (NV12 over USB ≈ 370 MB/s) |
| RGB 4K / 1080p `bgr8` | 24.9 / 6.2 MB | — | 498 / 124 MB/s |
| Two mono 1280×800 `mono8` | 2 × 1.02 MB | 4.1 MB/s | 41 MB/s |

- The current default (RGB 12 MP plus two mono, 2 fps) records about 78 MB/s, roughly 4.4 GiB/min.
- 12 MP RGB at 20 fps is not feasible raw (#13). IMU traffic is negligible.

## Open hardware questions

- Camera-to-IMU rotation and offset (#10): no factory or driver data exists.
- Absolute exposure latency, RGB rolling readout time, and 12 MP intrinsics accuracy (#14).
- IMU sample loss at 200 Hz (#18).
- Sensor maximum frame rates per resolution: not stated in [doc 1]; measure for #13.
- RGB 1080p ISP mode (crop and scaling of the 4:3 sensor) and whether the driver reports the factory intrinsics unchanged at 1080p (#13).

## Sources

1. Luxonis, *OAK-D* hardware page: <https://docs.luxonis.com/hardware/products/OAK-D> (sensors, FOV, baseline, IMU model, USB, power, temperature).
2. Luxonis, *BNO08X* IMU page: <https://docs.luxonis.com/hardware/platform/sensors/imu/bno08x> (stable rate points, maxima, rounding).
3. Luxonis, *IMU* node (DepthAI v3): <https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/imu> (rounding on RVC2, >400 Hz gyro jitter, 480/400 Hz raw examples, batching parameters).
4. depthai-ros `v2.12.2-humble` (tag commit `d316224`), `depthai_bridge/include/depthai_bridge/ImuConverter.hpp`: <https://github.com/luxonis/depthai-ros/blob/v2.12.2-humble/depthai_bridge/include/depthai_bridge/ImuConverter.hpp> (`ImuSyncMethod`, `FillImuData_LinearInterpolation`, `interpolate`).
5. depthai-ros `v2.12.2-humble`, `depthai_bridge/src/ImuConverter.cpp`: <https://github.com/luxonis/depthai-ros/blob/v2.12.2-humble/depthai_bridge/src/ImuConverter.cpp> (COPY path stamps with the accelerometer time).
6. DepthAI 3.10.0 Python API, `dai.IMUSensor` and `dai.CameraExposureOffset` docstrings and values (run `python -c "import depthai as dai; help(dai.IMUSensor)"` in the image). The IMUSensor docstring points to the CEVA BNO080/085 datasheet: <https://www.ceva-dsp.com/wp-content/uploads/2019/10/BNO080_085-Datasheet.pdf>.
7. depthai-ros `v2.12.2-humble`, `depthai_ros_driver/src/param_handlers/sensor_param_handler.cpp`: <https://github.com/luxonis/depthai-ros/blob/v2.12.2-humble/depthai_ros_driver/src/param_handlers/sensor_param_handler.cpp> (`i_add_exposure_offset` default false, `i_exposure_offset` default 0).
8. Luxonis, *Driver Parameters* (ROS): <https://docs.luxonis.com/software-v3/depthai/ros/parameters/> (`i_sync_method` options).
9. Bench evidence in this repository: [hardware acceptance](hardware-acceptance.md), especially "OAK IMU sync and native rate" and the firmware sections; factory calibration `*_calibration.json` and parameter dumps in each session; issue [#18](https://github.com/WalleringRobotics/mapping_functionality/issues/18).
