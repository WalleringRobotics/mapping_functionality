# Capture geometry and field procedure

Current acquisition uses [ROS2/MCAP recording](rosbag-recording.md). Direct-SDK
capture examples below are retained as legacy diagnostic procedures; they do not
run or qualify the current ROS recording stack.

## Begin with slow, static-scene surveys

For the first building survey, carry the mounted camera/Orin around the building.
Use good diffuse daylight, fixed focus and exposure, and enough translation to obtain
parallax. Rotate gradually while moving; pure rotation cannot establish depth.
Keep original photographs even if later selection rejects them.

The standard OAK-D's RGB sensor is rolling shutter; its mono pair is global shutter
[R1, R2](references.md). Short exposure reduces motion blur, but does not remove
rolling-row geometric distortion. Compare RGB reconstruction with a separate left-mono
run to distinguish detail/colour benefits from motion-induced geometry problems.
Never combine these streams under a single camera model.

## Derive FPS and speed from geometry

For a pinhole approximation, range `Z` and focal length `f_px` give:

- Ground/sample spacing near a fronto-parallel surface: `GSD ≈ Z / f_px` metres/pixel.
- Translational blur: `b_px ≈ v_perpendicular × exposure_seconds / GSD`.
- Angular blur: `b_px ≈ f_px × angular_rate_rad_s × exposure_seconds`.
- Timing-induced displacement: `e_translation ≈ v × delta_t`.
- Angular timing error at range: `e_rotation ≈ Z × angular_rate × delta_t`.

These are first-order engineering approximations. Oblique views, lens distortion,
surface slope and trajectory affect the local result; use calibrated intrinsics.

Illustration: 4056-pixel width and 69° horizontal FOV imply `f_px ≈ 2950`. At 10 m,
GSD is about 3.4 mm/pixel; at 30 m, about 10.2 mm/pixel. These are sampling distances,
not demonstrated coordinate accuracy. At 5 m/s and 3.4 mm/pixel, keeping translational
blur under 0.5 pixel needs exposure below about 340 µs. The default 1000 µs may be
appropriate for a slow walk but not that motion. Gain/noise and lighting then matter.

At 5 m/s, a 2 ms shutter-time error already corresponds to 1 cm of translation.
At 20 m range and 0.5 rad/s rotation, 1 ms contributes about 1 cm tangential error.
These examples explain why host receipt timestamps are inadequate for accurate
direct georeferencing, but do not imply all image-only SfM error is caused by timing.

For along-track footprint `L`, speed `v` and desired overlap `o`, require approximately
`fps >= v / ((1-o) × L)`. Use the narrow footprint dimension along motion and account
for occlusion. Start with 80% forward / 70% side overlap as **planning heuristics**,
then verify actual tie points, registration and coverage. A 2 fps default is not a
universal flight recommendation. Export interval must retain the required overlap.

## Building loop

1. Place visible measured targets/scale bars around the scene. Reserve independent
   checkpoints; do not use every measured point to fit the reconstruction.
2. Capture the whole loop with deliberate overlap across the start/end. Add connecting
   views around corners; avoid a sudden 90° viewpoint jump with no common features.
3. Add a second elevation/oblique loop where safe and practical. Roofs and recesses
   require visibility, not just another horizontal circle.
4. Avoid near-duplicate bursts and long stationary segments. Maintain enough baseline
   while retaining common features; inspect occlusions and textureless walls.
5. Record field notes: scene/weather, targets, focus, stand-off, speed and any unusual
   events. Check capture integrity and sharpness before leaving.

Start with perhaps 200–500 well-selected RGB views for a workstation trial; this is
a workload starting point, not a completeness limit. Exhaustive matching supports
loop connections but grows roughly quadratically with image count. Larger captures
need retrieval/spatial matching or hierarchical processing.

## Fields, vegetation and creeks

Use crossing flight lines plus oblique views where appropriate. Angular diversity
helps distinguish surfaces only when correspondence is stable. Wind-driven vegetation
violates the static-scene assumption; calm conditions and short capture intervals
help, but a reconstructed canopy is not bare earth. Water, reflections and transparent
surfaces are often unsuitable for image correspondence. Mark these regions as low
confidence/no-data; do not interpret mesh hole-filling as measurement. Repetitive crops
can produce false matches; inspect geometry against independent references.

OAK's short stereo baseline is not a substitute for a survey baseline. For stereo:
`Z = fB/d` and `sigma_Z ≈ Z² sigma_d / (fB)`. As an illustrative mono pair with
`f=760 px`, `B=0.075 m`, and disparity error `0.2 px`, at `Z=10 m` depth uncertainty
is about 0.35 m. These assumed values are not measured device performance. Multi-view
SfM can obtain larger baselines by moving the camera, but still needs texture and
calibration. Centimetre mapping must be demonstrated with independent checks.

## Focus and calibration discipline

Calibration is valid for a sensor mode, lens/focus state and image transformation.
Do not mix resized/cropped/rectified images with original intrinsics. Mount the OAK
rigidly, avoid changing focus during a session, and validate calibration across the
entire image. A small bundle-adjustment reprojection error can coexist with biased
3D geometry. Factory calibration is the initial estimate, not survey certification.

