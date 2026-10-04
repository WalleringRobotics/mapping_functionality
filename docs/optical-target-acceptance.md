# Absolute exposure timing and target verification

This is the physical acceptance procedure for issue #14. No optical-event or
printed-target measurement has been performed by this procedure's addition.
Software calibration, IMU correlation and a populated rig file do not establish
absolute shutter timing or camera accuracy. Use the
[repository hardware workflow](../skills/jetson-mapping-checks/SKILL.md) and retain
private artifacts under a new `runs/` directory. Begin with
`configs/optical-target-acceptance.template.json`; null values are unmeasured.

## Instrumented optical event

Use a timestamped pulse source driving an LED visible to all cameras being tested,
with a photodiode and oscilloscope/logic acquisition measuring the **optical** edge.
Document exact boards, pins, levels, isolation and shared-reference arrangement
before wiring. Use a current-limited suitable driver; an unverified PX4 or Orin
pin must not be connected to an external voltage source. Keep props removed and
actuator power isolated for bench work. A software GPIO-call timestamp alone is
insufficient because scheduling, driver and LED delays are unknown.

Record the source edge time, photodiode optical edge time, uncertainty, clock name,
clock-bridge observations and repeated event identifier. Establish and measure the
bridge from the instrument clock to the camera/ROS timeline. If no such bridge
exists, report relative optical behavior only and leave absolute acceptance pending.
Measure propagation delay/jitter rather than equating electrical and optical edges.

Warm up the rig in its final mount. Lock and record exposure, gain, focus and image
resolution for each run; repeat any mode expected in operation. Capture original
image, camera-info and both IMU streams with unmodified timestamps. Sweep pulse
phase relative to frame acquisition, use distinguishable event codes, and include
both on/off transitions. Pulses spanning multiple frames without unique IDs cannot
establish which edge was observed. Avoid saturation; inspect raw image intensity
and reject ambiguous/clipped events without silently removing their counts.

For each camera retain at least 100 independently identified usable events over
several acquisition phases, then a separate held-out sequence. Report all attempted,
accepted and rejected events and reasons. Fit the exposure interval against optical
edges using recorded exposure duration; an integrated intensity is an interval
observation, not an instantaneous point. Report whether the stamp means start,
midpoint or end, the offset sign/equation, latency estimate, repeatability, measured
worst residual, held-out residual distribution and instrument/bridge uncertainty.
Repeat under representative USB, storage and compute load and after a cold restart.

Choose acceptance limits **before** inspecting fit results, using allowed camera
position/ray error and measured speed/angular-rate limits. For example, a 0.02 m
translational allowance at 5 m/s leaves at most 4 ms before rotational and other
errors. This example is a budget calculation, not a repository acceptance threshold.
A one-sigma lag estimate is not a deterministic latency bound.

## RGB rolling shutter

Illuminate a substantial vertical image region with the same measured transitions.
Fit row-dependent exposure time, stating whether row coordinate is full sensor,
cropped or resized. Report signed row period, full-frame readout duration, nonlinear
residuals and uncertainty separately from frame offset. Use multiple rows and both
edge polarities across held-out events and repeated captures. Reject aliasing from
periodic pulses and record any exposure-mode dependence. A single centre-row offset
cannot qualify every RGB pixel. Global-shutter behavior on other cameras must also
be checked rather than inferred from RGB results.

## Checkerboard or AprilGrid

Measure the physical target dimensions with a traceable scale; record square/tag
size, spacing, row/column counts, units, printing scale, flatness and measurement
uncertainty. Preserve the exact target definition and photographs. Use a rigid flat
backing. Record camera serials, firmware, image mode, focus state and calibration
file hashes. Any focus or resolution change requires a separate verification.

Collect at least 30 sharp varied views per camera, covering all corners, edges and
centre, several distances and oblique orientations. Keep synchronized multi-camera
views to check stereo extrinsics. Avoid a collection made only from fronto-parallel
images at one distance. Split complete poses/captures into fitting and held-out
sets; neighbouring duplicate frames must not leak between them.

Compare factory/calibrated predictions against held-out target corners. Retain
corner coordinates, detection rejection reasons and overlays. Report median, RMS,
95th percentile and maximum reprojection residual in pixels, grouped by image
radius, camera and distance; inspect coherent distortion rather than only RMS.
Compare stereo relative rotation/baseline and estimated scale with independent
physical measurements and their uncertainties. Do not average unknown covariance
into zero. Optional camera/IMU target calibration is an independent cross-check of
the targetless result, with temporal offset and rotation compared using combined
uncertainty and consistent frame/clock conventions.

## Evidence and acceptance

Fill the template with original bag seals, event table, instrument trace, target
definition, corner files, analysis version/command and held-out reports. Hash every
referenced artifact. Predeclare each threshold and explain its mapping requirement.
Keep optical frame offset, row timing, target intrinsics, stereo extrinsics and
camera/IMU cross-check as separate outcomes. Missing reference clocks, target scale,
uncertainty, held-out data or provenance leave the relevant outcome pending.

The rig schema currently stores aggregate clock offsets and transform sigmas; it
does not encode a per-row shutter model or a measured worst-case exposure bound.
Do not replace unrelated gyro offsets or claim global survey readiness to force
these measurements into the file. Preserve the acceptance report separately and
require consumers to support its clock-domain and uncertainty semantics first.
