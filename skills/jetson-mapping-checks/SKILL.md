---
name: jetson-mapping-checks
description: Verify this repository's Jetson Orin Nano, OAK capture, PX4/MAVROS and RTK setup; run startup checks or diagnose silent serial links. Use for hardware commissioning and related deployment changes in mapping_functionality.
---

# Jetson mapping checks

Use repository commands to collect fresh evidence and identify which requested
parts of the setup pass, fail, or remain untested. The skill is stored under
`skills/` and is routed by the root `AGENTS.md`; it needs no global installation.

## Choose the workflow

- [Hardware command guide](../../docs/hardware-commands.md): setup, inventory,
  startup checks, baud probes, ROS graph inspection, capture and tests.
- [QGroundControl compatibility and USB diagnosis](../../docs/hardware-commands.md#qgroundcontrol-on-the-orin):
  confirm ARM64 and Ubuntu support before installing a release; inspect the
  controller through direct USB when TELEM2 is silent.
- [Acceptance record](../../docs/hardware-acceptance.md): measured bench results
  and unresolved blockers. Treat these as historical evidence; recheck changed hardware.
- [Jetson commissioning](../../docs/jetson-setup.md): storage, service deployment
  and loaded capture acceptance.
- [MAVROS timing](../../docs/mavlink-integration.md): clock and topic contracts.
  [Postprocessing](../../docs/postprocessing.md) covers engine and accuracy workflows.

Run commands from the repository root. Prefer the checked-in `deploy/check-*.sh`
wrappers or `wr-map` commands over reconstructing scripts in `/tmp`. Retain JSON
reports in `runs/` with new names. Reports deliberately refuse replacement.

## Hardware and runtime decisions

- Confirm the checked-out branch contains the PR stack before changing code.
  `main` may still contain only the original license.
- Reuse the pinned Python environment. Check imported binary-module versions as
  well as package metadata; ROS overlays can shadow venv packages.
- The sibling platform uses ROS Humble/Python 3.10 and DDS domain 1 by default.
  Its prebuilt image can differ from its checkout. Mapping wrappers bypass its
  entrypoint because it may start unrelated model preparation.
- Run OAK probes sequentially with one camera owner. USB 2 enumeration before
  boot does not establish runtime link speed; check the opened DepthAI device.
- `imu=auto` enables a detected IMU. A failed IMU is a failed survey setup. Use
  the explicit camera-only diagnostic profile to isolate image capture and label
  that result accordingly. Firmware commissioning is separate from startup probes.
  When an IMU firmware upgrade is requested, use the
  [commissioning procedure](../../docs/hardware-commands.md#oak-bno-imu-firmware-commissioning)
  and `deploy/update-oak-imu.py` with the inspected device ID and versions. Keep
  power connected until completion; verify firmware after reconnect, unchanged
  calibration, and both raw IMU streams during camera capture.
- Run the passive serial probe only after stopping the UART's existing owner.
  It changes host baud settings and reads bytes; it must not send flight commands
  or modify PX4 parameters. Use `--baud 115200` for a 115.2 kbaud test, or a bounded
  list of rates. Report byte counts separately from valid MAVLink and heartbeats.
- Serial silence does not establish a baud mismatch. Check port assignment,
  wiring and power using actual evidence. A heartbeat also does not qualify
  pose timing, RTK or camera geolocation.
- This setup uses MAV_1 on TELEM2 through USB-UART; MAV_0/TELEM1 is the SiK
  radio and MAV_2 is Ethernet. Select the adapter by `/dev/serial/by-id` and keep
  its individual identity in ignored reports. Confirm actual baud/flow-control
  settings, check all port owners and reuse routing before opening a second
  consumer. The [active link audit](../../docs/hardware-commands.md#telem2-parameter-stream-and-timing-audit)
  collects parameter readback, stream rates and TIMESYNC without changing PX4
  configuration. Report missing evidence before modifying the integration.
- Query actual MAVROS topics and time-plugin parameters. `/mavros/time` was the
  observed time node; verify it for the deployed graph. RTK requires running
  `gps_status`/`gps_rtk` plugins from `mavros_extras`, not only message definitions.
- For a short saved sample, use the five-second collection commands in the guide.
  The camera/PX4 diagnostic profile disables only the OAK IMU and allows a fresh
  timing window. Validate the immutable dataset, then run `sync` for every camera
  stream. Report rejected associations, GNSS fix validity and host NTP separately;
  never relax timing thresholds to make a bench sample pass.
- Require the configured storage mount and the capture user's actual write/fsync
  access. Preserve failed captures. Store credentials and survey profiles privately.

## Finish and verify

Update startup probes and regression tests for actual failures. Run focused tests,
Ruff and shell syntax checks, and exercise changed wrappers where hardware allows.
Run the ROS test in the platform image when ROS is absent on the host. If the
pinned ARM64 pycolmap dependency is unavailable, identify the excluded processing
tests explicitly instead of presenting the portable test run as the complete suite.

Keep installation, startup smoke checks, sustained throughput and physical
timing/survey acceptance distinct in the result. Startup checks must not claim
measured exposure latency, calibrated lever arms or centimetre accuracy.
