# Camera upgrades and delivery roadmap

## Initial acceptance milestones

| Milestone | Evidence required | Current state |
|---|---|---|
| M0: software foundation | IO fixtures, corruption/failure tests, command plans | Implemented; tests runnable without hardware |
| M1: Orin/OAK bench capture | Device/JetPack inventory; 20-minute capture, stop/fault checks | Requires target hardware |
| M2: static building reconstruction | Connected sparse model, dense result, scale/control and held-out checks | Requires real capture |
| M3: terrain product | GCP/CRS-verified ODM product and independent accuracy report | Designed; integration manual |
| M4: moving platform capture | Blur, vibration, exposure timing, power and throughput acceptance | Not demonstrated |
| M5: upgraded synchronized rig | Trigger/PTP/GNSS event evidence, calibration and metric rig solve | Planned |

## Upgrade decision matrix

| Candidate | Benefit | Integration requirements / limitation |
|---|---|---|
| Current OAK RGB | Colour/detail, simplest start | Rolling shutter on common SKU; daylight/slow motion calibration trial |
| Current OAK mono | Global shutter on standard OAK-D | Lower resolution; useful independent geometry comparison |
| Global-shutter RGB OAK variant | Existing DepthAI ecosystem | Verify exact sensor/SKU, exposure synchronization and lens calibration |
| GigE Vision global-shutter camera | Industrial lenses, trigger/PTP options | New SDK/GenICam adapter, packet/chunk metadata, network bandwidth, power |
| USB3 Vision global-shutter camera | Direct host link and industrial triggering | Vendor SDK or GenICam adapter, trigger wiring, per-image calibration |
| CSI/GMSL synchronized cameras | Compact integrated multicamera rig | Carrier, drivers, trigger distribution, ISP and timestamp-domain validation |

No product purchase is prescribed until required range/GSD, lens, motion, FPS, SWaP
and environmental protection are known. The camera interface should deliver an image,
native sequence, exposure start/mid/end evidence, sensor clock/domain, settings,
per-image pixel calibration and model/serial identity into the same session writer.

PTP aligns clocks; it does not inherently trigger all shutters. Hardware triggering
aligns exposure events; it does not automatically establish absolute UTC or calibrate
rolling-row timing. Require both a characterized exposure event and a traceable clock
mapping when integrating multiple cameras and navigation sensors.

## Prioritized engineering backlog

1. Benchmark actual OAK sensor modes, USB load, host JPEG/PNG encoding, IMU reports,
   memory and flush latency on the Nano; lock one qualified deployment profile.
2. Add a recording status display/LED and controlled start-stop input, plus field QA
   thumbnails without blocking the recording path.
3. Add optional camera-side MJPEG or raw chunk storage only if measured bandwidth/CPU
   requires it. Preserve exact frame/timestamp association and restart recovery.
4. Add a controlled recovery command for interrupted sessions; never mutate originals.
5. Add GNSS/flight-controller adapter with raw messages, clock mapping and event marks.
6. Add calibration tooling (intrinsics, distortion, camera-camera, camera-IMU, timing),
   explicit metre/unit/frame conventions, and uncertainty evidence.
7. Build a calibrated stereo/rig reconstruction adapter with validated exposure pairing.
   Do not assume nearest timestamps alone are sufficient.
8. Add ODM camera conversion and version-pinned execution, GCP/geo ingestion and product QA.
9. Add large-dataset matching, scene masks and repeat-survey comparisons after small
   surveys consistently meet their defined accuracy objectives.

## Configuration changes needing requalification

Camera/lens/focus, mounting stiffness, shutter mode/resolution, FPS, compression,
USB cable/hub, storage device/filesystem, power supply, thermal setup, DepthAI, JetPack,
and concurrent workloads can affect data quality or continuity. Retain versioned
profiles and calibration identities; do not infer equivalence from a successful launch.

