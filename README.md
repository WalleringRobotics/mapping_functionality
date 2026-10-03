# Wallering Robotics — photogrammetric mapping

Record on **Jetson Orin Nano + Luxonis OAK-D**, reconstruct offline on a workstation.
The acquisition foundation uses DepthAI **3.10.0**, independent full-resolution images,
optional raw IMU, factory calibration, image calibration metadata, and explicit clocks.

**Status:** implementation under development; hardware commissioning is still required.
Synthetic tests exercise storage, not OAK throughput or photogrammetric accuracy.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e '.[oak,dev]'
wr-map inspect
wr-map record --config configs/oakd-survey.json --output /mnt/nvme/mapping/session-001 --duration 60
```

Without a camera:

```bash
pip install -e '.[dev]'
wr-map simulate --output /tmp/mapping-fixture --frames 12
pytest
```

Use a **new output directory** for every session. `SIGINT`/`SIGTERM` stop gracefully;
writer/USB/storage faults mark the session failed. A power interruption leaves an
incomplete session for later validation. Source images must be retained.

The sample exposure and ISO are starting values for daylight bench tests, not tuned
flight settings. The exact OAK-D model, lens, USB throughput and JetPack version must
be established on the target. This repository does not yet claim centimetre accuracy.
