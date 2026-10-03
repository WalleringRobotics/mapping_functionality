"""Recorder lifecycle with simulated OAK and actual bounded telemetry handoff."""

import json
import time
from contextlib import contextmanager
from datetime import timedelta
from types import SimpleNamespace

import numpy as np
import pytest

from test_mavros import PARAMETERS, clock, header, timesync
from wallering_mapping import oak
from wallering_mapping.config import CaptureConfig
from wallering_mapping.dataset import jsonl, sha256_file
from wallering_mapping.mavros import TelemetryBuffer
from wallering_mapping.telemetry_config import TelemetryConfig


def test_camera_failure_stops_callbacks_drains_and_seals_accepted_telemetry(tmp_path, monkeypatch):
    config = TelemetryConfig(min_sync_samples=2)
    buffer = TelemetryBuffer(config, PARAMETERS)
    mono, ros = time.monotonic_ns(), time.time_ns()
    for index in range(2):
        buffer.clock(clock(mono + index * 1000, ros + index * 1000))
        buffer.ingest("timesync", timesync(10**9 + index * 1000), "mavros_msgs/msg/TimesyncStatus",
                      b"cdr", clock(mono + index * 1000, ros + index * 1000), ros)
        for role in ("state", "imu_raw", "pose"):
            buffer.ingest(role, {**header(ros), "connected": True}, "fixture/msg/Message",
                          b"cdr", clock(mono + index * 1000, ros + index * 1000), ros)
    closed = []
    source = SimpleNamespace(buffer=buffer, start=lambda: None, close=lambda: closed.append(True))

    @contextmanager
    def subscriber(_):
        yield source
        source.close()

    class Device:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def getUsbSpeed(self):
            return "super"

    class Pipeline:
        calls = 0
        stopped = False

        def start(self):
            pass

        def isRunning(self):
            self.calls += 1
            return self.calls == 1

        def stop(self):
            self.stopped = True

    class CameraQueue:
        sent = False

        def tryGet(self):
            if self.sent:
                return None
            self.sent = True
            return SimpleNamespace(getCvFrame=lambda: np.zeros((16, 16, 3), np.uint8))

    pipeline = Pipeline()
    dai = SimpleNamespace(Device=Device, UsbSpeed=SimpleNamespace(SUPER="super", SUPER_PLUS="plus"),
                          Clock=SimpleNamespace(now=lambda: timedelta(microseconds=time.monotonic_ns() // 1000)))
    monkeypatch.setattr(oak, "depthai", lambda: dai)
    monkeypatch.setattr(oak, "device_details", lambda *args: {})
    monkeypatch.setattr(oak, "build_pipeline", lambda *args: (pipeline, {"rgb": CameraQueue()}, {}, {}))
    monkeypatch.setattr(oak, "telemetry_subscriber", subscriber)
    monkeypatch.setattr(oak, "frame_metadata", lambda *args: {
        "sequence": 1, "device_ns": 10**9, "host_synced_ns": mono,
        "received_monotonic_ns": mono, "received_utc_ns": ros,
        "camera": {"K": [[20, 0, 8], [0, 20, 8], [0, 0, 1]], "distortion": [0] * 8,
                   "model": "CameraModel.Perspective"}})
    root = tmp_path / "capture"
    with pytest.raises(RuntimeError, match="pipeline stopped"):
        oak.record(root, CaptureConfig(streams=("rgb",), imu="off", warmup_seconds=0, min_free_gib=0),
                   telemetry_config=config)
    manifest = json.loads((root / "manifest.json").read_text())
    assert manifest["status"] == "failed" and manifest["counts"]["rgb"] == 1
    assert manifest["telemetry"]["summary"]["ever_qualified"]
    assert pipeline.stopped and closed
    assert buffer.queue.empty()
    assert len(list(jsonl(root / "telemetry.jsonl"))) == 10
    assert manifest["journals_sha256"]["telemetry"] == sha256_file(root / "telemetry.jsonl")
