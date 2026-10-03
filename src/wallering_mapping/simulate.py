"""Deterministic IO fixture, explicitly not a physically consistent SfM dataset."""

import time

import cv2
import numpy as np

from .config import CaptureConfig
from .dataset import AsyncWriter, Session


def simulate(root, frames=12):
    config = CaptureConfig(min_free_gib=0, warmup_seconds=0, imu="off")
    calibration = {"synthetic": True, "description": "IO fixture, no metric geometry"}
    session = Session(root, config, "synthetic", {"imu_enabled": False}, calibration)
    writer = AsyncWriter(session)
    rng = np.random.default_rng(17)
    texture = rng.integers(0, 256, (480, 640, 3), dtype=np.uint8)
    try:
        for sequence in range(frames):
            for stream in config.streams:
                frame = np.roll(texture, sequence * 4, axis=1)
                if stream != "rgb":
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                metadata = {
                    "sequence": sequence, "device_ns": 1_000_000_000 + sequence * 500_000_000,
                    "host_synced_ns": 2_000_000_000 + sequence * 500_000_000,
                    "received_monotonic_ns": time.monotonic_ns(),
                    "received_utc_ns": time.time_ns(), "exposure_us": 1000,
                    "iso": 400, "lens_position": 130, "white_balance_k": 5000,
                    "camera": {"K": [[500, 0, 320], [0, 500, 240], [0, 0, 1]],
                               "distortion": [0] * 8, "model": "CameraModel.Perspective",
                               "source_width": 640, "source_height": 480,
                               "processing": "synthetic"},
                }
                # Fixture generation is deliberately paced by the consumer.
                writer.check()
                writer.queue.put(("frame", (stream, frame, metadata)), timeout=10)
        writer.close()
        session.finish()
    except BaseException as error:
        try:
            writer.close()
        finally:
            session.finish("failed", str(error))
        raise
    return session.manifest

