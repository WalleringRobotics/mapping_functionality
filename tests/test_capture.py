import json
from datetime import timedelta

import numpy as np
import pytest

from wallering_mapping.config import CaptureConfig
from wallering_mapping.dataset import AsyncWriter, Session, jsonl, safe_path
from wallering_mapping.oak import nanoseconds
from wallering_mapping.simulate import simulate


def test_simulation_roundtrip(tmp_path):
    root = tmp_path / "capture"
    result = simulate(root, 4)
    assert result["status"] == "complete"
    assert result["counts"] == {"rgb": 4, "left": 4, "right": 4}
    assert len(list(jsonl(root / "frames.jsonl"))) == 12
    assert not list(root.rglob("*.partial"))
    with pytest.raises(FileExistsError):
        simulate(root, 2)


def test_timestamp_uses_integer_arithmetic():
    assert nanoseconds(timedelta(days=800, seconds=2, microseconds=3)) == 69120002000003000


@pytest.mark.parametrize("kwargs", [{"fps": 0}, {"fps": float("nan")},
                                    {"streams": ["rgb", "rgb"]}, {"imu": "yes"},
                                    {"queue_frames": 1.5}, {"lens_position": -1}])
def test_invalid_configuration(kwargs):
    with pytest.raises(ValueError):
        CaptureConfig(**kwargs)


def test_unknown_config_key(tmp_path):
    file = tmp_path / "config.json"
    file.write_text('{"fpps": 2}')
    with pytest.raises(TypeError):
        CaptureConfig.read(file)


def test_gap_and_reset(tmp_path):
    session = Session(tmp_path / "session", CaptureConfig(min_free_gib=0), "test", {}, {})
    image = np.zeros((32, 32), np.uint8)
    session.frame("left", image, {"sequence": 10, "device_ns": 10})
    session.frame("left", image, {"sequence": 13, "device_ns": 13})
    with pytest.raises(ValueError, match="Non-monotonic"):
        session.frame("left", image, {"sequence": 1, "device_ns": 1})
    session.finish("failed", "test reset")
    assert session.counts["left_sequence_gaps"] == 2
    assert list(jsonl(session.root / "events.jsonl"))[0]["missing"] == 2


def test_worker_failure_reaches_caller(tmp_path):
    session = Session(tmp_path / "session", CaptureConfig(min_free_gib=0), "test", {}, {})
    writer = AsyncWriter(session)
    writer.submit("does_not_exist")
    writer.thread.join(timeout=2)
    with pytest.raises(RuntimeError, match="writer failed"):
        writer.close()
    session.finish("failed", "expected")
    assert json.loads((session.root / "manifest.json").read_text())["status"] == "failed"


def test_path_escape(tmp_path):
    with pytest.raises(ValueError):
        safe_path(tmp_path, "../outside.png")


def test_disk_reserve_checked_before_session_creation(tmp_path, monkeypatch):
    import shutil
    from collections import namedtuple
    usage = namedtuple("usage", "total used free")
    monkeypatch.setattr(shutil, "disk_usage", lambda _: usage(100, 99, 1))
    with pytest.raises(OSError, match="reserve"):
        Session(tmp_path / "session", CaptureConfig(), "test", {}, {})
    assert not (tmp_path / "session").exists()
