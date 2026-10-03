import json
import time

import pytest

from wallering_mapping.config import CaptureConfig
from wallering_mapping.dataset import Session
from wallering_mapping.operations import doctor, status


def test_live_status_and_finished_status(tmp_path):
    session = Session(tmp_path / "run", CaptureConfig(min_free_gib=0), "test", {}, {})
    session.heartbeat("recording", {"free_bytes": 42}, 1)
    assert not status(session.root)["stale"]
    path = session.root / "status.json"
    data = json.loads(path.read_text())
    data["updated_utc_ns"] = time.time_ns() - 30_000_000_000
    path.write_text(json.dumps(data))
    assert status(session.root)["stale"]
    session.finish("failed", "test")
    assert status(session.root)["status"] == "failed"
    assert not status(session.root)["stale"]


def test_imu_reset_rejected_and_gap_counted(tmp_path):
    session = Session(tmp_path / "run", CaptureConfig(min_free_gib=0), "test", {}, {})
    session.imu_batch([{"sensor": "gyroscope", "sequence": 1, "device_ns": 10},
                       {"sensor": "gyroscope", "sequence": 4, "device_ns": 20}])
    assert session.counts["gyroscope_sequence_gaps"] == 2
    with pytest.raises(ValueError, match="Non-monotonic"):
        session.imu_batch([{"sensor": "gyroscope", "sequence": 5, "device_ns": 10}])
    session.finish("failed", "test")


def test_doctor_process_reports_missing_engine(tmp_path, monkeypatch):
    monkeypatch.setattr("wallering_mapping.operations.shutil.which", lambda _: None)
    result = doctor("process", tmp_path)
    assert not result["ready"]
    assert any(c["name"] == "colmap" and not c["ok"] for c in result["checks"])
    assert "versions" in result["provenance"]
