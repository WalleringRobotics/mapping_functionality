from dataclasses import replace

import pytest

from wallering_mapping.telemetry_config import TelemetryConfig
from wallering_mapping.timing import ClockTracker, SyncMonitor, sample_clock


def test_bracket_keeps_independent_clock_epoch_and_step_detection():
    iterator = iter([1000, 1100])
    sample = sample_clock(lambda: 5000000, "ros_system", lambda: next(iterator))
    assert sample["reference_ns"] == 1050 and sample["bracket_ns"] == 100
    tracker = ClockTracker(.1)
    tracker.observe(sample)
    tracker.observe({**sample, "reference_ns": 2050, "target_ns": 5001000})
    with pytest.raises(RuntimeError, match="jump/reset"):
        tracker.observe({**sample, "reference_ns": 3050, "target_ns": 7001000})


def packet(remote, estimated=1700000000000000000, residual=100, rtt=1):
    return {"remote_timestamp_ns": remote, "estimated_offset_ns": estimated,
            "observed_offset_ns": estimated + residual, "round_trip_time_ms": rtt}


def test_qualification_uses_actual_convergence_window_and_resets_on_bad_rtt():
    config = TelemetryConfig(min_sync_samples=2)
    monitor = SyncMonitor(config, {"timesync_mode": "MAVLINK", "convergence_window": 3, "max_rtt_sample": 5})
    for i in range(1, 4):
        assert monitor.observe(packet(i))["qualification"] != "qualified"
    assert monitor.observe(packet(4))["qualification"] == "qualified"
    assert monitor.observe(packet(5, rtt=6))["consecutive_good"] == 0
    assert monitor.observe(packet(6))["qualification"] != "qualified"
    with pytest.raises(RuntimeError, match="PX4 clock reset"):
        monitor.observe(packet(1))


def test_nonzero_offset_is_not_convergence_and_jump_is_not_silently_corrected():
    config = TelemetryConfig(min_sync_samples=2)
    monitor = SyncMonitor(config, {"timesync_mode": "MAVLINK", "convergence_window": 1, "max_rtt_sample": 10})
    assert monitor.observe(packet(1))["qualification"] != "qualified"
    assert monitor.observe(packet(2))["qualification"] == "qualified"
    with pytest.raises(RuntimeError, match="offset jump"):
        monitor.observe(packet(3, estimated=1700000000100000000))
    with pytest.raises(ValueError, match="MAVLINK"):
        SyncMonitor(config, {"timesync_mode": "NONE"})


def test_gate_diagnostics_distinguish_rtt_and_residual_without_relaxing_the_gate():
    monitor = SyncMonitor(TelemetryConfig(min_sync_samples=2),
                          {"timesync_mode": "MAVLINK", "convergence_window": 1, "max_rtt_sample": 10})
    monitor.observe(packet(1))
    assert monitor.observe(packet(2))["qualification"] == "qualified"
    assert monitor.observe(packet(3, rtt=10))["consecutive_good"] == 0
    monitor.observe(packet(4))
    monitor.observe(packet(5, residual=3_000_000))
    monitor.observe(packet(6, rtt=11, residual=3_000_000))
    report = monitor.diagnostics()
    assert report["any_rejected_samples"] == 3
    assert report["rtt_rejected_samples"] == report["residual_rejected_samples"] == 2
    assert report["interrupted_streaks"] == 2
    assert report["longest_interrupted_streak"] == 2
    assert report["qualified_streak_interruptions"] == 1


@pytest.mark.parametrize("changes", [{"max_rtt_ms": float("nan")}, {"queue_records": 1.5},
                                    {"required": ["pose"]}, {"topics": {"pose": "/pose"}}])
def test_strict_telemetry_configuration(changes):
    with pytest.raises(ValueError):
        replace(TelemetryConfig(), **changes)
