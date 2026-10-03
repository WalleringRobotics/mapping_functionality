"""Measured clock-domain bridges and conservative MAVROS timing qualification."""

import math
import time


def sample_clock(clock_ns, target, monotonic_ns=time.monotonic_ns):
    before = monotonic_ns()
    value = clock_ns()
    after = monotonic_ns()
    if type(value) is not int or before > after or value < 0:
        raise ValueError("Invalid bracketed clock observation")
    return {"kind": "clock_bridge", "reference": "python_monotonic", "target": target,
            "reference_ns": (before + after) // 2, "target_ns": value,
            "bracket_ns": after - before}


class ClockTracker:
    def __init__(self, jump_ms):
        self.previous = None
        self.jump_ns = int(jump_ms * 1e6)

    def observe(self, sample):
        if self.previous:
            dm = sample["reference_ns"] - self.previous["reference_ns"]
            dc = sample["target_ns"] - self.previous["target_ns"]
            allowance = (sample["bracket_ns"] + self.previous["bracket_ns"]) // 2
            if dm <= 0 or dc <= 0 or abs(dc - dm) > self.jump_ns + allowance:
                raise RuntimeError(f"{sample['target']} clock jump/reset; start a new session")
        self.previous = sample


class SyncMonitor:
    """Qualification is an evidence gate, not a hardware synchronization guarantee.

    TimesyncStatus lacks the UAS's applied/converged flag. Verify the actual plugin
    parameters, then require a fresh conservative window of accepted observations.
    Never label a status packet, or its nonzero offset, as proof of convergence.
    """

    def __init__(self, config, parameters):
        self.config = config
        if parameters.get("timesync_mode") != "MAVLINK":
            raise ValueError("MAVROS time plugin must use timesync_mode=MAVLINK")
        window = parameters.get("convergence_window")
        max_rtt = parameters.get("max_rtt_sample")
        if type(window) is not int or window < 1 or type(max_rtt) not in (int, float) or max_rtt <= 0:
            raise ValueError("Missing/invalid MAVROS time-plugin parameters")
        self.minimum = max(config.min_sync_samples, window + 1)
        self.max_rtt = min(config.max_rtt_ms, max_rtt)
        self.good = 0
        self.previous = None
        self.ever_qualified = False

    def observe(self, fields):
        remote = fields["remote_timestamp_ns"]
        observed, estimated = fields["observed_offset_ns"], fields["estimated_offset_ns"]
        rtt = fields["round_trip_time_ms"]
        if (any(type(v) is not int for v in (remote, observed, estimated)) or remote <= 0
                or type(rtt) not in (int, float) or not math.isfinite(rtt) or rtt < 0):
            raise ValueError("Invalid MAVROS timesync observation")
        if self.previous:
            if remote <= self.previous["remote_timestamp_ns"]:
                raise RuntimeError("PX4 clock reset or nonmonotonic TIMESYNC; start a new session")
            if self.ever_qualified and abs(estimated - self.previous["estimated_offset_ns"]) > self.config.clock_jump_ms * 1e6:
                raise RuntimeError("MAVROS offset jump; start a new session")
        residual = abs(observed - estimated)
        good = rtt < self.max_rtt and residual <= self.config.max_offset_residual_ms * 1e6
        self.good = self.good + 1 if good else 0
        qualified = self.good >= self.minimum
        self.ever_qualified |= qualified
        self.previous = fields
        return {"qualification": "qualified" if qualified else "warming_or_degraded",
                "consecutive_good": self.good, "required_good": self.minimum,
                "offset_residual_ns": residual,
                "timesync_budget_ns": int(rtt * 1e6 / 2) + residual,
                "meaning": "Conservative observation gate; applied UAS offset is not exposed"}
