"""ROS 2 MAVROS recording contract, independent of optional ROS imports."""

import json
import math
from dataclasses import asdict, dataclass, field


def default_topics():
    return {"state": "/mavros/state", "imu_raw": "/mavros/imu/data_raw",
            "attitude": "/mavros/imu/data", "pose": "/mavros/local_position/pose",
            "gnss": "/mavros/global_position/global", "timesync": "/mavros/timesync_status",
            "time_reference": "/mavros/time_reference"}


@dataclass(frozen=True)
class TelemetryConfig:
    topics: dict = field(default_factory=default_topics)
    time_node: str = "/mavros/uas_1/time"
    required: tuple = ("state", "imu_raw", "pose", "timesync")
    queue_records: int = 4096
    qos_depth: int = 100
    stall_seconds: float = 5
    parameter_timeout_seconds: float = 10
    clock_sample_hz: float = 10
    max_rtt_ms: float = 10
    max_offset_residual_ms: float = 2
    clock_jump_ms: float = 5
    min_sync_samples: int = 501
    max_clock_age_ms: float = 200
    max_sync_age_ms: float = 500
    max_pose_bracket_ms: float = 50
    sdk_sync_budget_ms: float = 1
    max_alignment_budget_ms: float = 5
    max_gnss_age_ms: float = 200

    def __post_init__(self):
        if (not isinstance(self.topics, dict) or set(self.topics) != set(default_topics())
                or any(not isinstance(v, str) or not v.startswith("/") or " " in v for v in self.topics.values())
                or len(set(self.topics.values())) != len(self.topics)):
            raise ValueError("Provide distinct absolute ROS topic names for all telemetry roles")
        if not isinstance(self.time_node, str) or not self.time_node.startswith("/"):
            raise ValueError("time_node must be an absolute MAVROS time-plugin node name")
        if not self.required or set(self.required) - set(self.topics) or len(set(self.required)) != len(self.required):
            raise ValueError("required must contain unique configured telemetry roles")
        if "timesync" not in self.required:
            raise ValueError("Timesync evidence is required for this integration")
        for name, low, high in (("queue_records", 16, 100000), ("qos_depth", 1, 10000),
                               ("stall_seconds", 1, 300), ("parameter_timeout_seconds", 1, 120),
                               ("clock_sample_hz", 1, 100), ("max_rtt_ms", .1, 100),
                               ("max_offset_residual_ms", .01, 100), ("clock_jump_ms", .1, 100),
                               ("min_sync_samples", 2, 100000), ("max_clock_age_ms", 10, 5000),
                               ("max_sync_age_ms", 10, 5000), ("max_pose_bracket_ms", 1, 1000),
                               ("sdk_sync_budget_ms", .001, 100),
                               ("max_alignment_budget_ms", .1, 100), ("max_gnss_age_ms", 1, 5000)):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{name} must be finite in [{low}, {high}]")
        for name in ("queue_records", "qos_depth", "min_sync_samples"):
            if type(getattr(self, name)) is not int:
                raise ValueError(f"{name} must be an integer")

    def to_dict(self):
        return asdict(self)

    @classmethod
    def read(cls, path):
        values = json.loads(path.read_text())
        if "required" in values:
            values["required"] = tuple(values["required"])
        return cls(**values)
