"""Strict, portable JSON configuration; no camera dependency."""

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path


@dataclass(frozen=True)
class CaptureConfig:
    streams: tuple[str, ...] = ("rgb", "left", "right")
    fps: float = 2.0
    exposure_us: int = 1000
    iso: int = 400
    white_balance_k: int = 5000
    lens_position: int | None = None
    rgb_format: str = "jpg"
    jpeg_quality: int = 97
    imu: str = "auto"
    imu_hz: int = 200
    queue_frames: int = 12
    min_free_gib: float = 5.0
    stall_seconds: float = 10.0
    warmup_seconds: float = 3.0
    fsync_every: int = 10

    def __post_init__(self):
        if (not self.streams or len(set(self.streams)) != len(self.streams)
                or set(self.streams) - {"rgb", "left", "right"}):
            raise ValueError("streams must be unique members of rgb, left, right")
        for key, low, high in (
            ("fps", 0.1, 30), ("exposure_us", 1, 33000), ("iso", 100, 1600),
            ("white_balance_k", 1000, 12000), ("jpeg_quality", 1, 100),
            ("imu_hz", 25, 400), ("queue_frames", 1, 256),
            ("min_free_gib", 0, 100000), ("stall_seconds", 1, 300),
            ("warmup_seconds", 0, 120), ("fsync_every", 1, 1000),
        ):
            value = getattr(self, key)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{key} must be numeric")
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{key} must be in [{low}, {high}]")
        for key in ("exposure_us", "iso", "white_balance_k", "jpeg_quality", "imu_hz",
                    "queue_frames", "fsync_every"):
            if not isinstance(getattr(self, key), int):
                raise ValueError(f"{key} must be an integer")
        if self.lens_position is not None and (
            type(self.lens_position) is not int or not 0 <= self.lens_position <= 255
        ):
            raise ValueError("lens_position must be null or integer 0..255")
        if self.rgb_format not in {"jpg", "png"} or self.imu not in {"auto", "required", "off"}:
            raise ValueError("rgb_format must be jpg/png; imu must be auto/required/off")
        if self.stall_seconds < 2 / self.fps:
            raise ValueError("stall_seconds must allow at least two frame periods")

    def to_dict(self):
        return asdict(self)

    @classmethod
    def read(cls, path: Path):
        values = json.loads(path.read_text())
        if "streams" in values:
            values["streams"] = tuple(values["streams"])
        return cls(**values)

