"""Versioned processing recipes shared by the two product backends."""

import json
import math
import re
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class ProcessConfig:
    product: str = "building"
    stream: str = "rgb"
    interval_seconds: float = 0.5
    min_sharpness: float = 0
    allow_gaps: bool = False
    matcher: str = "exhaustive"
    cpu: bool = False
    dense: bool = True
    mesh: bool = True
    max_image_size: int = 2000
    model_index: int | None = None
    min_registered_fraction: float = 0.9
    min_sparse_points: int = 100
    odm_image: str = "opendronemap/odm:3.6.2"
    orthophoto_cm: float = 2.0
    dem_cm: float = 5.0
    dtm: bool = False
    max_concurrency: int = 4

    def __post_init__(self):
        if self.product not in {"building", "terrain"} or self.stream not in {"rgb", "left", "right"}:
            raise ValueError("product must be building/terrain; stream must be rgb/left/right")
        if self.matcher not in {"exhaustive", "sequential"}:
            raise ValueError("Unknown matcher")
        for key in ("allow_gaps", "cpu", "dense", "mesh", "dtm"):
            if type(getattr(self, key)) is not bool:
                raise ValueError(f"{key} must be a boolean")
        for key, low, high in (
            ("interval_seconds", 0, 3600), ("min_sharpness", 0, 1e12),
            ("max_image_size", 128, 20000), ("min_registered_fraction", 0, 1),
            ("min_sparse_points", 1, 10000000), ("orthophoto_cm", 0.1, 1000),
            ("dem_cm", 0.1, 1000), ("max_concurrency", 1, 256),
        ):
            value = getattr(self, key)
            if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{key} must be finite in [{low}, {high}]")
        for key in ("max_image_size", "min_sparse_points", "max_concurrency"):
            if type(getattr(self, key)) is not int:
                raise ValueError(f"{key} must be an integer")
        if self.model_index is not None and (type(self.model_index) is not int or self.model_index < 0):
            raise ValueError("model_index must be null or a nonnegative integer")
        if self.mesh and not self.dense and self.product == "building":
            raise ValueError("Building meshing requires dense=true")
        if not isinstance(self.odm_image, str) or not self.odm_image.startswith("opendronemap/odm"):
            raise ValueError("odm_image must identify the official opendronemap/odm image")
        if self.odm_image != "opendronemap/odm:3.6.2" and not re.fullmatch(r"opendronemap/odm@sha256:[0-9a-f]{64}", self.odm_image):
            raise ValueError("Use ODM 3.6.2 or its verified digest; mutable latest tags are not supported")

    def to_dict(self):
        return asdict(self)

    @classmethod
    def read(cls, path):
        return cls(**json.loads(path.read_text()))

