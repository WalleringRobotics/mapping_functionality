"""Offline consumer of the versioned survey-wing design handoff.

Design geometry is useful for planning, never measured camera/rig calibration.
The existing OAK profiles and dataset contract deliberately remain independent.
"""

import hashlib
import json
import math
from pathlib import Path
import re

from .dataset import sha256_file, write_json

HANDOFF_SCHEMA = "wallering.uav-mapping-handoff/v1"
FORMATS = {"Bayer8_candidate": 1, "Bayer16_candidate": 2, "RGB8_candidate": 3}


def number(value, name, *, zero=False):
    if (type(value) not in (int, float) or not math.isfinite(value)
            or value < 0 or (not zero and value == 0)):
        raise ValueError(f"{name} must be a finite {'nonnegative' if zero else 'positive'} number")
    return float(value)


def fraction(value, name, *, zero=True):
    value = number(value, name, zero=zero)
    if value >= 1:
        raise ValueError(f"{name} must be below 1")
    return value


def fingerprints(values):
    if not isinstance(values, dict) or not values:
        raise ValueError("Source fingerprints are required")
    for name, digest in values.items():
        if (not name or Path(name).is_absolute() or ".." in Path(name).parts
                or not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest)):
            raise ValueError("Source fingerprints require relative paths and SHA-256")


def derive_handoff(data):
    """Recompute resource/geometry gates from inputs, not producer pass flags."""
    if (data.get("schema") != HANDOFF_SCHEMA or type(data.get("schema_version")) is not int
            or data["schema_version"] != 1 or data.get("platform") != "survey_wing"
            or data.get("status") != "design_proposal" or data.get("survey_ready") is not False):
        raise ValueError("Expected a v1 survey-wing design proposal with survey_ready=false")
    if data.get("consumer", {}).get("repository") != "WalleringRobotics/mapping_functionality":
        raise ValueError("Handoff names a different consumer")
    fingerprints(data.get("source_files"))
    navigation = data["navigation"]
    if (navigation.get("autopilot_family") != "ArduPilot"
            or navigation.get("vehicle_type") != "fixed_wing"
            or navigation.get("flight_gnss_independent") is not True):
        raise ValueError("Wing handoff must identify ArduPilot Plane with independent flight GNSS")
    if not {"UBX-RXM-RAWX", "UBX-RXM-SFRBX", "UBX-TIM-TM2"}.issubset(
            navigation.get("raw_messages_required", [])):
        raise ValueError("RAWX, SFRBX and TIM-TM2 raw evidence must be retained")
    camera = data["hardware"]["camera"]
    if not camera.get("part") or camera.get("calibration_state") != "nominal_design_only":
        raise ValueError("Camera must name its part and nominal_design_only calibration state")
    geometry, capture, mission = camera["geometry"], data["capture"], data["mission"]
    for key in ("pixels_across", "pixels_along"):
        if type(geometry[key]) is not int or geometry[key] <= 0:
            raise ValueError(f"{key} must be a positive integer")
    width, height = geometry["pixels_across"], geometry["pixels_along"]
    pitch = number(geometry["pixel_um"], "pixel_um")
    focal = number(geometry["focal_mm"], "focal_mm")
    fps = number(capture["requested_fps"], "requested_fps")
    maximum = number(geometry["max_fps"], "max_fps")
    if fps > maximum:
        raise ValueError("requested_fps exceeds the selected camera maximum")
    if capture.get("adapter") != "basler_gige_proposed":
        raise ValueError("Unsupported acquisition adapter in v1 handoff")
    mode = capture["pixel_format"]
    if mode not in FORMATS or capture["bytes_per_pixel"] != FORMATS[mode]:
        raise ValueError("Unsupported or inconsistent candidate pixel format/bytes_per_pixel")
    duration = number(mission["survey_duration_s"], "survey_duration_s")
    agl = number(mission["agl_m"], "agl_m")
    exposure = number(mission["exposure_s"], "exposure_s")
    blur = number(mission["blur_limit_px"], "blur_limit_px")
    speed = number(mission["max_ground_speed_m_s"], "max_ground_speed_m_s")
    forward = fraction(mission["forward_overlap"], "forward_overlap")
    side = fraction(mission["side_overlap"], "side_overlap")
    link = number(capture["link_bits_per_s"], "link_bits_per_s")
    utilisation = fraction(capture["link_utilisation_limit"], "link_utilisation_limit", zero=False)
    margin = number(capture["capacity_margin_fraction"], "capacity_margin_fraction", zero=True)
    window = duration + sum(number(capture[k], k, zero=True) for k in ("warmup_s", "postroll_s"))
    reserve = number(capture["free_space_reserve_bytes"], "free_space_reserve_bytes", zero=True)
    frame_bytes = width * height * FORMATS[mode]
    rate = frame_bytes * fps
    write_rate = rate * (1 + margin)
    capacity = math.ceil(window * write_rate + reserve)
    available, sustained = capture["available_storage_bytes"], capture["sustained_write_bytes_per_s"]
    for name, value in (("available_storage_bytes", available),
                        ("sustained_write_bytes_per_s", sustained)):
        if value is not None:
            number(value, name, zero=True)
    gsd = pitch * 1e-3 * agl / focal
    trigger_spacing = height * gsd * (1 - forward)
    resources = {
        "raw_bytes_per_frame": frame_bytes, "raw_bytes_per_s": rate,
        "minimum_sustained_write_bytes_per_s": write_rate,
        "minimum_free_storage_bytes": capacity,
        "link_budget_pass": rate <= link / 8 * utilisation,
        "storage_capacity_pass": None if available is None else available >= capacity,
        "storage_throughput_pass": None if sustained is None else sustained >= write_rate,
    }
    calculated = {
        "gsd_m_per_px": gsd, "required_trigger_hz": speed / trigger_spacing,
        "forward_motion_blur_px": speed * exposure / gsd,
        "frame_rate_pass": speed / trigger_spacing <= fps,
        "blur_pass": speed * exposure / gsd <= blur * (1 + 1e-12),
    }
    # Tolerate JSON floating-point roundoff, never stale producer acceptance flags.
    for section, expected in (("resources", resources), ("geometry", calculated)):
        for key, value in expected.items():
            stored = data[section].get(key)
            equal = (stored is value if value is None or type(value) is bool else
                     type(stored) in (int, float) and math.isclose(stored, value, rel_tol=1e-9))
            if not equal:
                raise ValueError(f"Stale handoff {section}.{key}; regenerate from its source inputs")
    return {"resources": resources, "geometry": calculated,
            "max_camera_groundspeed_m_s": min(blur * gsd / exposure, trigger_spacing * fps),
            "line_spacing_m": width * gsd * (1 - side)}


def load_handoff(path):
    data = json.loads(Path(path).read_text())
    try:
        derived = derive_handoff(data)
    except (KeyError, TypeError, AttributeError) as error:
        raise ValueError(f"Incomplete handoff: {error}") from error
    return data, derived


def camera_definition(data):
    """Export actual selected dimensions and requested FPS as nominal planner inputs."""
    camera = data["hardware"]["camera"]
    g = camera["geometry"]
    return {"schema_version": 1, "name": camera["part"], "stream": "rgb",
            "profile_resolution": f"{g['pixels_across']}x{g['pixels_along']}",
            "qgc_custom_camera": {
                "SensorWidth": g["pixels_across"] * g["pixel_um"] / 1000,
                "SensorHeight": g["pixels_along"] * g["pixel_um"] / 1000,
                "ImageWidth": g["pixels_across"], "ImageHeight": g["pixels_along"],
                "FocalLength": g["focal_mm"], "Landscape": True, "FixedOrientation": True,
                "MinTriggerInterval": 1 / data["capture"]["requested_fps"]},
            "calibration_state": "nominal_design_only", "survey_ready": False,
            "limitations": ["Planner definition; not measured intrinsics or optical-centre calibration"]}


def check_handoff(path, output):
    original = Path(path).read_bytes()
    data, derived = load_handoff(path)
    original_hash = hashlib.sha256(original).hexdigest()
    if json.loads(original) != data:
        raise ValueError("Handoff changed during checking")
    blockers = [f"{key}: {'unmeasured' if value is None else 'failed'}"
                for key, value in {**derived["resources"], **derived["geometry"]}.items()
                if key.endswith("_pass") and value is not True]
    report = {"schema_version": 1, "kind": "capability_check", "inputs_valid": True,
              "passed": not blockers, "handoff_sha256": original_hash,
              "source_files": data["source_files"], **derived, "blocking_reasons": blockers,
              "capture_ready": False, "flight_ready": False, "survey_ready": False,
              "limitations": [
                  "Design duration is a requested recording window, not achieved aircraft endurance",
                  "Candidate Bayer mode does not specify the actual sensor mosaic or effective settings",
                  "GigE acquisition, event timing, measured calibration and loaded soak remain pending"]}
    output = Path(output)
    # Read before creating output and check its fingerprint against what was validated.
    if sha256_file(Path(path)) != original_hash:
        raise ValueError("Handoff changed during checking")
    output.mkdir(parents=True, exist_ok=False)
    (output / "mapping_handoff.json").write_bytes(original)
    write_json(output / "camera.json", camera_definition(data))
    write_json(output / "capability-check.json", report)
    return report
