"""Disposable native-library probes, called with a deadline by hardware.py."""

import importlib
import importlib.metadata
import json
import math
import sys
import time
from pathlib import Path


def dependencies(packages):
    modules = {"opencv-python-headless": "cv2", "numpy": "numpy", "depthai": "depthai",
               "pycolmap": "pycolmap", "pyproj": "pyproj"}
    for name, expected in packages.items():
        actual = importlib.metadata.version(name)
        if actual != expected:
            raise ValueError(f"{name}: expected {expected}, installed {actual}")
        module = importlib.import_module(modules[name])
        expected_module = ".".join(expected.split(".")[:3]) if name == "opencv-python-headless" else expected
        if module.__version__ != expected_module:
            raise ValueError(f"{name}: imported {module.__version__}, expected {expected_module}; "
                             "check PYTHONPATH and the ROS overlay")
    import cv2
    import numpy as np
    for suffix in (".jpg", ".png"):
        ok, encoded = cv2.imencode(suffix, np.zeros((16, 16, 3), dtype=np.uint8))
        if not ok or cv2.imdecode(encoded, cv2.IMREAD_COLOR) is None:
            raise RuntimeError(f"OpenCV {suffix} codec failed")
    return packages


def camera(payload):
    from .config import CaptureConfig
    from .dataset import jsonl
    from .oak import record
    from .validate import validate
    root = Path(payload["root"])
    config = CaptureConfig(**payload["config"])
    manifest = record(root, config, payload["duration"], payload["device_id"])
    audit = validate(root)
    if not audit["valid"] or audit["warnings"]:
        raise ValueError(f"Short camera capture failed validation: {audit}")
    if any(stream["frames"] < 2 for stream in audit["streams"].values()):
        raise ValueError("Camera probe needs at least two frames per requested stream")
    settings = {}
    for row in jsonl(root / "frames.jsonl"):
        stream = row["stream"]
        actual = {key: row[key] for key in ("width", "height", "exposure_us", "iso",
                                            "lens_position", "white_balance_k")}
        if stream in settings and actual != settings[stream]:
            raise ValueError(f"Camera settings changed during startup on {stream}")
        settings[stream] = actual
        if row["exposure_us"] != config.exposure_us or row["iso"] != config.iso:
            raise ValueError(f"{stream} exposure/ISO readback differs from requested settings: {actual}")
        if stream == "rgb":
            lens = manifest["device"]["stream_settings"][stream]["lens_position_requested"]
            if lens is not None and row["lens_position"] != lens:
                raise ValueError("RGB lens-position readback differs from requested fixed focus")
            if row["white_balance_k"] != config.white_balance_k:
                raise ValueError("RGB white balance differs from requested settings")
    return {"device": manifest["device"], "streams": audit["streams"], "imu": audit["imu"],
            "settings": settings, "duration_seconds": payload["duration"]}


def profile_ready(path):
    from .gnss_accuracy import read_profile
    profile = read_profile(Path(path))
    missing = []
    for section in ("base", "receiver", "rig", "motion"):
        for key, value in profile[section].items():
            if value is None or value == "unknown" or (key.endswith("verified") and value is False):
                if key == "correction_age_field_verified" and not profile["policy"]["require_receiver_correction_age"]:
                    continue
                missing.append(f"{section}.{key}")
    if missing:
        raise ValueError("Incomplete GNSS commissioning profile: " + ", ".join(missing))
    return profile


def telemetry_ready(latest, config, now_ns):
    required = set(config.required) | {"state", "timesync"}
    if any(role not in latest or now_ns - latest[role]["received_monotonic_ns"] >
           config.stall_seconds * 1e9 for role in required):
        return False
    sync = latest["timesync"]
    return (latest["state"]["fields"]["connected"]
            and sync["sync_quality"]["qualification"] == "qualified"
            and now_ns - sync["received_monotonic_ns"] <= config.max_sync_age_ms * 1e6)


def telemetry(payload):
    from .mavros import MavrosSubscriber
    from .telemetry_config import TelemetryConfig
    from .rtcm import RTCMStream, describe
    config = TelemetryConfig(**payload["config"])
    profile = profile_ready(payload["gnss_profile"]) if payload["gnss_profile"] else None
    if profile and not {"gps_raw", "rtcm"} <= set(config.required):
        raise ValueError("GNSS startup requires gps_raw and rtcm in required telemetry topics")
    decoder = RTCMStream()
    rtcm_frames, base_verified, latest_observation = 0, False, None
    latest = {}
    source = MavrosSubscriber(config)
    try:
        source.start()
        deadline = time.monotonic() + payload["seconds"]
        while time.monotonic() < deadline:
            for row in source.buffer.drain():
                role = row.get("role")
                if role:
                    latest[role] = row
                if role == "rtcm":
                    for frame in decoder.feed(bytes(row["fields"]["data"])):
                        info = describe(frame)
                        rtcm_frames += 1
                        if profile:
                            from .gnss_accuracy import ecef_and_enu
                            base = profile["base"]
                            if "station_id" in info and info["station_id"] != base["station_id"]:
                                raise ValueError("RTCM station differs from commissioned base")
                            if "base_arp_ecef_m" in info:
                                expected = ecef_and_enu(base["wgs84_lat_lon_h"])[0]
                                if math.dist(expected, info["base_arp_ecef_m"]) > base["rtcm_coordinate_tolerance_m"]:
                                    raise ValueError("RTCM coordinates differ from surveyed base ARP")
                                base_verified = True
                        kind = info["message_type"]
                        if (1001 <= kind <= 1004 or 1009 <= kind <= 1012
                                or any(first <= kind <= first + 6 for first in (1071, 1081, 1091, 1101, 1111, 1121))):
                            latest_observation = row["received_monotonic_ns"]
            now = time.monotonic_ns()
            good = telemetry_ready(latest, config, now)
            if "rtcm" in config.required:
                good &= (latest_observation is not None
                         and now - latest_observation <= config.stall_seconds * 1e9)
            if "gps_raw" in config.required:
                fields = latest.get("gps_raw", {}).get("fields", {})
                good &= fields.get("fix_type") == 6
                good &= all(0 < fields.get(key, 0) < 2**32 - 1 for key in ("h_acc", "v_acc"))
            if profile:
                policy = profile["policy"]
                good &= base_verified and latest_observation is not None
                if latest_observation is not None:
                    good &= now - latest_observation <= policy["max_transport_age_seconds"] * 1e9
                gps = latest.get("gps_raw")
                good &= gps is not None and now - gps["received_monotonic_ns"] <= policy["max_gnss_age_ms"] * 1e6
                if policy["require_receiver_correction_age"]:
                    age = gps["fields"].get("dgps_age") if gps else None
                    good &= type(age) is int and 0 <= age <= policy["max_receiver_correction_age_ms"]
            if good:
                return {"parameters": source.buffer.parameters, "summary": source.buffer.summary(),
                        "rtcm_frames": rtcm_frames, "surveyed_base_verified": base_verified,
                        "gps_raw": {key: latest.get("gps_raw", {}).get("fields", {}).get(key)
                                    for key in ("fix_type", "h_acc", "v_acc", "dgps_age")},
                        "note": "Receipt freshness and live evidence only; run sync/image-accuracy on the survey."}
            time.sleep(.02)
        raise RuntimeError(f"No fresh connected/timing-qualified interval within {payload['seconds']}s; "
                           f"{source.buffer.summary()}; RTK base verified={base_verified}, RTCM frames={rtcm_frames}")
    finally:
        source.close()


def cuda(_):
    import ctypes as c
    driver = c.CDLL("libcuda.so.1")

    def call(name, *args):
        code = getattr(driver, name)(*args)
        if code:
            raise RuntimeError(f"{name} failed with CUDA status {code}")
    call("cuInit", 0)
    count, device = c.c_int(), c.c_int()
    call("cuDeviceGetCount", c.byref(count))
    if count.value < 1:
        raise RuntimeError("No CUDA devices visible")
    call("cuDeviceGet", c.byref(device), 0)
    name, major, minor = c.create_string_buffer(256), c.c_int(), c.c_int()
    call("cuDeviceGetName", name, len(name), device)
    call("cuDeviceComputeCapability", c.byref(major), c.byref(minor), device)
    context, pointer = c.c_void_p(), c.c_uint64()
    call("cuDevicePrimaryCtxRetain", c.byref(context), device)
    try:
        call("cuCtxSetCurrent", context)
        call("cuMemAlloc_v2", c.byref(pointer), c.c_size_t(4))
        try:
            value, received = c.c_int(42), c.c_int()
            call("cuMemcpyHtoD_v2", pointer, c.byref(value), c.c_size_t(4))
            call("cuMemcpyDtoH_v2", c.byref(received), pointer, c.c_size_t(4))
            if received.value != value.value:
                raise RuntimeError("CUDA device memory readback mismatch")
        finally:
            call("cuMemFree_v2", pointer)
    finally:
        call("cuDevicePrimaryCtxRelease_v2", device)
    return {"name": name.value.decode(), "compute_capability": f"{major.value}.{minor.value}",
            "memory_roundtrip": True, "note": "No reconstruction kernel or load test performed."}


def odm(_):
    from .odm import resolve_image
    return resolve_image("opendronemap/odm:3.6.2")


if __name__ == "__main__":
    try:
        detail = {"dependencies": dependencies, "camera": camera, "telemetry": telemetry,
                  "cuda": cuda, "odm": odm}[sys.argv[1]](json.loads(sys.argv[2]))
        result = {"ok": True, "detail": detail}
    except Exception as error:
        result = {"ok": False, "detail": f"{type(error).__name__}: {error}"}
    print("WR_HARDWARE_RESULT=" + json.dumps(result), flush=True)
