"""Operator-facing preflight, provenance and live status."""

import importlib.metadata
import json
import os
import platform
import shutil
import subprocess
import time
from pathlib import Path


def provenance():
    versions = {}
    for name in ("wallering-mapping", "depthai", "numpy", "opencv-python-headless"):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    repo = Path(__file__).resolve().parents[2]
    revision = None
    dirty = None
    if (repo / ".git").exists():
        try:
            revision = subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"],
                                               text=True, stderr=subprocess.DEVNULL).strip()
            dirty = bool(subprocess.check_output(["git", "-C", str(repo), "status", "--porcelain"],
                                                 text=True, stderr=subprocess.DEVNULL).strip())
        except (OSError, subprocess.CalledProcessError):
            pass
    l4t = Path("/etc/nv_tegra_release")
    return {"versions": versions, "git_commit": revision, "git_dirty": dirty,
            "machine": platform.machine(), "kernel": platform.release(),
            "jetson_release": l4t.read_text().strip() if l4t.is_file() else None}


def host_health(root):
    result = {"free_bytes": shutil.disk_usage(root).free,
              "load_average": list(os.getloadavg()), "sampled_utc_ns": time.time_ns()}
    meminfo = Path("/proc/meminfo")
    if meminfo.exists():
        for line in meminfo.read_text().splitlines():
            if line.startswith("MemAvailable:"):
                result["available_memory_bytes"] = int(line.split()[1]) * 1024
    return result


def status(root):
    manifest = json.loads((root / "manifest.json").read_text())
    live = root / "status.json"
    result = json.loads(live.read_text()) if live.exists() else {
        "updated_utc_ns": manifest["started_utc_ns"], "counts": {}, "phase": "starting"}
    age = max(0, (time.time_ns() - result["updated_utc_ns"]) / 1e9)
    result.update(session=str(root), status=manifest["status"], age_seconds=age,
                  stale=manifest["status"] == "recording" and age > 15,
                  stop_reason=manifest.get("stop_reason"))
    return result


def doctor(mode, output_root, config=None, probe=False, device_id=None, backend="building"):
    checks = []

    def check(name, ok, detail):
        checks.append({"name": name, "ok": bool(ok), "detail": detail})

    parent = output_root.resolve()
    while not parent.exists():
        parent = parent.parent
    check("output", parent.is_dir() and os.access(parent, os.W_OK | os.X_OK), str(parent))
    reserve = config.min_free_gib * 1024**3 if config else 0
    free = shutil.disk_usage(parent).free
    check("disk_reserve", free >= reserve, {"free_bytes": free, "reserve_bytes": reserve})
    details = None
    if mode == "capture":
        try:
            from .oak import depthai, inspect_device
            depthai()
            check("depthai", True, "3.10.0")
            if probe:
                details = inspect_device(device_id)
                check("usb3", details["usb_speed"].split(".")[-1] in {"SUPER", "SUPER_PLUS"},
                      details["usb_speed"])
                sockets = {camera["socket"].split(".")[-1] for camera in details["cameras"]}
                required = {"rgb": "CAM_A", "left": "CAM_B", "right": "CAM_C"}
                check("sensors", all(required[s] in sockets for s in config.streams), sorted(sockets))
                if config.imu == "required":
                    check("imu", details["imu_type"].upper() not in {"", "NONE", "UNKNOWN"},
                          details["imu_type"])
        except (ImportError, RuntimeError, OSError) as error:
            check("camera_dependency_or_probe", False, str(error))
    else:
        engine = "colmap" if backend == "building" else "docker"
        check(engine, shutil.which(engine) is not None, shutil.which(engine))
        if backend == "terrain":
            try:
                import pyproj
                check("pyproj", True, pyproj.__version__)
            except ImportError:
                check("pyproj", False, "Install .[terrain]")
    return {"ready": all(item["ok"] for item in checks), "mode": mode,
            "checks": checks, "device": details, "provenance": provenance(),
            "note": "Readiness checks are not a throughput or accuracy qualification."}

