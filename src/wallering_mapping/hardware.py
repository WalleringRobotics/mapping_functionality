"""Bounded startup checks. Importable even when optional binary modules are broken."""

import json
import os
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from .operations import provenance


def command(argv, timeout=15):
    result = subprocess.run(argv, capture_output=True, text=True, timeout=timeout,
                            env={**os.environ, "QT_QPA_PLATFORM": "offscreen"})
    if result.returncode:
        raise RuntimeError(f"{argv[0]} exited {result.returncode}: "
                           f"{(result.stderr or result.stdout)[-3000:].strip()}")
    return result.stdout


def probe(kind, payload, timeout):
    # Native SDKs can crash or hang. Keep those failures outside the report process.
    output = command([sys.executable, "-m", "wallering_mapping.hardware_probe",
                      kind, json.dumps(payload)], timeout)
    lines = [line.removeprefix("WR_HARDWARE_RESULT=") for line in output.splitlines()
             if line.startswith("WR_HARDWARE_RESULT=")]
    if len(lines) != 1:
        raise RuntimeError("Probe did not return a hardware result")
    result = json.loads(lines[0])
    if not result["ok"]:
        raise RuntimeError(result["detail"])
    return result["detail"]


def storage(root, reserve_gib, required_mount=None):
    root = root.resolve(strict=True)
    if not root.is_dir():
        raise ValueError("Output root must be an existing directory")
    if required_mount is not None:
        mount = required_mount.resolve(strict=True)
        if not mount.is_dir() or not os.path.ismount(mount):
            raise ValueError(f"Required storage mount is absent: {mount}")
        if not root.is_relative_to(mount) or root.stat().st_dev != mount.stat().st_dev:
            raise ValueError("Output root must reside on the required mount (no symlink fallback)")
    usage = shutil.disk_usage(root)
    if usage.free < reserve_gib * 1024**3:
        raise ValueError(f"Free storage {usage.free / 1024**3:.2f} GiB < reserve {reserve_gib} GiB")
    # Test the actual service user's create/write/fsync/read permissions, including ACLs.
    with tempfile.TemporaryFile(dir=root) as handle:
        handle.write(b"wallering mapping storage probe\n")
        handle.flush()
        os.fsync(handle.fileno())
        handle.seek(0)
        if handle.read() != b"wallering mapping storage probe\n":
            raise RuntimeError("Storage readback mismatch")
    return {"root": str(root), "free_bytes": usage.free,
            "required_mount": str(required_mount) if required_mount else None}


def jetson():
    model_path = Path("/proc/device-tree/model")
    if not model_path.exists():
        # Docker masks /sys/firmware even when a nested path is bind-mounted.
        model_path = Path("/run/wr-mapping/device-tree/model")
    model = model_path.read_text().rstrip("\0\n")
    release = Path("/etc/nv_tegra_release").read_text().strip()
    os_release = platform.freedesktop_os_release()
    if platform.machine() != "aarch64" or "Orin Nano" not in model:
        raise ValueError(f"Expected an aarch64 Orin Nano; found {platform.machine()}/{model}")
    if not re.match(r"# R36\b", release) or os_release.get("VERSION_ID") != "22.04":
        raise ValueError(f"Expected Jetson Linux R36 / Ubuntu 22.04; found {release}/{os_release}")
    return {"model": model, "release": release, "os": os_release["PRETTY_NAME"]}


def memory(minimum_gib):
    values = dict(line.split(":", 1) for line in Path("/proc/meminfo").read_text().splitlines())
    available = int(values["MemAvailable"].split()[0]) * 1024
    if available < minimum_gib * 1024**3:
        raise ValueError(f"Available RAM {available / 1024**3:.2f} GiB < {minimum_gib} GiB")
    return {"available_bytes": available, "minimum_gib": minimum_gib}


def thermals(root=Path("/sys/class/thermal")):
    readings = []
    for zone in sorted(root.glob("thermal_zone*")):
        name = (zone / "type").read_text().strip()
        # Power-gated Jetson CV engines legitimately return EAGAIN, which Python's
        # buffered sysfs reader can expose as a TypeError. os.read preserves errno.
        descriptor = os.open(zone / "temp", os.O_RDONLY)
        try:
            try:
                temperature = int(os.read(descriptor, 128)) / 1000
            except BlockingIOError:
                readings.append({"zone": name, "unavailable": "power-gated (EAGAIN)"})
                continue
        finally:
            os.close(descriptor)
        entry = {"zone": name, "celsius": temperature}
        for kind in zone.glob("trip_point_*_type"):
            if kind.read_text().strip() in {"hot", "critical"}:
                limit = int(kind.with_name(kind.name.replace("_type", "_temp")).read_text()) / 1000
                if entry["celsius"] >= limit:
                    raise ValueError(f"{entry['zone']} reached {entry['celsius']} C / {limit} C trip")
        readings.append(entry)
    if not any("celsius" in entry for entry in readings):
        raise ValueError("All thermal telemetry unavailable")
    return readings


def hardware_check(mode, output_root, config=None, *, require_jetson=False,
                   required_mount=None, telemetry_config=None, gnss_profile=None,
                   device_id=None, camera_seconds=5, telemetry_seconds=75,
                   min_memory_gib=1, serial_device=None):
    checks = []

    def run(name, action, remedy, required=True):
        try:
            detail = action()
            status = "pass"
        except (OSError, ValueError, RuntimeError, ImportError, KeyError, TypeError,
                subprocess.SubprocessError) as error:
            status, detail = ("fail" if required else "warn"), str(error)
        checks.append({"name": name, "status": status, "detail": detail,
                       "remedy": remedy if status != "pass" else None})
        return status == "pass"

    if require_jetson:
        run("jetson", jetson, "Use the commissioned Orin Nano / Jetson Linux R36 deployment.")
        run("power_mode", lambda: command(["nvpmodel", "-q"]).strip(),
            "Make nvpmodel query available; qualify performance in the deployed power mode.", False)
        run("thermals", thermals, "Check cooling and thermal telemetry.")
    run("memory", lambda: memory(min_memory_gib), "Free RAM or reduce competing workloads.")
    writable = run("storage", lambda: storage(output_root, config.min_free_gib if config else 5,
                                              required_mount),
                   "Mount the intended disk, create the output root, and grant the capture user access.")
    run("python", lambda: {"version": platform.python_version(), "executable": sys.executable}, "")
    packages = {"numpy": "1.26.4", "opencv-python-headless": "4.11.0.86"}
    if mode == "capture":
        packages["depthai"] = "3.10.0"
    elif mode == "building":
        packages["pycolmap"] = "3.12.6"
    elif mode == "terrain":
        packages["pyproj"] = "3.7.1"
    dependencies = run("binary_dependencies", lambda: probe("dependencies", packages, 30),
                       "Install the pinned extras in this interpreter. ARM64 pycolmap may require a source build.")
    if serial_device:
        def serial_access():
            import stat
            if not stat.S_ISCHR(serial_device.stat().st_mode):
                raise ValueError("Serial path is not a character device")
            if not os.access(serial_device, os.R_OK | os.W_OK):
                raise ValueError(f"No read/write access to {serial_device}")
            return {"path": str(serial_device), "target": str(serial_device.resolve()),
                    "note": "Access checked only; UART is owned by MAVROS."}
        run("serial_access", serial_access, "Grant the MAVROS user dialout access; check USB passthrough.")
    if mode == "capture":
        if dependencies and writable:
            # The parent owns the temporary directory so a timed-out native SDK cannot leak it.
            with tempfile.TemporaryDirectory(prefix=".wr-hardware-", dir=output_root) as temporary:
                duration = max(camera_seconds, 3 / config.fps)
                run("oak_capture", lambda: probe("camera", {
                    "config": config.to_dict(), "root": str(Path(temporary) / "capture"),
                    "device_id": device_id, "duration": duration},
                    config.warmup_seconds + duration + 45),
                    "Check OAK ownership, USB rules/cable, configured sensors, exposure/focus, and IMU.")
        else:
            checks.append({"name": "oak_capture", "status": "fail",
                           "detail": "Camera probe blocked by dependencies/storage", "remedy": "Fix checks above."})
        if telemetry_config:
            run("mavros", lambda: probe("telemetry", {
                "config": telemetry_config.to_dict(), "seconds": telemetry_seconds,
                "gnss_profile": str(gnss_profile) if gnss_profile else None},
                telemetry_seconds + 2 * telemetry_config.parameter_timeout_seconds + 15),
                "Source ROS Humble and the platform overlay, match ROS_DOMAIN_ID, and start the existing MAVROS link. "
                "Verify configured topics/time_node and RTK evidence; allow time for a fresh sync window.")
        else:
            checks.append({"name": "mavros", "status": "skip", "detail": "No telemetry config requested"})
        if gnss_profile and not telemetry_config:
            checks.append({"name": "gnss_profile", "status": "fail",
                           "detail": "GNSS checks require --telemetry-config"})
    elif mode == "building":
        run("cuda", lambda: probe("cuda", {}, 30), "Install a matching NVIDIA driver/runtime; allow GPU access.")
        def colmap():
            output = command(["colmap", "-h"])
            if not re.search(r"COLMAP\s+3\.12(?:\.|\s)", output):
                raise ValueError("Expected COLMAP 3.12.x")
            if not re.search(r"\bwith CUDA\b", output, re.IGNORECASE):
                raise ValueError("COLMAP does not advertise a CUDA build; dense reconstruction requires it")
            return output[:1000]
        run("colmap", colmap, "Build/install COLMAP 3.12.x with CUDA for the processing host.")
    elif mode == "terrain":
        run("odm", lambda: probe("odm", {}, 60),
            "Install Docker and a native-architecture ODM 3.6.2 image, or process on a supported workstation.")
    return {"schema_version": 1, "ready": all(c["status"] != "fail" for c in checks),
            "mode": mode, "sampled_utc_ns": time.time_ns(), "checks": checks,
            "provenance": provenance(), "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
            "limitations": "Startup smoke checks only. Sustained throughput, exposure timing, rig calibration, "
                            "receiver uncertainty semantics and survey accuracy require bench/field evidence."}
