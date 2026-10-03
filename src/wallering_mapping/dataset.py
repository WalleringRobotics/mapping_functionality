"""Append-only records, atomic image publication and bounded background writing."""

import hashlib
import json
import os
import platform
import queue
import shutil
import threading
import time
from collections import Counter
from pathlib import Path

import cv2

from . import __version__


def atomic_bytes(path: Path, data: bytes, durable=True):
    temporary = path.with_name(path.name + ".partial")
    with temporary.open("xb") as file:
        file.write(data)
        if durable:
            file.flush()
            os.fsync(file.fileno())
    os.replace(temporary, path)
    if durable:
        fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)


def write_json(path: Path, value):
    atomic_bytes(path, (json.dumps(value, indent=2, allow_nan=False) + "\n").encode())


def sha256_file(path: Path):
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for block in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def safe_path(root: Path, relative: str):
    path = (root / relative).resolve()
    if not path.is_relative_to(root.resolve()) or Path(relative).is_absolute():
        raise ValueError(f"Path escapes dataset: {relative}")
    return path


def jsonl(path: Path):
    with path.open() as file:
        for line_no, line in enumerate(file, 1):
            try:
                yield json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(f"{path.name}:{line_no}: incomplete/invalid JSON") from error


class Session:
    def __init__(self, root: Path, config, source: str, device: dict, calibration: dict):
        self.root = root
        self.config = config
        root.mkdir(parents=True, exist_ok=False)
        for stream in config.streams:
            (root / "images" / stream).mkdir(parents=True)
        write_json(root / "calibration.json", calibration)
        self.manifest = {
            "schema_version": 1, "software_version": __version__, "source": source,
            "status": "recording", "started_utc_ns": time.time_ns(),
            "host": {"platform": platform.platform(), "python": platform.python_version()},
            "config": config.to_dict(), "device": device,
            "calibration_sha256": sha256_file(root / "calibration.json"),
            "time_semantics": {
                "device_ns": "OAK monotonic clock; image exposure MIDDLE API offset",
                "host_synced_ns": "DepthAI host steady clock; not UNIX UTC",
                "received_monotonic_ns": "Python monotonic receipt; not exposure",
                "received_utc_ns": "host wall clock at receipt; not GNSS exposure time",
                "precision": "ns units; Python timedelta resolution is microseconds",
            },
        }
        write_json(root / "manifest.json", self.manifest)
        self.logs = {name: (root / f"{name}.jsonl").open("x")
                     for name in ("frames", "imu", "events", "clock")}
        self.counts = Counter()
        self.sequence = {}
        self.timestamps = {}
        self.pending = 0
        self.check_disk()

    def check_disk(self):
        if shutil.disk_usage(self.root).free < self.config.min_free_gib * 1024**3:
            raise OSError("Free disk space is below min_free_gib reserve")

    def append(self, kind, record):
        self.logs[kind].write(json.dumps(record, allow_nan=False) + "\n")
        self.pending += 1
        if self.pending >= self.config.fsync_every:
            self.flush()

    def flush(self):
        for file in self.logs.values():
            file.flush()
            os.fsync(file.fileno())
        self.pending = 0

    def event(self, kind, **fields):
        self.append("events", {"kind": kind, "utc_ns": time.time_ns(), **fields})

    def frame(self, stream, array, metadata):
        self.check_disk()
        sequence = metadata["sequence"]
        timestamp = metadata["device_ns"]
        if stream in self.sequence:
            if sequence <= self.sequence[stream] or timestamp <= self.timestamps[stream]:
                raise ValueError(f"Non-monotonic frame sequence/time on {stream}; start a new session")
            missing = sequence - self.sequence[stream] - 1
            if missing:
                self.counts[f"{stream}_sequence_gaps"] += missing
                self.event("sequence_gap", stream=stream, missing=missing,
                           previous=self.sequence[stream], current=sequence)
        self.sequence[stream], self.timestamps[stream] = sequence, timestamp
        extension = self.config.rgb_format if stream == "rgb" else "png"
        filename = f"images/{stream}/{sequence:012d}.{extension}"
        params = ([cv2.IMWRITE_JPEG_QUALITY, self.config.jpeg_quality] if extension == "jpg"
                  else [cv2.IMWRITE_PNG_COMPRESSION, 1])
        ok, encoded = cv2.imencode("." + extension, array, params)
        if not ok:
            raise OSError(f"Image encoding failed: {filename}")
        payload = encoded.tobytes()
        atomic_bytes(self.root / filename, payload)
        self.append("frames", {**metadata, "stream": stream, "path": filename,
                               "width": array.shape[1], "height": array.shape[0],
                               "bytes": len(payload), "sha256": hashlib.sha256(payload).hexdigest()})
        self.counts[stream] += 1

    def imu_batch(self, rows):
        for row in rows:
            self.append("imu", row)
            self.counts[row["sensor"]] += 1

    def finish(self, status="complete", reason="requested stop"):
        self.event("stop", status=status, reason=reason)
        self.flush()
        for file in self.logs.values():
            file.close()
        self.manifest.update(status=status, stop_reason=reason, counts=dict(self.counts),
                             finished_utc_ns=time.time_ns())
        write_json(self.root / "manifest.json", self.manifest)


class AsyncWriter:
    """One owner of all session writes; overload is an error, never a silent drop."""

    def __init__(self, session: Session):
        self.session = session
        self.queue = queue.Queue(maxsize=session.config.queue_frames)
        self.error = None
        self.thread = threading.Thread(target=self._run, name="dataset-writer", daemon=True)
        self.thread.start()

    def _run(self):
        try:
            while True:
                item = self.queue.get()
                if item is None:
                    return
                kind, args = item
                getattr(self.session, kind)(*args)
        except BaseException as error:
            self.error = error

    def check(self):
        if self.error:
            raise RuntimeError(f"Dataset writer failed: {self.error}") from self.error

    def submit(self, kind, *args):
        self.check()
        try:
            self.queue.put_nowait((kind, args))
        except queue.Full as error:
            raise RuntimeError("Writer backlog exceeded queue_frames; reduce FPS or improve disk/CPU") from error

    def close(self):
        while self.thread.is_alive():
            self.check()
            try:
                self.queue.put(None, timeout=0.1)
                break
            except queue.Full:
                pass
        self.thread.join()
        self.check()
