"""External process lifecycle and portable artifact manifests."""

import os
import shlex
import signal
import subprocess
from contextlib import contextmanager

from .dataset import safe_path, sha256_file


@contextmanager
def termination_signals():
    previous = signal.getsignal(signal.SIGTERM)

    def interrupted(*_):
        raise InterruptedError("Processing cancelled by SIGTERM")

    signal.signal(signal.SIGTERM, interrupted)
    try:
        yield
    finally:
        signal.signal(signal.SIGTERM, previous)


def run_logged(command, log_path, env=None):
    """Cancel owned child processes as a group, including engines that spawn workers."""
    with log_path.open("x") as log:
        log.write(shlex.join(command) + "\n")
        log.flush()
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   env=env, start_new_session=True)
        try:
            returncode = process.wait()
            if returncode:
                raise subprocess.CalledProcessError(returncode, command)
        except BaseException:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            raise


def inventory(root, files):
    result = []
    for path in sorted(set(files)):
        relative = path.relative_to(root).as_posix()
        file = safe_path(root, relative)
        if not file.is_file() or file.stat().st_size == 0:
            raise ValueError(f"Missing/empty output artifact: {relative}")
        result.append({"path": relative, "bytes": file.stat().st_size, "sha256": sha256_file(file)})
    return result


def verify_inventory(root, artifacts):
    if not artifacts:
        raise ValueError("Completed stage has no sealed artifacts")
    for item in artifacts:
        path = safe_path(root, item["path"])
        if not path.is_file() or path.stat().st_size != item["bytes"] or sha256_file(path) != item["sha256"]:
            raise ValueError(f"Completed output changed: {item['path']}; use a new run directory")

