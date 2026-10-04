"""Standard checksum interoperability and refusal to modify finalized sessions."""
import os
from pathlib import Path
import subprocess
import sys


def session_fixture(tmp_path):
    session = tmp_path / "session with spaces"
    (session / "bag").mkdir(parents=True)
    for name, contents in {
        "state": "stopping\n",
        "acquisition-start-ns.txt": "1\n",
        "acquisition-end-ns.txt": "2\n",
        "recorder-exit-code.txt": "0\n",
        "bag/metadata.yaml": "fixture\n",
        "bag/camera frames.mcap": "recording fixture\n",
    }.items():
        (session / name).write_text(contents)
    fake_ros = tmp_path / "ros2"
    fake_ros.write_text("#!/bin/sh\necho 'fixture bag metadata'\n")
    fake_ros.chmod(0o755)
    env = {**os.environ, "PATH": f"{tmp_path}:{Path(sys.executable).parent}:{os.environ['PATH']}"}
    return session, env


def test_ros_seal_matches_gnu_checksums_and_refuses_to_reseal(tmp_path):
    session, env = session_fixture(tmp_path)
    command = ["bash", "deploy/seal-rosbag.sh", str(session)]
    subprocess.run(command, env=env, check=True, capture_output=True)
    assert (session / "state").read_text().strip() == "complete"
    subprocess.run(["sha256sum", "--check", "SHA256SUMS"], cwd=session,
                   check=True, capture_output=True)
    before = {p.relative_to(session): p.read_bytes() for p in session.rglob("*") if p.is_file()}
    retry = subprocess.run(command, env=env, capture_output=True)
    assert retry.returncode != 0
    assert before == {p.relative_to(session): p.read_bytes() for p in session.rglob("*") if p.is_file()}


def test_ros_seal_preserves_failed_partial_seal(tmp_path):
    session, env = session_fixture(tmp_path)
    (session / "unrepresentable\nname").write_bytes(b"fixture")
    command = ["bash", "deploy/seal-rosbag.sh", str(session)]
    result = subprocess.run(command, env=env, capture_output=True)
    assert result.returncode != 0
    assert (session / "state").read_text().strip() == "failed"
    partial = (session / "SHA256SUMS").read_bytes()
    assert partial
    retry = subprocess.run(command, env=env, capture_output=True)
    assert retry.returncode != 0
    assert (session / "SHA256SUMS").read_bytes() == partial
    assert (session / "state").read_text().strip() == "failed"
