import os
import signal
import subprocess
import time


def field_tools(tmp_path, linger="yes", state="warming"):
    log = tmp_path / "calls"
    tools = tmp_path / "bin"
    tools.mkdir()
    # Simulate the recorder reaching the given state, with the files its checks write.
    files = ('touch "$out/started-utc.txt" "$out/topics.txt" "$out/mavros-state.yaml" '
             '"$out/oak-parameters.yaml" "$out/graph.txt"\n')
    if state == "failed":
        files = ('printf \'{"missing": ["/mavros/state", "/mavros/time"]}\' > "$out/readiness.json"\n'
                 'touch "$out/topics.txt"\n')
    scripts = {
        "loginctl": f'echo {linger}\n',
        "systemd-run": 'printf "%s\\n" systemd-run "$@" >> "$WR_TEST_LOG"\n'
                       'while (($#)); do [[ "$1" == --output ]] && out="$2"; shift; done\n'
                       f'mkdir -p "$out"; echo {state} > "$out/state"; {files}'
                       'touch "$WR_TEST_LOG.active"\n',
        "systemctl": 'printf "%s\\n" systemctl "$@" >> "$WR_TEST_LOG"\n'
                     'if [[ " $* " == *" is-active "* ]]; then\n'
                     '  [[ -e "$WR_TEST_LOG.active" ]] && { [[ " $* " == *" --quiet "* ]] || echo active; } ||\n'
                     '    { [[ " $* " == *" --quiet "* ]] || echo inactive; exit 3; }\n'
                     'fi\n',
    }
    for name, body in scripts.items():
        (tools / name).write_text("#!/bin/bash\n" + body)
        (tools / name).chmod(0o755)
    env = {**os.environ, "PATH": f"{tools}:{os.environ['PATH']}", "WR_TEST_LOG": str(log),
           "WR_MAPPING_FIELD_ROOT": str(tmp_path / "field"), "WR_MAPPING_SERIAL_DEVICE": "/dev/fake",
           "WR_MAPPING_READY_SECONDS": "3"}
    return env, log


def field_record(tmp_path, linger="yes", state="warming", *args):
    env, log = field_tools(tmp_path, linger, state)
    result = subprocess.run(["bash", "deploy/field-record.sh", "start", *args], env=env,
                            capture_output=True, text=True)
    return result, log.read_text().splitlines() if log.exists() else []


def test_field_start_requires_lingering(tmp_path):
    result, calls = field_record(tmp_path, linger="no")
    assert result.returncode == 2
    assert "NOT RECORDING" in result.stderr and "enable-linger" in result.stderr
    assert "systemd-run" not in calls  # Nothing may start that the SSH logout would kill.


def test_field_start_detaches_with_clean_stop_and_gives_the_all_clear(tmp_path):
    result, calls = field_record(tmp_path, "yes", "warming", "--duration", "900")
    assert result.returncode == 0, result.stderr
    # The operator unplugs on this banner, so each check and the verdict must show.
    for line in ("[ok] No other recorder or MAVROS running", "[ok] PX4 connected",
                 "[ok] OAK camera and IMU streaming", "[ok] 5 s of continuous data from every sensor",
                 "ALL CLEAR: recording. Safe to disconnect.", "Length:  900 s, then it stops by itself",
                 "Buzzer:  plays on the vehicle now"):
        assert line in result.stdout
    run = calls[calls.index("systemd-run"):]
    for expected in ("KillSignal=SIGINT", "TimeoutStopSec=infinity", "WR_MAPPING_SERIAL_DEVICE=/dev/fake",
                     "--start-mavros", "--announce"):
        assert expected in run
    # This host keeps no user journal; the unit's output must land beside the session.
    session = (tmp_path / "field" / "latest").resolve()
    assert f"StandardOutput=append:{session}.log" in run
    # The operator's duration follows the unbounded default, so argparse uses it.
    durations = [run[i + 1] for i, arg in enumerate(run) if arg == "--duration"]
    assert durations == ["0", "900"]
    assert (tmp_path / "field" / "latest" / "state").read_text().strip() == "warming"


def test_field_start_reports_and_stops_a_failed_startup(tmp_path):
    result, calls = field_record(tmp_path, "yes", "failed")
    assert result.returncode == 1
    assert "NOT RECORDING: startup failed" in result.stdout
    assert "Reason:  PX4 not connected" in result.stdout
    assert "ALL CLEAR" not in result.stdout
    assert "systemctl\n--user\nstop\nwr-field-record" in "\n".join(calls)


def test_field_start_wait_survives_ctrl_z(tmp_path):
    # Ctrl+Z once froze the wait over SSH, so the operator never saw a verdict.
    env, _log = field_tools(tmp_path, "yes", "starting")
    env["WR_MAPPING_READY_SECONDS"] = "20"
    process = subprocess.Popen(["bash", "deploy/field-record.sh", "start"], env=env,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        latest = tmp_path / "field" / "latest" / "state"
        deadline = time.monotonic() + 10
        while not latest.exists() and time.monotonic() < deadline:
            time.sleep(.05)
        process.send_signal(signal.SIGTSTP)
        time.sleep(.5)
        for name in ("topics.txt", "mavros-state.yaml", "oak-parameters.yaml", "graph.txt"):
            (latest.parent / name).touch()
        latest.write_text("warming\n")
        stdout, _stderr = process.communicate(timeout=10)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
    assert process.returncode == 0
    assert "ALL CLEAR" in stdout


def test_field_status_flags_a_recording_whose_recorder_is_gone(tmp_path):
    # After a power cut the state file still says recording; the verdict must not.
    env, log = field_tools(tmp_path, "yes", "recording")
    session = tmp_path / "field" / "20260101T000000Z"
    session.mkdir(parents=True)
    (session / "state").write_text("recording\n")
    (tmp_path / "field" / "latest").symlink_to(session)
    result = subprocess.run(["bash", "deploy/field-record.sh", "status"], env=env,
                            capture_output=True, text=True)
    assert result.stdout.splitlines()[0].startswith("NOT RECORDING")
    (tmp_path / "calls.active").touch()
    result = subprocess.run(["bash", "deploy/field-record.sh", "status"], env=env,
                            capture_output=True, text=True)
    assert result.stdout.splitlines()[0] == "RECORDING: safe to disconnect"
