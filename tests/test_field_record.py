import os
import subprocess


def field_record(tmp_path, linger="yes", state="warming", *args):
    log = tmp_path / "calls"
    tools = tmp_path / "bin"
    tools.mkdir()
    scripts = {
        "loginctl": f'echo {linger}\n',
        # Simulate the recorder reaching the given state once the unit starts.
        "systemd-run": 'printf "%s\\n" systemd-run "$@" >> "$WR_TEST_LOG"\n'
                       'while (($#)); do [[ "$1" == --output ]] && out="$2"; shift; done\n'
                       f'mkdir -p "$out"; echo {state} > "$out/state"; touch "$WR_TEST_LOG.active"\n',
        "systemctl": 'printf "%s\\n" systemctl "$@" >> "$WR_TEST_LOG"\n'
                     'if [[ " $* " == *" is-active "* ]]; then [[ -e "$WR_TEST_LOG.active" ]]; fi\n',
    }
    for name, body in scripts.items():
        (tools / name).write_text("#!/bin/bash\n" + body)
        (tools / name).chmod(0o755)
    result = subprocess.run(["bash", "deploy/field-record.sh", "start", *args], env={
        **os.environ, "PATH": f"{tools}:{os.environ['PATH']}", "WR_TEST_LOG": str(log),
        "WR_MAPPING_FIELD_ROOT": str(tmp_path / "field"), "WR_MAPPING_SERIAL_DEVICE": "/dev/fake",
        "WR_MAPPING_READY_SECONDS": "2"}, capture_output=True, text=True)
    return result, log.read_text().splitlines() if log.exists() else []


def test_field_start_requires_lingering(tmp_path):
    result, calls = field_record(tmp_path, linger="no")
    assert result.returncode == 2
    assert "enable-linger" in result.stderr
    assert "systemd-run" not in calls  # Nothing may start that the SSH logout would kill.


def test_field_start_detaches_with_clean_stop_and_waits_for_readiness(tmp_path):
    result, calls = field_record(tmp_path, "yes", "warming", "--duration", "900")
    assert result.returncode == 0, result.stderr
    assert "Startup checks passed" in result.stdout
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
    assert "did not start" in result.stderr
    assert "systemctl\n--user\nstop\nwr-field-record" in "\n".join(calls)
