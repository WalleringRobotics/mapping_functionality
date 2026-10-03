import os
import signal
import subprocess
import sys
import time

import pytest


def test_sigterm_stops_owned_worker_and_retains_log(tmp_path):
    worker = tmp_path / "worker.py"
    worker.write_text("import os,sys,time\nfrom pathlib import Path\n"
                      "Path(sys.argv[1]).write_text(str(os.getpid()))\n"
                      "while True: time.sleep(0.1)\n")
    pidfile, logfile = tmp_path / "pid", tmp_path / "run.log"
    parent = tmp_path / "runner.py"
    parent.write_text("from pathlib import Path\nimport sys\n"
                      "from wallering_mapping.process_utils import run_logged, termination_signals\n"
                      "with termination_signals():\n"
                      "    run_logged([sys.executable, sys.argv[1], sys.argv[2]], Path(sys.argv[3]))\n")
    process = subprocess.Popen([sys.executable, str(parent), str(worker), str(pidfile), str(logfile)],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        deadline = time.monotonic() + 10
        while not pidfile.exists() and time.monotonic() < deadline:
            if process.poll() is not None:
                pytest.fail("Runner exited before worker startup")
            time.sleep(.02)
        assert pidfile.exists()
        worker_pid = int(pidfile.read_text())
        process.send_signal(signal.SIGTERM)
        assert process.wait(timeout=15) != 0
        with pytest.raises(ProcessLookupError):
            os.kill(worker_pid, 0)
        assert "worker.py" in logfile.read_text()
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
