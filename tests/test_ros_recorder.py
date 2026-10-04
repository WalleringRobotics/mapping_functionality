"""Real ROS launch lifecycle with stand-in hardware and recorder executables."""
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import pytest

pytest.importorskip('launch')
pytest.importorskip('launch_ros')


@pytest.mark.parametrize('mode', ['timed', 'interrupt', 'calibration_interrupt', 'driver_failure', 'recorder_failure', 'preflight_failure'])
def test_launch_owns_children_and_only_clean_recordings_are_sealed(tmp_path, mode):
    real_ros2 = shutil.which('ros2')
    assert real_ros2
    index = tmp_path / 'share/ament_index/resource_index/packages'
    index.mkdir(parents=True)
    (index / 'depthai_ros_driver').touch()
    camera = tmp_path / 'lib/depthai_ros_driver/camera_node'
    camera.parent.mkdir(parents=True)
    fake = tmp_path / 'ros2'
    fake.write_text(f'#!{sys.executable}\n' + '''
import os, signal, sys, time
from pathlib import Path
args = sys.argv[1:]
root = Path(os.environ['WR_TEST_ROOT'])
mode = os.environ['WR_TEST_MODE']
if args[:1] == ['launch']:
    os.execv(os.environ['WR_TEST_ROS2'], [os.environ['WR_TEST_ROS2'], *args])
if Path(sys.argv[0]).name == 'camera_node' or args[:2] == ['bag', 'record']:
    kind = 'publisher' if Path(sys.argv[0]).name == 'camera_node' else 'recorder'
    with (root/'pids').open('a') as out:
        out.write(str(os.getpid())+'\\n')
    if kind == 'recorder':
        bag = Path(args[args.index('-o')+1])
        bag.mkdir()
        (bag/'test.mcap').write_bytes(b'lifecycle fixture')
    def stop(*_):
        # Both the foreground group and launch may deliver SIGINT. Model ROS's
        # idempotent shutdown rather than interrupting Python's interpreter exit.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        with (root/'events').open('a') as out:
            out.write(kind+'_stopped\\n')
        if kind == 'recorder':
            (bag/'metadata.yaml').write_text('lifecycle fixture\\n')
        sys.exit(0)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    while True:
        time.sleep(.05)
        if ((mode == 'driver_failure' and kind == 'publisher') or
            (mode == 'recorder_failure' and kind == 'recorder')):
            if (root/'session/acquisition-start-ns.txt').exists():
                sys.exit(7)
elif args[:2] == ['param', 'dump']:
    print('/oak: {ros__parameters: {}}')
elif args[:2] == ['topic', 'echo']:
    sys.exit(4 if mode == 'preflight_failure' else 0)
elif args[:2] == ['bag', 'info']:
    assert not any(Path('/proc', pid).exists() for pid in (root/'pids').read_text().splitlines())
    print('Lifecycle fixture bag')
''')
    fake.chmod(0o755)
    camera.symlink_to(fake)
    output = tmp_path / 'session'
    env = {**os.environ, 'PATH': f'{tmp_path}:{os.environ["PATH"]}',
           'AMENT_PREFIX_PATH': f'{tmp_path}:{os.environ.get("AMENT_PREFIX_PATH", "")}',
           'WR_TEST_ROOT': str(tmp_path), 'WR_TEST_MODE': mode, 'WR_TEST_ROS2': real_ros2}
    command = ['bash', 'deploy/record-rosbag.sh', '--output', str(output),
        '--camera-only', '--warmup', '0', '--duration', '1' if mode == 'timed' else '0']
    if mode == 'calibration_interrupt':
        command[-1] = '110'
        command.extend(['--kind', 'calibration'])
    process = subprocess.Popen(command,
        env=env, start_new_session=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        if mode in {'interrupt', 'calibration_interrupt'}:
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                marker = 'calibration-phases.json' if mode == 'calibration_interrupt' else 'acquisition-start-ns.txt'
                if (output / marker).exists():
                    break
                if process.poll() is not None:
                    pytest.fail(str(process.communicate()))
                time.sleep(.02)
            else:
                pytest.fail('Acquisition did not start')
            os.killpg(process.pid, signal.SIGINT)
        stdout, stderr = process.communicate(timeout=25)
        if 'failure' in mode:
            assert process.returncode != 0, (stdout, stderr)
            assert (output / 'state').read_text().strip() == 'failed'
            assert not (output / 'SHA256SUMS').exists()
        else:
            assert process.returncode == 0, (stdout, stderr)
            assert sorted((tmp_path / 'events').read_text().splitlines()) == ['publisher_stopped', 'recorder_stopped']
            assert (output / 'state').read_text().strip() == 'complete'
            assert 'bag/test.mcap' in (output / 'SHA256SUMS').read_text()
            assert '__pycache__' not in (output / 'SHA256SUMS').read_text()
            if mode == 'calibration_interrupt':
                import json
                assert json.loads((output / 'session.json').read_text())['kind'] == 'calibration'
                phases = json.loads((output / 'calibration-phases.json').read_text())
                assert phases['phases'][0]['phase'] == 'still_start'
                assert 'calibration-phases.json' in (output / 'SHA256SUMS').read_text()
            start = int((output / 'acquisition-start-ns.txt').read_text())
            end = int((output / 'acquisition-end-ns.txt').read_text())
            assert 0 < start < end
        assert all(not Path('/proc', pid).exists() for pid in (tmp_path / 'pids').read_text().splitlines())
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        if (tmp_path / 'pids').exists():
            for pid in (tmp_path / 'pids').read_text().splitlines():
                try:
                    os.kill(int(pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
