"""Official OAK/MAVROS drivers and standard rosbag2, owned by ROS launch.

record-rosbag.sh prepares the output directory and seals it after launch exits.
No sensor subscriptions, PID polling or signal escalation are implemented here.
"""
from pathlib import Path
import json
import runpy
import shutil
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, RegisterEventHandler, TimerAction,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def recording_actions(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    output = Path(value('output'))
    duration, warmup = int(value('duration')), int(value('warmup'))
    if duration < 0 or warmup < 0 or not (output / 'topics.txt').is_file():
        raise ValueError('Use record-rosbag.sh to prepare a new session first')
    guidance = runpy.run_path(str(output / 'recording.py'))
    calibration_phases = guidance['CALIBRATION_PHASES']
    phase_record = guidance['phase_record']

    def write(name, text):
        (output / name).write_text(f'{text}\n')

    def fail(reason):
        write('failure.txt', reason)
        return [EmitEvent(event=Shutdown(reason=reason))]

    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-s', 'mcap', '--max-cache-size', '104857600',
             '--max-bag-size', '1073741824', '--storage-config-file', str(output / 'mcap.yaml'),
             '--qos-profile-overrides-path', str(output / 'rosbag-qos.yaml'),
             '-o', str(output / 'bag'), *(output / 'topics.txt').read_text().splitlines()],
        name='rosbag2', output='log', sigterm_timeout='30', sigkill_timeout='5')
    checks = ExecuteProcess(
        cmd=['bash', str(output / 'readiness-script.sh'), str(output), value('camera_only'),
             value('px4_imu_rate')],
        name='readiness', output='log')

    announcers = []
    postroll = {'elapsed': False, 'announcer': None, 'announcement_finished': False}

    def announcer(event):
        # Operator notice only: its outcome is in announce-*.json and never stops recording.
        process = ExecuteProcess(
            cmd=['python3', str(output / 'recording.py'), '--announce', event,
                 '--report', str(output / f'announce-{event}.json')],
            name=f'announce-{event}', output='log')
        announcers.append(process)
        return process

    phases = {'schema_version': 1, 'kind': 'calibration',
              'meaning': 'Operator prompts; not measured motion or ground truth', 'phases': []}

    def prompt_phase(_context, index):
        row = phase_record(index, time.time_ns())
        phases['phases'].append(row)
        (output / 'calibration-phases.json').write_text(json.dumps(phases, indent=2) + '\n')
        actions = [LogInfo(msg=f"Calibration {row['started_utc_ns']}: {row['instruction']}")]
        if index + 1 < len(calibration_phases):
            actions.append(TimerAction(period=float(row['duration_seconds']), actions=[
                OpaqueFunction(function=prompt_phase, kwargs={'index': index + 1})]))
        return actions

    def finish_interval():
        if postroll['elapsed'] and (
                postroll['announcer'] is None or postroll['announcement_finished']):
            return [EmitEvent(event=Shutdown(reason='Requested recording duration complete'))]
        return []

    def postroll_elapsed(_context):
        postroll['elapsed'] = True
        return finish_interval()

    def announcement_deadline(_context):
        if not postroll['announcement_finished']:
            write('announce-stopped-timeout.txt', 'Stop announcer did not exit within 10 seconds')
            return [EmitEvent(event=Shutdown(reason='Stop announcement deadline reached'))]
        return []

    def end_interval(_context):
        write('acquisition-end-ns.txt', time.time_ns())
        write('state', 'postroll')
        actions = []
        if value('announce') == 'true':
            postroll['announcer'] = announcer('stopped')
            actions.extend([postroll['announcer'], TimerAction(period=10., actions=[
                OpaqueFunction(function=announcement_deadline)])])
        return actions + [TimerAction(period=2., actions=[
            OpaqueFunction(function=postroll_elapsed)])]

    def begin_interval(_context):
        write('acquisition-start-ns.txt', time.time_ns())
        write('state', 'recording')
        actions = [announcer('started')] if value('announce') == 'true' else []
        if value('kind') == 'calibration':
            actions.extend(prompt_phase(_context, 0))
        if duration:
            actions.append(TimerAction(period=float(duration), actions=[
                OpaqueFunction(function=end_interval)]))
        return actions

    def process_exited(event, launch_context):
        if event.action in announcers:
            if event.action is postroll['announcer']:
                postroll['announcement_finished'] = True
                return [] if launch_context.is_shutdown else finish_interval()
            return []
        if event.action is recorder:
            write('recorder-exit-code.txt', event.returncode)
            if event.returncode != 0:
                write('failure.txt', f'rosbag2 exited with {event.returncode}')
        if launch_context.is_shutdown:
            return []
        if event.action is checks and event.returncode == 0:
            write('state', 'warming')
            return [TimerAction(period=float(warmup), actions=[
                OpaqueFunction(function=begin_interval)])]
        return fail(f'{event.process_name} exited before shutdown ({event.returncode})')

    def shutdown(_event, _context):
        if not (output / 'acquisition-end-ns.txt').exists():
            write('acquisition-end-ns.txt', time.time_ns())
        write('state', 'stopping')
        return []

    def check_storage(_context):
        if shutil.disk_usage(output).free <= 5368709120:
            return fail('Disk reserve reached')
        return [TimerAction(period=5., actions=[OpaqueFunction(function=check_storage)])]

    parameters = [str(output / 'oak-requested.yaml')]
    if value('device_id'):
        parameters.append({'camera.i_mx_id': value('device_id')})
    actions = [
        RegisterEventHandler(OnProcessExit(on_exit=process_exited)),
        RegisterEventHandler(OnShutdown(on_shutdown=shutdown)),
        Node(package='depthai_ros_driver', executable='camera_node', name='oak',
             parameters=parameters, output='log', sigterm_timeout='10'),
        recorder,
        ExecuteProcess(cmd=['python3', str(output / 'record-resources.py'), str(output)],
                       name='resources', output='log'),
    ]
    if value('start_mavros') == 'true':
        actions.append(IncludeLaunchDescription(
            AnyLaunchDescriptionSource(str(
                Path(get_package_share_directory('mavros')) / 'launch/px4.launch')),
            launch_arguments={'fcu_url': value('fcu_url')}.items()))
    actions.extend([checks, OpaqueFunction(function=check_storage)])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output'),
        DeclareLaunchArgument('duration', default_value='60'),
        DeclareLaunchArgument('warmup', default_value='5'),
        DeclareLaunchArgument('device_id', default_value=''),
        DeclareLaunchArgument('camera_only', default_value='false'),
        DeclareLaunchArgument('start_mavros', default_value='false'),
        DeclareLaunchArgument('fcu_url', default_value='/dev/ttyUSB0:921600'),
        DeclareLaunchArgument('kind', default_value='survey'),
        DeclareLaunchArgument('px4_imu_rate', default_value='0'),
        DeclareLaunchArgument('announce', default_value='false'),
        OpaqueFunction(function=recording_actions),
    ])
