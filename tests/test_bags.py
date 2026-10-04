"""Exercise real MCAP/CDR decoding, evidence seals and offline image preparation."""
import json
import shutil
from pathlib import Path

import cv2
import numpy as np
import pytest
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore
from ruamel.yaml import YAML

from wallering_mapping.association import associate
from wallering_mapping.bags import (CAMERAS, IMU, audit_bag, connection_window, import_bag,
                                    imu_message_rate, sample_loss)
from wallering_mapping.dataset import jsonl, sha256_file
from wallering_mapping.export import export
from wallering_mapping.validate import validate

REPO = Path(__file__).resolve().parents[1]


def seal(root):
    (root / 'SHA256SUMS').write_text(''.join(
        f'{sha256_file(p)}  ./{p.relative_to(root)}\n' for p in sorted(root.rglob('*'))
        if p.is_file() and p.name not in {'state', 'SHA256SUMS'}))


def fixture_bag(root, *, missing=None, duplicate=False, short=False, tail_cut=False, imu_drop=(),
                imu_repeat=(), mono_fps=2, px4_burst=False):
    root.mkdir()
    # The fixture IMU publishes at 100 Hz on gyro times, whatever the default profile.
    yaml = YAML()
    profile = yaml.load((REPO / 'configs/oakd-ros.yaml').read_text())
    profile['/oak']['ros__parameters']['imu'].update(
        i_gyro_freq=100, i_acc_freq=125, i_sync_method='LINEAR_INTERPOLATE_ACCEL')
    for name in ('left', 'right'):
        profile['/oak']['ros__parameters'][name]['i_fps'] = mono_fps
    for name in ('oak-requested.yaml', 'oak-parameters.yaml'):
        with (root / name).open('w') as out:
            yaml.dump(profile, out)
    shutil.copyfile(REPO / 'configs/rosbag-mcap.yaml', root / 'mcap.yaml')
    (root / 'state').write_text('complete\n')
    (root / 'topics.txt').write_text('\n'.join([*CAMERAS.values(), IMU]))
    store = get_typestore(Stores.ROS2_HUMBLE)
    t = store.types
    zero = np.zeros(9)
    def header(ns, frame):
        return t['std_msgs/msg/Header'](t['builtin_interfaces/msg/Time'](
            ns // 10**9, ns % 10**9), frame)
    with Writer(root / 'bag', version=9, storage_plugin=StoragePlugin.MCAP) as writer:
        conns = {}
        for topic in CAMERAS.values():
            for name, kind in ((topic, 'Image'), (topic.replace('image_raw', 'camera_info'), 'CameraInfo')):
                conns[name] = writer.add_connection(name, f'sensor_msgs/msg/{kind}', typestore=store)
        conns[IMU] = writer.add_connection(IMU, 'sensor_msgs/msg/Imu', typestore=store)
        if px4_burst:
            conns['/mavros/imu/data'] = writer.add_connection('/mavros/imu/data', 'sensor_msgs/msg/Imu', typestore=store)
        for i in range(1501 if short else 501 if tail_cut else 151):
            ns = 1_790_000_000_000_000_000 + i * 10_000_000
            if i % min(50, 100 // mono_fps) == 0:
                for stream, topic in CAMERAS.items():
                    if i % (50 if stream == "rgb" else 100 // mono_fps):
                        continue
                    if topic == missing:
                        continue
                    if tail_cut and stream == 'right' and i >= 300:
                        continue
                    frame = f'{stream}_optical'
                    stamp = ns - (500_000_000 if duplicate and i == 100 else 0)
                    # RGB row padding and channel order must survive import correctly.
                    pixels = np.array([255, 0, 0, 0, 255, 0, 99, 99], dtype=np.uint8)
                    message = t['sensor_msgs/msg/Image'](header(stamp, frame), 1, 2, 'rgb8', 0, 8, pixels)
                    writer.write(conns[topic], ns, store.serialize_cdr(message, message.__msgtype__))
                    info = t['sensor_msgs/msg/CameraInfo'](header(stamp, frame), 1, 2,
                        'plumb_bob', np.zeros(5), np.array([2.,0,1,0,2,0.5,0,0,1]),
                        np.eye(3).ravel(), np.zeros(12), 0, 0,
                        t['sensor_msgs/msg/RegionOfInterest'](0,0,0,0,False))
                    conn = conns[topic.replace('image_raw', 'camera_info')]
                    writer.write(conn, ns, store.serialize_cdr(info, info.__msgtype__))
            if IMU != missing and (not short or i < 50) and i not in imu_drop:
                rate = (i - 1 if i in imu_repeat else i) * 1e-3
                msg = t['sensor_msgs/msg/Imu'](header(ns, 'imu'),
                    t['geometry_msgs/msg/Quaternion'](0.,0.,0.,1.), zero,
                    t['geometry_msgs/msg/Vector3'](rate, 0., 0.), zero,
                    t['geometry_msgs/msg/Vector3'](0.,0.,9.81), zero)
                writer.write(conns[IMU], ns, store.serialize_cdr(msg, msg.__msgtype__))
            if px4_burst and i <= 100:
                msg = t['sensor_msgs/msg/Imu'](header(ns, 'px4'),
                    t['geometry_msgs/msg/Quaternion'](0.,0.,0.,1.), zero,
                    t['geometry_msgs/msg/Vector3'](0.,0.,0.), zero,
                    t['geometry_msgs/msg/Vector3'](0.,0.,9.81), zero)
                writer.write(conns['/mavros/imu/data'], ns, store.serialize_cdr(msg, msg.__msgtype__))
    seal(root)
    return root


def test_mcap_import_export_retains_geometry_pixels_and_time(tmp_path):
    root = fixture_bag(tmp_path / 'recording')
    audit = audit_bag(root)
    assert audit['valid'], audit['errors']
    assert not audit['survey_ready']
    assert audit['imu_source_sequence_gaps'] is None
    derived = tmp_path / 'derived'
    manifest = import_bag(root, derived)
    assert manifest['source'] == 'rosbag2'
    assert manifest['rosbag_source']['seal_sha256'] == sha256_file(root / 'SHA256SUMS')
    rows = list(jsonl(derived / 'frames.jsonl'))
    assert len(rows) == 12
    assert rows[0]['source_stamp_ros_ns'] == 1_790_000_000_000_000_000
    image = cv2.imread(str(derived / rows[0]['path']))
    assert image.tolist() == [[[0,0,255], [0,255,0]]]
    report = validate(derived)
    assert report['valid'], report['errors']
    assert report['streams']['rgb']['sequence_gaps'] is None
    result = export(derived, tmp_path / 'export', interval=0)
    assert len(result['images']) == 4
    assert result['camera']['model'] == 'FULL_OPENCV'
    with pytest.raises(ValueError, match='qualified clock bridge'):
        associate(derived, tmp_path / 'sync')
    assert json.loads((derived / 'manifest.json').read_text())['status'] == 'complete'


@pytest.mark.parametrize('missing', [IMU, CAMERAS['right']])
def test_missing_required_stream_rejected(tmp_path, missing):
    report = audit_bag(fixture_bag(tmp_path / 'recording', missing=missing))
    assert not report['valid']
    assert any('Required topic missing' in error for error in report['errors'])


def test_nonmonotonic_headers_rejected(tmp_path):
    report = audit_bag(fixture_bag(tmp_path / 'recording', duplicate=True))
    assert not report['valid']
    assert any('Nonmonotonic' in error for error in report['errors'])


def test_sidecar_tamper_and_incomplete_seal_rejected(tmp_path):
    root = fixture_bag(tmp_path / 'recording')
    (root / 'topics.txt').write_text('/unrelated\n')
    assert 'checksum mismatch' in audit_bag(root)['errors'][0]
    seal(root)
    lines = (root / 'SHA256SUMS').read_text().splitlines()
    (root / 'SHA256SUMS').write_text('\n'.join(v for v in lines if 'oak-parameters.yaml' not in v))
    assert 'omits required' in audit_bag(root)['errors'][0]


def test_failed_capture_not_importable(tmp_path):
    root = fixture_bag(tmp_path / 'recording')
    (root / 'state').write_text('failed\n')
    with pytest.raises(ValueError, match='did not finish cleanly'):
        import_bag(root, tmp_path / 'derived')
    assert not (tmp_path / 'derived').exists()


def test_imu_stall_is_rejected_even_when_active_interval_rate_is_correct(tmp_path):
    report = audit_bag(fixture_bag(tmp_path / 'recording', short=True))
    assert not report['valid']
    assert any('stalled' in error for error in report['errors'])


def test_clean_bag_with_truncated_camera_tail_is_not_capture_ready(tmp_path):
    report = audit_bag(fixture_bag(tmp_path / 'recording', tail_cut=True))
    assert report['valid'], report['errors']
    assert not report['capture_ready']
    assert not report['coverage_complete']
    assert report['topics'][CAMERAS['right']]['last_receipt_gap_seconds'] == 2.5


def test_shutdown_drain_is_outside_acquisition_coverage_window(tmp_path):
    root = fixture_bag(tmp_path / 'recording', tail_cut=True)
    (root / 'acquisition-start-ns.txt').write_text('1790000000000000000\n')
    (root / 'acquisition-end-ns.txt').write_text('1790000002500000000\n')
    seal(root)
    report = audit_bag(root)
    assert report['valid'], report['errors']
    assert report['capture_ready']
    assert report['topics'][CAMERAS['right']]['last_receipt_gap_seconds'] == 0


def test_bag_ending_before_requested_window_does_not_shorten_coverage_check(tmp_path):
    root = fixture_bag(tmp_path / 'recording')
    (root / 'acquisition-start-ns.txt').write_text('1790000000000000000\n')
    (root / 'acquisition-end-ns.txt').write_text('1790000004000000000\n')
    seal(root)
    report = audit_bag(root)
    assert report['valid'], report['errors']
    assert not report['capture_ready']
    assert not report['coverage_complete']
    assert report['coverage_window']['duration_seconds'] == 4


@pytest.mark.parametrize('samples, expected', [
    ([(0, False), (1, True), (3, True)], True),
    ([(0, False), (3, True)], False),  # Disconnected state still held at interval start.
    ([(1, True), (3, False), (4, True)], False),
    ([(1, True), (3, True), (5, False)], True),
])
def test_px4_preroll_is_retained_but_connection_gate_covers_acquisition(samples, expected):
    result = connection_window(samples, 2, 4)
    assert result['connected_throughout_window'] is expected
    assert result['disconnected_before_window'] == sum(t < 2 and not v for t, v in samples)


def window(root, start_s, end_s):
    base = 1_790_000_000_000_000_000
    (root / 'acquisition-start-ns.txt').write_text(f'{base + int(start_s * 1e9)}\n')
    (root / 'acquisition-end-ns.txt').write_text(f'{base + int(end_s * 1e9)}\n')
    seal(root)
    return root


def test_clean_imu_reports_no_in_window_loss(tmp_path):
    root = fixture_bag(tmp_path / 'recording', tail_cut=True, imu_repeat=(50,))
    report = audit_bag(window(root, 0, 5))
    assert report['valid'], report['errors']
    loss = report['topics'][IMU]['in_window']
    assert loss['requested_hz'] == 100
    assert (loss['samples'], loss['expected_samples'], loss['missing_samples']) == (501, 500, 0)
    assert (loss['gaps'], loss['repeated_samples']) == (0, 1)
    assert not any('IMU sample loss' in warning for warning in report['warnings'])
    assert report['topics'][CAMERAS['rgb']]['in_window']['requested_hz'] == 2


def test_imu_loss_gaps_and_repeats_reported_against_requested_rate(tmp_path):
    root = fixture_bag(tmp_path / 'recording', tail_cut=True, imu_drop=(*range(100, 105), 300),
                       imu_repeat=(200,))
    report = audit_bag(window(root, 0, 5))
    assert report['valid'], report['errors']
    loss = report['topics'][IMU]['in_window']
    assert (loss['samples'], loss['expected_samples'], loss['missing_samples']) == (495, 500, 5)
    assert loss['loss_percent'] == 1
    assert (loss['gaps'], loss['missing_in_gaps'], loss['repeated_samples']) == (2, 6, 1)
    assert loss['max_gap_ms'] == pytest.approx(60)
    warning = next(w for w in report['warnings'] if 'IMU sample loss' in w)
    assert '495 samples, 500 expected at 100 Hz (1.00% missing), 2 gaps (max 60 ms' in warning


def test_imu_loss_restricted_to_acquisition_window(tmp_path):
    root = fixture_bag(tmp_path / 'recording', tail_cut=True, imu_drop=range(10, 20))
    loss = audit_bag(window(root, 1, 5))['topics'][IMU]['in_window']
    assert (loss['samples'], loss['gaps'], loss['missing_samples']) == (401, 0, 0)


@pytest.mark.parametrize('imu, rate', [
    ({'i_gyro_freq': 200, 'i_acc_freq': 250, 'i_sync_method': 'LINEAR_INTERPOLATE_ACCEL'}, 200),
    ({'i_gyro_freq': 100, 'i_acc_freq': 100, 'i_sync_method': 'COPY'}, 100),
    ({'i_gyro_freq': 150, 'i_acc_freq': 100, 'i_sync_method': 'LINEAR_INTERPOLATE_ACCEL'}, 200),
    ({'i_gyro_freq': 400, 'i_acc_freq': 100, 'i_sync_method': 'LINEAR_INTERPOLATE_GYRO'}, 125),
])
def test_imu_message_rate_follows_the_uninterpolated_sensor(imu, rate):
    assert imu_message_rate(imu) == rate


def test_sample_loss_without_requested_rate_uses_median_interval():
    stamps = [i * 20_000_000 for i in range(100) if i not in (40, 41, 42)]
    loss = sample_loss(stamps, 0, 99 * 20_000_000)
    assert loss['requested_hz'] is None
    assert (loss['gaps'], loss['missing_samples'], loss['expected_samples']) == (1, 3, 100)
    assert loss['max_gap_ms'] == pytest.approx(80)
    assert loss['repeated_samples'] is None


@pytest.mark.parametrize("name", ["factory_calibration.json", "session.json", "calibration-phases.json",
                                 "px4-rate-request.json", "rosbag-qos.yaml"])
def test_unsealed_optional_provenance_is_rejected(tmp_path, name):
    root = fixture_bag(tmp_path / "recording")
    (root / name).write_text("{}")
    assert "seal omits required files" in audit_bag(root)["errors"][0]


def test_zero_samples_reports_full_rate_deficit():
    loss = sample_loss([0, 1], 2_000_000_000, 3_000_000_000, 100)
    assert loss["samples"] == 0
    assert loss["missing_samples"] == 100
    assert loss["loss_percent"] == 100
    assert loss["hardware_sample_loss"] is None


def test_invalid_timestamps_do_not_divide_by_zero_or_claim_rate():
    loss = sample_loss([0, 0, 0], 0, 1_000_000_000, 100)
    assert loss["invalid_intervals"] == 2
    assert loss["measured_hz"] is None


def test_sample_loss_retains_clock_deficit_separate_from_gap_estimate():
    stamps = np.arange(1000, dtype=np.int64) * 10_010_000
    loss = sample_loss(stamps, 0, 10_000_000_000, 100)
    assert loss["gaps"] == 0
    assert loss["hardware_sample_loss"] is None




def test_mixed_rate_import_keeps_independent_mono_and_rgb_rates(tmp_path):
    root = fixture_bag(tmp_path / "recording", mono_fps=20)
    derived = tmp_path / "derived"
    manifest = import_bag(root, derived)
    assert manifest["device"]["stream_settings"]["left"]["fps"] == 20
    result = validate(derived)
    assert result["valid"], result["errors"]
    assert result["streams"]["left"]["requested_fps"] == 20
    assert result["streams"]["rgb"]["requested_fps"] == 2
    assert result["streams"]["left"]["observed_fps"] == 20


def test_mavlink_wire_load_and_sequence_gap_estimates_are_windowed():
    from wallering_mapping.bags import mavlink_window
    rows = [(0, 1, 1, 253, 74), (1_000_000_000, 1, 1, 255, 74),
            (2_000_000_000, 1, 1, 0, 74), (3_000_000_000, 1, 1, 2, 74),
            (3_500_000_000, 1, 1, 2, 74), (4_000_000_000, 1, 1, 1, 74)]
    report = mavlink_window(rows, 1_000_000_000, 4_000_000_000)
    assert report["inferred_sequence_gaps"] == 1
    assert report["duplicate_sequences"] == 1
    assert report["reordered_or_reset_sequences"] == 1
    assert report["received_wire_bytes_per_second"] == pytest.approx(5 * 74 / 3)


def test_px4_filtered_imu_brief_full_rate_burst_fails_acquisition_coverage(tmp_path):
    from types import SimpleNamespace
    from wallering_mapping.recording import request_imu_rate
    root = fixture_bag(tmp_path / 'recording', short=True, px4_burst=True)
    (root / 'session.json').write_text(json.dumps({'kind': 'survey', 'px4_imu_requested_hz': 100}))
    for name, rate in [('request', 100), ('restore', 0)]:
        request_imu_rate(rate, root / f'px4-rate-{name}.json',
                         lambda *_: SimpleNamespace(success=True, result=0))
    window(root, 0, 15)
    report = audit_bag(root)
    assert not report['capture_ready']
    loss = report['topics']['/mavros/imu/data']['in_window']
    assert loss['measured_hz'] == 100  # Active-span rate must not mask early stop.
    assert loss['window_hz'] == pytest.approx(101 / 15)
    assert 'PX4 IMU rate below 95% of request: /mavros/imu/data' in report['errors']
    assert 'Required stream absent/stalled for over 5 seconds: /mavros/imu/data' in report['errors']
