"""Exercise real MCAP/CDR decoding, evidence seals and offline image preparation."""
import json
import shutil
from pathlib import Path

import cv2
import numpy as np
import pytest
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore

from wallering_mapping.association import associate
from wallering_mapping.bags import CAMERAS, IMU, audit_bag, connection_window, import_bag
from wallering_mapping.dataset import jsonl, sha256_file
from wallering_mapping.export import export
from wallering_mapping.validate import validate

REPO = Path(__file__).resolve().parents[1]


def seal(root):
    (root / 'SHA256SUMS').write_text(''.join(
        f'{sha256_file(p)}  ./{p.relative_to(root)}\n' for p in sorted(root.rglob('*'))
        if p.is_file() and p.name not in {'state', 'SHA256SUMS'}))


def fixture_bag(root, *, missing=None, duplicate=False, short=False, tail_cut=False):
    root.mkdir()
    shutil.copyfile(REPO / 'configs/oakd-ros.yaml', root / 'oak-requested.yaml')
    shutil.copyfile(REPO / 'configs/oakd-ros.yaml', root / 'oak-parameters.yaml')
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
        for i in range(1501 if short else 501 if tail_cut else 151):
            ns = 1_790_000_000_000_000_000 + i * 10_000_000
            if i % 50 == 0:
                for stream, topic in CAMERAS.items():
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
            if IMU != missing and (not short or i < 50):
                msg = t['sensor_msgs/msg/Imu'](header(ns, 'imu'),
                    t['geometry_msgs/msg/Quaternion'](0.,0.,0.,1.), zero,
                    t['geometry_msgs/msg/Vector3'](0.,0.,0.), zero,
                    t['geometry_msgs/msg/Vector3'](0.,0.,9.81), zero)
                writer.write(conns[IMU], ns, store.serialize_cdr(msg, msg.__msgtype__))
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
