import json
from pathlib import Path

import numpy as np
import pytest

from wallering_mapping import cli
from wallering_mapping.calibration_imu import calibration_entries, solve_gyros
from wallering_mapping.rig_calibration import (BODY, OAK_IMU, RigCalibration, apply_entries, check,
                                               rotation_from_rpy_deg, rotation_from_xyzw)

TEMPLATE = Path(__file__).resolve().parents[1] / "configs/rig-calibration.template.json"
T0 = 1_791_116_822_000_000_000


def body_rates(seconds, axes=(0, 1, 2), seed=0):
    """Smooth multi-frequency body angular rate, still at both ends like the guided session."""
    rng = np.random.default_rng(seed)
    terms = [(axis, rng.uniform(0.15, 1.2), rng.uniform(0.4, 1.0), rng.uniform(0, 6.3))
             for axis in axes for _ in range(3)]

    def rate(t):
        t = np.asarray(t, float)
        w = np.zeros((len(t), 3))
        for axis, frequency, amplitude, phase in terms:
            w[:, axis] += amplitude * np.sin(2 * np.pi * frequency * t + phase)
        envelope = np.clip(np.minimum(t - 5, seconds - 5 - t) / 2, 0, 1)
        return w * envelope[:, None]
    return rate


def oak_stamps(seconds, rng):
    # Bench pattern: three ~7.8 ms intervals then one ~16 ms, averaging 100 Hz.
    pattern = np.tile([7.8e6, 7.8e6, 7.8e6, 16.6e6], int(seconds * 100 / 4) + 1)
    times = np.cumsum(pattern + rng.normal(0, 0.2e6, len(pattern)))
    return times[times < seconds * 1e9]


def synthetic(seconds=90, offset_ns=12_300_000, rotation=None, px4_hz=50, axes=(0, 1, 2),
              noise=0.01, seed=1, reflect=False, drop=None, step=None):
    rng = np.random.default_rng(seed)
    rotation = rotation_from_rpy_deg([10, -170, 95]) if rotation is None else rotation
    rate = body_rates(seconds, axes, seed)
    true_oak = oak_stamps(seconds, rng)
    oak_rates = rate(true_oak / 1e9) @ rotation + [0.004, -0.006, 0.003] + rng.normal(0, noise, (len(true_oak), 3))
    if reflect:
        oak_rates[:, 1] *= -1
    # OAK stamps run offset_ns behind the PX4 clock: px4 = oak + offset.
    oak_t = T0 + true_oak.astype(np.int64) - offset_ns
    if step is not None:
        # The OAK clock jumps by step[1] ns from step[0] seconds on.
        oak_t = oak_t - np.where(true_oak >= step[0] * 1e9, step[1], 0).astype(np.int64)
    px4_true = np.arange(0, seconds * 1e9, 1e9 / px4_hz) + rng.normal(0, 0.3e6, int(seconds * px4_hz))
    px4_true = np.sort(px4_true[px4_true > 0])
    px4_rates = rate(px4_true / 1e9) + rng.normal(0, noise, (len(px4_true), 3))
    px4_t = T0 + px4_true.astype(np.int64)
    if drop is not None:
        keep = (true_oak < drop[0] * 1e9) | (true_oak > drop[1] * 1e9)
        oak_t, oak_rates = oak_t[keep], oak_rates[keep]
    return (oak_t, oak_rates), (px4_t, px4_rates), rotation


def rotation_error(a, b):
    return float(np.degrees(np.arccos(np.clip((np.trace(a @ b.T) - 1) / 2, -1, 1))))


@pytest.mark.parametrize("offset_ns, px4_hz", [(12_300_000, 50), (-37_650_000, 100), (0, 50)])
def test_recovers_offset_and_rotation(offset_ns, px4_hz):
    oak, px4, rotation = synthetic(offset_ns=offset_ns, px4_hz=px4_hz)
    result = solve_gyros(oak, px4)
    error_ns = result["offset_ns"] - offset_ns
    assert abs(error_ns) < 500_000
    assert rotation_error(result["rotation"], rotation) < 0.2
    # Reported uncertainty must cover the actual error.
    assert abs(error_ns) <= 4 * result["offset_sigma_ns"]
    assert result["holdout"]["consistent"] is True
    assert result["skipped_for_oak_gaps"] == 0


def test_oak_gaps_are_skipped_and_reported():
    oak, px4, rotation = synthetic(drop=(40.0, 40.3))
    result = solve_gyros(oak, px4)
    assert abs(result["offset_ns"] - 12_300_000) < 500_000
    assert rotation_error(result["rotation"], rotation) < 0.2
    assert 10 <= result["skipped_for_oak_gaps"] <= 20


@pytest.mark.parametrize("axes", [(2,), (0, 2)])
def test_refuses_motion_that_does_not_excite_every_axis(axes):
    oak, px4, _ = synthetic(axes=axes)
    with pytest.raises(ValueError, match="Insufficient rotation"):
        solve_gyros(oak, px4)


def test_refuses_static_session():
    oak, px4, _ = synthetic(axes=())
    with pytest.raises(ValueError, match="Insufficient rotation"):
        solve_gyros(oak, px4)


def test_detects_axis_handedness_error():
    oak, px4, _ = synthetic(reflect=True)
    with pytest.raises(ValueError, match="reflection"):
        solve_gyros(oak, px4)


def test_timing_change_in_held_out_quarter_is_refused():
    oak, px4, _ = synthetic(step=(75, 2_000_000))
    with pytest.raises(ValueError, match="Held-out final quarter disagrees"):
        solve_gyros(oak, px4)


def test_offset_beyond_search_window_is_refused():
    oak, px4, _ = synthetic(offset_ns=180_000_000)
    with pytest.raises(ValueError, match="search limit"):
        solve_gyros(oak, px4, max_offset_ms=100)


def hand_measured():
    data = json.loads(TEMPLATE.read_text())
    data["transforms"][2].update(source="manual_measurement", rotation_rpy_deg=[180, 0, -90],
                                 translation_m=[0.12, 0.0, -0.05], rotation_sigma_rad=[0.05] * 3,
                                 translation_sigma_m=[0.01] * 3)
    return data


def test_entries_update_the_calibration():
    oak, px4, rotation = synthetic()
    result = solve_gyros(oak, px4)
    calibration = RigCalibration(hand_measured())
    updated = apply_entries(calibration.data, calibration_entries(result, calibration, "test"))
    parsed = RigCalibration(updated)
    imu = parsed.transforms[(BODY, OAK_IMU)]
    assert imu.sources == ("estimated",)
    assert rotation_error(imu.rotation, rotation) < 0.2
    # Position comes from the hand-measured camera, widened by the housing bound.
    assert np.allclose(imu.translation, [0.12, 0.0, -0.05])
    assert np.allclose(np.sqrt(np.diag(imu.covariance)[3:]), np.hypot(0.01, 0.03))
    offset = parsed.offsets[("oak_ros_stamp", "px4_ros_stamp")]
    assert offset["source"] == "estimated" and abs(offset["offset_ns"] - 12_300_000) < 500_000
    # The camera link is still hand-measured until the camera-to-IMU solver runs.
    assert check(parsed)["camera_path"] == "direct"


def test_without_camera_position_translation_stays_unknown():
    oak, px4, _ = synthetic()
    entries = calibration_entries(solve_gyros(oak, px4))
    data = apply_entries(json.loads(TEMPLATE.read_text()), entries)
    assert RigCalibration(data).transforms[(BODY, OAK_IMU)].covariance is None
    assert np.allclose(rotation_from_xyzw(entries[0]["rotation_xyzw"]),
                       RigCalibration(data).transforms[(BODY, OAK_IMU)].rotation)


def write_bag(path, oak, px4):
    from rosbags.rosbag2 import StoragePlugin, Writer
    from rosbags.typesys import Stores, get_typestore
    store = get_typestore(Stores.ROS2_HUMBLE)
    Imu, Header, Time, Vector3, Quaternion = (store.types[name] for name in (
        "sensor_msgs/msg/Imu", "std_msgs/msg/Header", "builtin_interfaces/msg/Time",
        "geometry_msgs/msg/Vector3", "geometry_msgs/msg/Quaternion"))
    with Writer(path / "bag", version=8, storage_plugin=StoragePlugin.MCAP) as writer:
        for topic, frame, (times, rates) in (("/oak/imu/data", "oak_imu_frame", oak),
                                             ("/mavros/imu/data_raw", "base_link", px4)):
            connection = writer.add_connection(topic, Imu.__msgtype__, typestore=store)
            for t, w in zip(times, rates):
                t = int(t)
                message = Imu(Header(Time(sec=t // 10**9, nanosec=t % 10**9), frame),
                              Quaternion(0.0, 0.0, 0.0, 1.0), np.zeros(9), Vector3(*map(float, w)),
                              np.zeros(9), Vector3(0.0, 0.0, 0.0), np.zeros(9))
                writer.write(connection, t, store.serialize_cdr(message, Imu.__msgtype__))


def recorded_session(root, oak, px4, device="TESTDEVICE"):
    from wallering_mapping.dataset import sha256_file
    root.mkdir()
    write_bag(root, oak, px4)
    # Every real session carries the OAK factory calibration; a single camera suffices here.
    (root / f"{device}_calibration.json").write_text(json.dumps(
        {"boardName": "TEST", "imuExtrinsics": {"toCameraSocket": -1},
         "cameraData": [[1, {"extrinsics": {"toCameraSocket": -1}}]]}))
    for name in ("oak-requested.yaml", "oak-parameters.yaml", "mcap.yaml"):
        (root / name).write_text("fixture\n")
    (root / "topics.txt").write_text("/oak/imu/data\n/mavros/imu/data_raw\n")
    (root / "SHA256SUMS").write_text("".join(
        f"{sha256_file(p)}  ./{p.relative_to(root)}\n" for p in sorted(root.rglob("*")) if p.is_file()))
    (root / "state").write_text("complete\n")
    return root


def solve_cli(session, calibration, output):
    return cli.main(["calibrate-solve", str(session), "--calibration", str(calibration),
                     "--output", str(output)])


def test_cli_solves_a_recorded_session(tmp_path, capsys):
    oak, px4, rotation = synthetic(seconds=60)
    session = recorded_session(tmp_path / "session", oak, px4)
    calibration = tmp_path / "rig.json"
    calibration.write_text(json.dumps(hand_measured()))
    output = tmp_path / "solved"
    assert solve_cli(session, calibration, output) == 0
    report = json.loads((output / "report.json").read_text())
    assert abs(report["imu_to_imu"]["offset_ns"] - 12_300_000) < 500_000
    assert report["imu_to_imu"]["rates_hz"]["px4"] == pytest.approx(50, rel=0.02)
    solved = RigCalibration.read(output / "rig-calibration.json")
    assert rotation_error(solved.transforms[(BODY, OAK_IMU)].rotation, rotation) < 0.2
    assert report["imu_to_imu"]["seal_sha256"] in json.dumps(json.loads((output / "rig-calibration.json").read_text()))
    assert json.loads(calibration.read_text()) == hand_measured()
    assert sorted(p.name for p in tmp_path.iterdir()) == ["rig.json", "session", "solved"]
    capsys.readouterr()
    assert solve_cli(session, calibration, output) == 2
    assert "new directory" in capsys.readouterr().err


@pytest.mark.parametrize("damage, message", [
    (lambda root: (root / "SHA256SUMS").unlink(), "no SHA256SUMS seal"),
    (lambda root: (root / "topics.txt").write_text("tampered\n"), "checksum mismatch"),
    (lambda root: (root / "state").write_text("recording\n"), "did not finish cleanly"),
])
def test_cli_refuses_unsealed_or_altered_sessions(tmp_path, capsys, damage, message):
    oak, px4, _ = synthetic(seconds=60)
    session = recorded_session(tmp_path / "session", oak, px4)
    damage(session)
    calibration = tmp_path / "rig.json"
    calibration.write_text(json.dumps(hand_measured()))
    assert solve_cli(session, calibration, tmp_path / "solved") == 2
    assert message in capsys.readouterr().err
    assert not (tmp_path / "solved").exists()


def test_cli_refuses_another_device_before_writing(tmp_path, capsys):
    oak, px4, _ = synthetic(seconds=60)
    session = recorded_session(tmp_path / "session", oak, px4, device="OTHERDEVICE")
    data = hand_measured()
    data["hardware"]["oak_device_id"] = "TESTDEVICE"
    calibration = tmp_path / "rig.json"
    calibration.write_text(json.dumps(data))
    assert solve_cli(session, calibration, tmp_path / "solved") == 2
    assert "Calibration is for OAK TESTDEVICE" in capsys.readouterr().err
    assert sorted(p.name for p in tmp_path.iterdir()) == ["rig.json", "session"]
