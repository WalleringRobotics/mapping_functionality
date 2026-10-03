"""Actual ROS service/DDS/CDR integration; PX4/OAK are simulated publishers."""

import threading
import time
from dataclasses import replace

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("mavros_msgs")

from geometry_msgs.msg import PoseStamped  # noqa: E402
from mavros_msgs.msg import GPSRAW, GPSRTK, RTCM, State, TimesyncStatus  # noqa: E402
from rclpy.context import Context  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402

from wallering_mapping.mavros import MavrosSubscriber  # noqa: E402
from wallering_mapping.telemetry_config import TelemetryConfig, default_topics, rtk_topics  # noqa: E402


def test_real_ros_parameter_snapshot_subscriptions_and_cdr_roundtrip():
    context = Context()
    rclpy.init(args=[], context=context)
    node = rclpy.create_node("time", namespace="/wr_test", context=context)
    node.declare_parameter("timesync_mode", "MAVLINK")
    node.declare_parameter("convergence_window", 1)
    node.declare_parameter("max_rtt_sample", 10)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    stop = threading.Event()
    topics = {key: "/wr_test/" + key for key in default_topics()}
    topics.update({key: "/wr_test/" + key for key in rtk_topics()})
    config = replace(TelemetryConfig(), topics=topics, time_node="/wr_test/time", min_sync_samples=2)
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    publishers = {role: node.create_publisher(cls, topics[role], qos)
                  for role, cls in {"state": State, "imu_raw": Imu, "pose": PoseStamped,
                                    "timesync": TimesyncStatus, "gps_raw": GPSRAW,
                                    "gps_rtk": GPSRTK, "rtcm": RTCM}.items()}
    origin = time.monotonic_ns()

    def publish():
        stamp = node.get_clock().now()
        remote = time.monotonic_ns() - origin + 10**9
        sync = TimesyncStatus()
        sync.header.stamp = stamp.to_msg()
        sync.remote_timestamp_ns = remote
        sync.observed_offset_ns = sync.estimated_offset_ns = stamp.nanoseconds - remote
        sync.round_trip_time_ms = 1.0
        state = State()
        state.header.stamp = stamp.to_msg()
        state.connected = True
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp.to_msg(), "base_link"
        imu.angular_velocity.x = .25
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp.to_msg(), "map"
        pose.pose.orientation.w = 1.0
        gps, rtk, correction = GPSRAW(), GPSRTK(), RTCM()
        gps.header.stamp = rtk.header.stamp = correction.header.stamp = stamp.to_msg()
        gps.fix_type, gps.h_acc, gps.v_acc = 6, 10, 20
        gps.dgps_age = 2**32 - 1
        correction.data = [1, 2, 3]
        for role, message in {"state": state, "imu_raw": imu, "pose": pose, "timesync": sync,
                              "gps_raw": gps, "gps_rtk": rtk, "rtcm": correction}.items():
            publishers[role].publish(message)

    timer = node.create_timer(.05, publish)

    def spin():
        while not stop.is_set():
            executor.spin_once(timeout_sec=.05)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    subscriber = None
    try:
        subscriber = MavrosSubscriber(config)
        assert subscriber.buffer.parameters["timesync_mode"] == "MAVLINK"
        subscriber.start()
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not subscriber.buffer.monitor.ever_qualified:
            subscriber.buffer.check()
            time.sleep(.05)
        subscriber.close()
        subscriber.buffer.finish_check()
        rows = subscriber.buffer.drain(check=False)
        assert any(r["record_type"] == "clock" for r in rows)
        row = next(r for r in rows if r.get("role") == "imu_raw")
        import base64
        message = deserialize_message(base64.b64decode(row["cdr_base64"]), Imu)
        assert message.angular_velocity.x == .25
        assert row["fields"]["header"]["frame_id"] == "base_link"
        assert row["ros_type"] == "sensor_msgs/msg/Imu"
        gps_row = next(r for r in rows if r.get("role") == "gps_raw")
        assert gps_row["fields"]["fix_type"] == 6
        assert gps_row["fields"]["dgps_age"] == 2**32 - 1
        assert deserialize_message(base64.b64decode(gps_row["cdr_base64"]), GPSRAW).h_acc == 10
    finally:
        if subscriber:
            subscriber.close()
        stop.set()
        thread.join(timeout=2)
        node.destroy_timer(timer)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
