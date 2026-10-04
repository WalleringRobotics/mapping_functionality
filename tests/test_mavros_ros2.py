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
    config = replace(
        TelemetryConfig(), topics=topics, time_node="/wr_test/time", min_sync_samples=2
    )
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    publishers = {
        role: node.create_publisher(cls, topics[role], qos)
        for role, cls in {
            "state": State,
            "imu_raw": Imu,
            "pose": PoseStamped,
            "timesync": TimesyncStatus,
            "gps_raw": GPSRAW,
            "gps_rtk": GPSRTK,
            "rtcm": RTCM,
        }.items()
    }
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
        imu.angular_velocity.x = 0.25
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp.to_msg(), "map"
        pose.pose.orientation.w = 1.0
        gps, rtk, correction = GPSRAW(), GPSRTK(), RTCM()
        gps.header.stamp = rtk.header.stamp = correction.header.stamp = stamp.to_msg()
        gps.fix_type, gps.h_acc, gps.v_acc = 6, 10, 20
        gps.dgps_age = 2**32 - 1
        correction.data = [1, 2, 3]
        for role, message in {
            "state": state,
            "imu_raw": imu,
            "pose": pose,
            "timesync": sync,
            "gps_raw": gps,
            "gps_rtk": rtk,
            "rtcm": correction,
        }.items():
            publishers[role].publish(message)

    timer = node.create_timer(0.05, publish)

    def spin():
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.05)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    subscriber = None
    try:
        subscriber = MavrosSubscriber(config)
        assert subscriber.buffer.parameters["timesync_mode"] == "MAVLINK"
        subscriber.start()
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not (
            subscriber.buffer.monitor.ever_qualified
            and subscriber.buffer.tracker.previous
            and all(subscriber.buffer.counts[r] >= 2 for r in rtk_topics())
        ):
            subscriber.buffer.check()
            time.sleep(0.05)
        subscriber.close()
        subscriber.buffer.finish_check()
        rows = subscriber.buffer.drain(check=False)
        assert any(r["record_type"] == "clock" for r in rows)
        row = next(r for r in rows if r.get("role") == "imu_raw")
        import base64

        message = deserialize_message(base64.b64decode(row["cdr_base64"]), Imu)
        assert message.angular_velocity.x == 0.25
        assert row["fields"]["header"]["frame_id"] == "base_link"
        assert row["ros_type"] == "sensor_msgs/msg/Imu"
        gps_row = next(r for r in rows if r.get("role") == "gps_raw")
        assert gps_row["fields"]["fix_type"] == 6
        assert gps_row["fields"]["dgps_age"] == 2**32 - 1
        assert deserialize_message(base64.b64decode(gps_row["cdr_base64"]), GPSRAW).h_acc == 10
        from wallering_mapping.hardware_probe import telemetry
        readiness = telemetry({"config": config.to_dict(), "seconds": 5, "gnss_profile": None})
        assert readiness["summary"]["connected_seen"]
        assert readiness["summary"]["sync"]["qualification"] == "qualified"
    finally:
        if subscriber:
            subscriber.close()
        stop.set()
        thread.join(timeout=2)
        node.destroy_timer(timer)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


def test_actual_ntrip_http_rtcm_guard_and_ros_forwarding(tmp_path, monkeypatch):
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    from test_rtcm import base_frame, frame
    from wallering_mapping.ntrip import NtripConfig, bridge
    from wallering_mapping.rtcm import RTCMStream

    monkeypatch.delenv("WR_TEST_NTRIP_USER", raising=False)
    monkeypatch.delenv("WR_TEST_NTRIP_PASSWORD", raising=False)
    packets = base_frame() + frame(bytes([0x43, 0x20, 42]) + bytes(900))
    http_stop = threading.Event()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            assert self.path == "/base"
            assert self.headers["Ntrip-Version"] == "Ntrip/2.0"
            self.send_response(200)
            self.send_header("Content-Type", "gnss/data")
            self.end_headers()
            while not http_stop.wait(0.03):
                try:
                    self.wfile.write(packets)
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    break

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    server.daemon_threads = True
    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    http_thread.start()
    context = Context()
    rclpy.init(args=[], context=context)
    node = rclpy.create_node("rtcm_sink", namespace="/wr_ntrip_test", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    received = []
    subscription = node.create_subscription(
        RTCM, "/wr_ntrip_test/corrections", received.append, 100
    )
    ros_stop = threading.Event()

    def spin():
        while not ros_stop.is_set():
            executor.spin_once(timeout_sec=0.05)

    ros_thread = threading.Thread(target=spin, daemon=True)
    ros_thread.start()
    try:
        config = NtripConfig(
            f"http://127.0.0.1:{server.server_port}/base",
            42,
            [4200000, 1000000, 4700000],
            0.01,
            "synthetic surveyed ARP",
            topic="/wr_ntrip_test/corrections",
            username_env="WR_TEST_NTRIP_USER",
            password_env="WR_TEST_NTRIP_PASSWORD",
        )
        result = bridge(config, tmp_path / "bridge", duration=0.3)
        assert result["status"] == "complete" and result["base"]["station_id"] == 42
        deadline = time.monotonic() + 3
        while len(received) < result["forwarded_chunks"] and time.monotonic() < deadline:
            time.sleep(0.02)
        assert len(received) == result["forwarded_chunks"]
        assert max(len(message.data) for message in received) <= 720
        decoder = RTCMStream()
        decoded = []
        for message in received:
            decoded.extend(decoder.feed(bytes(message.data)))
        assert any(len(packet) > 720 for packet in decoded)
        assert b"".join(decoded) == (tmp_path / "bridge/corrections.rtcm3").read_bytes()
    finally:
        ros_stop.set()
        ros_thread.join(timeout=2)
        node.destroy_subscription(subscription)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        http_stop.set()
        server.shutdown()
        server.server_close()
        http_thread.join(timeout=2)
