"""Receive-only ROS 2 adapter for the platform's existing PX4/MAVROS link."""

import base64
import math
import queue
import threading
import time
from collections import Counter

from .timing import ClockTracker, SyncMonitor, sample_clock


def json_fields(value):
    """Keep unavailable ROS sensor values explicit; the CDR retains original bits."""
    if isinstance(value, dict):
        return {key: json_fields(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)) or hasattr(value, "tolist"):
        return [json_fields(item) for item in (value.tolist() if hasattr(value, "tolist") else value)]
    if isinstance(value, float) and not math.isfinite(value):
        return {"nonfinite": repr(value)}
    if isinstance(value, (bytes, bytearray)):
        return {"bytes_base64": base64.b64encode(value).decode()}
    return value


def stamp_ns(fields):
    stamp = fields.get("header", {}).get("stamp")
    if stamp is None:
        return None
    sec, nano = stamp["sec"], stamp["nanosec"]
    if type(sec) is not int or type(nano) is not int or sec < 0 or not 0 <= nano < 10**9:
        raise ValueError("Invalid ROS source header timestamp")
    return sec * 10**9 + nano


class TelemetryBuffer:
    """Bounded callback handoff; only the dataset writer touches session files."""

    def __init__(self, config, parameters):
        self.config = config
        self.parameters = parameters
        self.monitor = SyncMonitor(config, parameters)
        self.tracker = ClockTracker(config.clock_jump_ms)
        self.queue = queue.Queue(maxsize=config.queue_records)
        self.counts = Counter()
        self.last_received = {}
        self.error = None
        self.was_connected = False
        self.started_ns = time.monotonic_ns()
        self.latest_sync = None

    def put(self, record):
        try:
            self.queue.put_nowait(record)
        except queue.Full as error:
            raise RuntimeError("Telemetry buffer overflow; no samples may be silently dropped") from error

    def clock(self, observation):
        self.tracker.observe(observation)
        self.put({"record_type": "clock", **observation})

    def ingest(self, role, fields, ros_type, cdr, receipt, utc_ns):
        received = receipt["reference_ns"]
        self.counts[role] += 1
        self.last_received[role] = received
        record = {"record_type": "message", "role": role, "topic": self.config.topics[role],
                  "ros_type": ros_type, "ordinal": self.counts[role],
                  "source_stamp_ros_ns": stamp_ns(fields), "receipt_clock": receipt,
                  "received_monotonic_ns": received, "received_utc_ns": utc_ns,
                  "fields": json_fields(fields), "cdr_base64": base64.b64encode(cdr).decode()}
        if role == "state":
            if self.was_connected and not fields["connected"]:
                raise RuntimeError("MAVROS disconnected during capture; start a new session")
            self.was_connected |= bool(fields["connected"])
        if role == "timesync":
            record["sync_quality"] = self.monitor.observe(fields)
            self.latest_sync = record
        self.put(record)

    def check(self, now_ns=None):
        if self.error:
            raise RuntimeError(f"Telemetry callback failed: {self.error}") from self.error
        now = time.monotonic_ns() if now_ns is None else now_ns
        for role in self.config.required:
            last = self.last_received.get(role, self.started_ns)
            if now - last > self.config.stall_seconds * 1e9:
                raise RuntimeError(f"Required MAVROS topic stalled: {role}/{self.config.topics[role]}")

    def drain(self, limit=1024, check=True):
        if check:
            self.check()
        rows = []
        for _ in range(limit):
            try:
                rows.append(self.queue.get_nowait())
            except queue.Empty:
                break
        return rows

    def summary(self):
        return {"counts": dict(self.counts), "queued_records": self.queue.qsize(),
                "connected_seen": self.was_connected,
                "sync": self.latest_sync.get("sync_quality") if self.latest_sync else None,
                "ever_qualified": self.monitor.ever_qualified}

    def finish_check(self):
        self.check()
        if any(not self.counts[role] for role in self.config.required):
            raise RuntimeError("One or more required MAVROS topics were never received")
        if not self.was_connected or not self.monitor.ever_qualified:
            raise RuntimeError("No connected, timing-qualified PX4 interval was recorded")


class MavrosSubscriber:
    """No publishers, flight services, serial transport or TIMESYNC transmitter.

    The only service requests read existing time-plugin parameters. All telemetry
    is kept independently; no image is dropped just because pose pairing fails.
    """

    def __init__(self, config):
        self.config = config
        self.context = self.node = self.executor = None
        self.thread = None
        self.stop = threading.Event()
        self.buffer = None
        try:
            self._open()
        except BaseException:
            self.close()
            raise

    def _open(self):
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import State, TimesyncStatus
        from rcl_interfaces.srv import GetParameters
        from rclpy.clock import Clock, ClockType
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rclpy.serialization import serialize_message
        from rclpy.signals import SignalHandlerOptions
        from rosidl_runtime_py.convert import message_to_ordereddict
        from sensor_msgs.msg import Imu, NavSatFix, TimeReference

        self.context = Context()
        rclpy.init(args=[], context=self.context, signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node("wr_mapping_telemetry", context=self.context)
        if self.node.get_parameter("use_sim_time").value:
            raise ValueError("Hardware capture does not support ROS simulation time")
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        client = self.node.create_client(GetParameters, self.config.time_node.rstrip("/") + "/get_parameters")
        timeout = self.config.parameter_timeout_seconds
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"MAVROS time node unavailable: {self.config.time_node}; inspect ros2 node list")
        names = ["timesync_mode", "convergence_window", "max_rtt_sample"]
        request = GetParameters.Request()
        request.names = names
        future = client.call_async(request)
        self.executor.spin_until_future_complete(future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError("MAVROS parameter read timed out")
        from rclpy.parameter import parameter_value_to_python
        parameters = {name: parameter_value_to_python(value)
                      for name, value in zip(names, future.result().values, strict=True)}
        self.node.destroy_client(client)
        self.buffer = TelemetryBuffer(self.config, parameters)
        types = {"state": State, "imu_raw": Imu, "attitude": Imu, "pose": PoseStamped,
                 "gnss": NavSatFix, "timesync": TimesyncStatus, "time_reference": TimeReference}
        qos = QoSProfile(depth=self.config.qos_depth, reliability=ReliabilityPolicy.BEST_EFFORT)

        def protect(action):
            try:
                action()
            except BaseException as error:
                self.buffer.error = error
                self.stop.set()

        def callback(role, ros_type):
            def receive(message):
                def action():
                    observation = sample_clock(lambda: self.node.get_clock().now().nanoseconds, "ros_system")
                    self.buffer.ingest(role, message_to_ordereddict(message), ros_type,
                                       serialize_message(message), observation, time.time_ns())
                protect(action)
            return receive

        self.subscriptions = []
        for role, message_class in types.items():
            ros_type = message_class.__module__.split(".")[0] + "/msg/" + message_class.__name__
            self.subscriptions.append(self.node.create_subscription(
                message_class, self.config.topics[role], callback(role, ros_type), qos))
        self.timer = self.node.create_timer(1 / self.config.clock_sample_hz,
            lambda: protect(lambda: self.buffer.clock(sample_clock(
                lambda: self.node.get_clock().now().nanoseconds, "ros_system"))),
            clock=Clock(clock_type=ClockType.STEADY_TIME))

    def start(self):
        self.buffer.started_ns = time.monotonic_ns()
        self.thread = threading.Thread(target=self._spin, name="mavros-subscriber", daemon=True)
        self.thread.start()

    def _spin(self):
        try:
            while not self.stop.is_set():
                self.executor.spin_once(timeout_sec=.05)
        except BaseException as error:
            self.buffer.error = error

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=2)
            if self.thread.is_alive():
                raise RuntimeError("MAVROS subscriber did not stop")
        if self.executor:
            self.executor.shutdown(timeout_sec=1)
            self.executor = None
        if self.node:
            self.node.destroy_node()
            self.node = None
        if self.context and self.context.ok():
            self.context.shutdown()
