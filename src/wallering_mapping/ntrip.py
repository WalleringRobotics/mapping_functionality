"""NTRIP v2 corrections to the existing MAVROS GPS-RTK input, with evidence."""

import base64
import json
import math
import os
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import asdict, dataclass

import numpy as np

from .dataset import sha256_file, write_json
from .oak import stop_signals
from .rtcm import RTCMStream, describe, mavros_chunks


@dataclass(frozen=True)
class NtripConfig:
    caster_url: str
    station_id: int
    base_arp_ecef_m: list
    base_coordinate_tolerance_m: float
    base_survey_evidence: str
    topic: str = "/mavros/gps_rtk/send_rtcm"
    username_env: str = "WR_NTRIP_USERNAME"
    password_env: str = "WR_NTRIP_PASSWORD"
    timeout_seconds: float = 10
    require_base_before_forwarding: bool = True

    def __post_init__(self):
        url = urllib.parse.urlsplit(self.caster_url)
        if (url.scheme not in {"http", "https"} or not url.hostname or not url.path.strip("/")
                or url.username or url.password or url.query or url.fragment):
            raise ValueError("Use an HTTP(S) mountpoint URL without credentials/query/fragment")
        if type(self.station_id) is not int or not 0 <= self.station_id <= 4095:
            raise ValueError("Expected RTCM station_id must be an integer in [0,4095]")
        point = np.asarray(self.base_arp_ecef_m, float)
        if point.shape != (3,) or not np.isfinite(point).all() or not 6e6 < np.linalg.norm(point) < 7e6:
            raise ValueError("Provide surveyed base ARP ECEF coordinates in metres")
        for value in (self.base_coordinate_tolerance_m, self.timeout_seconds):
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError("Coordinate tolerance and timeout must be positive finite values")
        if not self.base_survey_evidence.strip() or not self.topic.startswith("/"):
            raise ValueError("Provide base survey evidence and an absolute MAVROS RTCM topic")
        if self.require_base_before_forwarding is not True:
            raise ValueError("This fixed-base adapter requires station-coordinate verification before forwarding")

    @classmethod
    def read(cls, path):
        return cls(**json.loads(path.read_text()))


class BaseGuard:
    def __init__(self, config):
        self.config = config
        self.verified = False
        self.last_base = None

    def check(self, frame):
        details = describe(frame)
        if "station_id" in details and details["station_id"] != self.config.station_id:
            raise ValueError("RTCM station ID differs from configured surveyed base")
        if "base_arp_ecef_m" in details:
            discrepancy = np.linalg.norm(np.asarray(details["base_arp_ecef_m"]) - self.config.base_arp_ecef_m)
            if discrepancy > self.config.base_coordinate_tolerance_m:
                raise ValueError("RTCM base ARP coordinates differ from surveyed reference")
            self.last_base = {**details, "coordinate_difference_m": float(discrepancy)}
            self.verified = True
        return details, self.verified


def request(config):
    username, password = os.environ.get(config.username_env), os.environ.get(config.password_env)
    headers = {"Ntrip-Version": "Ntrip/2.0", "User-Agent": "NTRIP wallering-mapping/0.1", "Accept": "*/*"}
    if bool(username) != bool(password):
        raise ValueError("Provide both NTRIP credential environment variables, or neither")
    if username:
        if not config.caster_url.startswith("https://"):
            raise ValueError("Authenticated NTRIP requires HTTPS to protect credentials")
        headers["Authorization"] = "Basic " + base64.b64encode((username + ":" + password).encode()).decode()
    return urllib.request.Request(config.caster_url, headers=headers)


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        raise ValueError("NTRIP redirects are disabled; configure the exact caster mountpoint")


def bridge(config, output, duration=None):
    """Explicit correction forwarding only; never sends flight or clock commands."""
    import rclpy
    from mavros_msgs.msg import RTCM
    from rclpy.context import Context
    from rclpy.signals import SignalHandlerOptions

    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    report = {"schema_version": 1, "status": "starting", "config": asdict(config),
              "started_utc_ns": time.time_ns(), "frames": 0, "forwarded_chunks": 0,
              "receiver_applied_corrections": "Not observable; verify rover status separately"}
    write_json(output / "report.json", report)
    context, node = Context(), None
    try:
        rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node("wr_mapping_ntrip", context=context)
        publisher = node.create_publisher(RTCM, config.topic, 100)
        discovery_deadline = time.monotonic() + config.timeout_seconds
        while publisher.get_subscription_count() == 0:
            if time.monotonic() >= discovery_deadline:
                raise RuntimeError("No ROS subscriber for the configured MAVROS RTCM input")
            rclpy.spin_once(node, timeout_sec=.1)
        guard, decoder = BaseGuard(config), RTCMStream()
        opener = urllib.request.build_opener(NoRedirect)
        # A stalled/closed stream fails rather than silently stitching different bases.
        with opener.open(request(config), timeout=config.timeout_seconds) as response, \
                (output / "corrections.rtcm3").open("xb") as raw, \
                (output / "events.jsonl").open("x") as events, stop_signals() as stop:
            if response.status != 200:
                raise ValueError("NTRIP caster did not return HTTP 200")
            report["status"] = "forwarding"
            start = time.monotonic()
            while not stop.is_set() and (duration is None or time.monotonic() - start < duration):
                rclpy.spin_once(node, timeout_sec=0)
                data = response.read1(4096)
                if not data:
                    raise RuntimeError("NTRIP correction stream closed unexpectedly")
                for frame in decoder.feed(data):
                    details, allowed = guard.check(frame)
                    if not allowed and time.monotonic() - start > config.timeout_seconds:
                        raise RuntimeError("No surveyed-base RTCM 1005/1006 verification before deadline")
                    if allowed:
                        for chunk in mavros_chunks(frame):
                            message = RTCM()
                            message.header.stamp = node.get_clock().now().to_msg()
                            message.data = list(chunk)
                            publisher.publish(message)
                            report["forwarded_chunks"] += 1
                    raw.write(frame)
                    raw.flush()
                    os.fsync(raw.fileno())
                    events.write(json.dumps({**details, "received_monotonic_ns": time.monotonic_ns(),
                                            "received_utc_ns": time.time_ns(), "forwarded": allowed}) + "\n")
                    events.flush()
                    os.fsync(events.fileno())
                    report.update(frames=report["frames"] + 1, base=guard.last_base,
                                  updated_utc_ns=time.time_ns(), discarded_bytes=decoder.discarded_bytes)
                    write_json(output / "report.json", report)
            if not guard.verified or not report["forwarded_chunks"]:
                raise RuntimeError("No verified surveyed-base correction interval was forwarded")
            report.update(status="complete", partial_frame_bytes=len(decoder.pending))
    except BaseException as error:
        report.update(status="failed", error=type(error).__name__)
        # Do not write credential-bearing HTTP exception text to public logs.
        raise RuntimeError("NTRIP forwarding failed; inspect status, caster/base settings and private credentials") from None
    finally:
        if node:
            node.destroy_node()
        if context.ok():
            context.shutdown()
        report["finished_utc_ns"] = time.time_ns()
        report["sha256"] = {p.name: sha256_file(p) for p in output.iterdir()
                            if p.is_file() and p.name != "report.json"}
        write_json(output / "report.json", report)
    return report
