#!/usr/bin/env python3
"""Read GNSS evidence through MAVROS; never opens a UART or publishes messages."""

import argparse
import json
import math
import time
from collections import Counter
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=5)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--source-topic", default="/uas1/mavlink_source")
    args = parser.parse_args()
    if not 0 < args.seconds <= 300:
        parser.error("seconds must be finite and in (0, 300]")
    if args.output.exists() or not args.output.parent.is_dir():
        parser.error("output must be new, with an existing parent directory")

    import rclpy
    from mavros.mavlink import convert_to_bytes
    from mavros_msgs.msg import Mavlink
    from pymavlink.dialects.v20 import common
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.convert import message_to_ordereddict
    from sensor_msgs.msg import NavSatFix

    rclpy.init()
    node = rclpy.create_node("wr_mapping_gnss_check")
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    decoder = common.MAVLink(None)
    counts, samples, fixes, errors = Counter(), [], [], []

    def mavlink(message):
        counts[str(message.msgid)] += 1
        if message.msgid != 24:
            return
        try:
            decoded = decoder.parse_char(convert_to_bytes(message))
            if decoded is not None:
                samples.append({"received_monotonic_ns": time.monotonic_ns(),
                                "system_id": message.sysid, "component_id": message.compid,
                                "fields": decoded.to_dict()})
        except (ValueError, common.MAVError) as error:
            errors.append(str(error))

    def fix(message, topic):
        fields = message_to_ordereddict(message)
        valid = message.status.status >= 0 and all(math.isfinite(value) for value in
                                                   (message.latitude, message.longitude, message.altitude))
        # Keep invalid nonfinite coordinates out of JSON; raw GPS packets remain above.
        for key in ("latitude", "longitude", "altitude"):
            if not math.isfinite(fields[key]):
                fields[key] = None
        fields["position_covariance"] = [v if math.isfinite(v) else None
                                         for v in fields["position_covariance"]]
        fixes.append({"topic": topic, "valid_fix": valid, "fields": fields})

    subscriptions = [node.create_subscription(Mavlink, args.source_topic, mavlink, qos)]
    for topic in ("/mavros/global_position/global", "/mavros/global_position/raw/fix"):
        subscriptions.append(node.create_subscription(NavSatFix, topic,
                                                      lambda msg, t=topic: fix(msg, t), qos))
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    report = {"seconds": time.monotonic() - started, "mavlink_message_counts": dict(counts),
              "gps_raw_int": samples, "navsatfix": fixes, "errors": errors,
              "valid_global_fix_seen": any(f["valid_fix"] and f["topic"].endswith("/global")
                                           for f in fixes),
              "note": "Read-only receiver evidence; not RTK or survey accuracy acceptance."}
    with args.output.open("x") as file:
        json.dump(report, file, indent=2, allow_nan=False)
        file.write("\n")
    print(json.dumps({"report": str(args.output), "gps_raw_samples": len(samples),
                      "navsatfix_samples": len(fixes),
                      "valid_global_fix_seen": report["valid_global_fix_seen"], "errors": errors}))
    return 0 if report["valid_global_fix_seen"] and not errors else 2


if __name__ == "__main__":
    raise SystemExit(main())
