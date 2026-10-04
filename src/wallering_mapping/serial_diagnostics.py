"""Bounded, receive-only serial diagnostics; never starts a MAVLink transmitter."""

import math
import time
from collections import Counter


def check_serial(device, bauds=(115200,), seconds=20):
    if (not bauds or len(bauds) > 10 or len(set(bauds)) != len(bauds)
            or any(type(baud) is not int or not 0 < baud <= 4_000_000 for baud in bauds)):
        raise ValueError("Provide 1–10 distinct baud rates in 1..4000000")
    if (type(seconds) not in (int, float) or not math.isfinite(seconds)
            or not 0 < seconds <= 300):
        raise ValueError("Serial listen duration must be finite in (0, 300] seconds per baud")
    try:
        import serial
        from pymavlink.dialects.v20 import common
    except ImportError as error:
        raise ImportError("Install serial diagnostics with pip install -e '.[diagnostics]'") from error

    results = []
    for baud in bauds:
        parser = common.MAVLink(None)
        parser.robust_parsing = True
        counts = Counter()
        received = 0
        started = time.time_ns()
        # exclusive prevents another cooperating serial opener from sharing this
        # port. Stop the existing MAVROS owner before probing. Opening a UART
        # changes local termios and may toggle adapter modem-control lines.
        with serial.Serial(str(device), baudrate=baud, timeout=min(.2, seconds),
                           rtscts=False, dsrdtr=False, xonxoff=False, exclusive=True) as port:
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                data = port.read(4096)
                received += len(data)
                for message in parser.parse_buffer(data) or []:
                    counts[message.get_type()] += 1
        valid = sum(count for kind, count in counts.items() if kind != "BAD_DATA")
        heartbeats = counts["HEARTBEAT"]
        finding = ("heartbeat_received" if heartbeats else "mavlink_without_heartbeat" if valid
                   else "bytes_without_valid_mavlink" if received else "no_bytes")
        results.append({"baud": baud, "started_utc_ns": started, "duration_seconds": seconds,
                        "received_bytes": received, "valid_messages": valid,
                        "messages": dict(counts), "finding": finding})
    return {"schema_version": 1, "device": str(device), "receive_only": True,
            "ready": any(result["finding"] == "heartbeat_received" for result in results),
            "results": results,
            "note": "A heartbeat confirms MAVLink reception only. Check MAVROS state, timing and "
                    "required telemetry separately. No bytes alone does not distinguish wiring from configuration."}
