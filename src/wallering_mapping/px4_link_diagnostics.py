"""Bounded MAVLink link audit. Sends heartbeats, parameter reads and TIMESYNC only."""

import argparse
import fcntl
import json
import math
import statistics
import struct
import termios
import time
from collections import Counter
from pathlib import Path


PARAMETERS = ("MAV_1_CONFIG", "MAV_1_MODE", "SER_TEL2_BAUD", "MAV_1_RATE",
              "MAV_1_RADIO_CTL", "MAV_1_FLOW_CTRL")
GROUPS = {
    "imu": ("HIGHRES_IMU", "RAW_IMU", "SCALED_IMU", "SCALED_IMU2", "SCALED_IMU3"),
    "attitude": ("ATTITUDE", "ATTITUDE_QUATERNION"),
    "gnss": ("GPS_RAW_INT", "GPS2_RAW"),
}


def statistics_ns(values):
    if not values:
        return {"count": 0, "min": None, "median": None, "p95": None, "max": None,
                "stddev": None}
    ordered = sorted(values)
    return {"count": len(values), "min": ordered[0], "median": statistics.median(values),
            "p95": ordered[math.ceil(.95 * len(values)) - 1], "max": ordered[-1],
            "stddev": statistics.pstdev(values)}


def audit_link(device, baud, flow_control, seconds=60):
    if type(baud) is not int or not 0 < baud <= 4_000_000:
        raise ValueError("baud must be an integer in 1..4000000")
    if flow_control not in ("off", "rtscts"):
        raise ValueError("flow_control must be off or rtscts")
    if not math.isfinite(seconds) or not 0 < seconds <= 300:
        raise ValueError("seconds must be finite in (0, 300]")
    import serial
    from pymavlink.dialects.v20 import common

    report = {"schema_version": 1, "started_utc_ns": time.time_ns(), "device": str(device),
              "host_settings": {"baud": baud, "flow_control": flow_control, "format": "8N1"},
              "requested_seconds": seconds, "target": None, "parameters": {},
              "error": None, "complete": False}
    counts, sent = Counter(), Counter()
    bytes_received = 0
    rtts, offsets, pending = [], [], {}
    target = None
    start = time.monotonic()
    port = serial.Serial(port=None, baudrate=baud, timeout=.01, write_timeout=.2,
                         rtscts=flow_control == "rtscts", dsrdtr=False, xonxoff=False,
                         exclusive=True)
    # DTR is not used by the MAVLink UART. RTS remains asserted without hardware
    # flow control so a peer wired to CTS is not intentionally told to stop TX.
    port.dtr = False
    if flow_control == "off":
        port.rts = True
    try:
        port.port = str(device)
        port.open()
        # Kernel exclusion also rejects non-cooperating subsequent openers.
        # Check existing owners before starting; TIOCEXCL cannot evict them.
        fcntl.ioctl(port.fileno(), termios.TIOCEXCL)
        try:
            report["host_settings"]["cts_asserted_at_open"] = port.cts
        except OSError:
            report["host_settings"]["cts_asserted_at_open"] = None
        mav = common.MAVLink(port, srcSystem=255, srcComponent=191)
        mav.robust_parsing = True
        start = time.monotonic()
        heartbeat_at = sync_at = params_at = start
        while time.monotonic() - start < seconds:
            now = time.monotonic()
            if now >= heartbeat_at:
                mav.heartbeat_send(common.MAV_TYPE_ONBOARD_CONTROLLER,
                                   common.MAV_AUTOPILOT_INVALID, 0, 0,
                                   common.MAV_STATE_ACTIVE)
                sent["HEARTBEAT"] += 1
                heartbeat_at = now + 1
            if target and now >= sync_at:
                stamp = time.monotonic_ns()
                mav.timesync_send(0, stamp)
                pending[stamp] = stamp
                sent["TIMESYNC_REQUEST"] += 1
                sync_at = now + .1
                pending = {k: v for k, v in pending.items() if stamp - v < 5_000_000_000}
            if target and now >= params_at:
                for name in PARAMETERS:
                    if name not in report["parameters"]:
                        mav.param_request_read_send(*target, name.encode("ascii"), -1)
                        sent["PARAM_REQUEST_READ"] += 1
                params_at = now + 3
            data = port.read(min(16384, port.in_waiting or 1))
            received_ns = time.monotonic_ns()
            bytes_received += len(data)
            for message in mav.parse_buffer(data) or []:
                kind = message.get_type()
                if kind == "BAD_DATA":
                    counts["BAD_DATA"] += 1
                    continue
                peer = (message.get_srcSystem(), message.get_srcComponent())
                counts[f"{peer[0]}:{peer[1]}:{kind}"] += 1
                if peer == (255, 191):
                    raise RuntimeError("Received our source ID: possible echo or component ID conflict")
                if (kind == "HEARTBEAT" and message.autopilot == common.MAV_AUTOPILOT_PX4
                        and peer[1] == common.MAV_COMP_ID_AUTOPILOT1):
                    if target and peer != target:
                        raise RuntimeError("More than one PX4 autopilot on this link")
                    target = peer
                    report["target"] = {"system": peer[0], "component": peer[1]}
                if peer != target:
                    continue
                if kind == "PARAM_VALUE" and message.param_id in PARAMETERS:
                    # PX4 transports INT32 parameters as raw bytes in the float field.
                    if message.param_type == common.MAV_PARAM_TYPE_INT32:
                        packet = message.get_msgbuf()
                        start_byte = 10 if packet[0] == 0xFD else 6
                        value = struct.unpack("<i", packet[start_byte:start_byte + 4])[0]
                        report["parameters"][message.param_id] = value
                if kind == "TIMESYNC":
                    if message.tc1 == 0:
                        mav.timesync_send(time.monotonic_ns(), message.ts1)
                        sent["TIMESYNC_REPLY"] += 1
                    elif message.ts1 in pending:
                        tx_ns = pending.pop(message.ts1)
                        rtts.append(received_ns - tx_ns)
                        offsets.append((received_ns + tx_ns) // 2 - message.tc1)
    except (OSError, serial.SerialException, RuntimeError) as error:
        report["error"] = f"{type(error).__name__}: {error}"
    finally:
        try:
            if port.is_open:
                # Drop this probe's queued output on a stalled CTS line.
                port.reset_output_buffer()
        except (OSError, serial.SerialException) as error:
            report["error"] = report["error"] or f"{type(error).__name__}: {error}"
        finally:
            port.close()
    elapsed = time.monotonic() - start
    target_counts = {kind: count for key, count in counts.items()
                     if target and key.startswith(f"{target[0]}:{target[1]}:")
                     for kind in [key.split(":", 2)[2]]}
    present = {group: any(target_counts.get(kind, 0) for kind in kinds)
               for group, kinds in GROUPS.items()}
    heartbeat_count = target_counts.get("HEARTBEAT", 0)
    missing = [group for group, seen in present.items() if not seen]
    if not heartbeat_count:
        missing.append("PX4 heartbeat")
    if not rtts:
        missing.append("matched TIMESYNC replies")
    missing_params = [name for name in PARAMETERS if name not in report["parameters"]]
    report.update({"duration_seconds": elapsed, "received_bytes": bytes_received,
                   "received_messages_by_source": dict(counts), "host_writes": dict(sent),
                   "px4_stream_rates_hz": {kind: count / elapsed for kind, count in target_counts.items()},
                   "px4_heartbeat_count": heartbeat_count, "telemetry_present": present,
                   "missing_data": missing, "missing_parameters": missing_params,
                   "timesync": {"rtt_ns": statistics_ns(rtts),
                                "local_monotonic_minus_px4_ns": statistics_ns(offsets)},
                   "bidirectional_protocol_evidence": bool(rtts or report["parameters"]),
                   "peer_receipt_of_our_heartbeat": "not_acknowledged_by_protocol",
                   "complete": not (missing or missing_params or report["error"]),
                   "note": "Rates are receiver counts / observation duration, not configured rates. "
                           "TIMESYNC statistics are unfiltered host observations, not MAVROS convergence. "
                           "Host writes may be queued behind CTS; they do not prove wire transmission or "
                           "PX4 receipt. No parameters or stream rates set."})
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", required=True)
    parser.add_argument("--baud", type=int, required=True)
    parser.add_argument("--flow-control", choices=("off", "rtscts"), required=True)
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args(argv)
    # Reserve the report before opening the device; never overwrite prior evidence.
    with args.report.open("x") as output:
        result = audit_link(args.device, args.baud, args.flow_control, args.seconds)
        json.dump(result, output, indent=2, allow_nan=False)
        output.write("\n")
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result["complete"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
