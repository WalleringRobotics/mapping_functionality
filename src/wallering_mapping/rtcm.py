"""RTCM 3 framing, CRC and reference-station evidence; no receiver claims."""


def crc24q(data):
    value = 0
    for byte in data:
        value ^= byte << 16
        for _ in range(8):
            value <<= 1
            if value & 0x1000000:
                value ^= 0x1864CFB
    return value & 0xFFFFFF


def bits(payload, start, width, signed=False):
    if start + width > len(payload) * 8:
        raise ValueError("Truncated RTCM payload")
    value = (int.from_bytes(payload, "big") >> (len(payload) * 8 - start - width)) & ((1 << width) - 1)
    return value - (1 << width) if signed and value & (1 << (width - 1)) else value


def describe(frame):
    if len(frame) < 8 or frame[0] != 0xD3 or frame[1] & 0xFC:
        raise ValueError("Invalid RTCM 3 header")
    length = ((frame[1] & 3) << 8) | frame[2]
    if len(frame) != length + 6 or crc24q(frame[:-3]) != int.from_bytes(frame[-3:], "big"):
        raise ValueError("RTCM length/CRC mismatch")
    payload = frame[3:-3]
    message_type = bits(payload, 0, 12)
    result = {"message_type": message_type, "bytes": len(frame)}
    has_station = (1001 <= message_type <= 1013 or message_type in (1033, 1230)
                   or any(first <= message_type <= first + 6 for first in (1071, 1081, 1091, 1101, 1111, 1121)))
    if has_station:
        result["station_id"] = bits(payload, 12, 12)
    if message_type in (1005, 1006):
        result["itrf_realization_code"] = bits(payload, 24, 6)
        result["base_arp_ecef_m"] = [bits(payload, start, 38, signed=True) * .0001 for start in (34, 74, 114)]
        if message_type == 1006:
            result["antenna_height_m"] = bits(payload, 152, 16) * .0001
    return result


class RTCMStream:
    def __init__(self):
        self.pending = bytearray()
        self.discarded_bytes = 0

    def feed(self, data):
        self.pending.extend(data)
        frames = []
        while self.pending:
            if self.pending[0] != 0xD3:
                del self.pending[0]
                self.discarded_bytes += 1
                continue
            if len(self.pending) < 3:
                break
            if self.pending[1] & 0xFC:
                raise ValueError("Invalid RTCM reserved header bits")
            size = (((self.pending[1] & 3) << 8) | self.pending[2]) + 6
            if len(self.pending) < size:
                break
            frame = bytes(self.pending[:size])
            describe(frame)
            frames.append(frame)
            del self.pending[:size]
        return frames


def mavros_chunks(frame):
    """MAVROS drops ROS RTCM payloads >720 bytes; preserve the byte-stream order."""
    describe(frame)
    return [frame[start:start + 720] for start in range(0, len(frame), 720)]
