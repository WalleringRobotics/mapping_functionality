import base64
from dataclasses import replace

import pytest

from wallering_mapping.ntrip import BaseGuard, NtripConfig, request
from wallering_mapping.rtcm import RTCMStream, crc24q, describe, mavros_chunks


def frame(payload):
    head = bytes([0xD3, len(payload) >> 8, len(payload) & 255]) + payload
    return head + crc24q(head).to_bytes(3, 'big')


def base_frame(station=42, xyz=(4200000, 1000000, 4700000), kind=1005):
    parts = [(kind, 12), (station, 12), (0, 6), (0, 4)]
    for index, coordinate in enumerate(xyz):
        parts.append((round(coordinate * 10000) & ((1 << 38) - 1), 38))
        if index < 2:
            parts.append((0, 2))
    if kind == 1006:
        parts.append((12500, 16))
    binary = ''.join(format(value, f'0{width}b') for value, width in parts)
    assert len(binary) % 8 == 0
    return frame(int(binary, 2).to_bytes(len(binary) // 8, 'big'))


def config():
    return NtripConfig('https://caster.example.invalid/base', 42, [4200000, 1000000, 4700000],
                       .01, 'private survey record')


def test_stream_reassembles_crc_verified_frames_and_detects_corruption():
    packet = base_frame()
    decoder = RTCMStream()
    assert decoder.feed(b'noise' + packet[:5]) == []
    assert decoder.feed(packet[5:] + packet) == [packet, packet]
    assert decoder.discarded_bytes == 5
    damaged = bytearray(packet)
    damaged[7] ^= 1
    with pytest.raises(ValueError, match='CRC'):
        RTCMStream().feed(damaged)
    assert crc24q(b'123456789') == 0xCDE703


def test_station_and_surveyed_arp_guard_and_1006_height():
    guard = BaseGuard(config())
    details, allowed = guard.check(base_frame())
    assert allowed and details['base_arp_ecef_m'] == [4200000, 1000000, 4700000]
    assert describe(base_frame(kind=1006))['antenna_height_m'] == 1.25
    with pytest.raises(ValueError, match='station ID'):
        guard.check(base_frame(station=43))
    with pytest.raises(ValueError, match='coordinates'):
        guard.check(base_frame(xyz=(4200000.02, 1000000, 4700000)))
    assert describe(base_frame(xyz=(-4200000, 1000000, -4700000)))['base_arp_ecef_m'][0] == -4200000


def test_mavros_fragment_limit_preserves_entire_large_rtcm_frame():
    packet = frame(bytes([0x43, 0x20, 42]) + bytes(900))
    pieces = mavros_chunks(packet)
    assert len(pieces) == 2 and max(map(len, pieces)) <= 720
    assert b''.join(pieces) == packet
    assert RTCMStream().feed(b''.join(pieces)) == [packet]


def test_credentials_are_environment_only_https_and_redirect_free(monkeypatch):
    monkeypatch.setenv('WR_NTRIP_USERNAME', 'operator')
    monkeypatch.setenv('WR_NTRIP_PASSWORD', 'private-token')
    req = request(config())
    assert req.get_header('Authorization') == 'Basic ' + base64.b64encode(b'operator:private-token').decode()
    assert 'private-token' not in repr(config())
    with pytest.raises(ValueError, match='HTTPS'):
        request(replace(config(), caster_url='http://caster.example.invalid/base'))
    with pytest.raises(ValueError, match='without credentials'):
        replace(config(), caster_url='https://operator:private-token@caster.example.invalid/base')
