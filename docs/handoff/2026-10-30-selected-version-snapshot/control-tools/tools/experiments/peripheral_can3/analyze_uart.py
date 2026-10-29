#!/usr/bin/env python3
"""Offline framing check of captured PDO UART chunks; never controls hardware.

HI91 framing/CRC follows src/hardware/imu/hi91_protocol.cpp; SBUS framing
follows src/hardware/remote/wfly_sbus_protocol.cpp. A valid SBUS frame with
failsafe/frame_lost is reported separately from framing failure.
"""
import csv
import json
import math
import struct
import sys


def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xffff
    return crc


def analyze(path):
    streams = {p: bytearray() for p in (2, 5)}
    stats = {p: dict(blocks=0, bytes=0, discontinuities=0, sequence_errors=0,
                     frames=0, discarded_bytes=0) for p in streams}
    last_seq = {p: 0 for p in streams}
    stats[2].update(crc_errors=0, nonfinite_frames=0, device_time_nonadvancing=0)
    stats[5].update(frame_lost=0, failsafe=0)
    last_device_ms = None
    for row in csv.DictReader(open(path)):
        port, seq, flags = map(int, (row['port'], row['sequence'], row['flags']))
        data = bytes.fromhex(row['data_hex'])
        s, b = stats[port], streams[port]
        s['blocks'] += 1
        s['bytes'] += len(data)
        if flags & 4:
            s['discontinuities'] += 1
            b.clear()
        elif seq != ((last_seq[port] + 1) & 0xffffffff):
            s['sequence_errors'] += 1
            b.clear()
        last_seq[port] = seq
        b.extend(data)
        while b:
            if port == 2:
                if len(b) < 2:
                    break
                if b[:2] != b'\x5a\xa5':
                    del b[0]
                    s['discarded_bytes'] += 1
                    continue
                if len(b) < 6:
                    break
                length = int.from_bytes(b[2:4], 'little')
                if not 76 <= length <= 256:
                    del b[0]
                    s['discarded_bytes'] += 1
                    continue
                if len(b) < length + 6:
                    break
                if crc16(b[:4] + b[6:6+length]) != int.from_bytes(b[4:6], 'little'):
                    s['crc_errors'] += 1
                    del b[0]
                    continue
                if b[6] == 0x91:
                    values = struct.unpack_from('<7f', b, 6 + 48)
                    if not all(math.isfinite(v) for v in values):
                        s['nonfinite_frames'] += 1
                    device_ms = int.from_bytes(b[14:18], 'little')
                    if last_device_ms is not None:
                        delta = (device_ms - last_device_ms) & 0xffffffff
                        if not 0 < delta < 0x80000000:
                            s['device_time_nonadvancing'] += 1
                    last_device_ms = device_ms
                    s['frames'] += 1
                del b[:6+length]
            else:
                if b[0] != 0x0f:
                    del b[0]
                    s['discarded_bytes'] += 1
                    continue
                if len(b) < 25:
                    break
                if b[24] != 0:
                    del b[0]
                    s['discarded_bytes'] += 1
                    continue
                s['frames'] += 1
                s['frame_lost'] += bool(b[23] & 4)
                s['failsafe'] += bool(b[23] & 8)
                del b[:25]
    for port, b in streams.items():
        stats[port]['trailing_bytes'] = len(b)
    return stats


if __name__ == '__main__':
    print(json.dumps(analyze(sys.argv[1]), indent=2))
