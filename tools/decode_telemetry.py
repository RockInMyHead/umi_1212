#!/usr/bin/env python3
"""Decode one version-2 (112-byte) or ST version-3 (148-byte) frame to JSON."""
import argparse
import json
import struct
import zlib
from pathlib import Path


def decode(data):
    if len(data) not in (112, 148) or data[:2] != b"IN" or data[3] != len(data) or (data[2], len(data)) not in ((2,112),(3,148)):
        raise ValueError("Expected one 112-byte IN v2 or 148-byte IN v3 packet")
    if zlib.crc32(data[:-4]) != struct.unpack_from("<I", data, len(data)-4)[0]:
        raise ValueError("CRC32 mismatch")
    unpack = lambda fmt, offset: struct.unpack_from("<"+fmt, data, offset)
    sequence, epoch = unpack("II", 4)
    timestamp, status, faults = unpack("QII", 12)
    result = dict(version=data[2], sequence=sequence, origin_epoch=epoch, filter_time_us=timestamp,
                status=status, faults=faults, position_m=unpack("3f", 28),
                velocity_m_s=unpack("3f", 40), position_variance_m2=unpack("3f", 52),
                velocity_variance_m2_s2=unpack("3f", 64), quaternion_wxyz=unpack("4f", 76),
                temperature_c=unpack("f", 92)[0], dropped_samples=unpack("I", 96)[0],
                sample_count=unpack("I", 100)[0], max_compute_us=unpack("I", 104)[0])
    if data[2] == 3:
        result.update(accel_age_us=unpack("I",108)[0], gyro_age_us=unpack("I",112)[0],
                      mag_age_us=unpack("I",116)[0], pressure_age_us=unpack("I",120)[0],
                      accel_time_uncertainty_us=unpack("I",124)[0], gyro_time_uncertainty_us=unpack("I",128)[0],
                      pressure_hpa=unpack("f",132)[0], barometer_bias_variance_m2=unpack("f",136)[0],
                      auxiliary_overruns=unpack("I",140)[0])
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("packet")
    args = parser.parse_args()
    try:
        print(json.dumps(decode(Path(args.packet).read_bytes()), indent=2))
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
