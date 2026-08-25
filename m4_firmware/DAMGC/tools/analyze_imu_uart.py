#!/usr/bin/env python3
"""Capture and summarize DAMGC STM32 IMU packets from a serial port."""

import argparse
import math
import statistics
import struct
import time

import serial


SYNC = b"\xAA\x55"
HEADER = struct.Struct("<2sBBHHH")
IMU_PAYLOAD = struct.Struct("<Q6f4fhH")


def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def quaternion_to_rpy(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_term = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_term)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def unwrap_degrees(values):
    if not values:
        return []
    result = [values[0]]
    for value in values[1:]:
        candidate = value
        while candidate - result[-1] > 180.0:
            candidate -= 360.0
        while candidate - result[-1] < -180.0:
            candidate += 360.0
        result.append(candidate)
    return result


def summarize(name, values, unit):
    mean = statistics.fmean(values)
    stddev = statistics.pstdev(values)
    peak_to_peak = max(values) - min(values)
    print(f"{name:>12}: mean={mean:10.5f} {unit}, std={stddev:9.5f}, p-p={peak_to_peak:9.5f}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--seconds", type=float, default=20.0)
    args = parser.parse_args()

    samples = []
    buffer = bytearray()
    crc_errors = 0
    malformed = 0
    sequence_gaps = 0
    previous_sequence = None
    bytes_received = 0

    with serial.Serial(args.port, args.baud, timeout=0.1) as port:
        port.reset_input_buffer()
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            chunk = port.read(port.in_waiting or 1)
            bytes_received += len(chunk)
            buffer.extend(chunk)
            while True:
                start = buffer.find(SYNC)
                if start < 0:
                    buffer[:] = buffer[-1:]
                    break
                if start:
                    del buffer[:start]
                if len(buffer) < HEADER.size:
                    break
                _, version, msg_type, payload_length, sequence, flags = HEADER.unpack_from(buffer)
                if version != 1 or payload_length > 512:
                    malformed += 1
                    del buffer[:2]
                    continue
                total = HEADER.size + payload_length + 2
                if len(buffer) < total:
                    break
                frame = bytes(buffer[:total])
                del buffer[:total]
                body = frame[2:-2]
                received_crc = struct.unpack_from("<H", frame, total - 2)[0]
                if crc16_ccitt(body) != received_crc:
                    crc_errors += 1
                    continue
                if previous_sequence is not None and sequence != ((previous_sequence + 1) & 0xFFFF):
                    sequence_gaps += (sequence - previous_sequence - 1) & 0xFFFF
                previous_sequence = sequence
                if msg_type != 0x10 or payload_length != IMU_PAYLOAD.size:
                    continue
                values = IMU_PAYLOAD.unpack(frame[HEADER.size:-2])
                timestamp = values[0]
                accel = values[1:4]
                gyro = values[4:7]
                quaternion = values[7:11]
                temperature = values[11] / 100.0
                status = values[12]
                rpy = quaternion_to_rpy(*quaternion)
                samples.append((timestamp, accel, gyro, quaternion, temperature, status, rpy))

    print(f"port={args.port}, baud={args.baud}, duration={args.seconds:.1f}s")
    print(f"bytes={bytes_received}, valid_frames={len(samples)}, crc_errors={crc_errors}, malformed={malformed}, sequence_gaps={sequence_gaps}")
    if len(samples) < 2:
        raise SystemExit("Not enough valid IMU frames; check TX/RX/GND wiring, baud rate, and port ownership.")

    timestamps = [sample[0] for sample in samples]
    intervals = [(b - a) / 1000.0 for a, b in zip(timestamps, timestamps[1:]) if b > a]
    print(f"sample_rate={1000.0 / statistics.fmean(intervals):.2f} Hz, interval_mean={statistics.fmean(intervals):.3f} ms, interval_max={max(intervals):.3f} ms")

    rolls = [sample[6][0] for sample in samples]
    pitches = [sample[6][1] for sample in samples]
    yaws = unwrap_degrees([sample[6][2] for sample in samples])
    summarize("roll", rolls, "deg")
    summarize("pitch", pitches, "deg")
    summarize("yaw", yaws, "deg")
    print(f"   yaw drift: {yaws[-1] - yaws[0]:.5f} deg over {(timestamps[-1] - timestamps[0]) / 1e6:.2f}s")

    for axis, name in enumerate(("accel_x", "accel_y", "accel_z")):
        summarize(name, [sample[1][axis] for sample in samples], "m/s^2")
    for axis, name in enumerate(("gyro_x", "gyro_y", "gyro_z")):
        summarize(name, [math.degrees(sample[2][axis]) for sample in samples], "deg/s")
    summarize("quat_norm", [math.sqrt(sum(value * value for value in sample[3])) for sample in samples], "")
    summarize("temperature", [sample[4] for sample in samples], "C")
    statuses = sorted(set(sample[5] for sample in samples))
    print("status values:", ", ".join(f"0x{value:04X}" for value in statuses))


if __name__ == "__main__":
    main()
