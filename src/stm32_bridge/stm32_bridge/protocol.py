"""Transport-independent binary protocol for the Orin–STM32 link."""

import struct

SYNC = b"\xAA\x55"
VERSION = 1
MAX_PAYLOAD = 512

CMD_VELOCITY = 0x01
CMD_GRIPPER = 0x02
CMD_ESTOP_RESET = 0x03
IMU_DATA = 0x10
WHEEL_STATE = 0x11
SYSTEM_STATE = 0x12
TIME_SYNC_RESPONSE = 0x20
TIME_SYNC_REQUEST = 0x21
ACK_NACK = 0x7F

HEADER = struct.Struct("<2sBBHHH")
CMD_VELOCITY_PAYLOAD = struct.Struct("<hhHH")
IMU_PAYLOAD = struct.Struct("<Q6f4fhH")
WHEEL_PAYLOAD = struct.Struct("<QqqiiH")
SYSTEM_PAYLOAD = struct.Struct("<QHhhBBIH")


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(msg_type: int, seq: int, payload: bytes = b"", flags: int = 0) -> bytes:
    if not 0 <= len(payload) <= MAX_PAYLOAD:
        raise ValueError("payload is too large")
    body = struct.pack("<BBHHH", VERSION, msg_type, len(payload), seq & 0xFFFF, flags & 0xFFFF) + payload
    return SYNC + body + struct.pack("<H", crc16_ccitt(body))


class FrameParser:
    """Incremental parser that can recover after noise or a bad frame."""

    def __init__(self):
        self._buffer = bytearray()
        self.crc_errors = 0
        self.malformed_frames = 0

    def feed(self, data: bytes):
        self._buffer.extend(data)
        frames = []
        while True:
            start = self._buffer.find(SYNC)
            if start < 0:
                self._buffer[:] = self._buffer[-1:]
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < HEADER.size:
                break
            _, version, msg_type, payload_len, seq, flags = HEADER.unpack_from(self._buffer)
            if version != VERSION or payload_len > MAX_PAYLOAD:
                self.malformed_frames += 1
                del self._buffer[:2]
                continue
            total = HEADER.size + payload_len + 2
            if len(self._buffer) < total:
                break
            body = bytes(self._buffer[2:total - 2])
            received_crc = struct.unpack_from("<H", self._buffer, total - 2)[0]
            del self._buffer[:total]
            if crc16_ccitt(body) != received_crc:
                self.crc_errors += 1
                continue
            frames.append((msg_type, seq, flags, body[8:]))
        return frames


def pack_velocity(left_mm_s: int, right_mm_s: int, watchdog_ms: int = 200, control_flags: int = 1) -> bytes:
    return encode_frame(CMD_VELOCITY, 0, CMD_VELOCITY_PAYLOAD.pack(left_mm_s, right_mm_s, watchdog_ms, control_flags))


def unpack_imu(payload: bytes):
    values = IMU_PAYLOAD.unpack(payload)
    return {"timestamp_us": values[0], "accel": values[1:4], "gyro": values[4:7],
            "quaternion": values[7:11], "temperature_cdeg": values[11], "status": values[12]}


def unpack_wheel(payload: bytes):
    values = WHEEL_PAYLOAD.unpack(payload)
    return {"timestamp_us": values[0], "left_ticks": values[1], "right_ticks": values[2],
            "left_mm_s": values[3], "right_mm_s": values[4], "status": values[5]}


def unpack_system(payload: bytes):
    values = SYSTEM_PAYLOAD.unpack(payload)
    return {"timestamp_us": values[0], "battery_mv": values[1], "battery_ma": values[2],
            "motor_temp_cdeg": values[3], "mode": values[4], "estop": values[5],
            "fault_bits": values[6], "last_cmd_age_ms": values[7]}
