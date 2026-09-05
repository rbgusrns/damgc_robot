#!/usr/bin/env python3
"""Direct Orin controller for leader/follower RX-64 and RX-28.

This intentionally sends write commands only.  It does not use ROS and does
not require a Dynamixel UART on the STM32.  The Orin still needs a USB/RS-485
adapter connected to the Dynamixel bus.
"""

import argparse
import struct
import time

import serial


BAUDRATE = 115200
TORQUE_ENABLE = 24
GOAL_POSITION = 30

PROFILES = {
    "leader": {
        "rx64_id": 33,
        "rx64_min": 450,
        "rx64_max": 775,
        "rx28_id": 2,
        "rx28_min": 1,
        "rx28_max": 1021,
    },
    "follower": {
        "rx64_id": 50,
        "rx64_min": 260,
        "rx64_max": 670,
        "rx28_id": 1,
        "rx28_min": 1,
        "rx28_max": 1021,
    },
}


def checksum(data: bytes) -> int:
    return (~sum(data)) & 0xFF


def build_write_packet(device_id: int, address: int, data: bytes) -> bytes:
    body = bytes((device_id, len(data) + 3, 0x03, address)) + data
    return b"\xff\xff" + body + bytes((checksum(body),))


def clamp(value: int, minimum: int, maximum: int) -> int:
    return max(minimum, min(maximum, value))


def get_profile(robot: str) -> dict:
    try:
        return PROFILES[robot].copy()
    except KeyError as exc:
        raise ValueError(f"unknown robot profile: {robot}") from exc


class DynamixelOrin:
    def __init__(self, port: str, profile: dict, baudrate: int = BAUDRATE):
        self.profile = profile
        self.serial = serial.Serial(port, baudrate, timeout=0.05, write_timeout=0.2)

    def close(self):
        self.serial.close()

    def write(self, device_id: int, address: int, data: bytes):
        self.serial.reset_input_buffer()
        self.serial.write(build_write_packet(device_id, address, data))
        self.serial.flush()
        # RX-64/RX-28 may return a status packet. It is deliberately discarded
        # because this controller is open-loop by design.
        time.sleep(0.005)
        self.serial.reset_input_buffer()

    def set_torque(self, device_id: int, enabled: bool):
        self.write(device_id, TORQUE_ENABLE, bytes((1 if enabled else 0,)))

    def set_position(self, device_id: int, position: int, minimum: int, maximum: int):
        position = clamp(position, minimum, maximum)
        self.write(device_id, GOAL_POSITION, struct.pack("<H", position))


def main():
    parser = argparse.ArgumentParser(description="Direct U2D2 RX-64/RX-28 controller")
    parser.add_argument("--port", required=True, help="Orin USB/RS-485 port, e.g. /dev/ttyUSB0")
    parser.add_argument("--robot", choices=sorted(PROFILES), default="leader")
    parser.add_argument("--rx64-raw", type=int, help="RX-64 raw goal position")
    parser.add_argument("--rx28-raw", type=int, help="RX-28 raw goal position")
    parser.add_argument("--torque", action="store_true", help="enable torque before moving")
    parser.add_argument("--no-torque", action="store_true", help="disable torque")
    args = parser.parse_args()
    profile = get_profile(args.robot)

    if args.torque and args.no_torque:
        parser.error("--torque and --no-torque cannot be used together")
    if args.rx64_raw is None and args.rx28_raw is None and not (args.torque or args.no_torque):
        parser.error("provide a raw goal position or torque option")

    controller = DynamixelOrin(args.port, profile)
    try:
        if args.no_torque:
            controller.set_torque(profile["rx64_id"], False)
            controller.set_torque(profile["rx28_id"], False)
        else:
            if args.torque:
                controller.set_torque(profile["rx64_id"], True)
                controller.set_torque(profile["rx28_id"], True)
            if args.rx64_raw is not None:
                controller.set_position(
                    profile["rx64_id"], args.rx64_raw,
                    profile["rx64_min"], profile["rx64_max"])
            if args.rx28_raw is not None:
                controller.set_position(
                    profile["rx28_id"], args.rx28_raw,
                    profile["rx28_min"], profile["rx28_max"])
    finally:
        controller.close()


if __name__ == "__main__":
    main()
