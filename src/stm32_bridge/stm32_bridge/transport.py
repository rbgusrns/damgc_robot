"""Byte transports used by the STM32 bridge."""

import fcntl
import os


I2C_SLAVE = 0x0703


class I2cTransport:
    """Length-prefixed Linux i2c-dev transport for the STM32 mailbox."""

    def __init__(self, device: str, address: int, read_size: int):
        if not 0x08 <= address <= 0x77:
            raise ValueError(f"invalid 7-bit I2C address: 0x{address:02x}")
        if not 1 <= read_size <= 8192:
            raise ValueError(f"invalid I2C read size: {read_size}")

        self.device = device
        self.address = address
        self.read_size = read_size
        self._fd = os.open(device, os.O_RDWR | os.O_CLOEXEC)
        try:
            fcntl.ioctl(self._fd, I2C_SLAVE, address)
        except Exception:
            os.close(self._fd)
            self._fd = None
            raise

    @property
    def is_open(self):
        return self._fd is not None

    @property
    def description(self):
        return f"I2C {self.device} address 0x{self.address:02x}"

    def read(self):
        # Read one complete fixed-size queue slot in a single transaction. A
        # split length/read sequence races the STM32 producer, and the STM32
        # only pops the queue head after the complete slot has been clocked.
        block_size = self.read_size + 2
        block = os.read(self._fd, block_size)
        if len(block) != block_size:
            raise OSError(f"short I2C mailbox read: {len(block)}/{block_size} bytes")

        frame_size = block[0]
        if frame_size == 0:
            return b""
        if frame_size > self.read_size:
            raise OSError(
                f"STM32 I2C frame is too large: {frame_size}/{self.read_size} bytes")
        return block[2:2 + frame_size]

    def write(self, data: bytes):
        written = os.write(self._fd, data)
        if written != len(data):
            raise OSError(f"short I2C write: {written}/{len(data)} bytes")

    def close(self):
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None


class SerialTransport:
    """UART fallback for boards still using the original wiring."""

    def __init__(self, port: str, baudrate: int):
        import serial

        self._serial = serial.Serial(port, baudrate, timeout=0)

    @property
    def is_open(self):
        return self._serial.is_open

    @property
    def description(self):
        return f"UART {self._serial.port}"

    def read(self):
        return self._serial.read(self._serial.in_waiting or 1)

    def write(self, data: bytes):
        self._serial.write(data)

    def close(self):
        self._serial.close()
