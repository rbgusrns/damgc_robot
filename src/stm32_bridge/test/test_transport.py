import unittest
from unittest.mock import patch

from stm32_bridge.transport import I2C_SLAVE, I2cTransport


class I2cTransportTest(unittest.TestCase):
    @patch("stm32_bridge.transport.fcntl.ioctl")
    @patch("stm32_bridge.transport.os.open", return_value=12)
    def test_opens_device_and_selects_7bit_address(self, open_mock, ioctl_mock):
        transport = I2cTransport("/dev/i2c-7", 0x42, 64)

        open_mock.assert_called_once()
        ioctl_mock.assert_called_once_with(12, I2C_SLAVE, 0x42)
        self.assertEqual(transport.description, "I2C /dev/i2c-7 address 0x42")

    def test_rejects_reserved_or_out_of_range_address(self):
        for address in (0x07, 0x78):
            with self.subTest(address=address):
                with self.assertRaises(ValueError):
                    I2cTransport("/dev/i2c-7", address, 64)

    @patch("stm32_bridge.transport.os.read")
    @patch("stm32_bridge.transport.fcntl.ioctl")
    @patch("stm32_bridge.transport.os.open", return_value=12)
    def test_reads_one_length_prefixed_frame(self, _open_mock, _ioctl_mock, read_mock):
        read_mock.return_value = b"\x04\x1f\xaa\x55\x01\x02" + bytes(60)
        transport = I2cTransport("/dev/i2c-7", 0x42, 64)

        self.assertEqual(transport.read(), b"\xaa\x55\x01\x02")
        read_mock.assert_called_once_with(12, 66)

    @patch("stm32_bridge.transport.os.read", return_value=bytes(66))
    @patch("stm32_bridge.transport.fcntl.ioctl")
    @patch("stm32_bridge.transport.os.open", return_value=12)
    def test_empty_queue_slot_returns_no_frame(
            self, _open_mock, _ioctl_mock, _read_mock):
        transport = I2cTransport("/dev/i2c-7", 0x42, 64)

        self.assertEqual(transport.read(), b"")

    @patch("stm32_bridge.transport.os.read", return_value=bytes(65))
    @patch("stm32_bridge.transport.fcntl.ioctl")
    @patch("stm32_bridge.transport.os.open", return_value=12)
    def test_rejects_short_queue_slot(self, _open_mock, _ioctl_mock, _read_mock):
        transport = I2cTransport("/dev/i2c-7", 0x42, 64)

        with self.assertRaises(OSError):
            transport.read()

    @patch("stm32_bridge.transport.os.read", return_value=b"\x41\x00" + bytes(64))
    @patch("stm32_bridge.transport.fcntl.ioctl")
    @patch("stm32_bridge.transport.os.open", return_value=12)
    def test_rejects_frame_larger_than_configured_limit(
            self, _open_mock, _ioctl_mock, _read_mock):
        transport = I2cTransport("/dev/i2c-7", 0x42, 64)

        with self.assertRaises(OSError):
            transport.read()

    @patch("stm32_bridge.transport.os.write", return_value=3)
    @patch("stm32_bridge.transport.fcntl.ioctl")
    @patch("stm32_bridge.transport.os.open", return_value=12)
    def test_writes_complete_frame(self, _open_mock, _ioctl_mock, write_mock):
        transport = I2cTransport("/dev/i2c-7", 0x42, 64)

        transport.write(b"abc")
        write_mock.assert_called_once_with(12, b"abc")


if __name__ == "__main__":
    unittest.main()
