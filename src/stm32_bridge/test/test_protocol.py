import unittest

from stm32_bridge.protocol import (
    FrameParser,
    IMU_PAYLOAD,
    WHEEL_PAYLOAD,
    crc16_ccitt,
    encode_frame,
    pack_velocity,
)


class ProtocolTest(unittest.TestCase):
    def test_crc_known_vector(self):
        self.assertEqual(crc16_ccitt(b"123456789"), 0x29B1)

    def test_round_trip_with_fragmentation(self):
        frame = encode_frame(0x10, 42, b"payload", flags=3)
        parser = FrameParser()
        result = []
        for byte in frame:
            result.extend(parser.feed(bytes([byte])))
        self.assertEqual(result, [(0x10, 42, 3, b"payload")])

    def test_bad_crc_is_dropped(self):
        frame = bytearray(encode_frame(0x11, 1, b"abc"))
        frame[-1] ^= 0xFF
        parser = FrameParser()
        self.assertEqual(parser.feed(frame), [])
        self.assertEqual(parser.crc_errors, 1)

    def test_velocity_packet_uses_protocol_payload(self):
        frame = pack_velocity(-120, 340)
        parser = FrameParser()
        result = parser.feed(frame)
        self.assertEqual(result[0][0], 0x01)
        self.assertEqual(result[0][3], b"\x88\xffT\x01\xc8\x00\x01\x00")


if __name__ == "__main__":
    unittest.main()
