import struct
import unittest

from robot_bridge.protocol import (
    CONTROL_FRAME_SIZE, TELEMETRY_FRAME_SIZE, ControlFrame, ProtocolError,
    TelemetryFrame, is_newer_sequence,
)


class ProtocolTests(unittest.TestCase):
    def test_control_format_has_contract_size(self) -> None:
        self.assertEqual(struct.calcsize("<HIB3h5hBHHBB"), 30)
        self.assertEqual(CONTROL_FRAME_SIZE, 30)

    def test_control_known_vector_pack_unpack(self) -> None:
        frame = ControlFrame(
            0x1234, 0x78563412, 2, -1000, 0, 1000,
            1, -2, 3, -4, 5, 1, 100, 32, 0, 1,
        )
        expected = bytes.fromhex(
            "3412123456780218fc0000e8030100feff0300fcff050001640020000001"
        )
        self.assertEqual(frame.pack(), expected)
        self.assertEqual(ControlFrame.unpack(expected), frame)

    def test_rejects_wrong_size_and_out_of_range(self) -> None:
        with self.assertRaises(ProtocolError):
            ControlFrame.unpack(b"\0" * 29)
        invalid = ControlFrame(0, 0, 0, 1001, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        with self.assertRaises(ProtocolError):
            invalid.pack()

    def test_telemetry_roundtrip(self) -> None:
        frame = TelemetryFrame(65535, 88, 23, 1.25, -2.5, (1, 2, 3, 4, 5, 6), 0x1F, 1)
        packed = frame.pack()
        self.assertEqual(TELEMETRY_FRAME_SIZE, 21)
        self.assertEqual(len(packed), 21)
        self.assertEqual(TelemetryFrame.unpack(packed), frame)

    def test_sequence_comparison_wraps(self) -> None:
        self.assertTrue(is_newer_sequence(11, 10))
        self.assertFalse(is_newer_sequence(10, 10))
        self.assertFalse(is_newer_sequence(9, 10))
        self.assertTrue(is_newer_sequence(0, 65535))
        self.assertFalse(is_newer_sequence(65535, 0))


if __name__ == "__main__":
    unittest.main()


