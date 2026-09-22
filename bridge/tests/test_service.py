from __future__ import annotations

import socket
import unittest

from robot_bridge.interfaces import IArmController, ICameraSource, ILaserController
from robot_bridge.protocol import ControlFrame
from robot_bridge.service import BridgeService


class FakeArm(IArmController):
    def __init__(self) -> None:
        self.jogs: list[tuple[float, ...]] = []
        self.estops = 0

    def jog_cartesian(self, *values: float) -> None:
        self.jogs.append(values)

    def set_height_delta(self, delta_mm: float) -> None: pass
    def get_joint_margins(self) -> list[float]: return [50.0] * 6
    def get_pose_safety_ok(self) -> bool: return True
    def home(self) -> None: pass
    def estop(self) -> None: self.estops += 1


class FakeCamera(ICameraSource):
    def get_jpeg_frame(self) -> bytes: return b"jpeg"
    def get_depth_frame(self): return None


class FakeLaser(ILaserController):
    def __init__(self) -> None:
        self.requests: list[tuple[bool, int, float, bool]] = []
        self.off_count = 0
    def set_request(self, enabled: bool, power: int, speed: float, safe: bool) -> None:
        self.requests.append((enabled, power, speed, safe))
    def force_off(self) -> None: self.off_count += 1
    def is_active(self) -> bool: return False


class ResetThenStopSocket:
    def __init__(self) -> None:
        self.calls = 0
        self.stop_event = None
    def setsockopt(self, *args) -> None: pass
    def settimeout(self, value) -> None: pass
    def recvfrom(self, size):
        self.calls += 1
        if self.calls == 1:
            raise ConnectionResetError("simulated Windows UDP peer close")
        self.stop_event.set()
        raise socket.timeout()
    def close(self) -> None: pass


def control(seq: int, *, estop: int = 0) -> bytes:
    return ControlFrame(
        seq, 0, 2, 0, 0, 0, 0, 100, 0, 0, 0,
        0, 0, 0, estop, seq & 1,
    ).pack()


class ServiceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.arm, self.camera, self.laser = FakeArm(), FakeCamera(), FakeLaser()
        self.service = BridgeService(
            self.arm, self.camera, self.laser,
            {"udp_port": 9000, "http_port": 8080, "telemetry_hz": 15},
            {}, _legacy_test_transport=True, udp_socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM),
            start_video=False,
        )

    def tearDown(self) -> None:
        self.service.close()

    def test_old_sequence_is_dropped(self) -> None:
        self.assertTrue(self.service.process_datagram(control(10), ("127.0.0.1", 5000)))
        self.assertFalse(self.service.process_datagram(control(9), ("127.0.0.1", 5000)))
        self.assertEqual(len(self.arm.jogs), 1)

    def test_new_source_endpoint_establishes_new_sequence_baseline(self) -> None:
        self.assertTrue(self.service.process_datagram(control(500), ("127.0.0.1", 5000)))
        self.assertTrue(self.service.process_datagram(control(0), ("127.0.0.1", 5001)))
        self.assertEqual(len(self.arm.jogs), 2)

    def test_estop_bypasses_old_sequence_filter(self) -> None:
        self.service.process_datagram(control(10), ("127.0.0.1", 5000))
        self.assertFalse(self.service.process_datagram(control(9, estop=1), ("127.0.0.1", 5000)))
        self.assertEqual(self.arm.estops, 1)
        self.assertEqual(self.laser.off_count, 1)

    def test_unknown_interlocks_are_unsafe(self) -> None:
        self.service.process_datagram(control(1), ("127.0.0.1", 5000))
        telemetry = self.service.build_telemetry()
        self.assertEqual(telemetry.interlock_bits, 0b00011)

    def test_udp_loop_survives_windows_peer_reset(self) -> None:
        self.service.close()
        fake_socket = ResetThenStopSocket()
        self.service = BridgeService(
            self.arm, self.camera, self.laser,
            {"udp_port": 9000, "http_port": 8080, "telemetry_hz": 15},
            {}, _legacy_test_transport=True, udp_socket=fake_socket, start_video=False,
        )
        fake_socket.stop_event = self.service._stop
        self.service._udp_loop()
        self.assertEqual(fake_socket.calls, 2)


if __name__ == "__main__":
    unittest.main()


