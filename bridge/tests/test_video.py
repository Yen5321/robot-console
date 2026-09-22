import urllib.request
import unittest

from robot_bridge.interfaces import ICameraSource
from robot_bridge.video import MjpegServer


class FakeCamera(ICameraSource):
    def get_jpeg_frame(self) -> bytes:
        return b"\xff\xd8fake-jpeg\xff\xd9"

    def get_depth_frame(self):
        return None


class MjpegTests(unittest.TestCase):
    def setUp(self) -> None:
        self.server = MjpegServer(FakeCamera(), "127.0.0.1", 0, max_fps=100)
        self.server.start()
        self.port = self.server._server.server_address[1]

    def tearDown(self) -> None:
        self.server.close()

    def test_health_and_mjpeg_stream(self) -> None:
        with urllib.request.urlopen(f"http://127.0.0.1:{self.port}/healthz", timeout=1) as response:
            self.assertEqual(response.read(), b"ok\n")
        with urllib.request.urlopen(f"http://127.0.0.1:{self.port}/stream.mjpg", timeout=1) as response:
            self.assertIn("multipart/x-mixed-replace", response.headers["Content-Type"])
            self.assertIn(b"--frame\r\n", response.read(80))


if __name__ == "__main__":
    unittest.main()

