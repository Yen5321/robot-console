import json
import unittest
import urllib.request

from robot_bridge.hardware.piper6 import PiperArmController
from robot_bridge.video import MjpegServer
from tests.test_piper6 import FakePiperSdk, make_controller
from tests.test_video import FakeCamera


class DiagnosticsTests(unittest.TestCase):
    def test_out_of_range_pose_reports_exact_reason_without_command(self):
        sdk = FakePiperSdk()
        sdk.pose.RY_axis = 100000
        arm = make_controller(_sdk_interface=sdk)
        report = arm.get_diagnostics()
        self.assertIn('pitch=100.000', report['error'])
        self.assertEqual(report['pose'][4], 100)
        self.assertEqual(report['enabled'], [True]*6)
        self.assertFalse(any(c[0] == 'pose' for c in sdk.commands))

    def test_zero_input_holds_once_and_next_jog_resyncs(self):
        sdk = FakePiperSdk()
        arm = make_controller(_sdk_interface=sdk, _time_fn=lambda: 1)
        arm.jog_cartesian(1, 0, 0, 0, 0)
        count = len(sdk.commands)
        arm.jog_cartesian(0, 0, 0, 0, 0)
        self.assertEqual(len(sdk.commands), count+2)
        self.assertEqual(sdk.commands[-1], ('pose', 100000, 0, 200000, 0, 10000, 0))
        arm.jog_cartesian(0, 0, 0, 0, 0)
        self.assertEqual(len(sdk.commands), count+2)
        sdk.pose.X_axis = 150000
        arm.jog_cartesian(1, 0, 0, 0, 0)
        self.assertEqual(sdk.commands[-1][1], 150200)

    def test_axis_jog_mapping(self):
        sdk = FakePiperSdk()
        arm = make_controller(_sdk_interface=sdk, _time_fn=lambda: 1)
        arm.jog_cartesian(0, 0, .2, -.2, .2)
        self.assertEqual(sdk.commands[-1], ('pose', 100000, 0, 200040, 0, 9980, 20))

    def test_http_diagnostics_is_read_only(self):
        sdk = FakePiperSdk()
        arm = make_controller(_sdk_interface=sdk)
        server = MjpegServer(FakeCamera(), '127.0.0.1', 0,
                             diagnostics=lambda: {'arm': arm.get_diagnostics()})
        server.start()
        try:
            port = server._server.server_address[1]
            with urllib.request.urlopen(f'http://127.0.0.1:{port}/diagnostics', timeout=2) as response:
                report = json.load(response)
                self.assertEqual(response.headers['Cache-Control'], 'no-store')
            self.assertEqual(report['arm']['pose'], [100, 0, 200, 0, 10, 0])
            self.assertEqual(sdk.commands, [('connect',)])
        finally:
            server.close()

    def test_failed_enable_disconnects_sdk(self):
        sdk = FakePiperSdk()
        with self.assertRaises(RuntimeError):
            PiperArmController(_sdk_interface=sdk, enable_on_start=True, enable_timeout_s=0)
        self.assertEqual(sdk.commands[-1], ('disconnect',))


if __name__ == '__main__':
    unittest.main()

