import math
import socket
import unittest

from robot_bridge.hardware.piper6 import PiperArmController
from robot_bridge.service import BridgeService
from robot_bridge.protocol import ControlFrame
from tests.test_piper6 import FakePiperSdk, make_controller
from tests.test_service import FakeCamera, FakeLaser, control


class Clock:
    def __init__(self): self.now = 1.0
    def __call__(self): return self.now


class JogSafetyTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.sdk = FakePiperSdk()
        self.arm = make_controller(_sdk_interface=self.sdk, _time_fn=self.clock)

    def test_positive_timestamp_does_not_count_as_fresh_on_first_observation(self):
        sdk = FakePiperSdk(); sdk.frozen = True
        arm = PiperArmController(_sdk_interface=sdk, _time_fn=self.clock)
        self.assertFalse(arm.get_diagnostics()['ready'])
        with self.assertRaisesRegex(RuntimeError, 'not fresh'):
            arm.jog_cartesian(1,0,0,0,0)
        self.assertEqual(sdk.commands, [('connect',)])

    def freeze_at_current_observation(self):
        self.sdk.frozen = True
        # All public feedback clocks now have the same final stamp. Observe it,
        # then keep both pose and Hz unchanged while monotonic time advances.
        self.arm.check_feedback()

    def test_frozen_feedback_stops_at_250ms_even_with_positive_hz(self):
        self.arm.jog_cartesian(0,1,0,0,0)
        self.freeze_at_current_observation()
        self.clock.now += .249
        self.arm.check_feedback()
        self.clock.now += .002
        with self.assertRaisesRegex(RuntimeError, 'not fresh'):
            self.arm.check_feedback()
        self.assertEqual(self.sdk.commands[-1], ('estop',1))
        self.sdk.frozen = False
        self.arm.check_feedback()
        with self.assertRaises(RuntimeError): self.arm.jog_cartesian(0,1,0,0,0)

    def test_release_with_stale_pose_estops_instead_of_sending_stale_hold(self):
        self.arm.jog_cartesian(0,1,0,0,0)
        self.freeze_at_current_observation()
        count = len(self.sdk.commands)
        self.clock.now += .3
        with self.assertRaises(RuntimeError): self.arm.jog_cartesian(0,0,0,0,0)
        self.assertEqual(self.sdk.commands[count:], [('estop',1)])

    def test_translation_limit_and_orientation_preserved(self):
        self.arm.jog_cartesian(1,1,1,0,0)
        cmd=self.sdk.commands[-1]
        delta=[cmd[1]-100000,cmd[2],cmd[3]-200000]
        self.assertLessEqual(math.sqrt(sum(v*v for v in delta))/1000, .201)
        self.assertEqual(cmd[4:], (0,10000,0))
        self.assertEqual(self.sdk.commands[-2], ('mode',1,2,10,0))

    def test_release_resyncs_even_after_more_than_twenty_mm_target_lead(self):
        for _ in range(105):
            self.clock.now += .05
            self.arm.jog_cartesian(1,0,0,0,0)
        self.assertGreater(self.arm._target[0]-100,20)
        self.arm.hold_position()
        self.assertEqual(self.sdk.commands[-1][1:4],(100000,0,200000))

    def test_hold_submits_actual_not_accumulated_target_and_resyncs(self):
        self.arm.jog_cartesian(0,1,0,0,0)
        self.sdk.pose.Y_axis = 80
        self.arm.hold_position()
        self.assertEqual(self.sdk.commands[-1][2],80)
        self.sdk.pose.Y_axis = 100
        self.clock.now += .05
        self.arm.jog_cartesian(0,1,0,0,0)
        self.assertEqual(self.sdk.commands[-1][2],300)

    def test_hold_overshoot_faults_and_records_excursion(self):
        self.arm._safety['hold_translation_enabled']=True
        self.arm.jog_cartesian(0,1,0,0,0)
        self.arm.hold_position()
        self.sdk.pose.Y_axis=2100
        with self.assertRaisesRegex(RuntimeError, 'tolerance'):
            self.arm.check_feedback()
        self.assertGreater(self.arm.get_diagnostics()['last_hold']['max_translation_mm'],2)

    def test_disabled_or_teach_mode_blocks_target(self):
        self.sdk.enabled=False
        with self.assertRaisesRegex(RuntimeError, '未使能'): self.arm.jog_cartesian(1,0,0,0,0)
        self.sdk.enabled=True; self.sdk.ctrl_mode=2
        with self.assertRaisesRegex(RuntimeError, '控制模式'): self.arm.jog_cartesian(1,0,0,0,0)
        self.assertFalse(any(c[0]=='pose' for c in self.sdk.commands))

    def test_no_unsafe_joint_zero_home(self):
        with self.assertRaisesRegex(RuntimeError,'No recorded home'): self.arm.home()
        self.assertFalse(any(c[0]=='joints' for c in self.sdk.commands))

    def test_timeout_stops_and_new_packets_never_clear_fault(self):
        service=BridgeService(self.arm,FakeCamera(),FakeLaser(),{'udp_port':0,'http_port':0,'telemetry_hz':15},
            _legacy_test_transport=True, udp_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM),start_video=False,_time_fn=self.clock)
        try:
            self.assertTrue(service.process_datagram(control(1),('127.0.0.1',5000)))
            self.clock.now += .251
            service.safety_tick()
            self.assertEqual(self.sdk.commands[-1],('estop',1))
            self.assertFalse(service.process_datagram(control(2),('127.0.0.1',5000)))
            self.assertTrue(service.get_diagnostics()['fault_latched'])
        finally: service.close()

    def test_late_packet_cannot_hide_timeout_between_safety_ticks(self):
        service=BridgeService(self.arm,FakeCamera(),FakeLaser(),{'telemetry_hz':15},
            _legacy_test_transport=True, udp_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM),start_video=False,_time_fn=self.clock)
        try:
            service.process_datagram(control(1),('127.0.0.1',5000))
            self.clock.now += .251
            self.assertFalse(service.process_datagram(control(2),('127.0.0.1',5000)))
            self.assertEqual(self.sdk.commands[-1],('estop',1))
        finally: service.close()

    def test_standby_packet_holds_after_motion(self):
        service=BridgeService(self.arm,FakeCamera(),FakeLaser(),{'telemetry_hz':15},
            _legacy_test_transport=True, udp_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM),start_video=False,_time_fn=self.clock)
        try:
            service.process_datagram(control(1),('127.0.0.1',5000))
            standby=ControlFrame(2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0).pack()
            self.assertTrue(service.process_datagram(standby,('127.0.0.1',5000)))
            self.assertEqual(self.sdk.commands[-1],('pose',100000,0,200000,0,10000,0))
        finally: service.close()


