import tempfile
from pathlib import Path
import socket
import unittest

from robot_bridge.protocol import ControlFrame
from robot_bridge.service import BridgeService
from tests.test_piper6 import FakePiperSdk, make_controller
from tests.test_service import FakeCamera, FakeLaser
from tests.test_jog_safety import Clock


class ArmActionTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.clock=Clock();self.sdk=FakePiperSdk()
        self.arm=make_controller(_sdk_interface=self.sdk,_time_fn=self.clock,home_path=str(Path(self.temp.name)/'home.json'))
        self.service=BridgeService(self.arm,FakeCamera(),FakeLaser(),{'telemetry_hz':15},
            _legacy_test_transport=True, udp_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM),start_video=False,_time_fn=self.clock)
        self.seq=0;self.neutral()
    def tearDown(self): self.service.close();self.temp.cleanup()
    def neutral(self):
        self.seq+=1
        self.service.process_datagram(ControlFrame(self.seq,123,0,0,0,0,0,0,0,0,0,0,0,0,0,0).pack(),('127.0.0.1',5000))
    def action(self,name,id='request'):
        return self.service.arm_action({'action':name,'request_id':id,'token':self.service._action_token,'udp_port':5000},'127.0.0.1')
    def test_set_home_persists_actual_pose(self):
        self.assertTrue(self.action('set_home')['ok'])
        self.assertEqual(self.arm.get_diagnostics()['home_pose'],[100,0,200,0,10,0])
        self.assertTrue((Path(self.temp.name)/'home.json').exists())
        self.assertFalse(any(c[0]=='joints' for c in self.sdk.commands))
    def test_unrecorded_home_never_sends_joint_zero(self):
        result=self.action('home')
        self.assertFalse(result['ok']);self.assertIn('No recorded home',result['error'])
        self.assertFalse(any(c[0]=='joints' for c in self.sdk.commands))
    def test_recovery_verifies_controller_and_clears_bridge_latch(self):
        self.service._latch_fault('test estop')
        self.neutral()
        result=self.action('recover')
        self.assertTrue(result['ok'])
        self.assertFalse(self.service._fault_latched)
        self.assertFalse(self.arm._estop_latched)
        self.assertEqual(self.sdk.status,0)
        self.assertIn(('estop',2),self.sdk.commands)
        self.assertIn(('pose',100000,0,200000,0,10000,0),self.sdk.commands)
    def test_idempotent_recovery_does_not_send_resume_twice(self):
        self.action('recover')
        count=len(self.sdk.commands)
        self.action('recover')
        self.assertEqual(len(self.sdk.commands),count)
    def test_failed_recovery_reasserts_estop(self):
        self.service._latch_fault('test');self.neutral()
        def failed(**kwargs):raise RuntimeError('feedback gone')
        self.arm._wait_normal=failed
        self.assertFalse(self.action('recover')['ok'])
        self.assertTrue(self.service._fault_latched)
        self.assertEqual(self.sdk.commands[-1],('estop',1))
    def test_enable_without_any_calibration_check(self):
        self.sdk.enabled=False
        self.assertTrue(self.action('enable')['ok'])
        self.assertTrue(self.sdk.enabled)
    def test_enable_takes_can_control_from_teach_at_actual_pose(self):
        self.sdk.ctrl_mode=2
        self.assertFalse(self.arm.get_diagnostics()['ready'])
        self.assertTrue(self.action('enable')['ok'])
        self.assertEqual(self.sdk.ctrl_mode,1)
        self.assertIn(('pose',100000,0,200000,0,10000,0),self.sdk.commands)
        self.assertTrue(self.arm.get_diagnostics()['ready'])
        self.neutral()
        self.assertTrue(self.action('set_home','record-after-can')['ok'])
    def test_refused_can_switch_does_not_claim_ready(self):
        self.sdk.ctrl_mode=2
        self.sdk.ModeCtrl=lambda *args: None
        result=self.action('enable')
        self.assertFalse(result['ok'])
        self.assertIn('control_mode=2',result['error'])
        self.assertTrue(self.service._fault_latched)
        self.assertEqual(self.sdk.commands[-1],('estop',1))
    def test_wrong_token_and_wrong_udp_owner_rejected(self):
        with self.assertRaises(ValueError):
            self.service.arm_action({'action':'recover','request_id':'r','token':'bad','udp_port':5000},'127.0.0.1')
        with self.assertRaises(ValueError):
            self.service.arm_action({'action':'recover','request_id':'r','token':self.service._action_token,'udp_port':5001},'127.0.0.1')
        self.assertFalse(any(c[0]=='estop' for c in self.sdk.commands))
    def test_non_neutral_input_blocks_recovery(self):
        frame=ControlFrame(10,0,2,0,0,0,0,100,0,0,0,0,0,0,0,0)
        self.service.process_datagram(frame.pack(),('127.0.0.1',5000))
        with self.assertRaisesRegex(ValueError,'neutral'):self.action('recover')
    def test_return_home_advances_at_limited_speed_then_holds(self):
        self.action('set_home','record')
        self.sdk.pose.Y_axis=1000
        self.assertTrue(self.action('home','return')['ok'])
        previous=1000
        for _ in range(15):
            self.clock.now+=.051
            self.neutral()
            self.service.safety_tick()
            poses=[c for c in self.sdk.commands if c[0]=='pose']
            if poses:
                y=poses[-1][2]
                self.assertLessEqual(abs(y-previous),201)
                self.sdk.pose.Y_axis=y
                previous=y
            if self.service._home_goal is None:break
        self.assertIsNone(self.service._home_goal)
        self.assertLessEqual(abs(self.sdk.pose.Y_axis),500)
    def test_return_home_cancel_holds_current_pose(self):
        self.action('set_home','record');self.sdk.pose.Y_axis=500
        self.action('home','return')
        self.clock.now+=.051;self.neutral();self.service.safety_tick()
        self.assertTrue(self.action('stop','stop')['ok'])
        self.assertIsNone(self.service._home_goal)
        self.assertEqual(self.sdk.commands[-1][2],500)

