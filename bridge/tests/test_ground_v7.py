"""Ground v8 regression tests, fake SDK only: never connect to CAN."""
import math
import socket
import unittest
from dataclasses import replace

from robot_bridge.service import BridgeService
from robot_bridge.protocol import ControlFrame
from tests.test_piper6 import FakePiperSdk, make_controller
from tests.test_jog_safety import Clock
from tests.test_service import FakeCamera, FakeLaser, control


class GroundV7Tests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock(); self.sdk=FakePiperSdk()
        self.arm=make_controller(_sdk_interface=self.sdk,_time_fn=self.clock,
                                 max_linear_speed_mm_s=50,max_angular_speed_deg_s=10)
        self.service=BridgeService(self.arm,FakeCamera(),FakeLaser(),{},
            udp_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM),start_video=False,
            _time_fn=self.clock,_legacy_test_transport=True)
    def tearDown(self): self.service.close()
    def send(self,seq=1,**kw):
        frame=replace(ControlFrame.unpack(control(seq)),**kw)
        return self.service.process_datagram(frame.pack(),('127.0.0.1',5000))
    def stops(self): return [x for x in self.sdk.commands if x[0]=='estop']

    def test_idle_invalid_states_only_block_and_report(self):
        for attr,value,reason in [('enabled',False,'未使能'),('ctrl_mode',2,'控制模式'),('status',4,'控制器故障')]:
            with self.subTest(reason=reason):
                old=getattr(self.sdk,attr);setattr(self.sdk,attr,value)
                self.service._input_rearm_required=False
                self.send(self.service._last_seq+1 if self.service._last_seq else 1)
                self.service.safety_tick()
                self.assertIn(reason,self.service.get_diagnostics()['blocking_reason'])
                self.assertFalse(self.service._fault_latched)
                self.assertEqual(self.stops(),[])
                setattr(self.sdk,attr,old)

    def test_idle_stale_feedback_does_not_send_estop(self):
        self.sdk.frozen=True; self.arm.check_feedback();self.clock.now+=.3
        self.send(); self.service.safety_tick()
        self.assertIn('not fresh',self.service.get_diagnostics()['blocking_reason'])
        self.assertEqual(self.stops(),[])

    def test_motion_state_failure_stops_once_and_latches(self):
        self.send();self.sdk.ctrl_mode=2
        self.service.safety_tick(); first=self.service._fault_reason
        self.service.safety_tick();self.send(2)
        self.assertEqual(len(self.stops()),1)
        self.assertTrue(self.service._fault_latched)
        self.assertEqual(self.service._fault_reason,first)
        self.assertEqual(self.service.get_diagnostics()['fault_source'],'arm_feedback')

    def test_idle_rejection_requires_neutral_before_retry(self):
        self.sdk.enabled=False;self.send();self.sdk.enabled=True
        self.send(2)
        self.assertFalse(any(c[0]=='pose' for c in self.sdk.commands))
        self.send(3,arm_dy=0);self.send(4)
        self.assertTrue(any(c[0]=='pose' for c in self.sdk.commands))
        self.assertEqual(self.stops(),[])

    def test_workspace_rejection_during_motion_holds_without_estop(self):
        self.send(); self.arm._workspace['y']=[-1,.3]
        self.clock.now+=.05;self.send(2,arm_dy=1000)
        self.assertFalse(self.arm._moving)
        self.assertEqual(self.sdk.commands[-1][1:4],(100000,0,200000))
        self.assertEqual(self.stops(),[])
        self.assertTrue(self.service._input_rearm_required)

    def test_sdk_send_error_even_on_first_target_stops(self):
        def fail(*args): raise OSError('CAN send failed')
        self.sdk.EndPoseCtrl=fail
        self.send()
        self.assertTrue(self.service._fault_latched)
        self.assertEqual(len(self.stops()),1)

    def test_hold_disabled_still_submits_actual_and_records_drift(self):
        self.arm.jog_cartesian(0,1,0,0,0); self.arm.hold_position()
        self.assertEqual(self.sdk.commands[-1][1:],(100000,0,200000,0,10000,0))
        self.sdk.pose.Y_axis=4000;self.sdk.pose.RY_axis=12000
        self.arm.check_feedback()
        hold=self.arm.get_diagnostics()['last_hold']
        self.assertEqual(hold['max_translation_mm'],4)
        self.assertGreater(hold['max_rotation_deg'],1.99)
        self.assertEqual(self.stops(),[])
        self.arm._safety['hold_rotation_enabled']=True
        with self.assertRaisesRegex(RuntimeError,'tolerance'):self.arm.check_feedback()
        self.assertEqual(len(self.stops()),1)

    def test_v8_diagonal_ceiling_and_fixed_orientation(self):
        self.arm.jog_cartesian(0,1,1,0,0)
        self.assertAlmostEqual(math.dist(self.arm._target[:3],[100,0,200]),2.5)
        self.assertEqual(self.arm._target[3:],[0,10,0])

    def test_scaled_low_speed_not_lost_to_backend_deadband(self):
        self.arm.jog_cartesian(0,.01,0,.1,0)
        # Quarter-stick at 2/50 -> .01; deadband applied BEFORE scaling in WPF.
        self.assertAlmostEqual(self.arm._target[1],.025)
        self.assertAlmostEqual(self.arm._target[4],10.05)

    def test_home_independent_low_speed_ceiling(self):
        self.arm.advance_home([200,0,200,0,10,0],.05)
        self.assertAlmostEqual(self.arm._target[0],100.2)
        self.arm.hold_position()
        self.arm.advance_home([100,0,200,0,30,0],.05)
        self.assertAlmostEqual(self.arm._target[4],10.1)

    def test_old_profile_rejected_before_session_or_motion(self):
        payload={'action':'session','token':self.service._action_token,'udp_port':5000,'request_id':'old'}
        with self.assertRaisesRegex(ValueError,'v8'):self.service.arm_action(payload,'127.0.0.1')
        self.assertIsNone(self.service._session);self.assertEqual(self.stops(),[])
        reply=self.service.arm_action(dict(payload,control_profile='piper-l-cpv-v8.1'),'127.0.0.1')
        self.assertEqual(reply['control_profile'],'piper-l-cpv-v8.1')
        caps=self.service.get_diagnostics()['motion_capabilities']
        self.assertEqual(caps['linear_speed_mm_s'],50)
        self.assertEqual(caps['angular_speed_deg_s'],10)

    def test_recent_log_is_bounded_and_token_redacted(self):
        for i in range(305): self.service._record('sample',action_token='secret',index=i)
        records=self.service.get_recent_logs()['records']
        self.assertEqual(len(records),300)
        self.assertNotIn('action_token',records[-1])

