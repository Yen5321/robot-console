import copy,math,sys,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/robot_console_servo'))
sys.path.insert(0,str(ROOT/'vendor/pyAgxArm'))
from robot_console_servo.cpv_policy import CpvPolicy
from robot_console_servo.model import Model
from robot_console_servo.cpv_sdk import FIRMWARE

MODEL=ROOT/'ros2_ws/src/robot_console_servo/model/piper_l/piper_l_stock_gripper.urdf'
class Fake:
    def __init__(self,model):
        self.q=np.array([.5,1.3,-1.,.5,.8,.4]);self.model=model;self.t=10.;self.tx=[];self.stops=0;self.enables=0
        self.active=False;self.bad=None;self.stale=False;self.failed=False
    def poll(self):
        stamp=self.t-1 if self.stale else self.t
        return {'q':self.q.copy(),'pose':self.model.pose(self.q),'q_at':stamp,'pose_at':stamp,'enabled_at':stamp,'status_at':stamp,
                'firmware':FIRMWARE,'node_type':'Piper_L_MC','enabled':[self.active]*6,'status':self.bad or 0,
                'ctrl_mode':1 if self.active else 0,'cpv_mode':5 if self.active else 1,'driver_fault':False}
    def enable(self):self.enables+=1;self.active=True
    def parameter_snapshot_ready(self):return True
    def velocity(self,v):
        if self.failed:raise OSError('send failure')
        self.tx.append(np.array(v));self.q+=np.array(v)*.01
    def emergency(self):self.stops+=1

class CpvTests(unittest.TestCase):
    def setUp(self):
        self.m=Model(MODEL.read_text());self.sdk=Fake(self.m);self.p=CpvPolicy(self.m,self.sdk);self.tick()
    def tick(self,count=1):
        for _ in range(count):self.sdk.t+=.01;self.p.tick(self.sdk.t,.01)
    def enable(self):
        self.p.action('sample_fk',self.sdk.t);self.p.action('enable',self.sdk.t);self.tick(2)
    def authority(self,v,**fields):
        self.p.accept_authority({'origin':self.sdk.t,'input_origin':self.sdk.t,'input':v,'epoch':'test','active':any(v),**fields},self.sdk.t)
    def commissioned(self):
        # Fixture stands in for separate measured evidence, not a shipped bypass.
        self.enable();self.p.fk_samples=[self.sdk.q.copy()]*3;self.p.hold_passed=True
        self.p.directions={(j,s) for j in range(1,7) for s in (-1,1)};self.authority([0]*5)
    def test_no_automatic_tx_or_enable(self):
        self.tick(30);self.assertEqual(self.sdk.tx,[]);self.assertEqual(self.sdk.enables,0)
    def test_enable_requires_identity_fk_and_actual_confirmation(self):
        with self.assertRaisesRegex(ValueError,'FK_sample'):self.p.action('enable',self.sdk.t)
        self.enable();self.assertTrue(self.p.armed);self.assertGreater(self.sdk.enables,0)
    def test_hold_test_and_bounded_pulse(self):
        self.enable();self.p.action('hold_test',self.sdk.t);self.tick(510);self.assertTrue(self.p.hold_passed)
        self.p.action('pulse',self.sdk.t,joint=1,sign=1);self.tick(70)
        self.assertTrue(self.p.last_result['passed']);self.assertIn((1,1),self.p.directions)
        self.assertLessEqual(max(abs(x[0]) for x in self.sdk.tx),.01)
        self.assertTrue(np.allclose(self.sdk.tx[-1],0))
    def test_hold_drift_blocks_motion(self):
        self.enable();self.p.action('hold_test',self.sdk.t);self.sdk.q[1]+=.1;self.tick()
        self.assertIn('hold_drift',self.p.fault);self.assertEqual(self.sdk.stops,1)
    def test_missing_parameter_readback_blocks_enable_without_tx(self):
        self.p.action('sample_fk',self.sdk.t)
        self.sdk.parameter_snapshot_ready=lambda:False
        with self.assertRaisesRegex(ValueError,'CPV_parameter_snapshot_incomplete'):
            self.p.action('enable',self.sdk.t)
        self.assertEqual(self.sdk.enables,0);self.assertEqual(self.sdk.tx,[])
    def test_drift_before_first_hold_test_stops_before_next_velocity(self):
        self.enable();n=len(self.sdk.tx);self.sdk.q[1]+=.1;self.tick()
        self.assertEqual(self.p.fault,'zero_velocity_hold_drift')
        self.assertFalse(self.p.armed);self.assertEqual(len(self.sdk.tx),n)
    def test_mode_transition_drift_is_not_rebased_away(self):
        self.p.action('sample_fk',self.sdk.t);self.p.action('enable',self.sdk.t)
        self.sdk.q[1]+=.1;self.tick()
        self.assertEqual(self.p.fault,'zero_velocity_hold_drift');self.assertEqual(self.sdk.enables,0)
    def test_no_double_sign_flip_in_pinned_sdk(self):
        try:from pyAgxArm import AgxArmFactory,create_agx_arm_config,ArmModel,PiperFW
        except ModuleNotFoundError as exc:self.skipTest(str(exc))
        arm=AgxArmFactory.create_arm(create_agx_arm_config(ArmModel.PIPER_L,firmeware_version=PiperFW.V189))
        sent=[];arm._move_cpv=lambda **kw:sent.append(kw)
        for i in range(1,7):arm.move_cpv_vel(i,.01)
        self.assertEqual([x['value'] for x in sent],[.01,-.01,-.01,-.01,-.01,.01])
    def test_urdf_matches_official_sdk_mdh_offline(self):
        try:from pyAgxArm.utiles.mdh_kinematics import get_mdh,fk_from_mdh
        except ModuleNotFoundError as exc:self.skipTest(str(exc))
        from robot_console_servo.cpv_policy import pose_error
        rng=np.random.default_rng(81)
        for _ in range(50):
            q=rng.uniform(self.m.lower,self.m.upper)
            metres,radians=pose_error(self.m.pose(q),np.array(fk_from_mdh(get_mdh('piper_l'),q.tolist())))
            self.assertLess(metres,.005);self.assertLess(radians,math.radians(2))
    def test_old_input_not_refreshed_by_heartbeat(self):
        self.commissioned();old=self.sdk.t-.26
        with self.assertRaisesRegex(ValueError,'input_origin_expired'):self.authority([1,0,0,0,0],input_origin=old)
    def test_output_expiry_and_first_fault(self):
        self.commissioned();self.authority([1,0,0,0,0]);self.tick()
        self.assertEqual(self.p.fault,'servo_output_expired')
        self.p.stop('second');self.assertEqual(self.p.fault,'servo_output_expired')
        self.p.stop('manual',True);self.p.stop('manual',True);self.assertEqual(self.sdk.stops,1)
    def test_feedback_expiry_electronic_stop(self):
        self.enable();self.sdk.stale=True;self.tick();self.assertEqual(self.sdk.stops,1)
    def test_expired_origin_latches_and_cannot_resume(self):
        self.commissioned();self.authority([0]*5);self.tick(26)
        self.assertEqual(self.p.fault,'control_origin_expired')
        self.authority([1,0,0,0,0]);self.tick();self.assertTrue(np.allclose(self.sdk.tx[-1],0))
    def test_joint_output_uniform_limit_and_ramp(self):
        self.commissioned();previous=np.zeros(6)
        for _ in range(60):
            self.authority([1,0,0,0,0]);self.p.accept_trajectory(self.m.names,[1,.5,0,0,0,0],self.sdk.t,.01,self.sdk.t);self.tick()
            out=self.sdk.tx[-1];self.assertLessEqual(max(abs(out-previous)),.00100001);self.assertLessEqual(max(abs(out)),.05000001);previous=out
        self.authority([0]*5);self.tick(60);self.assertTrue(np.allclose(self.sdk.tx[-1],0))
    def test_recover_never_resets_or_enables(self):
        self.enable();self.p.stop('manual',True);n=self.sdk.enables
        with self.assertRaises(ValueError):self.p.action('recover',self.sdk.t)
        self.sdk.bad=1;self.tick();self.sdk.bad=None;self.tick(60)
        self.p.action('recover',self.sdk.t);self.tick();self.assertFalse(self.p.armed);self.assertEqual(self.sdk.enables,n)
    def test_restart_epoch_fault(self):
        self.commissioned();self.authority([0]*5,epoch='restarted');self.assertEqual(self.p.fault,'gateway_restarted')
    def test_controller_fault_only_once(self):
        self.enable();self.sdk.bad=4;self.tick(5);self.assertEqual(self.sdk.stops,1)
    def test_order_and_nan_rejected(self):
        with self.assertRaises(ValueError):self.p.accept_trajectory(list(reversed(self.m.names)),[0]*6,self.sdk.t,.01,self.sdk.t)
        with self.assertRaises(ValueError):self.p.accept_trajectory(self.m.names,[math.nan]*6,self.sdk.t,.01,self.sdk.t)
    def test_fixed_transform_chain_is_used(self):
        xml=MODEL.read_text().replace('<parent link="base_link"','<parent link="offset_base"',1)
        xml=xml.replace('</robot>','<link name="offset_base"/><joint name="fixed_offset" type="fixed"><parent link="base_link"/><child link="offset_base"/><origin xyz="0.1 0 0"/></joint></robot>')
        other=Model(xml);self.assertAlmostEqual((other.pose(self.sdk.q)-self.m.pose(self.sdk.q))[0],.1)
        np.testing.assert_allclose(other.pose(self.sdk.q)[1:],self.m.pose(self.sdk.q)[1:])
if __name__=='__main__':unittest.main()
