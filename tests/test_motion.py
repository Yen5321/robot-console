import sys
from pathlib import Path
import unittest
import math
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/robot_console_servo'))
from robot_console_servo.core import Motion,JointGuard
from robot_console_servo.model import Model


class MotionTests(unittest.TestCase):
    def ready(self):
        m=Motion();m.feedback([.2,0,.3,0,0,0],1)
        m.action('enable',1);m.accept([0]*5,1,1)
        return m
    def test_axis_mapping_and_diagonal(self):
        m=self.ready();m.accept([0,1,1,0,0],1.01,1.01)
        for i in range(1,70):
            t=1.01+i*.01;m.feedback(m.pose,t);m.stamp=m.input_stamp=t
            out=m.tick(t,.01)
            self.assertLessEqual(np.linalg.norm(out[:3]),.05+1e-10)
        self.assertEqual(out[0],0);self.assertGreater(out[1],0);self.assertGreater(out[2],0)
    def test_translation_does_not_change_attitude_reference(self):
        m=self.ready();m.accept([.2,0,0,0,0],1.01,1.01)
        before=m.reference.as_matrix().copy();m.tick(1.02,.01)
        np.testing.assert_allclose(before,m.reference.as_matrix())
        m.feedback([.2,0,.3,.1,0,0],1.03);m.tick(1.03,.01)
        np.testing.assert_allclose(before,m.reference.as_matrix())
    def test_rotation_and_translation_are_independent(self):
        a=self.ready();b=self.ready()
        a.accept([0,.04,0,.1,0],1.01,1.01)
        b.accept([0,.5,0,.1,0],1.01,1.01)
        np.testing.assert_allclose(a.tick(1.02,.01)[3:],b.tick(1.02,.01)[3:])
    def test_release_ramps_without_position_jump(self):
        m=self.ready();m.accept([0,1,0,0,0],1.01,1.01)
        for i in range(1,10):m.tick(1.01+i*.01,.01)
        previous=m.twist.copy();pose=m.pose.copy()
        m.accept([0]*5,1.11,1.11);m.tick(1.12,.01)
        self.assertLess(m.twist[1],previous[1]);self.assertGreater(m.twist[1],0)
        np.testing.assert_equal(m.pose,pose)
    def test_replaying_old_timestamp_rejected(self):
        m=self.ready()
        with self.assertRaises(ValueError):m.accept([0,1,0,0,0],.5,1.02)
        with self.assertRaises(ValueError):m.accept([0]*5,1,1.02)
    def test_heartbeat_cannot_extend_jog_input(self):
        m=self.ready();m.accept([0,.1,0,0,0],1.01,1.01)
        m.stamp=1.29;m.feedback(m.pose,1.29);m.tick(1.29,.01)
        self.assertTrue(m.fault);self.assertEqual(m.reason,'control_input_expired')
    def test_idle_stale_feedback_not_estop(self):
        m=Motion();m.tick(1,.01)
        self.assertFalse(m.fault);self.assertEqual(m.state,'not_ready')
    def test_active_stale_feedback_fault(self):
        m=self.ready();m.accept([0,.1,0,0,0],1.01,1.01);m.tick(1.3,.01)
        self.assertTrue(m.fault);self.assertEqual(m.reason,'feedback_expired')
    def test_fault_recovery_requires_enable_and_release(self):
        m=self.ready();m.stop('test');m.action('recover',1.01)
        self.assertFalse(m.enabled);m.action('enable',1.02)
        with self.assertRaises(ValueError):m.accept([0,.1,0,0,0],1.03,1.03)
        m.accept([0]*5,1.04,1.04);m.accept([0,.1,0,0,0],1.05,1.05)
    def test_servo_limit_is_restriction_not_emergency(self):
        m=self.ready();m.accept([0,.1,0,0,0],1.01,1.01);m.servo_status=2
        m.tick(1.02,.01)
        self.assertFalse(m.fault);self.assertTrue(m.rearm);self.assertEqual(np.linalg.norm(m.twist),0)
    def test_home_speed_and_abort(self):
        m=self.ready();m.action('set_home',1.01)
        m.pose[0]+=.1;m.action('home',1.02)
        out=m.tick(1.03,.01)
        self.assertLessEqual(np.linalg.norm(out[:3]),.004)
        m.action('stop',1.04);self.assertIsNone(m.goal)
    def test_nan_input_rejected(self):
        with self.assertRaises(ValueError):self.ready().accept([0,float('nan'),0,0,0],1.02,1.02)
    def test_execution_limits_and_braking(self):
        g=JointGuard([str(i) for i in range(6)],[-1]*6,[1]*6)
        out=g.step([1,2,3,0,0,0],[0]*6,.01)
        self.assertLessEqual(np.max(abs(out)),.006+1e-12)
        self.assertAlmostEqual(out[1]/out[0],2)
        before=out.copy();out=g.step([0]*6,[0]*6,.01)
        self.assertLess(np.linalg.norm(out),np.linalg.norm(before))
    def test_bad_trajectory_freshness_names_nan(self):
        g=JointGuard([str(i) for i in range(6)],[-1]*6,[1]*6)
        for names,v,age,horizon in [([], [0]*6,0,.01),(g.names,[math.nan]*6,0,.01),(g.names,[0]*6,.5,.01),(g.names,[0]*6,0,1)]:
            with self.assertRaises(ValueError):g.validate(names,v,age,horizon)
    def test_model_is_pinned_six_axis(self):
        p=ROOT/'ros2_ws/src/robot_console_servo/model/piper/urdf/piper_description.urdf'
        model=Model(p.read_text());self.assertEqual(model.names,[f'joint{i}' for i in range(1,7)])
        self.assertTrue(np.isfinite(model.pose([0,.8,-.7,.3,.5,.2])).all())
    def test_home_tracking_failure_stays_protected(self):
        m=self.ready();m.action('set_home',1.01);m.pose[0]+=.1;m.action('home',1.02)
        m.home_reference[0]+=.003
        m.tick(1.03,.01);self.assertTrue(m.fault);self.assertIn('home_tracking',m.reason)
    def test_selected_angular_speed_not_amplified(self):
        m=self.ready();m.accept([0,0,0,.1,0],1.01,1.01)
        for k in range(100):
            now=1.02+k*.01;m.feedback(m.pose,now);m.stamp=m.input_stamp=now
            self.assertLessEqual(np.linalg.norm(m.tick(now,.01)[3:]),math.radians(1)+1e-12)

if __name__=='__main__':unittest.main()
