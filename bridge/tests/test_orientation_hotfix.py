import unittest
from tests.test_piper6 import FakePiperSdk, make_controller
from robot_bridge.hardware.piper6 import PiperArmController


class OrientationHotfixTests(unittest.TestCase):
    def test_jog_distance_alone_no_longer_rejects_over_twenty(self):
        sdk=FakePiperSdk(); arm=make_controller(_sdk_interface=sdk)
        arm.jog_cartesian(0,1,0,0,0)
        sdk.pose.Z_axis=197000
        arm.jog_cartesian(0,1,0,0,0)
        sdk.pose.Z_axis=179000
        count=len(sdk.commands)
        arm.jog_cartesian(0,1,0,0,0)
        self.assertGreater(len(sdk.commands),count)
        self.assertEqual(sdk.commands[-1][0],'pose')
        self.assertIsNone(arm.get_diagnostics()['jog_translation_lead_limit_mm'])
        self.assertFalse(arm.get_diagnostics()['jog_translation_lead_check_enabled'])

    def test_rotation_geometry(self):
        distance = PiperArmController.rotation_distance_deg
        self.assertAlmostEqual(distance([0,0,179.9], [0,0,-179.9]), .2, places=6)
        self.assertAlmostEqual(distance([0,0,0], [0,0,2]), 2, places=6)
        self.assertAlmostEqual(distance([0,90,0], [80,90,80]), 0, places=5)
        # Recorded RPY component differences exceed 1 degree, but their combined
        # physical rotation is below 1 degree near this high-pitch pose.
        self.assertLess(distance([-131.203,78.69,-100.337], [-133.131,78.343,-102.241]), 1)

    def test_recorded_translation_preserves_target_and_accepts_small_rotation(self):
        sdk = FakePiperSdk()
        values = [149219,89245,535419,-131203,78690,-100337]
        keys = ('X_axis','Y_axis','Z_axis','RX_axis','RY_axis','RZ_axis')
        for key, value in zip(keys,values): setattr(sdk.pose,key,value)
        arm = make_controller(_sdk_interface=sdk)
        arm.jog_cartesian(0,.221,-.046,0,0)
        for key, value in zip(keys,[149395,89327,534035,-133131,78343,-102241]):
            setattr(sdk.pose,key,value)
        arm.jog_cartesian(0,.221,-.046,0,0)
        self.assertEqual(sdk.commands[-1][4:],tuple(values[3:]))
        self.assertFalse(any(c[0]=='estop' for c in sdk.commands))

    def test_jog_rotation_error_is_recorded_without_rejection(self):
        sdk = FakePiperSdk(); arm = make_controller(_sdk_interface=sdk)
        arm.jog_cartesian(0,1,0,0,0)
        sdk.pose.RZ_axis=2000
        before=len(sdk.commands)
        arm.jog_cartesian(0,1,0,0,0)
        self.assertGreater(len(sdk.commands),before)
        self.assertEqual(sdk.commands[-1][4:],(0,10000,0))
        tracking=arm.get_diagnostics()['last_jog_tracking']
        self.assertEqual(tracking['actual'][5],2)
        self.assertEqual(tracking['attempted_target'][5],0)
        self.assertAlmostEqual(tracking['rotation_error_deg'],2,places=6)
        self.assertFalse(arm.get_diagnostics()['jog_angular_lead_check_enabled'])

    def test_hold_monitor_still_faults_on_true_rotation(self):
        sdk=FakePiperSdk(); arm=make_controller(_sdk_interface=sdk)
        arm._safety['hold_rotation_enabled']=True
        arm.jog_cartesian(0,1,0,0,0)
        arm.hold_position()
        sdk.pose.RZ_axis=2000
        with self.assertRaisesRegex(RuntimeError,'tolerance'):
            arm.check_feedback()
        self.assertEqual(sdk.commands[-1],('estop',1))

    def test_priority_stop_keeps_first_reason_and_still_sends_stop(self):
        sdk=FakePiperSdk(); arm=make_controller(_sdk_interface=sdk)
        arm.fault_stop('original fault')
        arm.request_priority_stop('remote echo')
        self.assertEqual(arm.get_diagnostics()['error'],'original fault')
        self.assertEqual(sdk.commands[-1],('estop',1))
