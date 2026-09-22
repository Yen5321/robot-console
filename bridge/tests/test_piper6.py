from types import SimpleNamespace
import unittest

from robot_bridge.hardware.piper6 import PiperArmController


class FakePiperSdk:
    def __init__(self) -> None:
        self.pose = SimpleNamespace(
            X_axis=100_000, Y_axis=0, Z_axis=200_000,
            RX_axis=0, RY_axis=10_000, RZ_axis=0,
        )
        self.joints = SimpleNamespace(
            joint_1=0, joint_2=90_000, joint_3=-85_000,
            joint_4=0, joint_5=0, joint_6=0,
        )
        self.commands: list[tuple[object, ...]] = []
        self.stamp = 1.0
        self.frozen = False
        self.status = 0
        self.ctrl_mode = 1
        self.enabled = True

    def next_stamp(self):
        if not self.frozen: self.stamp += 0.001
        return self.stamp

    def ConnectPort(self) -> None:
        self.commands.append(("connect",))

    def DisconnectPort(self) -> None:
        self.commands.append(("disconnect",))

    def GetArmEndPoseMsgs(self):
        return SimpleNamespace(Hz=30, time_stamp=self.next_stamp(), end_pose=self.pose)

    def GetArmJointMsgs(self):
        return SimpleNamespace(Hz=30, time_stamp=self.next_stamp(), joint_state=self.joints)

    def GetArmStatus(self):
        return SimpleNamespace(time_stamp=self.next_stamp(), arm_status=SimpleNamespace(arm_status=self.status, ctrl_mode=self.ctrl_mode))

    def GetArmLowSpdInfoMsgs(self):
        return SimpleNamespace(time_stamp=self.next_stamp(), **{
            f"motor_{i}": SimpleNamespace(foc_status=SimpleNamespace(driver_enable_status=self.enabled)) for i in range(1,7)
        })

    def EnablePiper(self):
        self.enabled=True
        return True

    def ModeCtrl(self, *args: object) -> None:
        self.commands.append(("mode", *args))
        self.ctrl_mode = args[0]

    def EndPoseCtrl(self, *args: object) -> None:
        self.commands.append(("pose", *args))

    def JointCtrl(self, *args: object) -> None:
        self.commands.append(("joints", *args))

    def EmergencyStop(self, *args: object) -> None:
        self.commands.append(("estop", *args))
        self.status = 1 if args[0]==1 else 0


def make_controller(**kwargs):
    # Legacy unit vectors exercise a configured low-speed profile; v8 tests
    # explicitly pass 50/10 when testing the new negotiated maximum profile.
    kwargs.setdefault('max_linear_speed_mm_s',4)
    kwargs.setdefault('max_angular_speed_deg_s',2)
    result = PiperArmController(**kwargs)
    result.check_feedback()
    result.get_joint_margins()
    return result


class PiperTests(unittest.TestCase):
    def test_cartesian_jog_uses_documented_units_and_axis_mapping(self) -> None:
        sdk = FakePiperSdk()
        controller = make_controller(_sdk_interface=sdk, _time_fn=lambda: 1.0)
        controller.jog_cartesian(1.0, 0.0, 0.0, 1.0, -1.0)
        self.assertEqual(sdk.commands[-2], ("mode", 1, 2, 10, 0))
        self.assertEqual(
            sdk.commands[-1],
            ("pose", 100_200, 0, 200_000, 0, 10_071, -71),
        )

    def test_joint_margins_are_centered_at_100(self) -> None:
        controller = make_controller(_sdk_interface=FakePiperSdk())
        self.assertEqual(controller.get_joint_margins(), [100.0] * 6)

    def test_estop_latches_and_calls_official_sdk(self) -> None:
        sdk = FakePiperSdk()
        controller = make_controller(_sdk_interface=sdk)
        controller.estop()
        self.assertEqual(sdk.commands[-1], ("estop", 1))
        with self.assertRaises(RuntimeError):
            controller.jog_cartesian(0, 0, 0, 0, 0)

    def test_cartesian_target_cannot_leave_configured_workspace(self) -> None:
        sdk = FakePiperSdk()
        controller = make_controller(
            _sdk_interface=sdk,
            workspace_mm={"x": [0.0, 100.1], "y": [-1.0, 1.0], "z": [0.0, 300.0]},
        )
        with self.assertRaisesRegex(RuntimeError, "outside"):
            controller.jog_cartesian(1.0, 0.0, 0.0, 0.0, 0.0)
        self.assertFalse(any(command[0] == "pose" for command in sdk.commands))


if __name__ == "__main__":
    unittest.main()

