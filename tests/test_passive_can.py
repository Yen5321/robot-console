"""No socket/CAN hardware: real pinned SDK decoder + in-memory receive-only bus."""
import sys,time,unittest,struct
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2_ws/src/robot_console_servo'))
try:
    import can
    from robot_console_servo.passive_can import PassiveCan
    from piper_sdk.protocol.protocol_v2 import C_PiperParserV2
    AVAILABLE=True
except ImportError:AVAILABLE=False


@unittest.skipUnless(AVAILABLE,'Install pinned receive-only SDK dependencies to run decoder tests')
class PassiveTests(unittest.TestCase):
    def make(self,frames):
        class Bus:
            def recv(self,timeout):return frames.pop(0) if frames else None
            def shutdown(self):pass
            def send(self,*a,**k):raise AssertionError('Read-only reader transmitted!')
        with patch('can.Bus',return_value=Bus()):return PassiveCan('unused')
    def frame(self,can_id,data=None,age=0):
        return can.Message(arbitration_id=can_id,data=data or bytes(8),is_extended_id=False,timestamp=time.time()-age)
    def test_all_joint_pairs_required_and_real_units(self):
        reader=self.make([self.frame(0x2a5,struct.pack('>ii',1000,-2000))])
        q,stamp=reader.poll();self.assertAlmostEqual(q[0],0.0174532925);self.assertEqual(stamp,-float('inf'))
    def test_stale_kernel_frames_do_not_refresh(self):
        reader=self.make([self.frame(i,age=1) for i in (0x2a5,0x2a6,0x2a7)])
        self.assertEqual(reader.poll()[1],-float('inf'))
    def test_status_pose_and_enable_without_send(self):
        reader=self.make([self.frame(i) for i in (0x2a1,0x2a2,0x2a3,0x2a4,0x2a5,0x2a6,0x2a7,0x261,0x262,0x263,0x264,0x265,0x266)])
        q,stamp=reader.poll()
        self.assertGreater(stamp,0);self.assertEqual(reader.status,0)
        self.assertIsNotNone(reader.sdk_pose);self.assertEqual(len(reader.enabled),6)
        reader.close()
if __name__=='__main__':unittest.main()
