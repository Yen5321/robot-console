"""Real pinned SDK over python-can virtual bus, no SocketCAN/hardware access."""
import sys,time,unittest,uuid
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'vendor/pyAgxArm'),str(ROOT/'ros2_ws/src/robot_console_servo')]
try:
    import can,pyAgxArm
except ImportError:can=None

@unittest.skipIf(can is None,'python-can unavailable; run in Linux with pinned dependencies')
class SdkBoundaryTests(unittest.TestCase):
    def setUp(self):
        from robot_console_servo.cpv_sdk import CpvSdk
        original=pyAgxArm.create_agx_arm_config
        def config(*a,**kw):
            kw.update(interface='virtual',enable_check_can=False,local_loopback=True)
            return original(*a,**kw)
        self.channel='cpv-test-'+uuid.uuid4().hex
        self.peer=can.Bus(interface='virtual',channel=self.channel)
        with patch.object(pyAgxArm,'create_agx_arm_config',config):self.sdk=CpvSdk(self.channel)
    def tearDown(self):self.sdk.close();self.peer.shutdown()
    def drain(self):
        frames=[]
        while True:
            f=self.peer.recv(0)
            if f is None:return frames
            frames.append(f)
    def test_startup_only_identification_query(self):
        frames=self.drain();self.assertEqual([f.arbitration_id for f in frames],[0x4af])
        self.assertFalse(self.sdk.arm._auto_set_motion_mode_enabled)
    def test_six_cpv_commands_without_implicit_mode_tx(self):
        self.drain();self.sdk.velocity([.01]*6);frames=self.drain()
        self.assertEqual(len(frames),6);self.assertNotIn(0x151,[f.arbitration_id for f in frames])
        self.assertNotIn(0x150,[f.arbitration_id for f in frames])
    def test_send_exception_propagates(self):
        with patch.object(self.sdk.comm.send_bus,'send',side_effect=can.CanOperationError('queue full')):
            with self.assertRaisesRegex(RuntimeError,'queue full'):self.sdk.velocity([0]*6)
    def test_zero_velocity_wire_payload_is_seven_byte_wsp_zero(self):
        self.drain();self.sdk.velocity([0]*6);frames=self.drain()
        self.assertEqual([f.arbitration_id for f in frames],list(range(0x181,0x187)))
        self.assertTrue(all(f.dlc==7 and bytes(f.data)==b'wsp'+bytes(4) for f in frames))
    def test_short_cpv_response_and_ack_are_parsed(self):
        self.drain()
        self.peer.send(can.Message(arbitration_id=0x181,is_extended_id=False,data=bytes.fromhex('616b7000000046')))
        self.sdk.poll()
        self.assertAlmostEqual(self.sdk.arm.get_cpv_kp(1,timeout=0),.7)
        self.peer.send(can.Message(arbitration_id=0x181,is_extended_id=False,data=[0xac]))
        self.sdk.poll();self.assertTrue(self.sdk.arm._parser.cpv_response_1.msg.write_ack)
    def test_short_physical_or_malformed_cpv_cannot_refresh_feedback(self):
        self.peer.send(can.Message(arbitration_id=0x2a5,is_extended_id=False,data=[0]*7))
        self.peer.send(can.Message(arbitration_id=0x181,is_extended_id=False,data=b'asp'+bytes(3)))
        self.peer.send(can.Message(arbitration_id=0x181,is_extended_id=False,data=b'rsp'+bytes(4)))
        self.sdk.poll();self.assertNotIn(0x2a5,self.sdk.times);self.assertNotIn(0x181,self.sdk.times)
    def test_physical_groups_and_full_firmware(self):
        raw=bytes.fromhex('48 2d 56 31 2e 32 2d 31 00 00 00 00 00 00 00 00 31 30 00 00 00 00 00 00 00 00 00 00 00 00 00 00 50 69 70 65 72 5f 4c 5f 4d 43 00 00 00 00 00 00 33 30 41 48 00 00 00 00 00 00 00 00 53 2d 56 31 2e 39 2d 30 32 36 30 37 31 36 00 00 31 35 00 00 e8 6f 00 00 00 00 00 00')
        for i in range(0,len(raw),8):self.peer.send(can.Message(arbitration_id=0x4af,is_extended_id=False,data=raw[i:i+8]))
        self.peer.send(can.Message(arbitration_id=0x2a5,is_extended_id=False,data=[0]*8))
        f=self.sdk.poll();self.assertEqual(f['firmware'],'S-V1.9-0260716');self.assertEqual(f['node_type'],'Piper_L_MC')
        self.assertLess(f['q_at'],0) # one pair must not freshen the other four axes
        for cid in (0x2a6,0x2a7):self.peer.send(can.Message(arbitration_id=cid,is_extended_id=False,data=[0]*8))
        f=self.sdk.poll();self.assertLess(time.monotonic()-f['q_at'],.25)
        stamp=f['q_at'];self.sdk.poll();self.assertEqual(self.sdk.snapshot()['q_at'],stamp)
if __name__=='__main__':unittest.main()
