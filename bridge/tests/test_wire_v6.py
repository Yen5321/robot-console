import unittest
import socket
import struct
import threading
from dataclasses import replace
from robot_bridge.wire_v6 import *
from robot_bridge.protocol import TelemetryFrame
from robot_bridge.service import BridgeService
from tests.test_service import FakeArm, FakeCamera, FakeLaser
from tests.test_jog_safety import Clock

def frame(seq=1,stamp=1000,**kw):
    return replace(ControlFrame(seq,stamp,0,0,0,0,0,0,0,0,0,0,55,32,0,1),**kw)

class WireTests(unittest.TestCase):
    def test_crc_standard_check_and_control_roundtrip(self):
        self.assertEqual(crc16(b'123456789'),0x29b1)
        packet=encode_control(0x12345678,frame())
        self.assertEqual(len(packet),32)
        sid,alive,f=decode_control(packet)
        self.assertEqual(sid,0x12345678);self.assertTrue(alive);self.assertEqual(f,frame())
    def test_every_single_bit_corruption_rejected(self):
        packet=encode_control(1,frame())
        for bit in range(256):
            damaged=bytearray(packet);damaged[bit//8]^=1<<(bit%8)
            with self.assertRaises(ProtocolError):decode_control(damaged)
    def test_sequence_wrap_and_duplicates(self):
        g=SessionGuard(1,('a',1))
        g.accept(1,('a',1),frame(65535),1000)
        g.accept(1,('a',1),frame(0,1001),1001)
        for seq in (0,65535):
            with self.assertRaises(ProtocolError):g.accept(1,('a',1),frame(seq,1002),1002)
    def test_timestamp_window_and_wrap(self):
        for stamp in (799,1051):
            with self.assertRaises(ProtocolError):SessionGuard(1,('a',1)).accept(1,('a',1),frame(stamp=stamp),1000)
        SessionGuard(1,('a',1)).accept(1,('a',1),frame(stamp=0xfffffff0),10)
    def test_session_owner_and_neutral_required(self):
        for sid,peer,f in [(2,('a',1),frame()),(1,('b',1),frame()),(1,('a',1),frame(mode=2,arm_dy=10))]:
            with self.assertRaises(ProtocolError):SessionGuard(1,('a',1)).accept(sid,peer,f,1000)
    def test_estop_priority_but_not_foreign_session(self):
        g=SessionGuard(1,('a',1));g.accept(1,('a',1),frame(),1000)
        g.accept(1,('a',1),frame(seq=0,stamp=0,estop=1),99999)
        with self.assertRaises(ProtocolError):g.accept(2,('a',1),frame(estop=1),1000)
    def test_invalid_sensors_have_mask_and_nan(self):
        base=TelemetryFrame(1,0,0,0,0,(0,0,0,0,0,0),0,0)
        packet=telemetry(1,base,1000,{})
        self.assertEqual(len(packet),97);checked(packet,97)
        self.assertEqual(struct.unpack_from('<I',packet,14)[0],0)
        self.assertTrue(all(math.isnan(v) for v in struct.unpack_from('<14f',packet,39)))

class GatewayTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.peer=('127.0.0.1',51000)
        self.service=BridgeService(FakeArm(),FakeCamera(),FakeLaser(),{},udp_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM),start_video=False,_time_fn=self.clock)
        self.payload={'action':'session','token':self.service._action_token,'udp_port':51000,'request_id':'session-one','control_profile':'piper-l-cpv-v8.1'}
        self.sid=self.service.arm_action(self.payload,'127.0.0.1')['session']
    def tearDown(self):self.service.close()
    def send(self,f,alive=True):return self.service.process_datagram(encode_control(self.sid,f,alive),self.peer)
    def test_legacy_wire_is_rejected(self):
        self.assertFalse(self.service.process_datagram(frame().pack(),self.peer))
        self.assertIsNone(self.service._last_received_at)
    def test_ui_heartbeat_fault_latches_and_cannot_resume_on_heartbeat(self):
        self.assertTrue(self.send(frame()))
        self.assertFalse(self.send(frame(2,1001),False));self.assertTrue(self.service._fault_latched)
        self.assertFalse(self.send(frame(3,1002)))
        self.assertTrue(self.service._fault_latched)
    def test_rejected_packets_do_not_feed_watchdog(self):
        self.send(frame());self.clock.now+=.251
        self.assertFalse(self.send(frame(2,1000)))
        self.service.safety_tick();self.assertTrue(self.service._fault_latched)
    def test_new_path_session_requires_recovery_and_invalidates_old(self):
        self.send(frame());old=self.sid;self.clock.now+=1
        p={**self.payload,'request_id':'other'}
        self.sid=self.service.arm_action(p,'127.0.0.1')['session']
        self.assertNotEqual(old,self.sid);self.assertTrue(self.service._fault_latched)
        self.assertFalse(self.service.process_datagram(encode_control(old,frame(2,2000)),self.peer))
    def test_session_retry_is_idempotent(self):
        self.assertEqual(self.service.arm_action(self.payload,'127.0.0.1')['session'],self.sid)
    def test_estop_bypasses_slow_management_action(self):
        self.send(frame())
        entered=threading.Event();release=threading.Event();results=[]
        def enable(): entered.set();release.wait(2)
        self.service.arm.enable=enable
        p={**self.payload,'action':'enable','request_id':'slow-enable'}
        worker=threading.Thread(target=lambda:results.append(self.service.arm_action(p,'127.0.0.1')))
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            self.assertTrue(self.send(frame(2,1001))) # heartbeat is handled while SDK waits
            self.assertFalse(self.send(frame(3,1002,estop=1)))
            self.assertTrue(self.service._fault_latched)
        finally:release.set();worker.join(2)
        self.assertFalse(results[0]['ok'])

