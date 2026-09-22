"""Little-endian protocol v6. CRC-16/CCITT-FALSE; no legacy network fallback."""
import math
import struct
from .protocol import ControlFrame, ProtocolError, is_newer_sequence

CONTROL = struct.Struct('<IHI B 8h B H')  # 30 bytes + CRC16

def crc16(data):
    crc=0xffff
    for byte in data:
        crc ^= byte << 8
        for _ in range(8): crc=((crc << 1)^0x1021 if crc & 0x8000 else crc << 1)&0xffff
    return crc

def seal(data): return data+struct.pack('<H',crc16(data))
def checked(data, size):
    if len(data)!=size or crc16(data[:-2])!=struct.unpack('<H',data[-2:])[0]:
        raise ProtocolError('v6 length/CRC mismatch')
    return data[:-2]

def decode_control(data):
    sid,seq,stamp,flags,*v=CONTROL.unpack(checked(data,32))
    if flags & 0xe0: raise ProtocolError('reserved flags set')
    frame=ControlFrame(seq,stamp,flags&3,*v[:8],int(bool(flags&8)),v[8],v[9],int(bool(flags&4)),seq&1)
    frame.validate()
    return sid, bool(flags&16), frame

def encode_control(sid,frame,ui_alive=True):
    frame.validate()
    flags=frame.mode|(frame.estop<<2)|(frame.laser_enable<<3)|(int(ui_alive)<<4)
    return seal(CONTROL.pack(sid,frame.seq,frame.timestamp,flags,
        frame.drive_x,frame.drive_y,frame.drive_rotate,frame.arm_dx,frame.arm_dy,frame.arm_dz,
        frame.arm_pitch,frame.arm_yaw,frame.laser_power,frame.laser_speed))

def signed_delta(a,b,bits=32): return ((a-b+(1<<(bits-1)))&((1<<bits)-1))-(1<<(bits-1))

class SessionGuard:
    def __init__(self,sid,peer):
        self.sid=sid;self.peer=peer;self.last_seq=None;self.neutral_seen=False
        self.last_stamp=None
    def accept(self,sid,peer,frame,now_ms):
        if sid!=self.sid or peer!=self.peer: raise ProtocolError('session/owner mismatch')
        # A valid current-session estop dominates sequence and age checks.
        if frame.estop: return
        age=signed_delta(now_ms,frame.timestamp)
        if not -50<=age<=200: raise ProtocolError('expired/future control timestamp')
        if self.last_seq is not None and not is_newer_sequence(frame.seq,self.last_seq):
            raise ProtocolError('duplicate/reordered sequence')
        if self.last_stamp is not None and signed_delta(frame.timestamp,self.last_stamp)<0:
            raise ProtocolError('timestamp moved backwards')
        neutral=frame.mode==0 and not any(frame.arm_normalized+frame.drive_normalized) and not frame.laser_enable
        if not self.neutral_seen and not neutral: raise ProtocolError('neutral handshake required')
        self.neutral_seen=True;self.last_seq=frame.seq;self.last_stamp=frame.timestamp

def ack(sid,frame,status):
    return seal(struct.pack('<4sIHI B',b'ACK6',sid,frame.seq,frame.timestamp,status))

def telemetry(sid,base,now_ms,diagnostics,sensors=None):
    # Null/unconnected sensors never become plausible zero readings.
    sensors=sensors or {};mask=0
    pose=diagnostics.get('pose')
    if diagnostics.get('feedback_present') and diagnostics.get('feedback_age_ms') is not None and diagnostics['feedback_age_ms']<250 and pose and len(pose)==6 and all(math.isfinite(v) for v in pose): mask|=1
    else: pose=[float('nan')]*6
    currents=sensors.get('motor_current_a')
    if currents and len(currents)==6 and all(math.isfinite(v) for v in currents): mask|=2
    else: currents=[float('nan')]*6
    voltage=sensors.get('battery_voltage_v',float('nan'))
    magnet=sensors.get('magnetic_force_n',float('nan'))
    if math.isfinite(voltage): mask|=4
    if math.isfinite(magnet): mask|=8
    if sensors.get('base_attitude_valid'): mask|=16
    if sensors.get('battery_percent_valid'): mask|=32
    return seal(struct.pack('<4sIHI I',b'TLM6',sid,base.seq,now_ms,mask)+base.pack()+struct.pack('<14f',*pose,*currents,voltage,magnet))
