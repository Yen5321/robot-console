"""Pinned pyAgxArm boundary. Only the executor imports this module on hardware.

The SDK signs joints 2..5 itself. Receive timestamps come from SocketCAN, not
SDK aggregate getters. No background reader can hide a send error. Startup
queries identification/CPV parameters only; it never enables or switches mode.
"""
import math,re,time
from collections import deque
import numpy as np
SDK_COMMIT='e7aef17d54cac80cbaeb1b4110ab3d8f1337a95b'
FIRMWARE='S-V1.9-0260716'

def enum_value(v):return int(getattr(v,'value',v))

class CpvSdk:
    def __init__(self,channel):
        import can
        from pyAgxArm import create_agx_arm_config,AgxArmFactory,ArmModel,PiperFW
        self.arm=AgxArmFactory.create_arm(create_agx_arm_config(robot=ArmModel.PIPER_L,firmeware_version=PiperFW.V189,channel=channel,timeout=0,log_level='ERROR'))
        self.arm.set_auto_set_motion_mode_enabled(False)
        self.arm.connect(start_read_thread=False)
        self.comm=self.arm._ctx.get_comm()
        # Pinned SDK normally catches CAN send errors. Replace only transport TX
        # with bounded python-can send; all packing/sign conversions remain SDK.
        self.sent=0;self.velocity_calls=0;self.tx_recent=deque(maxlen=120)
        def strict_send(msg,timeout=None):
            self.comm.send_bus.send(msg,timeout=.001)
            self.sent+=1
            self.tx_recent.append({'time':time.monotonic(),'can_id':msg.arbitration_id,
                                   'dlc':msg.dlc,'data_hex':bytes(msg.data).hex()})
        self.comm.send=strict_send
        self.times={};self.firmware_bytes=bytearray();self.firmware='unknown';self.node_type='unknown'
        self.parameters={};self.parameter_received_at={};self.query_index=0;self.next_query=time.monotonic()+.1
        self.q=np.zeros(6);self.pose=None;self.enabled=[False]*6;self.status=None;self.ctrl=None;self.mode=None
        self.motor_velocity=[None]*6;self.driver_fault=False
        # One read request; raw parsing handles variable-length firmware strings.
        self.comm.send(can.Message(arbitration_id=0x4af,is_extended_id=False,data=[1,0,0,0,0,0,0,0]))

    def poll(self):
        now=time.monotonic()
        for _ in range(256):
            frame=self.comm.recv_bus.recv(timeout=0)
            if frame is None:break
            if frame.is_error_frame:raise RuntimeError('CAN_error_frame')
            if not self.valid_feedback_frame(frame):continue
            age=time.time()-frame.timestamp
            if not 0<=age<.25:continue
            stamp=time.monotonic()-age;cid=frame.arbitration_id
            self.comm._trigger_callback(frame)
            self.times[cid]=stamp
            if cid==0x4af:
                self.firmware_bytes.extend(frame.data)
                self.firmware_bytes=self.firmware_bytes[-512:]
                raw=bytes(self.firmware_bytes)
                match=re.search(rb'S-V\d+\.\d+-\d{7}(?=\x00)',raw)
                if match:self.firmware=match.group().decode('ascii')
                if b'Piper_L_MC' in raw:self.node_type='Piper_L_MC'
        joints=self.arm.get_joint_angles();pose=self.arm.get_flange_pose();status=self.arm.get_arm_status()
        if joints is not None:self.q=np.array(joints.msg,dtype=float)
        if pose is not None:self.pose=np.array(pose.msg,dtype=float)
        if status is not None:
            self.status=enum_value(status.msg.arm_status);self.ctrl=enum_value(status.msg.ctrl_mode);self.mode=enum_value(status.msg.mode_feedback)
        self.driver_fault=False
        for i in range(6):
            ds=self.arm.get_driver_states(i+1);ms=self.arm.get_motor_states(i+1)
            if ds:
                flags=ds.msg.foc_status;self.enabled[i]=bool(flags.driver_enable_status)
                self.driver_fault |= any(bool(getattr(flags,k,False)) for k in ('voltage_too_low','motor_overheating','driver_overcurrent','driver_overheating','collision_status','driver_error_status','stall_status'))
            self.motor_velocity[i]=float(ms.msg.velocity) if ms and now-self.times.get(0x251+i,-math.inf)<.25 else None
        # Slow, bounded queries only while disarmed; never write CPV parameters.
        return self.snapshot()

    @staticmethod
    def valid_feedback_frame(frame):
        # CPV query responses are seven bytes; write ACKs are one byte. The
        # previous global DLC==8 filter silently discarded both on real CAN.
        if frame.is_remote_frame or frame.is_extended_id or frame.is_fd:return False
        data=bytes(frame.data);cid=frame.arbitration_id
        if frame.dlc!=len(data):return False
        if 0x181<=cid<=0x186:
            return data==b'\xac' or (len(data) in (7,8) and data[:1]==b'a' and
                data[1:3] in (b'po',b'sp',b'ac',b'dc',b'vv',b'pp',b'kp',b'ki'))
        if cid==0x4af:return 1<=len(data)<=8
        return len(data)==8

    def parameter_snapshot_ready(self):
        return self.query_index>=48 and all(isinstance(self.parameters.get(f'{name}:{joint}'),(int,float))
            and math.isfinite(self.parameters[f'{name}:{joint}'])
            for name in ('pos','vel','acc','dcc','cv','pp','kp','ki') for joint in range(1,7))

    def read_parameter_step(self):
        if time.monotonic()<self.next_query or self.query_index>=48:return
        self.next_query=time.monotonic()+.1
        name=('pos','vel','acc','dcc','cv','pp','kp','ki')[self.query_index//6];joint=self.query_index%6+1
        value=getattr(self.arm,'get_cpv_'+name)(joint,timeout=0,min_interval=.1)
        key=f'{name}:{joint}'
        if value is not None:
            self.parameters[key]=float(getattr(value,'msg',value));self.parameter_received_at[key]=time.monotonic();self.query_index+=1
        else:
            attempts=self.parameters.get('_attempts_'+key,0)+1;self.parameters['_attempts_'+key]=attempts
            if attempts>=5:self.parameters[key]='no_response';self.query_index+=1

    def snapshot(self):
        oldest=lambda ids:min(self.times.get(i,-math.inf) for i in ids)
        return {'q':self.q.copy(),'pose':None if self.pose is None else self.pose.copy(),
                'q_at':oldest(range(0x2a5,0x2a8)),'pose_at':oldest(range(0x2a2,0x2a5)),
                'enabled_at':oldest(range(0x261,0x267)),'status_at':self.times.get(0x2a1,-math.inf),
                'enabled':self.enabled.copy(),'status':self.status,'ctrl_mode':self.ctrl,'cpv_mode':self.mode,
                'driver_fault':self.driver_fault,'motor_velocity_rad_s':self.motor_velocity.copy(),
                'firmware':self.firmware,'node_type':self.node_type}

    def enable(self):
        self.arm.set_motion_mode('cpv')
        self.arm.enable(255)

    def velocity(self,values):
        for i,v in enumerate(values):
            self.arm.move_cpv_vel(i+1,float(v));self.velocity_calls+=1

    def emergency(self):self.arm.electronic_emergency_stop()
    def close(self):self.arm.disconnect()
