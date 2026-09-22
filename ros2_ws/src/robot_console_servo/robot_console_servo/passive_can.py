"""Read-only CAN receiver: no Interface.ConnectPort, enable, mode or TX calls.

Every joint pair needs a fresh physical frame; republishing cannot refresh age.
Firmware is deliberately reported unknown because this reader sends no query.
"""
import math
import time
import numpy as np


class PassiveCan:
    def __init__(self, channel):
        import can
        from piper_sdk.protocol.protocol_v2 import C_PiperParserV2
        from piper_sdk.piper_msgs.msg_v2 import PiperMessage
        self.bus=can.Bus(channel=channel,interface='socketcan',receive_own_messages=False)
        self.parser=C_PiperParserV2(); self.Message=PiperMessage
        self.q=np.zeros(6); self.times=[-math.inf]*3
        self.status=None; self.ctrl_mode=None; self.status_at=-math.inf
        self.enabled=[False]*6; self.motor_times=[-math.inf]*6
        self.sdk_pose=None; self.pose_times=[-math.inf]*3; self._pose=np.zeros(6)

    def poll(self):
        for _ in range(100):
            frame=self.bus.recv(timeout=0)
            if frame is None: break
            if frame.is_error_frame or frame.is_remote_frame: continue
            msg=self.Message()
            if not self.parser.DecodeMessage(frame,msg): continue
            name=msg.type_.name
            # SocketCAN kernel receive time survives a slow reader / queued frames.
            age=time.time()-frame.timestamp
            if not 0<=age<.25: continue
            now=time.monotonic()-age
            for i, suffix in enumerate(('12','34','56')):
                if name=='PiperMsgJointFeedBack_'+suffix:
                    for k in (i*2,i*2+1): self.q[k]=math.radians(getattr(msg.arm_joint_feedback,f'joint_{k+1}')*.001)
                    self.times[i]=now
            if name=='PiperMsgStatusFeedback':
                self.status=int(msg.arm_status_msgs.arm_status)
                self.ctrl_mode=int(msg.arm_status_msgs.ctrl_mode); self.status_at=now
            for i in range(3):
                if name==f'PiperMsgEndPoseFeedback_{i+1}':
                    for k in (2*i,2*i+1):
                        raw=getattr(msg.arm_end_pose,('X_axis','Y_axis','Z_axis','RX_axis','RY_axis','RZ_axis')[k])
                        self._pose[k]=raw*.001 if k<3 else raw*.001
                    self.pose_times[i]=now
                    self.sdk_pose=self._pose.copy()
            for i in range(6):
                if name==f'PiperMsgLowSpdFeed_{i+1}':
                    self.enabled[i]=bool(getattr(msg,f'arm_low_spd_feedback_{i+1}').foc_status.driver_enable_status)
                    self.motor_times[i]=now
        return self.q.copy(),min(self.times)

    def close(self): self.bus.shutdown()
