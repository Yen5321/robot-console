"""Testable independent executor policy. All deadlines use local monotonic time.

No boolean 'real enabled' configuration. Evidence is measured within the current
executor session; restarting never silently restores motion authorization.
"""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from .core import JointGuard,vector
from .cpv_sdk import FIRMWARE

def pose_error(a,b):
    return float(np.linalg.norm(a[:3]-b[:3])),float(np.linalg.norm((Rotation.from_euler('xyz',a[3:])*Rotation.from_euler('xyz',b[3:]).inv()).as_rotvec()))

class CpvPolicy:
    def __init__(self,model,sdk,event=lambda *a,**k:None,clock=None):
        self.model=model;self.sdk=sdk;self.event=event
        self.clock=clock
        self.guard=JointGuard(model.names,model.lower,model.upper,.05,.1)
        self.feedback=None;self.armed=False;self.enabling=False;self.enable_deadline=0.;self.next_enable=0.
        self.fault='';self.emergency_sent=False;self.emergency_ack_seen=False;self.phase='not_ready';self.desired=np.zeros(6)
        self.authority=None;self.authority_at=-math.inf;self.servo_at=-math.inf;self.servo_seq=-math.inf
        self.rearm=True;self.fk_samples=[];self.hold_passed=False;self.directions=set();self.task=None
        self.last_result=None;self.zero_reference=None;self.zero_started=0.;self.last_drift=None
        self.previous_feedback_q=None;self.stationary_since=math.inf

    def fresh(self,now):
        f=self.feedback
        return bool(f is not None and all(0<=now-f[k]<.25 for k in ('q_at','pose_at','enabled_at','status_at')) and f['pose'] is not None and np.isfinite(f['q']).all() and np.isfinite(f['pose']).all())

    def identity_ok(self):
        return self.feedback is not None and self.feedback['firmware']==FIRMWARE and self.feedback['node_type']=='Piper_L_MC'

    def ready_to_jog(self):return len(self.fk_samples)>=3 and self.hold_passed and len(self.directions)==12

    def stop(self,reason,emergency=False):
        if not self.fault:
            self.fault=reason;self.event('fault',reason=reason,source='cpv_executor')
        self.task=None;self.desired[:]=0;self.rearm=True;self.enabling=False
        if emergency and not self.emergency_sent:
            self.armed=False;self.phase='electronic_stop'
            self.guard.previous[:]=0  # command reference only; not measured stop
            self.sdk.emergency();self.emergency_sent=True
        elif not self.emergency_sent:self.phase='fault_decelerating'

    def accept_authority(self,payload,now):
        at=float(payload['origin']);v=vector(payload['input'],5)
        if not math.isfinite(at) or not 0<=now-at<.25:raise ValueError('control_origin_expired')
        if np.any(v) and not 0<=now-float(payload['input_origin'])<.25:raise ValueError('jog_input_origin_expired')
        if at<self.authority_at:raise ValueError('control_origin_reordered')
        if np.max(np.abs(v))>1:raise ValueError('input_out_of_range')
        self.authority=dict(payload);self.authority_at=at
        # Neutral input, not traffic alone, rearms. A restarted gateway must enable
        # again: its epoch changes and the executor latches before accepting input.
        epoch=str(payload['epoch'])
        if hasattr(self,'epoch') and self.epoch!=epoch and self.armed:self.stop('gateway_restarted')
        self.epoch=epoch
        if not np.any(v) and not payload.get('home',False):self.rearm=False

    def accept_trajectory(self,names,velocity,source_at,horizon,now):
        if source_at<=self.servo_seq:return
        self.desired=self.guard.validate(names,velocity,now-source_at,horizon)
        self.servo_seq=source_at;self.servo_at=source_at

    def action(self,name,now,**kw):
        if name=='estop':self.stop('manual_emergency_stop',True);return
        if name=='stop':
            self.task=None;self.desired[:]=0;self.rearm=True
            if self.authority:self.authority['input']=[0]*5;self.authority['home']=False
            return
        if not self.fresh(now):raise ValueError('physical_feedback_expired')
        f=self.feedback
        if name=='recover':
            if f['status']!=0 or f['driver_fault']:raise ValueError('controller_not_normal_support_arm_and_reset_on_site; no SDK reset is sent')
            if self.emergency_sent and not self.emergency_ack_seen:raise ValueError('electronic_stop_state_not_confirmed; support_arm_and_check_controller_on_site')
            if np.any(self.guard.previous):raise ValueError('wait_for_zero')
            if now-self.stationary_since<.5:raise ValueError('wait_for_measured_stationary_feedback_500ms')
            self.fault='';self.emergency_sent=False;self.emergency_ack_seen=False;self.armed=False;self.enabling=False;self.rearm=True;self.phase='not_ready';self.authority=None
            return
        if self.fault:raise ValueError('explicit_recovery_required:'+self.fault)
        if not self.identity_ok():raise ValueError('PIPER_L_or_full_firmware_mismatch')
        if name=='sample_fk':
            if self.armed or self.enabling:raise ValueError('FK_sample_requires_disarmed_readback')
            error=pose_error(self.model.pose(f['q']),f['pose'])
            if error[0]>.005 or error[1]>math.radians(2):raise ValueError(f'FK_mismatch:{error}')
            if any(np.linalg.norm(f['q']-q)<.15 for q in self.fk_samples):raise ValueError('choose_a_distinct_representative_pose')
            self.fk_samples.append(f['q'].copy());self.event('fk_sample',q=f['q'].tolist(),sdk_pose=f['pose'].tolist(),error=list(error));return
        if name=='enable':
            if getattr(self.sdk,'query_index',48)<48:raise ValueError('wait_for_initial_read_only_CPV_parameter_snapshot')
            if not self.sdk.parameter_snapshot_ready():raise ValueError('CPV_parameter_snapshot_incomplete; no mode switch or enable sent')
            if not self.fk_samples:raise ValueError('read_only_FK_sample_required')
            if np.any(f['q']<self.guard.lower) or np.any(f['q']>self.guard.upper):raise ValueError('joint_position_outside_model_before_enable')
            if f['status']!=0 or f['driver_fault']:raise ValueError('controller_not_normal')
            if self.armed:
                if not self.ready_to_jog() or self.task or np.any(self.guard.previous):raise ValueError('commissioning_or_stop_incomplete')
                self.rearm=True;self.authority=None;self.authority_at=now
                return  # explicit handoff after commissioning, no mode re-send
            if self.enabling:raise ValueError('already_enabling')
            # Anchor before switching mode: drift during transition must not be
            # hidden by taking a new reference only after the controller ACK.
            self.zero_reference=f['pose'].copy();self.zero_started=now;self.last_drift=None
            self.enabling=True;self.enable_deadline=now+3.;self.next_enable=now;self.phase='enabling';self.rearm=True
            return
        if not self.armed:raise ValueError('explicit_enable_required')
        if np.any(self.guard.previous) or self.task:raise ValueError('wait_for_stop_or_task_completion')
        if name=='hold_test':
            self.task={'kind':'hold','start':now,'end':now+5.,'pose':f['pose'].copy(),'max_mm':0.,'max_deg':0.}
        elif name=='pulse':
            if not self.hold_passed:raise ValueError('zero_hold_test_required')
            joint=int(kw['joint']);sign=int(kw['sign'])
            if not 1<=joint<=6 or sign not in (-1,1):raise ValueError('invalid_pulse')
            # .4 s including acceleration, then <=.1 s deceleration: <=.5 s total.
            self.task={'kind':'pulse','start':now,'end':now+.4,'joint':joint,'sign':sign,'q':f['q'].copy()}
        else:raise ValueError('unknown_executor_action')
        self.rearm=True;self.last_result=None

    def tick(self,now,dt):
        self.feedback=self.sdk.poll();f=self.feedback
        if self.clock is not None:now=self.clock()  # sample age after receive work
        if self.fresh(now):
            if self.previous_feedback_q is None or max(abs(f['q']-self.previous_feedback_q))>.0002:
                self.stationary_since=now;self.previous_feedback_q=f['q'].copy()
        else:self.stationary_since=math.inf;self.previous_feedback_q=None
        if self.emergency_sent and self.fresh(now) and f['status']==1:self.emergency_ack_seen=True
        if not 0<dt<=.05 and (self.armed or self.enabling):self.stop('executor_cycle_overrun',True)
        if (self.armed or self.enabling) and not self.fresh(now):self.stop('physical_feedback_expired',True)
        if (self.armed or self.enabling) and self.fresh(now) and self.zero_reference is not None and not np.any(self.guard.previous):
            mm,rad=pose_error(f['pose'],self.zero_reference)
            self.last_drift={'duration_s':now-self.zero_started,'translation_mm':mm*1000,'rotation_deg':math.degrees(rad)}
            # Applies from the first enable, including before hold-test passes.
            if mm>.002 or rad>math.radians(1):
                self.event('unexpected_zero_motion',q=f['q'].tolist(),pose=f['pose'].tolist(),reference=self.zero_reference.tolist(),**self.last_drift)
                self.hold_passed=False;self.stop('zero_velocity_hold_drift',True)
        if self.armed and self.fresh(now):
            if f['status']!=0 or f['driver_fault']:self.stop('controller_fault:'+str(f['status']),True)
            elif not all(f['enabled']) or f['ctrl_mode']!=1 or f['cpv_mode']!=5:self.stop('enable_or_CAN_CPV_mode_changed',True)
        if self.enabling and not self.fault:
            if now>=self.enable_deadline:self.stop('enable_CAN_CPV_confirmation_timeout',True)
            elif all(f['enabled']) and f['ctrl_mode']==1 and f['cpv_mode']==5:
                self.enabling=False;self.armed=True;self.phase='zero_hold';self.authority=None;self.authority_at=now;self.rearm=True
                self.event('enable_confirmed')
            elif now>=self.next_enable:self.sdk.enable();self.next_enable=now+.2
        if not self.armed:return np.zeros(6)
        requested=np.zeros(6)
        if self.task and not self.fault:
            t=self.task
            if t['kind']=='hold':
                mm,angle=pose_error(f['pose'],t['pose']);t['max_mm']=max(t['max_mm'],mm*1000);t['max_deg']=max(t['max_deg'],math.degrees(angle))
                if t['max_mm']>2 or t['max_deg']>1:
                    self.hold_passed=False;self.stop('zero_velocity_hold_drift',True)
                elif now>=t['end']:
                    self.hold_passed=True;self.last_result={'kind':'hold','passed':True,'max_mm':t['max_mm'],'max_deg':t['max_deg']};self.task=None;self.event('commission_result',**self.last_result)
            elif now<t['end']:requested[t['joint']-1]=t['sign']*.01
            elif not np.any(self.guard.previous):
                dq=f['q']-t['q'];i=t['joint']-1
                passed=.0005<dq[i]*t['sign']<=.006 and max(abs(np.delete(dq,i)))<.002
                self.last_result={'kind':'pulse','joint':i+1,'sign':t['sign'],'delta_rad':dq.tolist(),'passed':bool(passed)}
                self.event('commission_result',**self.last_result);self.task=None
                if passed:self.directions.add((i+1,t['sign']))
                else:self.stop('joint_direction_or_drift_test_failed')
        elif not self.fault and self.authority is not None:
            a=self.authority
            if now-self.authority_at>=.25:self.stop('control_origin_expired')
            elif a.get('fault'):self.stop(str(a['fault']),bool(a.get('emergency')))
            elif np.any(a['input']) and now-float(a['input_origin'])>=.25:self.stop('jog_input_origin_expired')
            elif a.get('active') and not self.rearm:
                if not self.ready_to_jog():self.stop('commissioning_incomplete')
                elif now-self.servo_at>=.1:self.stop('servo_output_expired')
                else:requested=self.desired.copy()
        if self.fault:requested[:]=0
        out=self.guard.step(requested,f['q'],min(max(dt,.001),.05))
        self.sdk.velocity(out)  # six zeros continue while armed, including fault hold
        self.phase=('fault_' if self.fault else '')+('decelerating' if not np.any(requested) and np.any(out) else 'jogging' if np.any(out) else 'zero_hold')
        if not np.any(out):
            if self.zero_reference is None:self.zero_reference=f['pose'].copy();self.zero_started=now
            mm,rad=pose_error(f['pose'],self.zero_reference);self.last_drift={'duration_s':now-self.zero_started,'translation_mm':mm*1000,'rotation_deg':math.degrees(rad)}
            if mm>.002 or rad>math.radians(1):
                self.hold_passed=False;self.stop('zero_velocity_hold_drift',True)
        else:self.zero_reference=None
        return out
