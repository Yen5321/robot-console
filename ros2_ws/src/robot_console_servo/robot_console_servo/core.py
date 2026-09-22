"""Hardware-independent motion policy. SI internally, monotonic times on one host.

Servo owns differential IK. This layer does not integrate an unbounded Cartesian
position target. It owns input expiry, orientation reference and bounded stopping.
"""
from dataclasses import dataclass, field
import math
import numpy as np
from scipy.spatial.transform import Rotation


def vector(values, count):
    a = np.asarray(values, dtype=float)
    if a.shape != (count,) or not np.isfinite(a).all():
        raise ValueError(f"Expected {count} finite values")
    return a


def cap(v, limit):
    n = np.linalg.norm(v)
    return v * min(1., limit / n) if n else v.copy()


@dataclass
class Motion:
    timeout: float = .25
    linear_speed: float = .05
    angular_speed: float = math.radians(10)
    linear_acceleration: float = .1
    angular_acceleration: float = math.radians(20)
    state: str = "not_ready"
    reason: str = "waiting_for_feedback"
    enabled: bool = False
    fault: bool = False
    rearm: bool = True
    stamp: float = -math.inf
    input_stamp: float = -math.inf
    feedback_stamp: float = -math.inf
    input: np.ndarray = field(default_factory=lambda: np.zeros(5))
    pose: np.ndarray = field(default_factory=lambda: np.zeros(6))
    twist: np.ndarray = field(default_factory=lambda: np.zeros(6))
    reference: object = None
    home: object = None
    goal: object = None
    home_reference: object = None
    servo_status: int = 0
    workspace: np.ndarray = field(default_factory=lambda: np.array([[-.5,.5],[-.5,.5],[0.,.6]]))

    def feedback(self, pose, stamp):
        self.pose = vector(pose, 6)
        self.feedback_stamp = stamp

    def accept(self, values, stamp, now):
        values = vector(values, 5)
        if np.max(np.abs(values)) > 1: raise ValueError("Input outside [-1,1]")
        if not math.isfinite(stamp) or stamp <= self.input_stamp or not 0 <= now-stamp < self.timeout:
            raise ValueError("stale_or_reordered_input")
        self.stamp = max(self.stamp,stamp)
        self.input_stamp = stamp
        if not np.any(values): self.rearm = False
        elif self.fault or self.rearm or not self.enabled:
            raise ValueError("release_and_enable_before_jog")
        if np.any(values) and not np.any(self.input):
            self.reference = Rotation.from_euler('xyz', self.pose[3:])
        self.input = values
        if np.any(values): self.goal = None

    def stop(self, reason, emergency=False):
        if self.fault:
            if emergency:self.state='estop'
            return
        self.input[:] = 0
        self.goal = None
        self.home_reference = None
        self.twist[:] = 0
        self.fault = True
        self.rearm = True
        self.reason = reason
        self.state = 'estop' if emergency else 'fault'

    def action(self, action, now):
        if action == 'stop':
            self.input[:] = 0; self.goal = None; self.home_reference=None; self.rearm = True
            return
        if action == 'estop': self.stop('manual_emergency_stop', True); return
        if now-self.feedback_stamp >= self.timeout: raise ValueError('feedback_expired')
        if np.any(self.input) or np.linalg.norm(self.twist) > 1e-6:
            raise ValueError('release_and_wait_for_stop')
        if action == 'recover':
            self.fault = False; self.enabled = False; self.rearm = True
            self.state = 'not_ready'; self.reason = 'not_enabled'
        elif action == 'enable':
            if self.fault: raise ValueError('explicit_recovery_required')
            self.enabled = True; self.rearm = True; self.state = 'ready'; self.reason = ''
        elif action == 'set_home':
            if not self.enabled or self.fault: raise ValueError('not_ready')
            self.home = self.pose.copy()
        elif action == 'home':
            if self.home is None: raise ValueError('home_not_recorded_in_this_model')
            if not self.enabled or self.fault: raise ValueError('not_ready')
            self.goal = self.home.copy()
            self.home_reference = self.pose.copy()
        else: raise ValueError('unknown_action')

    def tick(self, now, dt):
        active = np.any(self.input) or np.any(self.twist) or self.goal is not None
        if self.fault: return np.zeros(6)
        if now-self.feedback_stamp >= self.timeout:
            if active: self.stop('feedback_expired')
            else: self.state='not_ready'; self.reason='feedback_expired'
            return np.zeros(6)
        # Session heartbeats may keep a home operation alive, but cannot extend a
        # nonzero manual jog command. Its original input timestamp is independent.
        if (now-self.stamp >= self.timeout and self.stamp != -math.inf) or (np.any(self.input) and now-self.input_stamp>=self.timeout):
            self.stop('control_input_expired'); return np.zeros(6)
        if not self.enabled:
            self.state='not_ready'; self.reason='not_enabled'; return np.zeros(6)
        if not 0 < dt <= .05:
            self.stop('control_loop_overrun'); return np.zeros(6)
        actual = Rotation.from_euler('xyz', self.pose[3:])
        desired = np.zeros(6)
        if self.goal is not None:
            error = self.goal[:3]-self.pose[:3]
            rot_error = (Rotation.from_euler('xyz', self.goal[3:])*actual.inv()).as_rotvec()
            if np.linalg.norm(error)<.0005 and np.linalg.norm(rot_error)<math.radians(.2):
                self.goal=None
            else:
                reference_rotation=Rotation.from_euler('xyz',self.home_reference[3:])
                lead_rotation=(reference_rotation*actual.inv()).as_rotvec()
                if np.linalg.norm(self.home_reference[:3]-self.pose[:3])>.002 or np.linalg.norm(lead_rotation)>math.radians(1):
                    self.stop('home_tracking_exceeded_2mm_1deg');return np.zeros(6)
                self.home_reference[:3]+=cap(self.goal[:3]-self.home_reference[:3],.004*dt)
                increment=(Rotation.from_euler('xyz',self.goal[3:])*reference_rotation.inv()).as_rotvec()
                reference_rotation=Rotation.from_rotvec(cap(increment,math.radians(2)*dt))*reference_rotation
                self.home_reference[3:]=reference_rotation.as_euler('xyz')
                desired[:3]=cap((self.home_reference[:3]-self.pose[:3])*4,.004)
                desired[3:]=cap((reference_rotation*actual.inv()).as_rotvec()*4,math.radians(2))
        elif np.any(self.input):
            desired[:3] = cap(self.input[:3], 1)*self.linear_speed
            # UI pitch/yaw mean fixed-axis XYZ (RPY) Euler rates; convert to base angular
            # velocity using the finite SO(3) increment, never treat RPY rates as omega.
            angles = self.reference.as_euler('xyz')
            rates=cap(self.input[3:],1)*self.angular_speed
            angles[1:] += rates*dt
            candidate = Rotation.from_euler('xyz', angles)
            correction = (candidate*actual.inv()).as_rotvec()
            # Bound orientation reference lead (anti-windup), including near limits.
            if np.any(rates):
                self.reference = Rotation.from_rotvec(cap(correction, math.radians(1)))*actual
            # Pure translation must not drag the reference toward an actual
            # orientation error. Keep the press-time attitude unchanged.
            # Never amplify selected rotational input to catch up. During pure
            # translation, bounded posture correction has a separate 1 deg/s cap.
            angular_cap=np.linalg.norm(rates) if np.any(rates) else math.radians(1)
            desired[3:] = cap((self.reference*actual.inv()).as_rotvec()*4, angular_cap)
        # Smooth both start and release. TCP bounds predict stopping distance.
        accel = np.array([self.linear_acceleration]*3+[self.angular_acceleration]*3)
        out = self.twist + np.clip(desired-self.twist, -accel*dt, accel*dt)
        for i in range(3):
            distance = self.workspace[i,1]-self.pose[i] if out[i]>0 else self.pose[i]-self.workspace[i,0]
            allowed = math.sqrt(max(0., 2*self.linear_acceleration*(distance-.001)))
            out[i] = math.copysign(min(abs(out[i]),allowed),out[i])
        # Servo halt statuses are motion restrictions, not SDK emergencies.
        if self.servo_status in (2,4,5):
            out[:] = 0; self.input[:] = 0; self.goal=None; self.rearm=True
            self.reason=f'servo_restricted:{self.servo_status}'
        else: self.reason=''
        self.twist = out
        self.state = ('jogging' if np.any(self.input) or self.goal is not None else 'decelerating') if np.linalg.norm(out)>1e-7 else 'holding'
        return out.copy()


class JointGuard:
    """Final output limiter; bounds derive from model and commissioning config."""
    def __init__(self, names, lower, upper, velocity=.3, acceleration=.6):
        self.names = list(names)
        self.lower=vector(lower,6); self.upper=vector(upper,6)
        self.velocity=float(velocity); self.acceleration=float(acceleration)
        self.previous=np.zeros(6)

    def validate(self, names, velocities, stamp_age, point_time):
        if list(names)!=self.names: raise ValueError('joint_order_mismatch')
        if not math.isfinite(stamp_age) or not -.02<=stamp_age<.1: raise ValueError('servo_output_expired')
        if not math.isfinite(point_time) or not 0<=point_time<=.25: raise ValueError('invalid_trajectory_horizon')
        return vector(velocities,6)

    def step(self, requested, q, dt):
        q=vector(q,6); v=vector(requested,6)
        if not 0<dt<=.05: raise ValueError('execution_loop_overrun')
        if np.any(q<self.lower) or np.any(q>self.upper): raise ValueError('joint_position_outside_model')
        scale=min(1., self.velocity/max(np.max(np.abs(v)),1e-12))
        distance=np.where(v>=0,self.upper-q,q-self.lower)-.02
        allowed=np.sqrt(np.maximum(0,2*self.acceleration*distance))
        scale=min(scale,float(np.min(allowed/np.maximum(np.abs(v),1e-12))))
        target=v*scale
        delta=target-self.previous
        factor=min(1.,self.acceleration*dt/max(np.max(np.abs(delta)),1e-12))
        out=self.previous+factor*delta
        if np.any(q+out*dt<self.lower) or np.any(q+out*dt>self.upper):
            raise ValueError('cannot_stop_before_joint_limit')
        self.previous=out
        return out.copy()
