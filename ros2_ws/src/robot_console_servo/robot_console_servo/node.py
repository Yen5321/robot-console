"""MoveIt Servo I/O and a receive-only Piper commissioning mode.

The simulator integrates guarded joint commands. The real backend delegates
CAN exclusively to the independently timed CPV executor and commissioning gates.
"""
import json
import uuid
import urllib.request
import math
import os
from pathlib import Path
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger
from .core import Motion, JointGuard, vector
from .model import Model
from .settings import load as load_settings


class ServoGateway(Node):
    def __init__(self):
        super().__init__('console_motion')
        self.declare_parameter('mode','sim')
        self.declare_parameter('robot_description','')
        self.declare_parameter('can_name','can0')
        self.declare_parameter('home_path','recorded-home-v8-sim.json')
        self.declare_parameter('motion_config','')
        self.mode=self.get_parameter('mode').value
        if self.mode not in ('sim','readonly','real'):
            raise RuntimeError('REAL_MOTION_NOT_COMMISSIONED: use sim or readonly; no firmware upgrade or MOVE L fallback')
        self.model=Model(self.get_parameter('robot_description').value)
        self.settings=load_settings(self.get_parameter('motion_config').value)
        s=self.settings
        self.motion=Motion(timeout=s['input_timeout_s'],linear_speed=s['linear_speed_m_s'],angular_speed=s['angular_speed_rad_s'],linear_acceleration=s['linear_acceleration_m_s2'],angular_acceleration=s['angular_acceleration_rad_s2'],workspace=np.array(s['workspace_m']))
        self.guard=JointGuard(self.model.names,self.model.lower,self.model.upper,s['joint_velocity_rad_s'],s['joint_acceleration_rad_s2'])
        # Synthetic starting configuration, NOT a physical home command. Chosen
        # away from the model's singular zone (numerical condition number ~12).
        self.q=np.array([.72171815,1.55710658,-.99765818,.85081249,.87539701,.56477063]); self.qd=np.zeros(6)
        self.desired=np.zeros(6); self.output_at=-math.inf; self.output_ros_stamp=-1
        self.mutex=threading.RLock()
        self.events=deque(maxlen=1000); self.periods=deque(maxlen=3000)
        self.counters={'received_input':0,'servo_generated':0,'sim_executed':0,'sdk_calls':0,'observed_motion':0,'overruns':0}
        self.passive=None
        if self.mode=='readonly':
            from .passive_can import PassiveCan
            self.passive=PassiveCan(self.get_parameter('can_name').value)
        self.home_verified=False;self.home_test_active=False
        self.cpv_report={};self.epoch=uuid.uuid4().hex;self.enable_pending=False
        self.authority_pub=self.create_publisher(String,'/console/cpv_authority',1)
        self.create_subscription(String,'/console/cpv_feedback',self.executor_feedback,1)
        if self.mode=='real':
            self.motion.linear_speed=.002;self.motion.angular_speed=math.radians(1)
            self.guard.velocity=.05;self.guard.acceleration=.1
        self.home_path=Path(self.get_parameter('home_path').value)
        self.model_id=self.model.id
        if self.home_path.exists():
            saved=json.loads(self.home_path.read_text())
            if saved.get('model_id')==self.model_id and saved.get('mode')==self.mode:
                self.motion.home=vector(saved['pose_si'],6)
        self.js=self.create_publisher(JointState,'/joint_states',10)
        self.tw=self.create_publisher(TwistStamped,'/servo_node/delta_twist_cmds',1)
        self.diag=self.create_publisher(String,'/console/diagnostics',1)
        self.create_subscription(JointTrajectory,'/console/servo_output_stamped',self.trajectory,QoSProfile(depth=1))
        self.create_subscription(Int8,'/servo_node/status',self.servo_status,1)
        self.start_client=self.create_client(Trigger,'/servo_node/start_servo')
        self.start_future=None; self.servo_started=False
        self.last=time.monotonic(); self.next_diag=0.
        self.timer=self.create_timer(1./s['cycle_hz'],self.tick)
        self.create_timer(.5,self.start_servo)
        self.http=self.make_http()
        self.http_thread=threading.Thread(target=self.http.serve_forever,daemon=True)
        self.http_thread.start()
        self.event('started',mode=self.mode,real_motion_available=False)

    def executor_feedback(self,msg):
        if self.mode!='real':return
        with self.mutex:
            try:self.cpv_report=json.loads(msg.data)
            except ValueError:pass

    def executor_action(self,action):
        data=json.dumps({'action':action}).encode()
        req=urllib.request.Request('http://127.0.0.1:9091/action',data=data,headers={'Content-Type':'application/json'})
        with urllib.request.urlopen(req,timeout=.3) as response:result=json.load(response)
        if not result.get('ok'):raise ValueError(result.get('error','executor_action_rejected'))

    def event(self,event,**fields):
        self.events.append({'time':time.monotonic(),'event':event,**fields})

    def servo_status(self,msg):
        with self.mutex:
            if self.motion.servo_status!=msg.data: self.event('servo_status',code=msg.data)
            self.motion.servo_status=int(msg.data)

    def start_servo(self):
        if self.mode not in ('sim','real') or self.servo_started: return
        if self.start_future is not None:
            if self.start_future.done():
                self.servo_started=bool(self.start_future.result().success)
                self.start_future=None
            return
        if self.start_client.service_is_ready(): self.start_future=self.start_client.call_async(Trigger.Request())

    def trajectory(self,msg):
        if self.mode not in ('sim','real'): return
        with self.mutex:
            try:
                if not msg.points: raise ValueError('empty_servo_trajectory')
                # Humble Servo uses header.stamp=0 (execute immediately). DDS
                # source time preserves message freshness instead of rewriting it
                # at receipt. This is local-host DDS, not cross-host wall clocks.
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
                if stamp<=self.output_ros_stamp:
                    self.event('servo_output_discarded',reason='duplicate_or_reordered')
                    return  # do not renew freshness; watchdog remains independent
                age=time.time()-stamp
                point=msg.points[0]
                horizon=point.time_from_start.sec+point.time_from_start.nanosec/1e9
                self.desired=self.guard.validate(msg.joint_names,point.velocities,age,horizon)
                if point.positions:
                    p=vector(point.positions,6)
                    if np.any(p<self.guard.lower) or np.any(p>self.guard.upper): raise ValueError('servo_position_out_of_bounds')
                self.output_at=time.monotonic();self.output_ros_stamp=stamp
                self.counters['servo_generated']+=1
            except ValueError as exc:
                if np.any(self.motion.input) or np.any(self.qd) or np.any(self.motion.twist):
                    self.motion.stop(str(exc)); self.event('fault',source='execution_adapter',reason=str(exc))
                elif not self.motion.fault:self.motion.reason=str(exc)

    def tick(self):
        with self.mutex:
            now=time.monotonic(); dt=now-self.last; self.last=now; self.periods.append(dt)
            before=(self.motion.state,self.motion.reason)
            try:
                if dt>.05: self.counters['overruns']+=1
                if self.passive is not None:
                    self.q,feedback_at=self.passive.poll()
                    if math.isfinite(feedback_at): self.motion.feedback(self.model.pose(self.q),feedback_at)
                elif self.mode=='real':
                    e=self.cpv_report
                    if e.get('q') is not None and e.get('q_at') is not None:
                        self.q=vector(e['q'],6);self.motion.feedback(self.model.pose(self.q),e['q_at'])
                    if self.enable_pending and e.get('armed'):
                        self.enable_pending=False;self.motion.action('enable',now)
                    if e.get('fault') and not self.motion.fault:self.motion.stop(e['fault'],e.get('emergency_sent',False))
                    if self.motion.enabled and not e.get('armed'):self.motion.enabled=False
                else:
                    # Fake plant follows guarded joint velocities only; no fake Cartesian IK.
                    self.motion.feedback(self.model.pose(self.q),now)
                if self.mode in ('sim','real'):
                    had_home=self.motion.goal is not None
                    twist=self.motion.tick(now,dt)
                    if self.home_test_active and had_home and self.motion.goal is None:
                        from .cpv_policy import pose_error
                        error=pose_error(self.motion.pose,self.motion.home)
                        self.home_verified=not self.motion.fault and error[0]<.0005 and error[1]<math.radians(.2)
                        self.home_test_active=False
                        self.event('home_acceptance',passed=self.home_verified,position_error_m=error[0],rotation_error_rad=error[1])
                    if now-self.output_at>.1 and (np.linalg.norm(self.qd)>1e-6 or np.any(self.motion.twist)):
                        self.motion.stop('servo_output_timeout');self.event('fault',source='servo_watchdog',reason='servo_output_timeout')
                    if self.mode=='sim':
                        requested=self.desired if now-self.output_at<.1 and self.motion.enabled and not self.motion.fault else np.zeros(6)
                        if self.motion.fault:
                            # Simulated emergency stop only. Never described as hardware hold.
                            self.qd[:]=0;self.guard.previous[:]=0
                        else: self.qd=self.guard.step(requested,self.q,min(dt,.05))
                        self.q+=self.qd*min(dt,.05)
                        self.counters['sim_executed']+=1
                        if np.linalg.norm(self.qd)>1e-6:self.counters['observed_motion']+=1
                    else:
                        m=self.motion
                        origin=max(m.stamp,m.input_stamp)
                        if math.isfinite(origin) and now-origin<.25:
                            self.authority_pub.publish(String(data=json.dumps({'epoch':self.epoch,'origin':origin,
                                'input':m.input.tolist(),'input_origin':m.input_stamp if math.isfinite(m.input_stamp) else origin,'active':bool(np.any(m.input) or np.any(m.twist) or m.goal is not None),
                                'home':m.goal is not None,'fault':m.reason if m.fault else '', 'emergency':m.state=='estop'})))
                    cmd=TwistStamped();cmd.header.stamp=self.get_clock().now().to_msg();cmd.header.frame_id='base_link'
                    cmd.twist.linear.x,cmd.twist.linear.y,cmd.twist.linear.z=map(float,twist[:3])
                    cmd.twist.angular.x,cmd.twist.angular.y,cmd.twist.angular.z=map(float,twist[3:])
                    self.tw.publish(cmd)
                # Only fresh feedback is published. Repeated reads don't renew CAN age.
                if now-self.motion.feedback_stamp<.25:
                    js=JointState()
                    js.header.stamp=(self.get_clock().now()-Duration(seconds=max(0.,now-self.motion.feedback_stamp))).to_msg()
                    js.name=self.model.names;js.position=self.q.tolist()
                    # Empty means unmeasured, not an invented zero physical speed.
                    js.velocity=[] if self.mode!='sim' else self.qd.tolist()
                    self.js.publish(js)
                if now>=self.next_diag:
                    self.diag.publish(String(data=json.dumps(self.report(),allow_nan=False)))
                    self.next_diag=now+.1
            except Exception as exc:
                self.motion.stop('execution_error: '+str(exc));self.qd[:]=0
                self.event('fault',source='runtime',reason=str(exc))
            after=(self.motion.state,self.motion.reason)
            if after!=before:self.event('state_transition',previous=before[0],state=after[0],reason=after[1])

    def report(self):
        now=time.monotonic();m=self.motion
        fresh=now-m.feedback_stamp<.25
        pose=np.r_[m.pose[:3]*1000,np.rad2deg(m.pose[3:])].tolist() if fresh else None
        model_pose=pose
        if self.passive:
            pose=self.passive.sdk_pose.tolist() if self.passive.sdk_pose is not None and now-min(self.passive.pose_times)<.25 else None
        primary_stamp=min(self.passive.pose_times) if self.passive else m.feedback_stamp
        if self.mode=='real':
            primary_stamp=self.cpv_report.get('pose_at') or -math.inf
            actual=self.cpv_report.get('pose')
            pose=np.r_[np.array(actual[:3])*1000,np.rad2deg(actual[3:])].tolist() if actual is not None and now-primary_stamp<.25 else None
        status_fresh=self.passive is None or now-self.passive.status_at<.25
        margins=np.minimum(self.q-self.guard.lower,self.guard.upper-self.q)/(self.guard.upper-self.guard.lower)*200
        reason=m.reason
        if self.mode=='readonly':reason='只读反馈：固件、模型与 CPV 停止保持尚未验证，运动禁用'
        elif not self.servo_started:reason='waiting_for_moveit_servo'
        elif self.mode=='real' and not self.cpv_report.get('jog_validated'):reason='实机验证未完成：请用 commission-cpv.py 检查 FK、零速保持及逐轴方向'
        if self.mode=='real' and self.cpv_report.get('fault'):reason=self.cpv_report['fault']
        ages=self.periods
        return {'ok':True,'backend':'ros2_moveit_servo','backend_mode':self.mode,'real_motion_available':self.mode=='real' and self.cpv_report.get('jog_validated',False),
                'pose':pose,'model_pose_unverified':model_pose if self.mode!='sim' else None,'target':None,'feedback_present':pose is not None,'feedback_age_ms':round((now-primary_stamp)*1000,1) if math.isfinite(primary_stamp) else None,
                'joint_feedback_age_ms':round((now-m.feedback_stamp)*1000,1) if math.isfinite(m.feedback_stamp) else None,
                'enabled':[m.enabled]*6 if self.mode=='sim' else (self.cpv_report.get('enabled',[]) if self.mode=='real' else (self.passive.enabled if now-min(self.passive.motor_times)<.25 else [])),
                'status':self.cpv_report.get('status') if self.mode=='real' else (self.passive.status if status_fresh else None) if self.passive else (1 if m.fault else 0),
                'ctrl_mode':self.cpv_report.get('ctrl_mode') if self.mode=='real' else (self.passive.ctrl_mode if status_fresh else None) if self.passive else (1 if m.enabled else 0),
                'ready':self.mode in ('sim','real') and self.servo_started and fresh and m.enabled and not m.fault and (self.mode!='real' or self.cpv_report.get('jog_validated',False)),
                'fault_latched':m.fault,'error':reason,'blocking_code':reason,'motion_state':m.state,
                'motion_may_be_active':bool(np.any(m.twist) or np.any(self.qd) or (self.mode=='real' and any(self.cpv_report.get('target_joint_velocity_rad_s',[0]*6)))),
                'input_rearm_required':m.rearm,'moving_input':bool(np.any(m.input)),
                'returning_home':m.goal is not None,'home_pose':None if m.home is None else np.r_[m.home[:3]*1000,np.rad2deg(m.home[3:])].tolist(),
                'joint_margins':np.clip(margins,0,100).tolist() if fresh else [0]*6,'actual_joint_velocity_rad_s':self.qd.tolist() if self.mode=='sim' else None,
                'linear_speed_mm_s':self.motion.linear_speed*1000,'angular_speed_deg_s':min(10.,math.degrees(self.motion.angular_speed)),'input_deadband':.02,
                'servo_status':m.servo_status,'servo_started':self.servo_started,'input_age_ms':(now-m.stamp)*1000 if math.isfinite(m.stamp) else None,
                'servo_output_age_ms':(now-self.output_at)*1000 if math.isfinite(self.output_at) else None,
                'cycle_ms':{'max':max(ages,default=0)*1000,'p99':float(np.percentile(ages,99))*1000 if ages else 0},
                'effective_motion_config':self.settings,'counters':dict(self.counters),'model_id':self.model_id,'firmware':self.cpv_report.get('firmware','unknown'),'executor':self.cpv_report,'home_validated':self.mode=='sim' or self.home_verified,'control_point':'SDK flange / base_link -> link6','stop_behavior':'CPV ramp to zero; hardware holding requires commissioning; emergency stop may descend'}

    def make_http(self):
        owner=self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args): pass
            def reply(self,result):
                data=json.dumps(result,ensure_ascii=False,allow_nan=False).encode()
                self.send_response(200);self.send_header('Content-Type','application/json; charset=utf-8')
                self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
            def do_GET(self):
                with owner.mutex:
                    self.reply(owner.report() if self.path=='/diagnostics' else {'events':list(owner.events)})
            def do_POST(self):
                try:
                    length=int(self.headers.get('Content-Length','0'))
                    if not 0<length<=8192: raise ValueError('invalid_request_length')
                    payload=json.loads(self.rfile.read(length));now=time.monotonic()
                    # IPC waits happen outside the motion lock. Slow HTTP cannot
                    # stall the 100 Hz timer and manufacture a loop-overrun stop.
                    if owner.mode=='real':
                        if self.path=='/stop':owner.executor_action('estop' if payload.get('emergency',True) else 'stop')
                        elif self.path=='/action' and payload.get('action') in ('enable','recover','stop'):
                            action=payload['action']
                            if action=='enable' and not owner.cpv_report.get('jog_validated'):raise ValueError('commissioning_incomplete; use bounded commissioning tool')
                            owner.executor_action(action)
                    with owner.mutex:
                        if owner.mode not in ('sim','real'): raise ValueError('READ_ONLY: all control actions disabled')
                        if self.path=='/input':
                            owner.motion.accept(payload['input'],float(payload['received_at']),now)
                            owner.counters['received_input']+=1
                            owner.event('input_received',input=payload['input'],origin=payload['received_at'])
                        elif self.path=='/heartbeat':
                            stamp=float(payload['received_at'])
                            if not owner.motion.stamp<stamp<=now or now-stamp>=.25:raise ValueError('stale_heartbeat')
                            owner.motion.stamp=stamp
                        elif self.path=='/stop':
                            owner.motion.stop(str(payload['reason']),payload.get('emergency',True));owner.event('fault',source='bridge',reason=payload['reason'])
                        elif self.path=='/action':
                            action=payload['action']
                            if action in ('enable','home') and not owner.servo_started:raise ValueError('Servo not started')
                            if owner.mode=='real' and action in ('enable','recover','stop'):
                                if action=='enable' and not owner.cpv_report.get('jog_validated'):raise ValueError('commissioning_incomplete; use bounded commissioning tool')
                                if action=='enable':owner.enable_pending=True
                                else:owner.motion.action(action,now)
                            elif owner.mode=='real' and action=='home' and not owner.home_verified:raise ValueError('home_not_hardware_validated')
                            elif owner.mode=='real' and action=='home_test':
                                from .cpv_policy import pose_error
                                if owner.motion.home is None:raise ValueError('record_new_home_first')
                                error=pose_error(owner.motion.pose,owner.motion.home)
                                if error[0]>.01 or error[1]>math.radians(3):raise ValueError('home_test_start_must_be_within_10mm_3deg_of_recorded_home')
                                if not owner.cpv_report.get('jog_validated'):raise ValueError('commissioning_incomplete')
                                owner.motion.action('home',now);owner.home_test_active=True;owner.home_verified=False
                            else:owner.motion.action(action,now)
                            if action=='set_home':
                                owner.home_verified=False
                                saved={'mode':owner.mode,'model_id':owner.model_id,'pose_si':owner.motion.home.tolist()}
                                temp=owner.home_path.with_suffix('.tmp');temp.write_text(json.dumps(saved));temp.replace(owner.home_path)
                            owner.event('action',action=action)
                        else: raise ValueError('unknown_endpoint')
                    self.reply({'ok':True})
                except Exception as exc:self.reply({'ok':False,'error':str(exc)})
        server=ThreadingHTTPServer(('127.0.0.1',9088),Handler)
        server.daemon_threads=True
        return server

    def destroy_node(self):
        self.http.shutdown();self.http.server_close()
        if self.passive:self.passive.close()
        super().destroy_node()


def main():
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Set ROS_LOCALHOST_ONLY=1; ROS remains local to N100')
    # This lock guards gateways; the CPV process has a separate CAN writer lock.
    import fcntl
    lock=open('/tmp/robot-console-motion.lock','a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    rclpy.init();node=None
    try:
        node=ServoGateway();rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        lock.close()

if __name__=='__main__':main()
