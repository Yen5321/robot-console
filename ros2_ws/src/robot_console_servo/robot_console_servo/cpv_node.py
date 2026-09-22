"""Sole CAN writer process. HTTP only queues actions; the timer owns all SDK I/O.

Normal node shutdown requests an electronic stop if armed. SIGKILL/power loss
cannot be handled here and require separate controller/hardware acceptance.
"""
import json,math,os,queue,threading,time
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
from collections import deque
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from .model import Model
from .cpv_sdk import CpvSdk,SDK_COMMIT
from .cpv_policy import CpvPolicy

def clean(value):
    if isinstance(value,np.ndarray):return clean(value.tolist())
    if isinstance(value,dict):return {k:clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [clean(v) for v in value]
    if isinstance(value,float) and not math.isfinite(value):return None
    return value

class Executor(Node):
    def __init__(self):
        super().__init__('piper_l_cpv_executor')
        self.declare_parameter('robot_description','');self.declare_parameter('can_name','can0')
        self.actions=queue.Queue(8);self.events=deque(maxlen=5000);self.periods=deque(maxlen=60000)
        self.manual_stop=threading.Event();self.snapshot={'armed':False,'fault':'waiting_for_executor'}
        self.model=Model(self.get_parameter('robot_description').value)
        self.sdk=CpvSdk(self.get_parameter('can_name').value)
        self.policy=CpvPolicy(self.model,self.sdk,self.event,clock=time.monotonic)
        self.pub=self.create_publisher(String,'/console/cpv_feedback',1)
        self.create_subscription(String,'/console/cpv_authority',self.authority,1)
        self.create_subscription(JointTrajectory,'/console/servo_output_stamped',self.trajectory,1)
        self.last=time.monotonic();self.next_report=0.;self.next_statistics=0.;self.statistics={};self.last_ros_stamp=-math.inf
        self.logged_origin=-math.inf;self.observed_q=None;self.observed_motion_samples=0
        self.parameters_logged=False;self.fault_logged=False
        self.timer=self.create_timer(.01,self.tick)
        self.server=self.http();threading.Thread(target=self.server.serve_forever,daemon=True).start()
        # Disk work on a separate thread. Queue full is reported; never stalls CAN.
        self.logs=queue.Queue(20000);self.log_dropped=0;self.log_stop=threading.Event()
        self.log_thread=threading.Thread(target=self.write_logs,daemon=True);self.log_thread.start()
        self.event('executor_started',sdk_commit=SDK_COMMIT,model_id=self.model.id,automatic_enable=False)

    def event(self,name,**fields):
        item=clean({'time':time.monotonic(),'wall_time':time.time(),'event':name,**fields});self.events.append(item)
        if hasattr(self,'logs'):
            try:self.logs.put_nowait(item)
            except queue.Full:self.log_dropped+=1

    def write_logs(self):
        Path('logs').mkdir(exist_ok=True)
        handler=RotatingFileHandler('logs/cpv-v8.1.jsonl',maxBytes=20_000_000,backupCount=5,encoding='utf-8')
        try:
            while not self.log_stop.is_set() or not self.logs.empty():
                try:item=self.logs.get(timeout=.1)
                except queue.Empty:continue
                handler.emit(logging.LogRecord('cpv',logging.INFO,'',0,json.dumps(item,ensure_ascii=False),(),None))
        finally:handler.close()

    def authority(self,msg):
        try:
            data=json.loads(msg.data);self.policy.accept_authority(data,time.monotonic())
            if data['origin']>self.logged_origin:
                self.logged_origin=data['origin'];self.event('received_control',**data)
        except (ValueError,KeyError,TypeError) as exc:
            self.event('authority_rejected',reason=str(exc))
            if self.policy.armed:self.policy.stop('invalid_control_authority')

    def trajectory(self,msg):
        try:
            if not msg.points:raise ValueError('empty_trajectory')
            p=msg.points[0];now=time.monotonic()
            source_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if source_stamp<=self.last_ros_stamp:return
            age=time.time()-source_stamp
            self.policy.accept_trajectory(msg.joint_names,p.velocities,now-age,p.time_from_start.sec+p.time_from_start.nanosec/1e9,now)
            self.last_ros_stamp=source_stamp
            self.event('servo_command',source_stamp=source_stamp,age_s=age,velocity_rad_s=list(p.velocities))
            if p.positions:
                positions=np.asarray(p.positions)
                if positions.shape!=(6,) or not np.isfinite(positions).all() or np.any(positions<self.policy.guard.lower) or np.any(positions>self.policy.guard.upper):raise ValueError('trajectory_position_invalid')
        except (ValueError,TypeError) as exc:
            self.event('trajectory_rejected',reason=str(exc))
            if self.policy.armed and np.any(self.policy.guard.previous):self.policy.stop('invalid_servo_output:'+str(exc))

    def tick(self):
        now=time.monotonic();dt=now-self.last;self.last=now;self.periods.append(dt)
        try:
            if self.manual_stop.is_set():self.manual_stop.clear();self.policy.stop('manual_emergency_stop',True)
            # Update physical state before validating queued management actions.
            self.policy.feedback=self.sdk.poll()
            now=time.monotonic()
            try:request,result,done,deadline=self.actions.get_nowait()
            except queue.Empty:pass
            else:
                try:
                    if now>deadline:raise ValueError('management_request_expired')
                    self.policy.action(request['action'],now,**{k:v for k,v in request.items() if k!='action'})
                    result.update(ok=True,accepted=True)
                except Exception as exc:result.update(ok=False,error=str(exc))
                finally:done.set()
            out=self.policy.tick(now,dt)
            if not self.policy.armed and not self.policy.enabling:self.sdk.read_parameter_step()
        except Exception as exc:
            self.event('sdk_or_runtime_error',reason=str(exc))
            try:self.policy.stop('SDK_send_or_runtime_error:'+str(exc),self.policy.armed or self.policy.enabling)
            except Exception as stop_exc:self.event('electronic_stop_send_failed',reason=str(stop_exc))
            out=np.zeros(6)
        now=time.monotonic();p=self.policy;f=p.feedback or {}
        if p.fault and not self.fault_logged:
            self.fault_logged=True
            self.event('first_fault_evidence',reason=p.fault,feedback=f,
                       recent_can_tx=list(getattr(self.sdk,'tx_recent',[])),
                       zero_hold_drift=p.last_drift,parameters=self.sdk.parameters)
        if not p.fault:self.fault_logged=False
        if getattr(self.sdk,'query_index',0)>=48 and not self.parameters_logged:
            self.parameters_logged=True
            self.event('cpv_parameter_snapshot',complete=self.sdk.parameter_snapshot_ready(),parameters=self.sdk.parameters)
        if p.fresh(now) and (self.observed_q is None or max(abs(f['q']-self.observed_q))>.0001):
            if self.observed_q is not None:self.observed_motion_samples+=1;self.event('observed_joint_motion',q=f['q'],physical_stamp=f['q_at'])
            self.observed_q=f['q'].copy()
        if now>=self.next_statistics:
            self.statistics={'max':max(self.periods)*1000,'p99':float(np.percentile(self.periods,99))*1000};self.next_statistics=now+.5
        # Ages and original stamps both published; subscriber cannot refresh them.
        self.snapshot=clean({**f,'executor_build':'8.1.1-cpv-diagnostics','armed':p.armed,'enabling':p.enabling,'fault':p.fault,'emergency_sent':p.emergency_sent,'phase':p.phase,
            'target_joint_velocity_rad_s':out,'actual_joint_velocity_rad_s':None,
            'actual_velocity_note':'motor feedback is not verified joint velocity',
            'firmware':self.sdk.firmware,'model_id':self.model.id,'sdk_commit':SDK_COMMIT,
            'fk_samples':len(p.fk_samples),'hold_passed':p.hold_passed,'directions_verified':sorted(p.directions),
            'jog_validated':p.ready_to_jog(),'home_validated':False,'last_result':p.last_result,'zero_hold_drift':p.last_drift,
            'physical_feedback_fresh':p.fresh(now),'servo_age_ms':(now-p.servo_at)*1000,'control_age_ms':(now-p.authority_at)*1000,
            'feedback_ages_ms':{key:(now-f.get(key,-math.inf))*1000 for key in ('q_at','pose_at','enabled_at','status_at')},
            'stamp':now,'cycle_ms':self.statistics,
            'sdk_send_count':self.sdk.sent,'sdk_velocity_call_count':getattr(self.sdk,'velocity_calls',None),
            'observed_joint_motion_samples':self.observed_motion_samples,'cpv_parameters_read_only':self.sdk.parameters,
            'cpv_parameter_snapshot_complete':self.sdk.parameter_snapshot_ready(),'log_dropped':self.log_dropped})
        self.pub.publish(String(data=json.dumps(self.snapshot)))
        if now>=self.next_report:
            # Full parameter snapshot is an event once per query cycle, not 20
            # copies/second; preserve the diagnostic HTTP payload unchanged.
            self.event('executor_sample',**{k:v for k,v in self.snapshot.items() if k!='cpv_parameters_read_only'})
            self.next_report=now+.05

    def http(self):
        owner=self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def reply(self,data):
                raw=json.dumps(clean(data),ensure_ascii=False,allow_nan=False).encode();self.send_response(200)
                self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
            def do_GET(self):self.reply({'events':list(owner.events)} if self.path=='/events' else owner.snapshot)
            def do_POST(self):
                try:
                    count=int(self.headers.get('Content-Length',0))
                    if self.path!='/action' or not 0<count<=4096:raise ValueError('invalid_action_request')
                    request=json.loads(self.rfile.read(count))
                    if request.get('action')=='estop':owner.manual_stop.set();self.reply({'ok':True});return
                    result={};done=threading.Event();owner.actions.put_nowait((request,result,done,time.monotonic()+.15))
                    if not done.wait(.2):raise ValueError('executor_action_timeout; inspect diagnostics before retrying')
                    self.reply(result)
                except Exception as exc:self.reply({'ok':False,'error':str(exc)})
        server=ThreadingHTTPServer(('127.0.0.1',9091),Handler);server.daemon_threads=True;return server

    def destroy_node(self):
        if self.policy.armed or self.policy.enabling:
            try:self.policy.stop('executor_process_exit',True)
            except Exception as exc:self.event('exit_stop_failed',reason=str(exc))
        self.sdk.close();self.server.shutdown();self.server.server_close()
        self.log_stop.set();self.log_thread.join(1.)
        super().destroy_node()

def main():
    import fcntl
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise RuntimeError('ROS must stay local')
    # Advisory lock prevents our own duplicate writers; audit other CAN processes
    # before starting. Linux flock cannot lock out unrelated vendor tools.
    lock=open('/tmp/robot-console-can-writer.lock','a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    rclpy.init();node=None
    try:node=Executor();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        lock.close()
