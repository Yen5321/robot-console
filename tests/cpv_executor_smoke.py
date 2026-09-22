"""Run the production executor process against an in-memory fake SDK only.

Uses port 9091; never launches real mode or opens a CAN bus. Tests real timer,
HTTP management, shutdown stop and asynchronous evidence logging together.
"""
import json,os,signal,subprocess,sys,time,urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/robot_console_servo'))

def child():
    import numpy as np,rclpy
    from robot_console_servo import cpv_node
    from robot_console_servo.model import Model
    from robot_console_servo.cpv_sdk import FIRMWARE
    from unittest.mock import patch
    xml=(ROOT/'ros2_ws/src/robot_console_servo/model/piper_l/piper_l_stock_gripper.urdf').read_text()
    class FakeSDK:
        def __init__(self,channel):
            self.q=np.array([.5,1.3,-1.,.5,.8,.4]);self.model=Model(xml);self.active=False;self.status=0;self.sent=0;self.parameters={};self.firmware=FIRMWARE;self.v=np.zeros(6);self.last=time.monotonic()
        def poll(self):
            now=time.monotonic();self.q+=self.v*min(now-self.last,.02);self.last=now
            return dict(q=self.q.copy(),pose=self.model.pose(self.q),q_at=now,pose_at=now,enabled_at=now,status_at=now,enabled=[self.active]*6,status=self.status,ctrl_mode=1 if self.active else 0,cpv_mode=5 if self.active else 1,driver_fault=False,firmware=FIRMWARE,node_type='Piper_L_MC',motor_velocity_rad_s=[None]*6)
        def enable(self):self.active=True;self.sent+=1
        def velocity(self,v):self.v=np.array(v);self.sent+=6
        def emergency(self):self.v[:]=0;self.status=1;self.sent+=1
        def close(self):pass
        def read_parameter_step(self):pass
        def parameter_snapshot_ready(self):return True
    rclpy.init(args=['--ros-args','-p','robot_description:='+xml])
    node=None
    try:
        with patch.object(cpv_node,'CpvSdk',FakeSDK):node=cpv_node.Executor()
        rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

def request(body=None):
    req=urllib.request.Request('http://127.0.0.1:9091'+('/action' if body else ''),data=json.dumps(body).encode() if body else None,headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=1) as r:result=json.load(r)
    assert result.get('ok',True),result
    return result

def main():
    env={**os.environ,'ROS_LOCALHOST_ONLY':'1','ROS_DOMAIN_ID':'81'}
    process=subprocess.Popen([sys.executable,__file__,'--fake-child'],env=env)
    try:
        deadline=time.monotonic()+10
        while True:
            try:d=request();break
            except OSError:
                if time.monotonic()>deadline:raise
                time.sleep(.1)
        assert not d['armed'] and d['sdk_send_count']==0,d
        request({'action':'sample_fk'});request({'action':'enable'});time.sleep(.1)
        assert request()['armed']
        request({'action':'hold_test'});time.sleep(5.3)
        assert request()['hold_passed'],request()
        request({'action':'pulse','joint':1,'sign':1});time.sleep(.9)
        d=request();assert d['last_result']['passed'],d
        request({'action':'estop'});time.sleep(.05);d=request();assert d['fault']=='manual_emergency_stop' and d['emergency_sent'],d
        print(json.dumps({'result':'PASS','kind':'fake SDK process (no hardware)','cycle_ms':d['cycle_ms']}))
    finally:
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.kill();process.wait()
if __name__=='__main__':child() if '--fake-child' in sys.argv else main()
