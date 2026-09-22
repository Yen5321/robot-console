"""Run against the sim ROS launch only; never connects to CAN or remote robot."""
import json,time,urllib.request
import numpy as np

BASE='http://127.0.0.1:9088'
def request(path,payload=None):
    data=None if payload is None else json.dumps(payload).encode()
    with urllib.request.urlopen(urllib.request.Request(BASE+path,data=data,headers={'Content-Type':'application/json'}),timeout=2) as r:result=json.load(r)
    if not result.get('ok',True):raise AssertionError(result)
    return result
def send(u):request('/input',{'input':u,'received_at':time.monotonic()})
def action(a):return request('/action',{'action':a})
def stream(u,duration):
    until=time.monotonic()+duration
    while time.monotonic()<until:
        send(u);time.sleep(.04)

report=request('/diagnostics')
assert report['backend_mode']=='sim' and not report['real_motion_available']
assert report['servo_started'],report
send([0]*5)
if report['fault_latched']:action('recover')
action('enable');stream([0]*5,.15)
action('set_home')
before=request('/diagnostics')
stream([0,.04,0,0,0],1.2)  # 2 mm/s Y intent
moving=request('/diagnostics')
stream([0]*5,.5)
stopped=request('/diagnostics')
assert not stopped['fault_latched'],stopped
assert np.linalg.norm(stopped['actual_joint_velocity_rad_s'])<.002,stopped
distance=np.linalg.norm(np.array(moving['pose'][:3])-before['pose'][:3])
assert distance>.2,('No observed simulated movement',moving)
assert moving['counters']['servo_generated']>before['counters']['servo_generated']
# With fresh pose but no new intent: independent motion lease expires.
time.sleep(.35)
fault=request('/diagnostics');assert fault['fault_latched'],fault
stream([0]*5,.1);assert request('/diagnostics')['fault_latched']
action('recover');assert not request('/diagnostics')['ready']
print(json.dumps({'result':'PASS','simulated_displacement_mm':distance,'stopped_joint_velocity':stopped['actual_joint_velocity_rad_s'],'watchdog_reason':fault['error'],'cycle_ms':stopped['cycle_ms']},indent=2))
