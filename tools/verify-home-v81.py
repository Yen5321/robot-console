"""Attended short real home validation. Bridge must be stopped; ROS stays running.

Only a previously recorded, model-bound zero within 10 mm / 3 degrees is allowed.
The gateway uses the same Servo home algorithm, independent 4 mm/s, 2 deg/s caps.
"""
import json,time,urllib.request
def request(path,payload=None):
    req=urllib.request.Request('http://127.0.0.1:9088'+path,data=json.dumps(payload).encode() if payload is not None else None,headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=1) as response:result=json.load(response)
    if not result.get('ok',True):raise RuntimeError(result.get('error',result))
    return result
def main():
    d=request('/diagnostics')
    if d['backend_mode']!='real' or not d.get('real_motion_available'):raise SystemExit('Requires completed PIPER-L commissioning')
    print('停止桥接和 Windows 输入，ROS 保持运行。必须已记录新零位，当前位置距零位 ≤10 mm / 3°。')
    print('本次会显式恢复软件故障、使能并执行一次低速回零；不发送 SDK reset。')
    if input('现场准备好后输入 RUN：').strip()!='RUN':return
    try:
        request('/heartbeat',{'received_at':time.monotonic()})
        request('/action',{'action':'recover'});request('/action',{'action':'enable'})
        until=time.monotonic()+4
        while not request('/diagnostics')['ready']:
            request('/input',{'received_at':time.monotonic(),'input':[0]*5})
            if time.monotonic()>until:raise RuntimeError('enable not confirmed')
            time.sleep(.04)
        request('/action',{'action':'home_test'})
        until=time.monotonic()+10
        while True:
            request('/heartbeat',{'received_at':time.monotonic()});d=request('/diagnostics')
            if d.get('fault_latched'):raise RuntimeError(d['error'])
            if d.get('home_validated'):print('回零误差检查通过；日志记录在 ROS diagnostics/events。重新连接后需显式使能。');break
            if time.monotonic()>until:raise RuntimeError('home test timeout')
            time.sleep(.04)
    finally:request('/action',{'action':'stop'})
if __name__=='__main__':main()
