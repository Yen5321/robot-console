"""Bounded, read-only debug bundle. Tokens/addresses are redacted before export."""
import argparse,json,os,re,signal,subprocess,tempfile,time,urllib.request,zipfile
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
def clean(value):
    if isinstance(value,dict):return {k:('[redacted]' if any(x in k.lower() for x in ('token','password','secret','client')) else clean(v)) for k,v in value.items()}
    if isinstance(value,list):return [clean(v) for v in value]
    if isinstance(value,str):return re.sub(r'\b(?:\d{1,3}\.){3}\d{1,3}\b','[IP]',value)
    return value
def main():
    p=argparse.ArgumentParser();p.add_argument('--http-port',type=int,default=8080);p.add_argument('--seconds',type=int,default=10);p.add_argument('--no-bag',action='store_true');args=p.parse_args()
    if not 1<=args.seconds<=30 or not 1<=args.http_port<=65535:raise ValueError('seconds=1..30, valid HTTP port required')
    out=ROOT/'diagnostics';out.mkdir(exist_ok=True)
    target=out/f'v8.1-{time.strftime("%Y%m%d-%H%M%S")}.zip'
    with tempfile.TemporaryDirectory() as temp:
        folder=Path(temp);events=[]
        for name,url in [('bridge',f'http://127.0.0.1:{args.http_port}/diagnostics'),('configuration',f'http://127.0.0.1:{args.http_port}/configuration'),('bridge-events',f'http://127.0.0.1:{args.http_port}/logs/recent'),('ros','http://127.0.0.1:9088/diagnostics'),('ros-events','http://127.0.0.1:9088/events'),('cpv','http://127.0.0.1:9091'),('cpv-events','http://127.0.0.1:9091/events')]:
            try:
                with urllib.request.urlopen(url,timeout=2) as r:data=json.load(r)
                events.extend(data.get('events',data.get('records',[])))
            except Exception as e:data={'error':str(e)}
            (folder/(name+'.json')).write_text(json.dumps(clean(data),ensure_ascii=False,indent=2),encoding='utf-8')
        events.sort(key=lambda x:x.get('time',x.get('monotonic_time',0)))
        (folder/'timeline.json').write_text(json.dumps(clean(events),ensure_ascii=False,indent=2),encoding='utf-8')
        for name,command in [('environment',['python3',str(ROOT/'tools/preflight.py')]),('nodes',['ros2','node','list']),('topics',['ros2','topic','list','-t'])]:
            try:r=subprocess.run(command,capture_output=True,text=True,timeout=15);content=r.stdout+r.stderr
            except Exception as e:content=str(e)
            (folder/(name+'.txt')).write_text(clean(content),encoding='utf-8')
        if not args.no_bag:
            # Explicit topic allowlist: no session tokens, video or arbitrary DDS data.
            topics=['/joint_states','/servo_node/delta_twist_cmds','/console/servo_output_stamped','/servo_node/status','/console/cpv_feedback']
            try:
                with (folder/'rosbag.log').open('w') as log:
                    proc=subprocess.Popen(['ros2','bag','record','-o',str(folder/'bag'),*topics],stdout=log,stderr=log,start_new_session=True,env={**os.environ,'ROS_LOCALHOST_ONLY':'1'})
                    try:proc.wait(timeout=args.seconds)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid,signal.SIGINT)
                        try:proc.wait(timeout=5)
                        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
            except Exception as e:(folder/'bag-error.txt').write_text(str(e))
        (folder/'dependencies.lock.json').write_bytes((ROOT/'config/dependencies.v8.1.lock.json').read_bytes())
        with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED) as z:
            for path in folder.rglob('*'):
                if path.is_file():z.write(path,path.relative_to(folder))
    print(target)
if __name__=='__main__':main()
