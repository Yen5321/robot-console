#!/usr/bin/env python3
"""Bounded CPV readback; never enables, switches modes or writes parameters."""
import argparse,datetime,json,math,sys,time
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'.deps-readonly'),str(ROOT/'vendor/pyAgxArm'),str(ROOT/'ros2_ws/src/robot_console_servo')]

def main():
    import fcntl
    from robot_console_servo.cpv_sdk import CpvSdk
    p=argparse.ArgumentParser();p.add_argument('--can-name',default='can0');args=p.parse_args()
    folder=ROOT/'diagnostics'/('cpv-readonly-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
    folder.mkdir(parents=True)
    def clean(value):
        if hasattr(value,'tolist'):return clean(value.tolist())
        if isinstance(value,dict):return {k:clean(v) for k,v in value.items()}
        if isinstance(value,list):return [clean(v) for v in value]
        if isinstance(value,float) and not math.isfinite(value):return None
        return value
    with open('/tmp/robot-console-can-writer.lock','a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        sdk=CpvSdk(args.can_name);frames=[];samples=[];error=None
        send=sdk.comm.send
        def read_only_send(msg,timeout=None):
            data=bytes(msg.data)
            if not (0x181<=msg.arbitration_id<=0x186 and len(data)==7 and data[:1]==b'r'):
                raise RuntimeError('Readonly probe blocked non-query CAN frame')
            send(msg,timeout);frames.append({'time':time.monotonic(),'id':hex(msg.arbitration_id),'data':data.hex(),'dlc':msg.dlc})
        sdk.comm.send=read_only_send
        try:
            deadline=time.monotonic()+30
            while time.monotonic()<deadline and sdk.query_index<48:
                samples.append(clean(sdk.poll()));sdk.read_parameter_step();time.sleep(.01)
        except Exception as exc:error=str(exc)
        finally:
            report={'kind':'read-only queries; no enable/mode/motion/Flash writes','error':error,
                    'firmware':sdk.firmware,'node_type':sdk.node_type,'parameters':sdk.parameters,
                    'parameter_received_at':sdk.parameter_received_at,'complete':sdk.parameter_snapshot_ready(),
                    'tx':frames,'samples':samples}
            sdk.close()
            path=folder/'report.json';path.write_text(json.dumps(report,indent=2),encoding='utf-8')
            print(json.dumps({k:report[k] for k in ('kind','error','firmware','node_type','parameters','complete')},indent=2))
            print('REPORT:',path)
        if error or not report['complete']:raise SystemExit(2)

if __name__=='__main__':main()
