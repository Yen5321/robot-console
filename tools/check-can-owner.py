#!/usr/bin/env python3
"""Reject known competing project CAN writers. Does not kill any process."""
import os,sys
from pathlib import Path
import yaml

def bridge_config(args,cwd):
    for i,arg in enumerate(args):
        if arg=='--config' and i+1<len(args):return cwd/args[i+1]
        if arg.startswith('--config='):return cwd/arg.split('=',1)[1]
    return cwd/'config.yaml'

def conflicts(proc=Path('/proc')):
    found=[]
    for item in proc.iterdir():
        if not item.name.isdigit() or int(item.name)==os.getpid():continue
        args=[]
        try:
            args=[s.decode(errors='replace') for s in (item/'cmdline').read_bytes().split(b'\0') if s]
            if any(Path(a).name=='cpv_executor' for a in args):
                found.append((int(item.name),'CPV executor already running'));continue
            if '-m' not in args or 'robot_bridge' not in args:continue
            cwd=(item/'cwd').resolve()
            config=yaml.safe_load(bridge_config(args,cwd).read_text())
            if config.get('arm',{}).get('driver')=='piper6':
                found.append((int(item.name),'legacy piper6 bridge: '+str(cwd)))
        except FileNotFoundError:
            if item.exists() and 'robot_bridge' in args:found.append((int(item.name),'bridge config missing; cannot verify CAN ownership'))
        except (OSError,ValueError,yaml.YAMLError,AttributeError) as exc:
            if 'robot_bridge' in args:found.append((int(item.name),'cannot verify bridge config: '+str(exc)))
    return found

if __name__=='__main__':
    found=conflicts()
    if found:
        for pid,reason in found:print(f'STOP PID={pid}: {reason}',file=sys.stderr)
        raise SystemExit('先停止现有 CAN 控制程序，再启动实机执行器。不会自动终止进程。')
    print('PASS: no other known project CAN writer; unrelated vendor tools still require operator check')
