"""Fault injection for an explicitly selected LOCAL SIM Servo PID; no CAN use.

Stop the UDP bridge and Windows client before running. Always resumes the child
in finally; never selects processes by name or sends signals to a real backend.
"""
import argparse
import json
import os
import signal
import socket
import time
import urllib.request
from pathlib import Path

def call(path,data=None):
    req=urllib.request.Request('http://127.0.0.1:9088'+path,
        data=None if data is None else json.dumps(data).encode(),headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=2) as response:result=json.load(response)
    assert result.get('ok',True),result
    return result

def stream(values,seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        call('/input',{'input':values,'received_at':time.monotonic()});time.sleep(.04)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--servo-pid',required=True,type=int);args=parser.parse_args()
    with socket.socket() as probe:
        assert probe.connect_ex(('127.0.0.1',8080))!=0,'Stop bridge and await its exit first'
    report=call('/diagnostics')
    assert report['backend_mode']=='sim' and not report['real_motion_available'],report
    cmdline=Path(f'/proc/{args.servo_pid}/cmdline').read_bytes().split(b'\0')
    assert cmdline[0].endswith(b'/moveit_servo/servo_node_main'),cmdline
    stream([0]*5,.1)
    if report['fault_latched']:call('/action',{'action':'recover'})
    call('/action',{'action':'enable'});stream([0]*5,.1)
    stream([0,.04,0,0,0],.5)
    try:
        os.kill(args.servo_pid,signal.SIGSTOP)
        # Keep the source intent alive so this isolates the Servo output watchdog.
        end=time.monotonic()+.3
        while time.monotonic()<end:
            report=call('/diagnostics')
            if report['fault_latched']:break
            call('/input',{'input':[0,.04,0,0,0],'received_at':time.monotonic()});time.sleep(.02)
        assert report['fault_latched'] and report['error']=='servo_output_timeout',report
        assert all(v==0 for v in report['actual_joint_velocity_rad_s']),report
    finally:
        os.kill(args.servo_pid,signal.SIGCONT)
    stream([0]*5,.2)
    assert call('/diagnostics')['fault_latched'],'Resume must not clear fault'
    print(json.dumps({'result':'PASS','reason':report['error'],'resume_requires_explicit_recovery':True}))

if __name__=='__main__':main()
