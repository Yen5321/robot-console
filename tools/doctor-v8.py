#!/usr/bin/env python3
"""One-pass, read-only Ubuntu v8 audit. No nodes, CAN TX or installs are started.

Only standard library is required. APT repair is generated, never executed.
"""
import argparse
import datetime
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import zipfile

ROOT=Path(__file__).resolve().parents[1]
REQUIRED=['ros-humble-moveit-servo','ros-humble-moveit-core','ros-humble-moveit-ros-planning',
 'ros-humble-moveit-ros-occupancy-map-monitor','ros-humble-moveit-ros-planning-interface',
 'ros-humble-launch','ros-humble-launch-ros','ros-humble-launch-xml','ros-humble-launch-yaml',
 'ros-humble-ros2launch','ros-humble-rclcpp','ros-humble-rclpy','ros-humble-robot-state-publisher']
PREFIXES=('ros-humble-chomp-motion-planner','ros-humble-moveit-','ros-humble-launch','ros-humble-rcl','ros-humble-rmw',
 'ros-humble-rosidl','ros-humble-tf2','ros-humble-pluginlib','ros-humble-class-loader')

def run(args,timeout=25):
    try:
        r=subprocess.run(args,cwd=ROOT,env={**os.environ,'LC_ALL':'C'},capture_output=True,text=True,errors='replace',timeout=timeout)
        return {'exit':r.returncode,'output':r.stdout+r.stderr}
    except (OSError,subprocess.TimeoutExpired) as e:return {'exit':-1,'output':str(e)}

def parse_candidates(text):
    result={};name=None
    for line in text.splitlines():
        if line and not line[0].isspace() and line.endswith(':'):name=line[:-1].split(':')[0]
        match=re.match(r'\s+Candidate:\s+(\S+)',line)
        if match and name and match[1]!='(none)':result[name]=match[1]
    return result

def clean(text):
    return re.sub(r'(https?://)[^\s/@]+:[^\s/@]+@',r'\1[redacted]@',text)

def main():
    p=argparse.ArgumentParser();p.add_argument('--can-name',default='can0');a=p.parse_args()
    if sys.platform!='linux':raise SystemExit('Run on Ubuntu N100, not Windows.')
    if not re.fullmatch(r'[a-zA-Z0-9_-]+',a.can_name):raise SystemExit('Invalid CAN name')
    folder=ROOT/'diagnostics'/('environment-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+str(os.getpid()))
    folder.mkdir(parents=True)
    checks={};issues=[]
    def check(name,args,timeout=25,optional=False):
        result=run(args,timeout);checks[name]=result
        (folder/(name+'.txt')).write_text(clean(result['output']),encoding='utf-8')
        good=result['exit']==0
        print(('OK   ' if good else 'CHECK ')+name,flush=True)
        if not good:issues.append(name+(' (optional/readonly)' if optional else ''))
        return result
    # Inspect the user's current environment separately from a clean system shell.
    check('system',['bash','-c','cat /etc/os-release; uname -m; df -h . /tmp; free -m; command -v python3; python3 --version'])
    (folder/'python-environment.json').write_text(json.dumps({k:os.environ.get(k) for k in
        ['VIRTUAL_ENV','CONDA_PREFIX','ROS_DISTRO','PYTHONPATH','AMENT_PREFIX_PATH','LD_LIBRARY_PATH']},indent=2))
    dpkg=check('dpkg-audit',['dpkg','--audit'])
    if dpkg['exit']==0 and dpkg['output'].strip():issues.append('dpkg reports incomplete package state; see dpkg-audit.txt')
    holds=check('held-packages',['apt-mark','showhold'])
    if 'ros-humble-' in holds['output']:issues.append('Held ROS packages may prevent coherent upgrades; see held-packages.txt')
    installed=check('installed-ros',['dpkg-query','-W','-f=${binary:Package}\t${Version}\t${db:Status-Status}\n','ros-humble-*'])
    names=set(REQUIRED)
    for line in installed['output'].splitlines():
        fields=line.split('\t')
        if len(fields)==3 and fields[2]=='installed' and fields[0].startswith(PREFIXES):names.add(fields[0].split(':')[0])
    policy=check('apt-candidates',['apt-cache','policy',*sorted(names)],60)
    candidates=parse_candidates(policy['output'])
    (folder/'candidate-lock.json').write_text(json.dumps({'status':'UNVERIFIED mirror candidates; not a tested lock',
        'ros_distribution':'humble','requested_debian_packages':candidates},indent=2))
    absent=sorted(names-set(candidates))
    if absent:issues.append('APT candidates missing: '+', '.join(absent))
    specs=[k+'='+v for k,v in sorted(candidates.items())]
    apt=check('apt-repair-preview',['apt-get','--simulate','--no-remove','install',*specs],90) if specs else {'exit':1}
    if specs and not absent and apt['exit']==0:
        # Exact versions frozen at audit time, never auto-run or auto-confirm.
        command=shlex.join(['sudo','apt-get','--no-remove','install',*specs])
        (folder/'repair-ros.sh').write_text('#!/usr/bin/env bash\nset -euo pipefail\n# REVIEW apt-repair-preview.txt first. Not hardware validated.\n'+command+'\n',encoding='utf-8')
    setup=Path('/opt/ros/humble/setup.bash')
    overlay=ROOT/'ros2_ws/install/setup.bash'
    def ros(code,extra_path='',timeout=25):
        prefix='unset PYTHONPATH PYTHONHOME AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH; source /opt/ros/humble/setup.bash; '
        if overlay.exists():prefix+='source '+shlex.quote(str(overlay))+'; '
        if extra_path:prefix+='export PYTHONPATH='+shlex.quote(extra_path)+'${PYTHONPATH:+:$PYTHONPATH}; '
        return ['bash','-c',prefix+'export ROS_LOCALHOST_ONLY=1; '+code]
    if setup.exists():
        for name,code in {
            'import-launch':'from launch.substitutions import LaunchConfiguration; from launch_ros.actions import Node; print("launch OK")',
            'import-rclpy':'import rclpy; from rclpy.node import Node; from trajectory_msgs.msg import JointTrajectory; print("rclpy/messages OK")',
            'import-math':'import numpy, scipy, yaml; from scipy.spatial.transform import Rotation; print(numpy.__version__,scipy.__version__,yaml.__version__)',
            'import-project':'import robot_console_servo.node; print(robot_console_servo.node.__file__)',
            'import-sdk':'import can; from piper_sdk.protocol.protocol_v2 import C_PiperParserV2; from piper_sdk.piper_msgs.msg_v2 import PiperMessage; print("receive decoder OK")'
        }.items():
            check(name,ros('/usr/bin/python3 -c '+shlex.quote(code),str(ROOT/'.deps-readonly') if name=='import-sdk' else ''))
        launch=ROOT/'ros2_ws/src/robot_console_servo/launch/console.launch.py'
        code='import runpy; d=runpy.run_path('+repr(str(launch))+'); d["generate_launch_description"](); print("launch description loaded; no nodes started")'
        check('launch-description',ros('/usr/bin/python3 -c '+shlex.quote(code)))
        config=ROOT/'ros2_ws/src/robot_console_servo/config/motion.yaml'
        code='from robot_console_servo.settings import load; print(load('+repr(str(config))+'))'
        check('motion-config',ros('/usr/bin/python3 -c '+shlex.quote(code)))
        # System-owned ELF files only. Check all installed MoveIt libraries as well
        # as the three entrypoints, not merely the first missing dependency.
        paths=[Path('/opt/ros/humble/lib/moveit_servo/servo_node_main'),Path('/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher'),
            ROOT/'ros2_ws/install/console_servo_stamp/lib/console_servo_stamp/stamp_servo']
        paths+=sorted(Path('/opt/ros/humble/lib').glob('libmoveit*.so*'))
        seen=set();missing=[];outputs=[]
        for path in paths:
            target=path.resolve()
            if target in seen:continue
            seen.add(target)
            if not path.is_file():missing.append(str(path)+' missing');continue
            result=run(ros('ldd '+shlex.quote(str(path))),10)
            outputs.append(str(path)+'\n'+result['output'])
            if result['exit'] or 'not found' in result['output']:missing.append(str(path))
        (folder/'dynamic-libraries.txt').write_text(clean('\n\n'.join(outputs)),encoding='utf-8')
        if missing:issues.append('Missing/broken ELF dependencies: '+', '.join(missing))
        print(('CHECK' if missing else 'OK   ')+' dynamic-libraries ('+str(len(seen))+' files)',flush=True)
        for config in ['bridge.sim.yaml','bridge.readonly.local.yaml']:
            path=ROOT/'config'/config
            if path.exists():check('config-'+config,ros('/usr/bin/python3 -m robot_bridge --check-config --config '+shlex.quote(str(path)),str(ROOT/'bridge')))
        for module,file in [('URDF','piper/urdf/piper_description.urdf'),('SRDF','official_piper.srdf')]:
            path=ROOT/'ros2_ws/src/robot_console_servo/model'/file
            code='import xml.etree.ElementTree as E; print(E.parse('+repr(str(path))+').getroot().tag)'
            check('model-'+module,['/usr/bin/python3','-c',code])
    else:issues.append('ROS Humble setup missing')
    can=check('can-status',['ip','-details','-statistics','link','show',a.can_name],optional=True)
    if can['exit']==0 and ('state DOWN' in can['output'] or 'can state STOPPED' in can['output']):
        issues.append('CAN is DOWN/STOPPED (readonly hardware blocker; simulation does not need CAN)')
    check('listening-ports',['ss','-lntup'],optional=True)
    # Import camera SDK without creating a pipeline or touching USB hardware.
    legacy=Path('/home/tieta/robot-bridge-track-b/.venv/bin/python')
    if legacy.exists():check('camera-import',[str(legacy),'-c','import cv2, pyrealsense2; print("camera imports OK; no device opened")'],optional=True)
    else:issues.append('Legacy camera interpreter not found (optional)')
    summary='V8 ENVIRONMENT AUDIT — no hardware commands sent\n\n'+('\n'.join('- '+i for i in issues) if issues else 'No failures detected in these static checks.')
    summary+='\n\nRead apt-repair-preview.txt before running any generated repair-ros.sh.\nPassing this audit does not validate runtime timing, CAN feedback or real motion.\n'
    (folder/'SUMMARY.txt').write_text(summary,encoding='utf-8')
    archive=ROOT/'diagnostics/environment-audit-latest.zip'
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
        for f in folder.iterdir():z.write(f,f.name)
    print('\n'+summary+'\nREPORT: '+str(archive)+'\nDETAILS: '+str(folder))

if __name__=='__main__':main()
