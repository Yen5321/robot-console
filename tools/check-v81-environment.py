#!/usr/bin/env python3
"""Fail-fast dependency check; no apt upgrades and no CAN access."""
import json,hashlib,subprocess,sys,datetime
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def moveit_core_family(name):
    # moveit_msgs and moveit_resources are separate upstream projects with their
    # own versions. Comparing them to MoveIt2 would reject a healthy install.
    return name.startswith(('ros-humble-moveit-ros-','ros-humble-moveit-planners-','ros-humble-moveit-setup-')) or name in {
        'ros-humble-chomp-motion-planner','ros-humble-moveit-core','ros-humble-moveit-servo','ros-humble-moveit-kinematics',
        'ros-humble-moveit-common','ros-humble-moveit-configs-utils','ros-humble-moveit-simple-controller-manager'}

def main():
    if sys.platform!='linux':raise SystemExit('Run on Ubuntu 22.04 after sourcing /opt/ros/humble/setup.bash')
    folder=ROOT/'diagnostics'/('v81-environment-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'));folder.mkdir(parents=True)
    result=subprocess.run(['dpkg-query','-W','-f=${binary:Package}\t${Version}\t${db:Status-Status}\n','ros-humble-*'],text=True,capture_output=True)
    (folder/'installed-ros.tsv').write_text(result.stdout)
    issues=[];versions={}
    for line in result.stdout.splitlines():
        fields=line.split('\t')
        if len(fields)==3 and fields[2]=='installed' and moveit_core_family(fields[0]):versions[fields[0]]=fields[1].split('-')[0]
    if len(set(versions.values()))!=1:issues.append('MoveIt package upstream versions are missing/mixed: '+str(versions))
    for module in ('rclpy','launch_ros','numpy','scipy','yaml','can'):
        try:__import__(module)
        except Exception as exc:issues.append(module+': '+str(exc))
    executable=Path('/opt/ros/humble/lib/moveit_servo/servo_node_main')
    libraries=[executable,*Path('/opt/ros/humble/lib').glob('libmoveit*.so')]
    library_report=[]
    for library in libraries:
        r=subprocess.run(['ldd',str(library)],text=True,capture_output=True);text=r.stdout+r.stderr
        library_report.append(str(library)+'\n'+text)
        if r.returncode or 'not found' in text:issues.append('dynamic_library_error:'+str(library))
    (folder/'dynamic-libraries.txt').write_text('\n'.join(library_report))
    lock=json.loads((ROOT/'config/dependencies.v8.1.lock.json').read_text())
    try:
        import pyAgxArm
        if not Path(pyAgxArm.__file__).resolve().is_relative_to((ROOT/'vendor/pyAgxArm').resolve()):issues.append('Another installed pyAgxArm shadows the pinned vendor source')
    except Exception as exc:issues.append('pinned SDK import: '+str(exc))
    for name,digest in lock['sdk_source_sha256'].items():
        p=ROOT/'vendor/pyAgxArm'/name
        if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=digest:issues.append('pinned SDK source mismatch: '+name)
    (folder/'summary.json').write_text(json.dumps({'issues':issues,'moveit_versions':versions,'hardware_verified':False},indent=2))
    print('Installed versions and library checks saved:',folder)
    if issues:
        print('\n'.join(issues));raise SystemExit('STOP: use tools/doctor-v8.py to generate a coherent repair plan; no partial upgrade performed')
    print('PASS: software dependency checks; physical motion is still gated')
if __name__=='__main__':main()
