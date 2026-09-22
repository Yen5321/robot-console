"""Read-only environment inventory. Does not open CAN or enable any motor."""
import argparse,json,platform,subprocess,sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
def run(args):
    try:
        p=subprocess.run(args,capture_output=True,text=True,timeout=8)
        return {'exit':p.returncode,'output':(p.stdout+p.stderr)[-12000:]}
    except (OSError,subprocess.TimeoutExpired) as e:return {'error':str(e)}
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--can-name',default='can0');args=parser.parse_args()
    if not args.can_name.replace('_','').isalnum():raise ValueError('Invalid CAN name')
    report={'platform':platform.platform(),'python':sys.version,'real_motion_available':False,
            'missing_hardware_evidence':['exact_model_variant','firmware_version','joint_sign_and_model_alignment','CPV_command_and_feedback_semantics','stop_and_load_holding_measurements']}
    if sys.platform=='linux':
        report['os_release']=Path('/etc/os-release').read_text()
        report['cpu']=run(['lscpu']);report['memory']=run(['free','-m'])
        report['can']=run(['ip','-details','-statistics','link','show',args.can_name])
        report['ros']=run(['ros2','pkg','prefix','moveit_servo'])
        lock=json.loads((ROOT/'config/dependencies.lock.json').read_text())
        report['packages']=run(['dpkg-query','-W',*lock['tested_debian_packages']])
    print(json.dumps(report,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
