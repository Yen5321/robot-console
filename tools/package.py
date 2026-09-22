"""Create reproducible-content release ZIPs and a checksum inventory."""
from pathlib import Path
import hashlib,json,os,shutil,zipfile

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'releases'
EXCLUDED={'bin','obj','__pycache__','.git','.venv','.deps-readonly','.pytest_cache','build','install','log','logs','releases','diagnostics','agx_arm_urdf','.log'}

def files(include_windows):
    lock=json.loads((ROOT/'config/dependencies.v8.1.lock.json').read_text(encoding='utf-8'))
    locked={('vendor/pyAgxArm/'+name):digest for name,digest in lock['sdk_source_sha256'].items()}
    seen=set()
    for directory,dirs,names in os.walk(ROOT):
        dirs[:]=sorted(d for d in dirs if d not in EXCLUDED|{'imported-legacy'} and not d.endswith('.egg-info') and (include_windows or d!='windows-client'))
        for name in sorted(names):
            p=Path(directory)/name;relative=p.relative_to(ROOT)
            if p.name.startswith('recorded-home') or '.local.' in p.name:continue
            if p.name in ('imported-connection.json','migration.json'):continue
            if p.suffix in ('.png','.zip','.pyc'):continue
            seen.add(relative.as_posix())
            yield p,relative.as_posix()
    # Runtime log exclusions must not omit pinned upstream source in .log/.
    for name,digest in sorted(locked.items()):
        p=ROOT/name
        if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=digest:
            raise RuntimeError('Pinned SDK source mismatch before packaging: '+name)
        if name not in seen:
            yield p,name

def archive(path,entries):
    checks={}
    with zipfile.ZipFile(path,'w',zipfile.ZIP_DEFLATED) as z:
        for p,name in entries:
            b=p.read_bytes();z.writestr(name,b);checks[name]=hashlib.sha256(b).hexdigest()
        z.writestr('MANIFEST.sha256.json',json.dumps(checks,indent=2))

def main():
    OUT.mkdir(exist_ok=True)
    archive(OUT/'RobotConsole-v8.1-Source.zip',files(True))
    archive(OUT/'RobotConsole-v8.1-RobotUpdate.zip',files(False))
    app=OUT/'RobotConsole-v8.1-PIPER-L-CPV'
    if not (app/'RustRemoval.RobotConsole.exe').exists():raise RuntimeError('Publish Windows application first')
    (app/'docs').mkdir(exist_ok=True)
    for name in ['DEPLOYMENT.md','V8.1-DEPLOYMENT.md','STATUS.md','TEST_RESULTS-V8.1.md','CPV-ARCHITECTURE.md','CPV-INCIDENT-20260917.md']:
        shutil.copy2(ROOT/'docs'/name,app/'docs'/name)
    (app/'README.md').write_text('# v8.1 PIPER-L CPV 地面调试版\n\n运行本目录 RustRemoval.RobotConsole.exe；保留整个目录。\n\n[从部署到实机验证](docs/V8.1-DEPLOYMENT.md) · [完成与待验证](docs/STATUS.md) · [软件测试](docs/TEST_RESULTS-V8.1.md)\n\n配套机器人更新包和源码ZIP在同级发布目录。实机先完成只读/FK、零速保持和逐轴验证，不自动使能。更高速度和高处运行未验收。\n',encoding='utf-8')
    archive(OUT/'RobotConsole-v8.1-PIPER-L-CPV.zip',[(p,p.relative_to(app).as_posix()) for p in sorted(app.rglob('*')) if p.is_file() and not any(x in p.parts for x in ('EBWebView','Crashpad'))])
    shutil.copy2(ROOT/'tools/install_v8.py',OUT/'install_v81.py')
    shutil.copy2(ROOT/'tools/deploy-v8.ps1',OUT/'deploy-v8.1.ps1')
    shutil.copy2(ROOT/'docs/V8.1-DEPLOYMENT.md',OUT/'README-v8.1.md')
    examples=sorted((ROOT/'diagnostics').glob('v8.1-*.zip'))
    if examples:shutil.copy2(examples[-1],OUT/'RobotConsole-v8.1-Diagnostics-SIM-Example.zip')
    checks={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(OUT.glob('RobotConsole-v8.1-*.zip'))}
    (OUT/'SHA256SUMS-v8.1.txt').write_text('\n'.join(v+'  '+k for k,v in checks.items())+'\n')
    lock=json.loads((ROOT/'config/dependencies.v8.1.lock.json').read_text(encoding='utf-8'))
    inventory={k:lock[k] for k in ('version','robot','reported_firmware_full','firmware_profile','pyAgxArm_commit','model_commit','hardware_validation')}
    inventory.update(windows_product_version='8.1.0-piper-l-cpv',real_jog_limits={'mm_s':2,'deg_s':1},
        python_tests_passed=146,executor_build='8.1.1-cpv-diagnostics',remote_deployed=False,
        physical_motion_tested=True,physical_motion_passed=False,
        hardware_status='Failed initial zero-velocity commissioning; keep emergency stop; see CPV-INCIDENT-20260917.md',sha256=checks)
    (OUT/'VERSION-v8.1.json').write_text(json.dumps(inventory,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:{'sha256':v,'bytes':(OUT/k).stat().st_size} for k,v in checks.items()},indent=2))

if __name__=='__main__':main()
