"""Install into a NEW sibling directory; preserve the existing v7 runtime and YAML.

No service start, CAN command, firmware write or OS change. Atomic publish of a
validated extracted source tree. Do not run this from inside the target directory.
"""
import argparse,datetime,hashlib,json,os,shutil,stat,subprocess,sys,tempfile,zipfile
from pathlib import Path


def install(archive,legacy,target):
    import yaml
    archive=Path(archive).resolve();legacy=Path(legacy).resolve();target=Path(target).resolve()
    if target==legacy or legacy in target.parents or target in legacy.parents:
        raise ValueError('Target must be separate from legacy runtime')
    if target.exists():raise ValueError('Target already exists. Choose a new version directory; no overwrite performed.')
    config_path=legacy/'config.yaml'
    if not config_path.is_file():config_path=legacy/'config/bridge.readonly.local.yaml'
    if not config_path.is_file():raise ValueError('Legacy config.yaml or migrated local YAML not found')
    original=yaml.safe_load(config_path.read_text(encoding='utf-8'))
    if not isinstance(original,dict):raise ValueError('Legacy YAML must be a mapping')
    target.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.v8-stage-',dir=target.parent) as temp:
        stage=Path(temp)
        with zipfile.ZipFile(archive) as z:
            for entry in z.infolist():
                path=stage/entry.filename
                if stage not in path.resolve().parents:raise ValueError('Archive path escapes stage')
                if stat.S_ISLNK(entry.external_attr>>16):raise ValueError('Symlink in archive')
                if not entry.is_dir():path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(z.read(entry))
        if not (stage/'bridge/robot_bridge/main.py').is_file():raise ValueError('Not a v8 robot update archive')
        manifest=json.loads((stage/'MANIFEST.sha256.json').read_text())
        for name,digest in manifest.items():
            path=(stage/name).resolve()
            if stage not in path.parents or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
                raise ValueError('Archive checksum mismatch: '+name)
        actual={p.relative_to(stage).as_posix() for p in stage.rglob('*') if p.is_file()}-{'MANIFEST.sha256.json'}
        if actual!=set(manifest):raise ValueError('Archive manifest file list mismatch')
        backup=stage/'config/imported-legacy';backup.mkdir(parents=True,exist_ok=True)
        shutil.copy2(config_path,backup/'config.yaml.original')
        old_home=Path(original.get('arm',{}).get('home_path','recorded-home.json'))
        if not old_home.is_absolute():old_home=legacy/old_home
        if old_home.is_file():shutil.copy2(old_home,backup/'recorded-home.original.json')
        config=dict(original);config['arm']={'driver':'ros_servo','endpoint':'http://127.0.0.1:9088'}
        config['camera']=dict(config['camera'])
        if config['camera'].get('driver')=='d435':config['camera']['driver']='d435_process'
        local=stage/'config/bridge.readonly.local.yaml'
        local.write_text(yaml.safe_dump(config,allow_unicode=True,sort_keys=False),encoding='utf-8')
        can_name=original.get('arm',{}).get('can_name','can0')
        old_connection=legacy/'config/imported-connection.json'
        if old_connection.exists():can_name=json.loads(old_connection.read_text()).get('can_name',can_name)
        for existing_home in legacy.glob('recorded-home*.json'):shutil.copy2(existing_home,backup/existing_home.name)
        shutil.copy2(local,stage/'config/bridge.real.local.yaml')
        (stage/'config/imported-connection.json').write_text(json.dumps({'can_name':can_name,'legacy_path':str(legacy),'zero_requires_frame_validation':True},indent=2),encoding='utf-8')
        changes={'arm.driver':['piper6','ros_servo'],'arm.endpoint':'localhost:9088','camera.driver':config['camera'].get('driver'),
                 'preserved':['network','camera settings','telemetry','laser','original CAN settings in backup','original zero file in backup'],
                 'motion':'NOT STARTED; v8.1 real entry requires measured commissioning evidence','legacy_modified':False}
        (stage/'config/migration.json').write_text(json.dumps(changes,ensure_ascii=False,indent=2),encoding='utf-8')
        env={**os.environ,'PYTHONPATH':str(stage/'bridge')}
        subprocess.run([sys.executable,'-m','robot_bridge','--config',str(local),'--check-config'],env=env,cwd=stage,check=True,stdout=subprocess.DEVNULL)
        # Copy to final path only after validation. Stage remains an unrelated sibling.
        stage.rename(target)
        print(json.dumps({'installed':str(target),'changes':changes},ensure_ascii=False,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--archive',required=True);p.add_argument('--legacy',required=True);p.add_argument('--target',required=True);a=p.parse_args()
    install(a.archive,a.legacy,a.target)
