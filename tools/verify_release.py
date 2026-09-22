"""Verify generated archives and test actual robot ZIP installation in a temp tree."""
import hashlib
import importlib.util
import json
import tempfile
import zipfile
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'releases'

def main():
    for line in (OUT/'SHA256SUMS-v8.1.txt').read_text().splitlines():
        digest,name=line.split('  ',1)
        path=OUT/name
        assert hashlib.sha256(path.read_bytes()).hexdigest()==digest,name
        with zipfile.ZipFile(path) as archive:
            assert archive.testzip() is None,name
            if 'MANIFEST.sha256.json' in archive.namelist():
                checks=json.loads(archive.read('MANIFEST.sha256.json'))
                assert set(archive.namelist())==set(checks)|{'MANIFEST.sha256.json'},name
                for entry,expected in checks.items():
                    assert hashlib.sha256(archive.read(entry)).hexdigest()==expected,(name,entry)
        print('VERIFIED',name)
    spec=importlib.util.spec_from_file_location('installer',ROOT/'tools/install_v8.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    with tempfile.TemporaryDirectory() as temp:
        base=Path(temp);legacy=base/'legacy';legacy.mkdir()
        original='arm:\n  driver: piper6\n  can_name: can7\ncamera:\n  driver: none\nnetwork:\n  udp_port: 19000\n  http_port: 18080\n'
        (legacy/'config.yaml').write_text(original)
        (legacy/'recorded-home.json').write_text('[1,2,3,4,5,6]')
        module.install(OUT/'RobotConsole-v8.1-RobotUpdate.zip',legacy,base/'v8')
        assert (legacy/'config.yaml').read_text()==original
        assert (base/'v8/ros2_ws/src/robot_console_servo/model/piper/urdf/piper_description.urdf').exists()
        assert (base/'v8/config/imported-legacy/recorded-home.original.json').is_file()
        print('VERIFIED actual archive migration, legacy config/home preserved')

if __name__=='__main__':main()
