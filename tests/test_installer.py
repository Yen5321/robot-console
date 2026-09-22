import hashlib,importlib.util,json,tempfile,unittest,zipfile
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('install_v8',ROOT/'tools/install_v8.py')
installer=importlib.util.module_from_spec(spec);spec.loader.exec_module(installer)

class InstallerTests(unittest.TestCase):
    def setup_tree(self,base):
        legacy=base/'old';legacy.mkdir()
        legacy.joinpath('config.yaml').write_text('arm:\n  driver: piper6\n  can_name: can7\n  home_path: recorded-home.json\ncamera:\n  driver: none\nnetwork:\n  udp_port: 19090\n  http_port: 18090\n  command_timeout_s: 0.25\n')
        legacy.joinpath('recorded-home.json').write_text('[1,2,3,4,5,6]')
        archive=base/'update.zip'
        entries={}
        for p in (ROOT/'bridge/robot_bridge').rglob('*.py'):entries[p.relative_to(ROOT).as_posix()]=p.read_bytes()
        with zipfile.ZipFile(archive,'w') as z:
            for name,b in entries.items():z.writestr(name,b)
            z.writestr('MANIFEST.sha256.json',json.dumps({name:hashlib.sha256(b).hexdigest() for name,b in entries.items()}))
        return legacy,archive
    def test_migration_preserves_original_and_ports_and_home(self):
        with tempfile.TemporaryDirectory() as t:
            base=Path(t);legacy,archive=self.setup_tree(base);before=(legacy/'config.yaml').read_bytes();target=base/'v8'
            installer.install(archive,legacy,target)
            self.assertEqual((legacy/'config.yaml').read_bytes(),before)
            self.assertEqual((target/'config/imported-legacy/config.yaml.original').read_bytes(),before)
            self.assertEqual((target/'config/imported-legacy/recorded-home.original.json').read_text(),'[1,2,3,4,5,6]')
            self.assertIn('19090',(target/'config/bridge.readonly.local.yaml').read_text())
            self.assertEqual(json.loads((target/'config/imported-connection.json').read_text())['can_name'],'can7')
            with self.assertRaises(ValueError):installer.install(archive,legacy,target)
    def test_traversal_and_tamper_rejected(self):
        for bad in ('../escape.py','bridge/robot_bridge/main.py'):
            with tempfile.TemporaryDirectory() as t:
                base=Path(t);legacy,archive=self.setup_tree(base)
                with zipfile.ZipFile(archive,'a') as z:z.writestr(bad,b'tampered')
                with self.assertRaises(ValueError):installer.install(archive,legacy,base/'v8')
                self.assertFalse((base/'v8').exists());self.assertFalse((base/'escape.py').exists())
    def test_v8_migration_keeps_current_yaml_can_and_zero_backups(self):
        with tempfile.TemporaryDirectory() as t:
            base=Path(t);legacy,archive=self.setup_tree(base)
            (legacy/'config').mkdir()
            (legacy/'config.yaml').rename(legacy/'config/bridge.readonly.local.yaml')
            (legacy/'config/imported-connection.json').write_text('{"can_name":"can8"}')
            (legacy/'recorded-home-v8-sim.json').write_text('{"old":true}')
            target=base/'v81';installer.install(archive,legacy,target)
            self.assertEqual(json.loads((target/'config/imported-connection.json').read_text())['can_name'],'can8')
            self.assertEqual((target/'config/bridge.real.local.yaml').read_bytes(),(target/'config/bridge.readonly.local.yaml').read_bytes())
            self.assertTrue((target/'config/imported-legacy/recorded-home-v8-sim.json').exists())
            self.assertFalse((target/'recorded-home-v8-sim.json').exists())

if __name__=='__main__':unittest.main()
