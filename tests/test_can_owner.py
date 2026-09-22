import importlib.util,tempfile,unittest
from pathlib import Path
spec=importlib.util.spec_from_file_location('owner',Path(__file__).resolve().parents[1]/'tools/check-can-owner.py')
owner=importlib.util.module_from_spec(spec);spec.loader.exec_module(owner)

class OwnerTests(unittest.TestCase):
    def test_legacy_writer_conflicts_but_ros_network_bridge_does_not(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)
            for pid,driver in ((900000001,'piper6'),(900000002,'ros_servo')):
                p=root/str(pid);(p/'cwd').mkdir(parents=True)
                (p/'cmdline').write_bytes(b'python\0-m\0robot_bridge\0--config\0config.yaml\0')
                (p/'cwd/config.yaml').write_text('arm:\n  driver: '+driver+'\n')
            self.assertEqual([p for p,_ in owner.conflicts(root)],[900000001])
    def test_missing_bridge_config_is_not_assumed_safe(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);p=root/'900000001';p.mkdir()
            (p/'cmdline').write_bytes(b'python\0-m\0robot_bridge\0')
            self.assertEqual(len(owner.conflicts(root)),1)
    def test_existing_executor_blocks_second_instance(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);p=root/'900000001';p.mkdir()
            (p/'cmdline').write_bytes(b'python\0/install/lib/pkg/cpv_executor\0--ros-args\0')
            self.assertEqual(len(owner.conflicts(root)),1)
