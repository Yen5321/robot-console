import copy
import sys
import tempfile
import unittest
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'ros2_ws/src/robot_console_servo'))
from robot_console_servo.settings import load


class SettingsTests(unittest.TestCase):
    def test_ranges_duplicates_and_nonfinite_rejected(self):
        original=ROOT/'ros2_ws/src/robot_console_servo/config/motion.yaml'
        valid=load(original)
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'motion.yaml'
            for key,value in [('cycle_hz',101),('input_timeout_s',.3),('linear_speed_m_s',float('nan')),
                              ('joint_acceleration_rad_s2',True),('workspace_m',[[0,1]]),('typo',1)]:
                data=copy.deepcopy(valid);data[key]=value
                path.write_text(yaml.safe_dump(data))
                with self.subTest(key=key),self.assertRaises(ValueError):load(path)
            path.write_text(original.read_text(encoding='utf-8')+'\ncycle_hz: 100\n',encoding='utf-8')
            with self.assertRaisesRegex(ValueError,'Duplicate'):load(path)
