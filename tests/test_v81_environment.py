import importlib.util,unittest
from pathlib import Path
path=Path(__file__).resolve().parents[1]/'tools/check-v81-environment.py'
spec=importlib.util.spec_from_file_location('v81_environment',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
class EnvironmentTests(unittest.TestCase):
    def test_only_same_upstream_release_family_requires_equal_versions(self):
        for name in ('ros-humble-chomp-motion-planner','ros-humble-moveit-core','ros-humble-moveit-servo','ros-humble-moveit-ros-occupancy-map-monitor'):
            self.assertTrue(module.moveit_core_family(name))
        for name in ('ros-humble-moveit-msgs','ros-humble-moveit-resources-panda-description','ros-humble-rclcpp'):
            self.assertFalse(module.moveit_core_family(name))
