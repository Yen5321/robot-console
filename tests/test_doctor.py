import importlib.util
from pathlib import Path
import unittest

spec=importlib.util.spec_from_file_location('doctor',Path(__file__).resolve().parents[1]/'tools/doctor-v8.py')
doctor=importlib.util.module_from_spec(spec);spec.loader.exec_module(doctor)

class DoctorTests(unittest.TestCase):
    def test_chomp_is_included_in_installed_family_repair(self):
        self.assertTrue('ros-humble-chomp-motion-planner'.startswith(doctor.PREFIXES))
    def test_apt_candidate_not_installed_or_none(self):
        data='ros-humble-launch:\n  Installed: 1.0.1\n  Candidate: 1.0.14-1jammy\nros-humble-rclcpp:amd64:\n  Installed: 16.0.1\n  Candidate: 16.0.21-1jammy\nmissing:\n  Candidate: (none)\n'
        self.assertEqual(doctor.parse_candidates(data),{'ros-humble-launch':'1.0.14-1jammy','ros-humble-rclcpp':'16.0.21-1jammy'})
    def test_repository_credentials_redacted(self):
        self.assertEqual(doctor.clean('https://name:secret@example.test/foo'),'https://[redacted]@example.test/foo')
