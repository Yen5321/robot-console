import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]

@unittest.skipUnless(sys.platform=='linux' and Path('/opt/ros/humble/setup.bash').exists(),'Needs Ubuntu/Humble shell environment; never installs packages')
class DependencyScriptTests(unittest.TestCase):
    def run_script(self,profile):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp);sudo=folder/'sudo'
            sudo.write_text('#!/bin/sh\nprintf "%s\\n" "$@"\n');sudo.chmod(0o755)
            return subprocess.run(['bash',str(ROOT/'tools/install-deps.sh'),str(profile),'--dry-run'],
                cwd=ROOT,env={**os.environ,'PATH':temp+':'+os.environ['PATH']},capture_output=True,text=True)
    def test_site_profile_dry_run_exact_versions_no_install(self):
        profile=ROOT/'config/dependencies.n100-20260917.lock.json'
        result=self.run_script(profile)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn('--simulate',result.stdout);self.assertIn('--no-remove',result.stdout)
        self.assertIn('ros-humble-launch',result.stdout.splitlines())
        self.assertIn('ros-humble-launch-ros',result.stdout.splitlines())
        for name,version in json.loads(profile.read_text())['requested_debian_packages'].items():
            self.assertIn(name+'='+version,result.stdout)
    def test_invalid_profile_stops_before_apt(self):
        with tempfile.TemporaryDirectory() as temp:
            profile=Path(temp)/'bad.json';profile.write_text('{}')
            result=self.run_script(profile)
            self.assertNotEqual(result.returncode,0);self.assertNotIn('apt-get',result.stdout)
