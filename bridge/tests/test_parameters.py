import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
import urllib.request
import urllib.error
from unittest.mock import patch

from robot_bridge.config import load_config
from robot_bridge.parameters import resolve_safety, effective_config
from robot_bridge.main import main
from robot_bridge.video import MjpegServer
from tests.test_video import FakeCamera
from tests.test_piper6 import FakePiperSdk, make_controller


class ParameterTests(unittest.TestCase):
    def test_v8_examples_use_local_ros_backend(self):
        root=Path(__file__).resolve().parents[1]
        for file in ("config.yaml","config.annotated.yaml"):
            cfg=effective_config(load_config(root/file))
            self.assertEqual(cfg['arm']['driver'],'ros_servo')
            self.assertEqual(cfg['arm']['endpoint'],'http://127.0.0.1:9088')
            self.assertLessEqual(cfg['network']['command_timeout_s'],.25)

    def test_bad_types_ranges_and_unknown_switches_rejected(self):
        for settings in ({'jog_rotation_enabled':'false'}, {'jog_translation_mm':float('nan')},
                         {'jog_translation_mm':0}, {'hold_rotation_deg':2}, {'hold_enabled':False},
                         {'feedback_max_age_s':False}, {'feedback_max_age_s':.3}, {'disable_all':True}):
            with self.subTest(settings=settings), self.assertRaises(ValueError): resolve_safety(settings)

    def test_jog_switches_independent_and_thresholds_effective(self):
        for linear, angular in ((True,False),(False,True),(False,False)):
            sdk=FakePiperSdk()
            arm=make_controller(_sdk_interface=sdk, safety_checks={
                'jog_translation_enabled':linear,'jog_translation_mm':2,
                'jog_rotation_enabled':angular,'jog_rotation_deg':1})
            arm.jog_cartesian(0,1,0,0,0)
            sdk.pose.Z_axis=197000
            sdk.pose.RZ_axis=2000
            before=len(sdk.commands)
            if linear or angular:
                with self.assertRaisesRegex(RuntimeError,'Configured jog tracking check exceeded'):
                    arm.jog_cartesian(0,1,0,0,0)
                self.assertEqual(len(sdk.commands),before)
                self.assertEqual(arm.get_diagnostics()['last_rejection']['checks'],['translation'] if linear else ['rotation'])
            else:
                arm.jog_cartesian(0,1,0,0,0)
                self.assertGreater(len(sdk.commands),before)

    def test_custom_hold_threshold_does_not_disable_hold(self):
        sdk=FakePiperSdk(); arm=make_controller(_sdk_interface=sdk,safety_checks={'hold_translation_mm':.5,'hold_translation_enabled':True})
        arm.jog_cartesian(0,1,0,0,0); arm.hold_position()
        sdk.pose.Y_axis=600
        with self.assertRaisesRegex(RuntimeError,'limit 0.5'): arm.check_feedback()
        self.assertEqual(sdk.commands[-1],('estop',1))

    def test_duplicate_yaml_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'config.yaml'
            path.write_text('arm: {}\narm: {}\ncamera: {}\nnetwork: {}\n')
            with self.assertRaisesRegex(ValueError,'Duplicate YAML key'): load_config(path)

    def test_check_cli_never_constructs_hardware(self):
        path=Path(__file__).resolve().parents[1]/'config.annotated.yaml'
        output=io.StringIO()
        with patch('sys.argv',['robot_bridge','--config',str(path),'--check-config']), patch('robot_bridge.main.create_arm') as create, patch('logging.basicConfig'), contextlib.redirect_stdout(output):
            main()
            create.assert_not_called()
        result=json.loads(output.getvalue())
        self.assertEqual(result['bridge_build'],'8.1-piper-l-cpv')
        self.assertEqual(result['effective_config']['arm']['driver'],'ros_servo')
        self.assertEqual(result['effective_config']['network']['command_timeout_s'],.25)

    def test_configuration_http_is_read_only(self):
        server=MjpegServer(FakeCamera(),'127.0.0.1',0,configuration=lambda:{'read_only':True,'safety_checks':resolve_safety()})
        server.start()
        try:
            url=f'http://127.0.0.1:{server._server.server_address[1]}/configuration'
            with urllib.request.urlopen(url,timeout=2) as response:
                result=json.load(response)
            self.assertFalse(result['safety_checks']['jog_rotation_enabled'])
            with self.assertRaises(urllib.error.HTTPError) as error:
                urllib.request.urlopen(urllib.request.Request(url,data=b'{}'),timeout=2)
            self.assertEqual(error.exception.code,404)
        finally: server.close()

    def test_config_rejects_typo_before_hardware(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'config.yaml'
            path.write_text('arm: {driver: piper6}\ncamera: {driver: d435}\nnetwork: {disable_timeout: true}\n')
            with self.assertRaisesRegex(ValueError,'Unknown network keys'): load_config(path)
