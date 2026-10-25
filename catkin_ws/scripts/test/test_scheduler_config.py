"""Offline configuration validation; no ROS master or external services."""

import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


SCRIPTS = Path(__file__).resolve().parents[1]
ROOT = SCRIPTS.parents[1]
spec = importlib.util.spec_from_file_location("check_scheduler_config", SCRIPTS / "check_scheduler_config.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class SchedulerConfigurationTest(unittest.TestCase):
    def test_segment_sampling_defaults_and_invalid_values(self):
        public = ROOT / "catkin_ws/src/grinder_scheduler/config/scheduler.yaml"
        self.assertEqual(.3, module.validate_configs(public)["active_segment_point_spacing"])
        with tempfile.TemporaryDirectory() as directory:
            local = Path(directory) / "device.yaml"
            for value in ("0", "-1", ".001", ".nan", ".inf", "true", "null"):
                local.write_text("active_segment_point_spacing: " + value, encoding="utf-8")
                with self.assertRaises(ValueError):
                    module.validate_configs(public, local)
            local.write_text("active_segment_point_spacing: 0.2", encoding="utf-8")
            self.assertEqual(.2, module.validate_configs(public, local)["active_segment_point_spacing"])

    def test_public_defaults_disable_external_services_and_clear_credentials(self):
        config = module.validate_configs(ROOT / "catkin_ws/src/grinder_scheduler/config/scheduler.yaml")
        for key in ("file_upload_on_map_save", "mqtt_enabled", "stream_enabled"):
            self.assertIs(config[key], False)
        for key in ("platform_username", "platform_password", "mqtt_username", "mqtt_password",
                    "mqtt_dev_code", "platform_base_url", "file_server_base_url", "stream_url"):
            self.assertEqual(config[key], "")

    def test_optional_override_takes_precedence(self):
        with tempfile.TemporaryDirectory() as directory:
            public = Path(directory) / "public.yaml"
            local = Path(directory) / "local.yaml"
            public.write_text("mqtt_enabled: false\nvehicle_width: 0.8\n", encoding="utf-8")
            local.write_text("mqtt_enabled: true\nmqtt_password: local-only\n", encoding="utf-8")
            config = module.validate_configs(public, str(local))
            self.assertIs(config["mqtt_enabled"], True)
            self.assertEqual(config["vehicle_width"], 0.8)
            self.assertEqual(config["mqtt_password"], "local-only")

    def test_explicit_missing_override_fails(self):
        public = ROOT / "catkin_ws/src/grinder_scheduler/config/scheduler.yaml"
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "missing"):
                module.validate_configs(public, str(Path(directory) / "absent.yaml"))

    def test_invalid_yaml_does_not_echo_secret(self):
        with tempfile.TemporaryDirectory() as directory:
            broken = Path(directory) / "broken.yaml"
            broken.write_text("mqtt_password: [secret-that-must-not-be-logged\n", encoding="utf-8")
            result = subprocess.run(
                [sys.executable, str(SCRIPTS / "check_scheduler_config.py"), "--default-config", str(broken)],
                text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.assertNotEqual(result.returncode, 0)
            self.assertNotIn("secret-that-must-not-be-logged", result.stderr)
            self.assertIn("invalid scheduler YAML", result.stderr)

    def test_non_mapping_and_non_boolean_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "invalid.yaml"
            for value in ("- a\n- b\n", 'mqtt_enabled: "false"\n'):
                config.write_text(value, encoding="utf-8")
                with self.assertRaises(ValueError):
                    module.validate_configs(config)


if __name__ == "__main__":
    unittest.main()
