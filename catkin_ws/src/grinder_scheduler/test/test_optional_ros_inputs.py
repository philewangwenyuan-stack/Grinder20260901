"""ROS input wiring; mocks subscriptions, never connects to hardware or master."""
import ast
import importlib.util
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "src"))
SOURCE = Path(os.environ.get("GRINDER_ROS_INPUT_SOURCE", str(PACKAGE / "src/grinder_scheduler")))
try:
    spec = importlib.util.spec_from_file_location("optional_input_bridge", str(SOURCE / "ros_data_bridge.py"))
    bridge = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(bridge)
except ImportError:
    bridge = None


class LegacyStatusTest(unittest.TestCase):
    def test_scheduler_does_not_register_legacy_status_subscribers(self):
        tree = ast.parse((SOURCE / "scheduler_node.py").read_text(encoding="utf-8"))
        for call in ast.walk(tree):
            if isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute) and call.func.attr == "Subscriber":
                for arg in call.args:
                    if isinstance(arg, ast.Name):
                        self.assertNotIn(arg.id, ("RadarSystemStatus", "RadarRelocalizationStatus"))


@unittest.skipIf(bridge is None, "Requires ROS Noetic and cv_bridge")
class OptionalImageInputsTest(unittest.TestCase):
    def topics(self, **kwargs):
        with patch.object(bridge.rospy, "Subscriber") as subscriber, patch.object(bridge, "CvBridge"):
            bridge.RosDataBridge("/map", "/lio/odom", "/left", "/right",
                                 depth_image_colorized_topic="/depth", **kwargs)
        return [call.args[0] for call in subscriber.call_args_list]

    def test_disabled_keeps_map_and_odometry_for_both_image_modes(self):
        for depth in (False, True):
            with self.subTest(depth=depth):
                self.assertEqual(["/map", "/lio/odom"], self.topics(
                    image_inputs_enabled=False, use_depth_colorized_image=depth))

    def test_default_board_has_no_image_subscriptions(self):
        self.assertEqual(["/map", "/lio/odom"], self.topics())

    def test_explicit_camera_enable_subscribes_both_cameras(self):
        self.assertEqual(["/map", "/lio/odom", "/left", "/right"], self.topics(image_inputs_enabled=True))

    def test_enabled_depth_mode_preserves_depth_subscription(self):
        self.assertEqual(["/map", "/lio/odom", "/depth"], self.topics(image_inputs_enabled=True, use_depth_colorized_image=True))


if __name__ == "__main__":
    unittest.main()
