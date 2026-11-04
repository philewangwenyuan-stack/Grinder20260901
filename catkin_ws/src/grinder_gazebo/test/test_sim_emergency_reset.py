"""Exercise the simulated safety latch without a ROS master or motion commands."""
import importlib.util
import os
from pathlib import Path
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

source = os.environ.get("GRINDER_SIM_CHASSIS_SOURCE", str(
    Path(__file__).resolve().parents[1] / "src/grinder_gazebo/chassis_sim_node.py"))
spec = importlib.util.spec_from_file_location("sim_chassis_test", source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class SimEmergencyResetTest(unittest.TestCase):
    def setUp(self):
        self.node = module.ChassisSimNode.__new__(module.ChassisSimNode)
        self.node._lock = threading.RLock()
        self.node._sim_cmd_pub = Mock()
        self.node._emergency_stop = False
        self.node._enabled = True
        self.node._task_enabled = True
        self.node._auto_stamp = self.node._manual_stamp = 10.0
        self.node._auto_command = module.Twist()
        self.node._auto_command.linear.x = 0.2
        self.node._manual_left_rpm = self.node._manual_right_rpm = 100.0
        self.node._disc_enabled = True
        self.node._last_output = module.Twist()

    def test_stop_latches_and_reset_discards_commands_received_while_latched(self):
        self.node._on_emergency_stop(SimpleNamespace(data=True))
        self.node._on_emergency_stop(SimpleNamespace(data=False))
        self.assertTrue(self.node._emergency_stop)
        self.assertEqual("safety_stop", self.node._select_output_locked(10.1)[1])
        self.node._auto_stamp = self.node._manual_stamp = 10.1
        self.node._manual_left_rpm = 50.0
        self.node._disc_enabled = True
        self.assertTrue(self.node._reset_emergency_stop(None).success)
        self.assertFalse(self.node._emergency_stop)
        self.assertFalse(self.node._task_enabled)
        self.assertFalse(self.node._disc_enabled)
        self.assertEqual((0.0, 0.0), (self.node._auto_stamp, self.node._manual_stamp))
        self.assertEqual("idle", self.node._select_output_locked(10.2)[1])
        self.assertEqual(0.0, self.node._sim_cmd_pub.publish.call_args.args[0].linear.x)

    def test_clear_fault_cannot_clear_emergency_latch(self):
        self.node._on_emergency_stop(SimpleNamespace(data=True))
        self.assertTrue(self.node._handle_clear_fault(None).success)
        self.assertTrue(self.node._emergency_stop)


if __name__ == "__main__":
    unittest.main()
