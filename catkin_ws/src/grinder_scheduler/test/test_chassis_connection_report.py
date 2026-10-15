"""Exercise the real report helper without starting ROS or accessing serial hardware."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest


source = Path(__file__).resolve().parents[1] / "src/grinder_scheduler/scheduler_node.py"
tree = ast.parse(source.read_text(encoding="utf-8"))
node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "SchedulerNode")
methods = [n for n in node.body if isinstance(n, ast.FunctionDef) and n.name in (
    "_chassis_connection_for_report", "_chassis_status_callback",
)]
clock = SimpleNamespace(monotonic=lambda: 10.0)
namespace = {"time": clock}
exec(compile(ast.fix_missing_locations(ast.Module(body=methods, type_ignores=[])), str(source), "exec"), namespace)


class ChassisConnectionReportTest(unittest.TestCase):
    def setUp(self):
        self.node = SimpleNamespace(last_chassis_status=None, _chassis_status_received_monotonic=None)

    def snapshot(self):
        return namespace["_chassis_connection_for_report"](self.node)

    def test_no_sample_is_unknown(self):
        self.assertIsNone(self.snapshot())

    def test_actual_driver_disconnection_is_reported(self):
        namespace["_chassis_status_callback"](self.node, SimpleNamespace(connected=False))
        self.assertFalse(self.snapshot())

    def test_fresh_feedback_and_driver_silence(self):
        namespace["_chassis_status_callback"](self.node, SimpleNamespace(connected=True))
        self.assertTrue(self.snapshot())
        self.node._chassis_status_received_monotonic = 6.9
        self.assertFalse(self.snapshot())

    def test_new_feedback_recovers_after_timeout(self):
        self.node.last_chassis_status = SimpleNamespace(connected=True)
        self.node._chassis_status_received_monotonic = 1.0
        self.assertFalse(self.snapshot())
        namespace["_chassis_status_callback"](self.node, SimpleNamespace(connected=True))
        self.assertTrue(self.snapshot())


if __name__ == "__main__":
    unittest.main()
