import os
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../src")))
from grinder_scheduler.models import SchedulerState
from grinder_scheduler.task_runtime_parameters import TaskRuntimeParameters


class TaskRuntimeParametersTest(unittest.TestCase):
    def setUp(self):
        self.rpp = {"desired_linear_vel": 0.06, "max_linear_vel": 0.15, "max_linear_accel": 0.2}
        self.writes = []
        self.rpm = []
        self.node = SimpleNamespace(
            task_config=SimpleNamespace(task_id="task-a"), state=SchedulerState.RUNNING,
            _navigation_speed_reconfigure_parameter="desired_linear_vel",
            _max_chassis_run_speed=0.2, _task_runtime_max_disc_speed_rpm=1500,
            _chassis_settings={"run_speed": 0.2, "disc_speed_rpm": 1200},
            _disc_auto_cover_desired=True, _read_rpp_settings=lambda: dict(self.rpp),
            _apply_rpp_settings=self.apply, _publish_task_runtime_disc_speed=self.rpm.append,
        )
        self.runtime = TaskRuntimeParameters()

    def apply(self, values):
        self.writes.append(dict(values))
        self.rpp.update(values)
        return dict(values)

    def test_query_has_no_side_effects(self):
        result = self.runtime.execute(self.node, "task-a")
        self.assertEqual(1200, result["disc_speed_rpm"])
        self.assertEqual(0.15, result["max_run_speed"])
        self.assertEqual([], self.writes)
        self.assertEqual([], self.rpm)

    def test_temporary_update_and_restore_preserves_defaults_and_limits(self):
        self.runtime.execute(self.node, "task-a", 0.1, 1300)
        self.assertEqual({"desired_linear_vel": 0.1}, self.writes[-1])
        self.assertEqual({"run_speed": 0.2, "disc_speed_rpm": 1200}, self.node._chassis_settings)
        self.assertEqual(0.15, self.rpp["max_linear_vel"])
        self.assertEqual(0.2, self.rpp["max_linear_accel"])
        self.assertEqual([1300], self.rpm)
        self.runtime.clear(self.node)
        self.assertEqual(0.06, self.rpp["desired_linear_vel"])
        self.assertEqual(1200, self.runtime.disc_speed("task-a", 1200))

    def test_repeated_updates_restore_original_not_previous_temporary_speed(self):
        self.runtime.execute(self.node, "task-a", 0.1)
        self.runtime.execute(self.node, "task-a", 0.12)
        self.runtime.clear(self.node)
        self.assertEqual(0.06, self.rpp["desired_linear_vel"])

    def test_pause_and_transition_change_setpoint_without_output(self):
        self.node.state = SchedulerState.PAUSED
        self.runtime.execute(self.node, "task-a", disc_speed=1400)
        self.assertEqual([], self.rpm)
        self.assertEqual(1400, self.runtime.disc_speed("task-a", 1200))
        self.node.state = SchedulerState.RUNNING
        self.node._disc_auto_cover_desired = False
        self.runtime.execute(self.node, "task-a", disc_speed=1350)
        self.assertEqual([], self.rpm)

    def test_wrong_task_and_stopped_emergency_state_reject(self):
        with self.assertRaises(ValueError):
            self.runtime.execute(self.node, "other", 0.1, 1300)
        self.node.state = SchedulerState.STOPPED
        with self.assertRaises(ValueError):
            self.runtime.execute(self.node, "task-a", 0.1, 1300)
        self.assertEqual([], self.writes)
        self.assertEqual([], self.rpm)

    def test_invalid_values_reject_before_any_output(self):
        for speed, rpm in [(float("nan"), 1000), (float("inf"), 1000), (0, 1000), (0.16, 1000), (0.1, 1501), (0.1, -1)]:
            with self.assertRaises(ValueError):
                self.runtime.execute(self.node, "task-a", speed, rpm)
        self.assertEqual([], self.writes)
        self.assertEqual([], self.rpm)

    def test_partial_dynamic_failure_rolls_back_and_keeps_previous_rpm(self):
        self.runtime.execute(self.node, "task-a", 0.1, 1300)
        def fail_once(values):
            self.node._apply_rpp_settings = self.apply
            self.apply(values)
            raise RuntimeError("reply lost")
        self.node._apply_rpp_settings = fail_once
        with self.assertRaises(RuntimeError):
            self.runtime.execute(self.node, "task-a", 0.12, 1400)
        self.assertEqual(0.1, self.rpp["desired_linear_vel"])
        self.assertEqual(1300, self.runtime.disc_speed("task-a", 1200))
        self.runtime.clear(self.node)
        self.assertEqual(0.06, self.rpp["desired_linear_vel"])

    def test_stop_during_dynamic_application_cannot_publish_rpm(self):
        def stop(values):
            self.apply(values)
            self.node.state = SchedulerState.STOPPED
            return dict(values)
        self.node._apply_rpp_settings = stop
        with self.assertRaises(ValueError):
            self.runtime.execute(self.node, "task-a", 0.1, 1400)
        self.assertEqual([], self.rpm)
        self.assertEqual(0.06, self.rpp["desired_linear_vel"])

    def test_next_task_cannot_inherit_rpm(self):
        self.runtime.execute(self.node, "task-a", disc_speed=1400)
        self.assertEqual(1200, self.runtime.disc_speed("task-b", 1200))


if __name__ == "__main__":
    unittest.main()
