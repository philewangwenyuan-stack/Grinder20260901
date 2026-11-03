"""Task start applies process speed before enabling motion; no ROS/hardware required."""
import ast
import math
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
import tempfile
from functools import wraps
from unittest import mock

SOURCE = Path(__file__).resolve().parents[1] / "src/grinder_scheduler/scheduler_node.py"
sys.path.insert(0, str(SOURCE.parent.parent))
from grinder_scheduler.models import SchedulerState, TaskConfigModel
from grinder_scheduler.region_process import base_region_id, point_region_id, capture_region_recipes
sys.path.insert(0, str(SOURCE.parents[3] / 'grinder_chassis_driver/src'))
from grinder_chassis_driver.motion_safety import MotionInterlock, MotionCancelled, motion_action


def production_scheduler(params, writes):
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    names = {
        "_execution_run_speed", "_apply_execution_speed_limit", "_update_navigation_speed_limit",
        "_read_rpp_settings", "_start_execution", "_prepare_region_process_execution", "_apply_region_process_for_path_point",
    }
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "SchedulerNode")
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    defaults = next(n for n in tree.body if isinstance(n, ast.Assign)
                    and any(isinstance(t, ast.Name) and t.id == "RPP_PARAMETER_DEFAULTS" for t in n.targets))

    class Client:
        def __init__(self, namespace, **kwargs):
            self.namespace = namespace

        def update_configuration(self, values):
            writes.append(dict(values))
            params.update({self.namespace + "/" + key: value for key, value in values.items()})
            return dict(values)

    namespace = dict(
        base_region_id=base_region_id, point_region_id=point_region_id, capture_region_recipes=capture_region_recipes,
        MotionCancelled=MotionCancelled, math=math, time=time, SchedulerState=SchedulerState, DynamicReconfigureClient=Client, motion_action=motion_action,
        Bool=lambda **kw: SimpleNamespace(**kw), UInt16=lambda **kw: SimpleNamespace(**kw),
        Int16=lambda **kw: SimpleNamespace(**kw),
        rospy=SimpleNamespace(
            get_param=lambda name, default=None: params.get(name, default),
            set_param=lambda name, value: params.__setitem__(name, value),
            logwarn=mock.Mock(), logerr=mock.Mock(), loginfo=mock.Mock(), logwarn_throttle=mock.Mock()),
    )
    decorator = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "localization_arming")
    namespace["wraps"] = wraps
    exec(compile(ast.Module(body=[defaults, decorator, cls], type_ignores=[]), str(SOURCE), "exec"), namespace)
    return namespace["SchedulerNode"]


class TaskStartSpeedTest(unittest.TestCase):
    def setUp(self):
        self.params = {}
        self.writes = []
        n = self.node = production_scheduler(self.params, self.writes)()
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        n._motion = MotionInterlock(str(Path(directory.name) / 'motion.json'))
        n._localization_arm_lock = threading.Lock()
        n._localization_arm_count = 0
        n._localization_arming_cancelled = False
        n.state = SchedulerState.READY
        n.task_config = TaskConfigModel(task_id="task-a")
        n._chassis_settings = dict(run_speed=.03)
        n._max_chassis_run_speed = .2
        n._navigation_speed_reconfigure_namespace = "/move_base/RegulatedPurePursuitController"
        n._navigation_speed_reconfigure_parameter = "desired_linear_vel"
        n._navigation_speed_reconfigure_timeout = 1.
        n._navigation_speed_client = None
        n._navigation_speed_lock = threading.Lock()
        self.params[n._navigation_speed_reconfigure_namespace + "/desired_linear_vel"] = .05
        n.current_path = SimpleNamespace(points=[dict(x=0., y=0.)])
        n._collision_imminent_lock = threading.Lock()
        n._collision_imminent_active = False
        n._super_lio_status_proxy = lambda: SimpleNamespace(
            state="READY", localization_ready=True,
            active_map_id="map-a", active_map_revision="revision-a")
        n._localization_motion_status = lambda: (
            True, "ready", {"map_id": "map-a", "map_revision": "revision-a"})
        n._tf_buffer = None
        n._continuation_checkpoint = None
        n._sorted_work_region_ids = lambda: []
        n._active_segments = [dict()]
        n._exec_mode = "path"
        n._configured_disc_speed_rpm = lambda: 1200
        n._safe_stop_motion = mock.Mock()
        for name in ("_ensure_super_lio_proxies", "_sync_task_regions_from_overlay",
                     "_prepare_task_continuation_start", "_set_chassis_enabled", "_reset_disc_motion_guard",
                     "_init_segment_execution_cursor", "_set_cmd_vel_forward_runtime_active",
                     "_publish_path_to_navigation", "_publish_active_segment_plan", "_send_active_segment_goal",
                     "_begin_task_execution_record", "_rebuild_active_segments"):
            setattr(n, name, mock.Mock())
        for name in ("work_mode_pub", "disc_speed_pub", "disc_enable_pub", "light_pub"):
            setattr(n, name, SimpleNamespace(publish=mock.Mock()))

    def test_start_uses_point_03_process_speed_instead_of_point_05_rpp_setting(self):
        n = self.node
        n._set_chassis_enabled.side_effect = lambda enabled: self.assertEqual(
            .03, self.params[n._navigation_speed_reconfigure_namespace + "/desired_linear_vel"])
        self.assertTrue(n._start_execution()[0])
        self.assertEqual([{"desired_linear_vel": .03}], self.writes)
        self.assertEqual(SchedulerState.RUNNING, n.state)

    def test_start_can_increase_previous_rpp_target_within_safety_limit(self):
        self.node._chassis_settings["run_speed"] = .1
        self.assertTrue(self.node._start_execution()[0])
        self.assertEqual({"desired_linear_vel": .1}, self.writes[-1])

    def test_start_fails_closed_when_navigation_update_fails(self):
        n = self.node
        n._update_navigation_speed_limit = mock.Mock(return_value=False)
        self.assertFalse(n._start_execution()[0])
        n._safe_stop_motion.assert_called_once()
        n._set_chassis_enabled.assert_not_called()
        n.disc_enable_pub.publish.assert_not_called()
        self.assertEqual(SchedulerState.READY, n.state)

    def test_zero_process_speed_cannot_start(self):
        n = self.node
        n._chassis_settings["run_speed"] = 0.
        self.assertFalse(n._start_execution()[0])
        n._safe_stop_motion.assert_called_once()
        n._set_chassis_enabled.assert_not_called()
        self.assertEqual([], self.writes)

    def test_each_start_reapplies_map_recipe_after_manual_settings_change(self):
        n = self.node
        n.map_service = SimpleNamespace(get_process_settings=lambda region_id: dict(run_speed=.03, disc_speed_rpm=925))
        n.current_path.points[0]["path_type"] = "region-a"
        n._configured_disc_speed_rpm = lambda: n._chassis_settings["disc_speed_rpm"]
        for manual_speed, manual_rpm in ((.15, 500), (.1, 1200)):
            n.state = SchedulerState.READY
            n._chassis_settings.update(run_speed=manual_speed, disc_speed_rpm=manual_rpm)
            self.assertTrue(n._start_execution()[0])
            self.assertEqual({"desired_linear_vel": .03}, self.writes[-1])
            self.assertEqual(925, n._chassis_settings["disc_speed_rpm"])
            self.assertEqual(925, n.disc_speed_pub.publish.call_args[0][0].data)

    def region_node(self):
        n = self.node
        recipes = {"region-a": dict(run_speed=.03, disc_speed_rpm=925),
                   "region-b": dict(run_speed=.07, disc_speed_rpm=1200)}
        n.map_service = SimpleNamespace(get_process_settings=lambda rid: dict(recipes.get(rid, {})))
        n.current_path.points = [dict(x=0., y=0., path_type="region-a__lap_1"),
                                 dict(x=1., y=0., path_type="connection"),
                                 dict(x=2., y=0., path_type="region-b__lap_1")]
        n._configured_disc_speed_rpm = lambda: n._chassis_settings["disc_speed_rpm"]
        n._pause_execution = mock.Mock()
        return n

    def test_regions_have_independent_speed_and_rpm_and_connection_does_not_switch(self):
        n = self.region_node()
        self.assertTrue(n._start_execution()[0])
        self.assertEqual(.03, n._chassis_settings["run_speed"])
        n._set_cmd_vel_forward_runtime_active.reset_mock()
        before = len(self.writes)
        self.assertTrue(n._apply_region_process_for_path_point(dict(path_type="connection")))
        self.assertTrue(n._apply_region_process_for_path_point(dict(path_type="region-a__lap_2")))
        self.assertEqual(before, len(self.writes))
        self.assertEqual("region-a", n._applied_process_region_id)
        self.assertTrue(n._apply_region_process_for_path_point(dict(path_type="region-b__lap_1")))
        self.assertEqual({"desired_linear_vel": .07}, self.writes[-1])
        self.assertEqual(1200, n.disc_speed_pub.publish.call_args[0][0].data)
        self.assertEqual(False, n._set_cmd_vel_forward_runtime_active.call_args_list[0].args[0])
        self.assertEqual(True, n._set_cmd_vel_forward_runtime_active.call_args_list[-1].args[0])
        self.assertTrue(n._apply_region_process_for_path_point(dict(path_type="region-a__lap_3")))
        self.assertEqual({"desired_linear_vel": .03}, self.writes[-1])
        self.assertEqual(925, n.disc_speed_pub.publish.call_args[0][0].data)

    def test_missing_region_recipe_blocks_start_instead_of_inheriting_a_neighbor(self):
        n = self.region_node()
        n.map_service.get_process_settings = lambda rid: dict(run_speed=.03, disc_speed_rpm=925) if rid == "region-a" else {}
        success, message = n._start_execution()
        self.assertFalse(success)
        self.assertIn("region-b", message)
        n.disc_enable_pub.publish.assert_not_called()
        self.assertEqual([], self.writes)

    def test_start_from_second_region_uses_its_recipe_before_tool_enable(self):
        n = self.region_node()
        n.current_path_index = 2
        self.assertTrue(n._start_execution()[0])
        self.assertEqual("region-b", n._applied_process_region_id)
        self.assertEqual({"desired_linear_vel": .07}, self.writes[-1])
        self.assertEqual(1200, n.disc_speed_pub.publish.call_args[0][0].data)

    def test_region_speed_apply_failure_stops_and_never_reenables_motion(self):
        n = self.region_node()
        self.assertTrue(n._start_execution()[0])
        n._set_cmd_vel_forward_runtime_active.reset_mock()
        n._apply_execution_speed_limit = mock.Mock(return_value=False)
        self.assertFalse(n._apply_region_process_for_path_point(dict(path_type="region-b")))
        n._pause_execution.assert_called_once_with(reason="region_process_apply_failed")
        n._safe_stop_motion.assert_called_once()
        self.assertEqual("region-a", n._applied_process_region_id)
        self.assertTrue(all(not c.args[0] for c in n._set_cmd_vel_forward_runtime_active.call_args_list))

    def test_stop_during_region_apply_cannot_reenable_wheels(self):
        n = self.region_node()
        self.assertTrue(n._start_execution()[0])
        n._set_cmd_vel_forward_runtime_active.reset_mock()
        def stopped():
            n._motion.invalidate()
            raise MotionCancelled("stopped")
        n._apply_execution_speed_limit = stopped
        self.assertFalse(n._apply_region_process_for_path_point(dict(path_type="region-b")))
        self.assertTrue(all(not c.args[0] for c in n._set_cmd_vel_forward_runtime_active.call_args_list))

    def test_process_speed_does_not_raise_navigation_safety_cap(self):
        self.node._chassis_settings["run_speed"] = .2
        self.params[self.node._navigation_speed_reconfigure_namespace + "/max_linear_vel"] = .08
        self.assertTrue(self.node._start_execution()[0])
        self.assertEqual({"desired_linear_vel": .08}, self.writes[-1])
        self.assertEqual(.08, self.node._read_rpp_settings()["max_linear_vel"])

    def test_invalid_or_zero_navigation_limit_blocks_start(self):
        n = self.node
        for limit in (0., -1., float("nan"), float("inf")):
            with self.subTest(limit=limit):
                self.params[n._navigation_speed_reconfigure_namespace + "/max_linear_vel"] = limit
                self.assertFalse(n._start_execution()[0])
        n._set_chassis_enabled.assert_not_called()
        self.assertEqual([], self.writes)

    def test_start_uses_configured_teb_parameter_without_reading_rpp(self):
        n = self.node
        n._navigation_speed_reconfigure_parameter = "max_vel_x"
        n._read_rpp_settings = mock.Mock(side_effect=AssertionError("TEB must not query RPP"))
        self.assertTrue(n._start_execution()[0])
        self.assertEqual({"max_vel_x": .03}, self.writes[-1])


if __name__ == "__main__":
    unittest.main()
