"""Read-only history and map recipe isolation; no ROS or hardware."""
import ast
import math
import os
import shutil
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "third_party/sl_linka/sdk/python"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from sl_link.message_gen import sl_link_pb2 as pb
from grinder_scheduler.map_service import MapService
from grinder_scheduler.models import SchedulerState, TaskConfigModel


def scheduler_class():
    source = Path(__file__).resolve().parents[1] / "src/grinder_scheduler/scheduler_node.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    cls = next(x for x in tree.body if isinstance(x, ast.ClassDef) and x.name == "SchedulerNode")
    cls.body = [x for x in cls.body if isinstance(x, ast.FunctionDef) and x.name in (
        "handle_map_region_point_request", "_fill_polygon_region_message", "_fill_pose2d_message", "handle_settings_write_request", "handle_path_plan_request",
        "_copy_map_overlay_state")]
    namespace = dict(math=math, os=os, shutil=shutil, SchedulerState=SchedulerState, MapService=MapService, rospy=mock.Mock())
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(source), "exec"), namespace)
    return namespace["SchedulerNode"]


class HistoryProcessTest(unittest.TestCase):
    def node(self, directory):
        node = scheduler_class()()
        node.sl_link_server = SimpleNamespace(pb=pb)
        node.state = SchedulerState.READY
        node.map_service = MapService()
        for rid in ("region-a", "region-b"):
            node.map_service._upsert_region(node.map_service._work_regions,
                dict(region_id=rid, points=[dict(x=0.,y=0.),dict(x=1.,y=0.),dict(x=1.,y=1.)]), rid, "work", 1)
        node._is_live_map_id = lambda key: key == "LIVE_MAP"
        node.task_config = TaskConfigModel(task_id="running", map_id="map-a")
        node._current_map_id = lambda: "map-a"
        node._find_recorded_map_by_id = lambda _: None
        node._persist_state_enabled = True
        node._map_state_dir = lambda _: directory
        node._max_chassis_run_speed = .2
        node._task_runtime_max_disc_speed_rpm = 1500
        node._ensure_offline_map_service_for_current_map = mock.Mock()
        node._chassis_settings = dict(run_speed=.15, disc_speed_rpm=500)
        node.disc_enable_pub = mock.Mock()
        return node

    def test_region_recipe_is_saved_with_map_and_does_not_change_other_region_or_manual_preferences(self):
        with tempfile.TemporaryDirectory() as directory:
            node = self.node(directory)
            request = pb.SettingsWriteRequest(process_map_id="map-a", process_region_id="region-a")
            request.process.CopyFrom(pb.MapProcessSettings(
                run_speed=.03, disc_speed_rpm=925, process_type="ROUGH",
                grinding_parameter="GRIT_30", concrete_strength="C25", pressure=345))
            response = pb.SettingsWriteResponse.FromString(node.handle_settings_write_request(request.SerializeToString())[0])
            self.assertEqual(pb.RESULT_SUCCESS, response.result)
            self.assertTrue(response.HasField("process"))
            self.assertEqual("map-a", response.process_map_id)
            self.assertEqual(dict(run_speed=.15, disc_speed_rpm=500), node._chassis_settings)
            node.disc_enable_pub.publish.assert_not_called()
            # Manual preference changes cannot mutate the independently saved recipe.
            node._chassis_settings.update(run_speed=.1, disc_speed_rpm=1200)
            restored = MapService()
            self.assertTrue(restored.load_local_state(directory))
            self.assertEqual("GRIT_30", restored.get_process_settings("region-a")["grinding_parameter"])
            self.assertAlmostEqual(.03, restored.get_process_settings("region-a")["run_speed"])
            self.assertEqual(925, restored.get_process_settings("region-a")["disc_speed_rpm"])
            self.assertEqual({}, restored.get_process_settings("region-b"))
            self.assertFalse(restored.load_local_state(str(Path(directory) / "other-map")))
            self.assertEqual({}, restored.get_process_settings("region-a"))

    def test_two_recipes_survive_geometry_edit_save_and_reload_independently(self):
        with tempfile.TemporaryDirectory() as directory:
            node = self.node(directory)
            a = dict(run_speed=.03, disc_speed_rpm=925)
            b = dict(run_speed=.07, disc_speed_rpm=1200)
            node.map_service.set_process_settings("region-a", a)
            node.map_service.set_process_settings("region-b", b)
            node.map_service._upsert_region(node.map_service._work_regions,
                dict(region_id="region-a", name="renamed"), "A", "work", 1)
            node.map_service.save_local_state(directory)
            restored = MapService()
            self.assertTrue(restored.load_local_state(directory))
            self.assertEqual(a, restored.get_process_settings("region-a"))
            self.assertEqual(b, restored.get_process_settings("region-b"))
            with self.assertRaises(ValueError):
                restored.set_process_settings("missing-region", a)
            restored.reset_local_state()
            self.assertEqual({}, restored.get_process_settings("region-a"))

    def test_wrong_map_invalid_recipe_and_io_failure_cannot_replace_recipe(self):
        with tempfile.TemporaryDirectory() as directory:
            node = self.node(directory)
            previous = dict(run_speed=.03, disc_speed_rpm=925)
            node.map_service.set_process_settings("region-a", previous)
            for target, speed in (("other-map", .1), ("map-a", float("nan")), ("map-a", .3)):
                request = pb.SettingsWriteRequest(process_map_id=target, process_region_id="region-a",
                    process=pb.MapProcessSettings(run_speed=speed, disc_speed_rpm=900))
                response = pb.SettingsWriteResponse.FromString(node.handle_settings_write_request(request.SerializeToString())[0])
                self.assertEqual(pb.RESULT_FAILED, response.result)
                self.assertEqual(previous, node.map_service.get_process_settings("region-a"))
            node.map_service.save_local_state = mock.Mock(side_effect=OSError("disk full"))
            request.process.run_speed = .1
            response = pb.SettingsWriteResponse.FromString(node.handle_settings_write_request(request.SerializeToString())[0])
            self.assertEqual(pb.RESULT_FAILED, response.result)
            self.assertEqual(previous, node.map_service.get_process_settings("region-a"))

    def test_running_and_paused_tasks_reject_planning_before_map_or_task_changes(self):
        with tempfile.TemporaryDirectory() as directory:
            node = self.node(directory)
            node._validate_requested_map_id = mock.Mock(side_effect=AssertionError("must not switch map"))
            node._handle_path_plan_request_impl = mock.Mock(side_effect=AssertionError("must not plan"))
            request = pb.PathPlanRequest(task_id="history-task", map_id="history-map", force_replan=False)
            for state in (SchedulerState.RUNNING, SchedulerState.PAUSED):
                node.state = state
                response = pb.PathPlanResponse.FromString(node.handle_path_plan_request(request.SerializeToString())[0])
                self.assertEqual(pb.RESULT_BUSY, response.result)
                self.assertEqual(state, node.state)
                self.assertEqual("running", node.task_config.task_id)
                self.assertEqual("map-a", node.task_config.map_id)
                settings = pb.SettingsWriteRequest(chassis=pb.ChassisSettings(run_speed=.15))
                reply = pb.SettingsWriteResponse.FromString(node.handle_settings_write_request(settings.SerializeToString())[0])
                self.assertEqual(pb.RESULT_BUSY, reply.result)
                self.assertEqual(.15, node._chassis_settings["run_speed"])

    def test_editing_another_saved_map_recipe_does_not_switch_or_mutate_current_map(self):
        with tempfile.TemporaryDirectory() as directory:
            node = self.node(directory)
            node._find_recorded_map_by_id = lambda key: {"map_id": key}
            node._map_state_dir = lambda key: str(Path(directory) / key)
            node.map_service.set_process_settings("region-a", dict(run_speed=.03, disc_speed_rpm=925))
            other_source = MapService()
            other_source._upsert_region(other_source._work_regions, dict(region_id="region-a"), "A", "work", 1)
            other_source.save_local_state(node._map_state_dir("map-b"))
            request = pb.SettingsWriteRequest(process_map_id="map-b", process_region_id="region-a",
                process=pb.MapProcessSettings(run_speed=.07, disc_speed_rpm=1200))
            response = pb.SettingsWriteResponse.FromString(node.handle_settings_write_request(request.SerializeToString())[0])
            self.assertEqual(pb.RESULT_SUCCESS, response.result)
            self.assertEqual("map-b", response.process_map_id)
            self.assertEqual("map-a", node._current_map_id())

            self.assertAlmostEqual(.03, node.map_service.get_process_settings("region-a")["run_speed"])
            other = MapService()
            self.assertTrue(other.load_local_state(node._map_state_dir("map-b")))
            self.assertAlmostEqual(.07, other.get_process_settings("region-a")["run_speed"])
            read = pb.MapRegionPointRequest(map_id="map-b")
            node.state = SchedulerState.RUNNING
            reply = pb.MapRegionPointResponse.FromString(node.handle_map_region_point_request(read.SerializeToString())[0])
            self.assertEqual("map-b", reply.map_id)
            self.assertAlmostEqual(.07, reply.work_regions[0].region.process.run_speed)
            self.assertEqual("region-a", reply.work_regions[0].region.region_id)
            self.assertEqual(SchedulerState.RUNNING, node.state)
            self.assertEqual("map-a", node._current_map_id())

    def test_new_saved_map_copy_carries_the_recipe(self):
        with tempfile.TemporaryDirectory() as directory:
            node = self.node(directory)
            node._map_state_dir = lambda key: str(Path(directory) / key)
            node.map_service.set_process_settings("region-a", dict(run_speed=.03, disc_speed_rpm=925, pressure=345.))
            node.map_service.save_local_state(node._map_state_dir("LIVE_MAP"))
            self.assertTrue(node._copy_map_overlay_state("LIVE_MAP", "saved-map", overwrite=True))
            restored = MapService()
            self.assertTrue(restored.load_local_state(node._map_state_dir("saved-map")))
            self.assertEqual(node.map_service.get_process_settings("region-a"), restored.get_process_settings("region-a"))


if __name__ == "__main__":
    unittest.main()
