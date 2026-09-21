#!/usr/bin/env python3

import json
import os
import sys
import tempfile
import types
import unittest
from types import SimpleNamespace
from unittest import mock


os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
PYTHON_SDK = os.path.join(REPO_ROOT, "third_party", "sl_linka", "sdk", "python")
if PYTHON_SDK not in sys.path:
    sys.path.insert(0, PYTHON_SDK)


def _stub_module(name, **symbols):
    module = types.ModuleType(name)
    for symbol_name, value in symbols.items():
        setattr(module, symbol_name, value)
    sys.modules[name] = module


class _Dummy:
    pass


# Generated ROS modules are unavailable before Catkin runs on edit-only hosts.
_stub_module("rospy", logwarn=lambda *_args, **_kwargs: None)
_stub_module("diagnostic_msgs")
_stub_module(
    "diagnostic_msgs.msg",
    DiagnosticArray=_Dummy,
    DiagnosticStatus=_Dummy,
    KeyValue=_Dummy,
)
_stub_module("actionlib_msgs")
_stub_module("actionlib_msgs.msg", GoalID=_Dummy)
_stub_module("geometry_msgs")
_stub_module(
    "geometry_msgs.msg",
    Pose=_Dummy,
    PoseStamped=_Dummy,
    PoseWithCovarianceStamped=_Dummy,
    Twist=_Dummy,
)
_stub_module("nav_msgs")
_stub_module("nav_msgs.msg", OccupancyGrid=_Dummy, Odometry=_Dummy, Path=_Dummy)
_stub_module("nav_msgs.srv", LoadMap=_Dummy)
_stub_module("std_msgs")
_stub_module("std_msgs.msg", Bool=_Dummy, Int16=_Dummy, UInt16=_Dummy)
_stub_module("std_srvs")
_stub_module("std_srvs.srv", Empty=_Dummy, Trigger=_Dummy, TriggerResponse=_Dummy)
_stub_module(
    "grinder_chassis_driver.msg",
    ChassisStatus=_Dummy,
    WheelSpeedCommand=_Dummy,
    WheelSpeedState=_Dummy,
)
_stub_module("grinder_chassis_driver.srv", EnableChassis=_Dummy)
_stub_module(
    "grinder_scheduler.msg",
    MapPreviewMetadata=_Dummy,
    SchedulerStatus=_Dummy,
)
_stub_module(
    "grinder_scheduler.srv",
    GetSuperLioStatus=_Dummy,
    SaveSuperLioMap=_Dummy,
    StartSuperLioLocalization=_Dummy,
)
_stub_module("grinder_scheduler.aurora_bridge", AuroraBridge=_Dummy)
_stub_module("grinder_scheduler.local_rtsp_server", LocalRtspStreamServer=_Dummy)
_stub_module("grinder_scheduler.map_service", MapService=_Dummy)
_stub_module("grinder_scheduler.media_streamer", FFmpegMediaStreamer=_Dummy)
_stub_module(
    "grinder_scheduler.map_catalog_response",
    fill_map_catalog_response=lambda **_kwargs: (0, 0, 0),
)
_stub_module(
    "grinder_scheduler.models",
    PlannerPath=_Dummy,
    SchedulerState=_Dummy,
    TaskConfigModel=_Dummy,
    VideoStreamState=_Dummy,
    is_planning_direction=lambda _value: False,
    normalize_planning_direction=lambda value: value,
)
_stub_module("grinder_scheduler.planner_adapter", PlannerAdapter=_Dummy)
_stub_module(
    "grinder_scheduler.platform_integration",
    MqttDeviceReporter=_Dummy,
    PlatformFileSync=_Dummy,
)
_stub_module("grinder_scheduler.sl_linka_adapter", SlLinkAServer=_Dummy)

from sl_link.message_gen import sl_link_pb2 as pb  # noqa: E402
from grinder_scheduler import scheduler_node as scheduler_module  # noqa: E402


SchedulerNode = scheduler_module.SchedulerNode


def _node(map_root=""):
    node = SchedulerNode.__new__(SchedulerNode)
    node._live_map_id = "LIVE_MAP"
    node._map_registry = {}
    node._super_lio_map_root = map_root
    return node


class LiveMapSaveTest(unittest.TestCase):
    def test_map_delete_commits_local_delete_before_remote_queue(self):
        with tempfile.TemporaryDirectory() as map_root:
            bundle_dir = os.path.join(map_root, "saved-map")
            os.makedirs(bundle_dir)
            with open(os.path.join(bundle_dir, "loc_map.pcd"), "w", encoding="utf-8") as handle:
                handle.write("map")

            node = _node(map_root)
            node._map_registry["saved-map"] = {
                "map_id": "saved-map",
                "name": "测试地图",
                "bundle_dir": bundle_dir,
            }
            node._map_delete_require_remote_success = False
            node._active_map_id = "LIVE_MAP"
            node._task_bindings = {}
            node._task_obstacle_regions = {}
            node._task_obstacle_regions_lock = mock.Mock()
            node._last_task_result = {}
            node._current_map_id = mock.Mock(return_value="LIVE_MAP")
            node._prepare_for_map_mode_switch = mock.Mock()
            node._ensure_super_lio_proxies = mock.Mock()
            node._unregister_saved_map = mock.Mock()
            node._remove_map_overlay_states_for_aliases = mock.Mock()
            node._remove_planned_path_debug_for_map = mock.Mock()
            node._remove_task_bindings_for_map_aliases = mock.Mock()
            node._save_local_state = mock.Mock()
            node.platform_file_sync = SimpleNamespace(
                enqueue_map_delete=mock.Mock(return_value=(True, "queued"))
            )
            node.sl_link_server = SimpleNamespace(pb=pb)

            request = pb.MapDeleteRequest(map_id="saved-map")
            payload, message_id, component_id = node.handle_map_delete_request(
                request.SerializeToString()
            )
            response = pb.MapDeleteResponse.FromString(payload)

            self.assertEqual(message_id, pb.MSG_ID_MAP_DELETE_RESPONSE)
            self.assertEqual(component_id, pb.COMP_SCHEDULER)
            self.assertEqual(response.result, pb.RESULT_SUCCESS)
            self.assertTrue(response.deleted)
            self.assertTrue(response.local_deleted)
            self.assertFalse(response.remote_deleted)
            self.assertTrue(response.remote_delete_pending)
            self.assertFalse(os.path.exists(bundle_dir))
            node.platform_file_sync.enqueue_map_delete.assert_called_once_with("saved-map")

    def test_stale_live_registry_record_cannot_enter_metadata_only_branch(self):
        node = _node()
        node.sl_link_server = SimpleNamespace(pb=pb)
        node._map_registry["LIVE_MAP"] = {
            "map_id": "LIVE_MAP",
            "bundle_dir": "stale-live-bundle",
        }
        node._saved_map_id_for_request = mock.Mock(
            side_effect=RuntimeError("live_request_reached_export_path")
        )
        request = pb.MapSaveRequest(map_id="LIVE_MAP", map_name="地下")

        payload, message_id, component_id = node.handle_map_save_request(
            request.SerializeToString()
        )
        response = pb.MapSaveResponse.FromString(payload)

        self.assertEqual(message_id, pb.MSG_ID_MAP_SAVE_RESPONSE)
        self.assertEqual(component_id, pb.COMP_SCHEDULER)
        self.assertEqual(response.result, pb.RESULT_FAILED)
        self.assertEqual(response.message, "live_request_reached_export_path")
        node._saved_map_id_for_request.assert_called_once_with("LIVE_MAP")

    def test_live_request_gets_unique_saved_map_id(self):
        with tempfile.TemporaryDirectory() as map_root:
            os.makedirs(os.path.join(map_root, "20260918_153000_01"))
            node = _node(map_root)
            node._map_registry["20260918_153000"] = {
                "map_id": "20260918_153000",
                "bundle_dir": os.path.join(map_root, "20260918_153000"),
            }
            with mock.patch.object(scheduler_module, "datetime") as fake_datetime:
                fake_datetime.now.return_value.strftime.return_value = "20260918_153000"
                target_map_id = node._saved_map_id_for_request("LIVE_MAP")

            self.assertEqual(target_map_id, "20260918_153000_02")
            self.assertFalse(node._is_live_map_id(target_map_id))

    def test_non_live_request_preserves_explicit_map_id(self):
        node = _node()
        self.assertEqual(node._saved_map_id_for_request("factory_floor"), "factory_floor")

    def test_registry_load_removes_and_rewrites_stale_live_map(self):
        with tempfile.TemporaryDirectory() as state_dir:
            node = _node()
            node._persist_state_enabled = True
            node._persist_state_dir = state_dir
            node._map_registry_state_file = os.path.join(state_dir, "map_registry.json")
            valid_bundle = os.path.join(state_dir, "saved-map")
            payload = {
                "schema_version": 2,
                "map_registry": {
                    "LIVE_MAP": {
                        "map_id": "LIVE_MAP",
                        "bundle_dir": os.path.join(state_dir, "LIVE_MAP"),
                    },
                    "saved-map": {
                        "map_id": "saved-map",
                        "bundle_dir": valid_bundle,
                    },
                },
            }
            with open(node._map_registry_state_file, "w", encoding="utf-8") as handle:
                json.dump(payload, handle)

            node._load_map_registry_state()

            self.assertNotIn("LIVE_MAP", node._map_registry)
            self.assertIn("saved-map", node._map_registry)
            with open(node._map_registry_state_file, "r", encoding="utf-8") as handle:
                rewritten = json.load(handle)
            self.assertNotIn("LIVE_MAP", rewritten["map_registry"])
            self.assertIn("saved-map", rewritten["map_registry"])

    def test_registry_save_filters_live_map_defensively(self):
        with tempfile.TemporaryDirectory() as state_dir:
            node = _node()
            node._persist_state_enabled = True
            node._persist_state_dir = state_dir
            node._map_registry_state_file = os.path.join(state_dir, "map_registry.json")
            node._map_registry = {
                "LIVE_MAP": {"map_id": "LIVE_MAP", "bundle_dir": "reserved"},
                "saved-map": {"map_id": "saved-map", "bundle_dir": "valid"},
            }

            node._save_map_registry_state()

            self.assertNotIn("LIVE_MAP", node._map_registry)
            with open(node._map_registry_state_file, "r", encoding="utf-8") as handle:
                saved = json.load(handle)
            self.assertNotIn("LIVE_MAP", saved["map_registry"])


if __name__ == "__main__":
    unittest.main()
