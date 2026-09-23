#!/usr/bin/env python3

import os
import queue
import sys
import tempfile
import threading
import types
import unittest


def _stub_module(name, **attributes):
    module = types.ModuleType(name)
    for key, value in attributes.items():
        setattr(module, key, value)
    sys.modules[name] = module
    return module


class _Dummy:
    pass


# Asset validation is intentionally ROS-independent. Stub imports so the test
# can also run on development hosts without a ROS installation.
_stub_module("roslaunch", rlutil=_Dummy(), parent=_Dummy(), configure_logging=lambda _uuid: None)
_stub_module(
    "rosnode",
    get_node_names=lambda: [],
    rosnode_ping=lambda _name, **_kwargs: True,
    kill_nodes=lambda _names: None,
)
_stub_module(
    "rospy",
    ServiceException=RuntimeError,
    is_shutdown=lambda: False,
    logerr=lambda *_args: None,
    logwarn=lambda *_args: None,
    logwarn_throttle=lambda *_args: None,
    sleep=lambda _seconds: None,
)
_stub_module("nav_msgs")
_stub_module("nav_msgs.msg", OccupancyGrid=_Dummy, Odometry=_Dummy)
_stub_module("sensor_msgs")
_stub_module("sensor_msgs.msg", Imu=_Dummy, PointCloud2=_Dummy)
_stub_module("geometry_msgs")
_stub_module("geometry_msgs.msg", PoseWithCovarianceStamped=_Dummy)
_stub_module("diagnostic_msgs")
_stub_module("diagnostic_msgs.msg", DiagnosticArray=_Dummy)
_stub_module("std_msgs")
_stub_module("std_msgs.msg", Bool=_Dummy)
_stub_module("std_srvs")
_stub_module("std_srvs.srv", Trigger=_Dummy, TriggerResponse=_Dummy)
_stub_module(
    "grinder_scheduler.srv",
    GetSuperLioStatus=_Dummy,
    GetSuperLioStatusResponse=_Dummy,
    SaveSuperLioMap=_Dummy,
    SaveSuperLioMapResponse=_Dummy,
    SetSuperLioInitialPose=_Dummy,
    SetSuperLioInitialPoseResponse=_Dummy,
    StartSuperLioLocalization=_Dummy,
    StartSuperLioLocalizationResponse=_Dummy,
)
_stub_module("super_lio")
_stub_module("super_lio.srv", GetMap=_Dummy, GetMapRequest=_Dummy)

from grinder_scheduler.super_lio_mode_manager import (  # noqa: E402
    SuperLioModeManager,
    SuperLioShutdownTimeout,
)


class SuperLioAssetValidationTest(unittest.TestCase):
    @staticmethod
    def _write_pcd(path, points=1):
        with open(path, "wb") as handle:
            handle.write(
                (
                    "# .PCD v0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n"
                    "TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {0}\nHEIGHT 1\n"
                    "POINTS {0}\nDATA ascii\n0 0 0 1\n"
                ).format(points).encode("ascii")
            )

    def test_valid_bundle(self):
        with tempfile.TemporaryDirectory() as directory:
            self._write_pcd(os.path.join(directory, "loc_map.pcd"))
            self._write_pcd(os.path.join(directory, "plan_map.pcd"))
            with open(os.path.join(directory, "map.pgm"), "wb") as handle:
                handle.write(b"P5\n1 1\n255\n\xfe")
            with open(os.path.join(directory, "map.yaml"), "w", encoding="utf-8") as handle:
                handle.write("image: map.pgm\nresolution: 0.05\n")
            manager = SuperLioModeManager.__new__(SuperLioModeManager)
            paths = manager._validate_bundle(directory)
            self.assertTrue(paths["loc"].endswith("loc_map.pcd"))

    def test_empty_pcd_rejected(self):
        with tempfile.NamedTemporaryFile() as handle:
            with self.assertRaisesRegex(RuntimeError, "missing or empty"):
                SuperLioModeManager._validate_pcd(handle.name)

    def test_yaml_must_reference_bundle_image(self):
        with tempfile.TemporaryDirectory() as directory:
            yaml_path = os.path.join(directory, "map.yaml")
            image_path = os.path.join(directory, "map.pgm")
            with open(yaml_path, "w", encoding="utf-8") as handle:
                handle.write("image: other.pgm\n")
            with open(image_path, "wb") as handle:
                handle.write(b"x")
            with self.assertRaisesRegex(RuntimeError, "must reference map.pgm"):
                SuperLioModeManager._validate_grid(yaml_path, image_path)

    def test_recursive_delete_refuses_map_root(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = SuperLioModeManager.__new__(SuperLioModeManager)
            manager._map_root = directory
            with self.assertRaisesRegex(RuntimeError, "refuse to remove"):
                manager._safe_rmtree(directory)

    def test_service_operation_runs_on_main_thread(self):
        manager = SuperLioModeManager.__new__(SuperLioModeManager)
        manager._main_thread_id = threading.get_ident()
        manager._operation_queue = queue.Queue()
        result = {}

        def handler(request):
            return request, threading.get_ident()

        def invoke_from_service_thread():
            result["value"] = manager._call_on_main_thread(handler, "mapping")

        worker = threading.Thread(target=invoke_from_service_thread)
        worker.start()
        self.assertTrue(manager._process_next_operation(timeout=1.0))
        worker.join(timeout=1.0)
        self.assertFalse(worker.is_alive())
        self.assertEqual(result["value"], ("mapping", manager._main_thread_id))

    def test_stop_owned_launch_rechecks_and_does_not_kill_by_ros_name(self):
        manager = SuperLioModeManager.__new__(SuperLioModeManager)
        active_uris = {"mapping": "http://owned-node"}
        checks = []
        manager._launch_parent = types.SimpleNamespace(
            shutdown=lambda: active_uris.clear()
        )
        manager._owned_mode = "mapping"
        manager._owned_node_uris = {"/super_lio_node": "http://owned-node"}
        manager._expected_owned_node_names = set(SuperLioModeManager._MAPPING_NODES)
        manager._accept_new_owned_node_uris = False
        manager._shutdown_timeout = 0.0
        manager._cleanup_timeout = 0.0
        rosnode_module = sys.modules["rosnode"]
        old_get_node_names = rosnode_module.get_node_names
        old_kill_nodes = rosnode_module.kill_nodes

        def get_node_names():
            return ["/super_lio_node"] if active_uris else []

        def kill_nodes(node_names):
            raise AssertionError("stop must not kill nodes by ROS name")

        rosnode_module.get_node_names = get_node_names
        rosnode_module.kill_nodes = kill_nodes
        manager._node_uri_is_reachable = lambda uri: uri in active_uris.values()
        manager._lookup_node_uri = lambda node_name: "http://owned-node"
        original_residual_check = manager._owned_residual_nodes

        def tracked_residual_check():
            checks.append(True)
            return original_residual_check()

        manager._owned_residual_nodes = tracked_residual_check
        try:
            manager._stop_owned_launch()
        finally:
            rosnode_module.get_node_names = old_get_node_names
            rosnode_module.kill_nodes = old_kill_nodes

        self.assertGreaterEqual(len(checks), 2)
        self.assertEqual(manager._owned_node_uris, {})
        self.assertIsNone(manager._launch_parent)

    def test_stop_timeout_reports_only_residual_node_names(self):
        manager = SuperLioModeManager.__new__(SuperLioModeManager)
        manager._launch_parent = types.SimpleNamespace(shutdown=lambda: None)
        manager._owned_mode = "mapping"
        manager._owned_node_uris = {"/super_lio_node": "http://owned-node"}
        manager._expected_owned_node_names = set(SuperLioModeManager._MAPPING_NODES)
        manager._accept_new_owned_node_uris = False
        manager._shutdown_timeout = 0.0
        manager._cleanup_timeout = 0.0
        manager._refresh_owned_node_uris = lambda: None
        manager._node_uri_is_reachable = lambda _uri: True
        manager._node_names = lambda: {"/super_lio_node", "/map_server"}
        manager._lookup_node_uri = lambda _node_name: "http://owned-node"
        manager._owned_residual_nodes = lambda: ["/super_lio_node"]

        module = sys.modules["grinder_scheduler.super_lio_mode_manager"]
        old_server_proxy = module.ServerProxy

        class _NodeProxy:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def shutdown(self, *_args):
                return 1, "ok", 0

        module.ServerProxy = lambda *_args, **_kwargs: _NodeProxy()
        try:
            with self.assertRaises(SuperLioShutdownTimeout) as caught:
                manager._stop_owned_launch()
        finally:
            module.ServerProxy = old_server_proxy

        self.assertEqual(caught.exception.residual_nodes, ["/super_lio_node"])
        self.assertIsNotNone(manager._launch_parent)
        self.assertEqual(manager._owned_node_uris, {"/super_lio_node": "http://owned-node"})

    def test_unreachable_registration_does_not_block_manager(self):
        manager = SuperLioModeManager.__new__(SuperLioModeManager)
        manager._state = manager.IDLE
        manager._message = "idle"
        manager._launch_parent = None
        manager._live_managed_nodes = lambda: set()
        manager._detect_unowned_conflicts()
        manager._assert_no_unowned_nodes()

        self.assertEqual(manager._state, manager.IDLE)
        self.assertEqual(manager._message, "idle")

    def test_reachable_registration_still_blocks_manager(self):
        manager = SuperLioModeManager.__new__(SuperLioModeManager)
        manager._state = manager.IDLE
        manager._message = "idle"
        manager._launch_parent = None
        manager._live_managed_nodes = lambda: {"/relocation_node"}
        manager._detect_unowned_conflicts()
        with self.assertRaisesRegex(RuntimeError, "unowned Super-LIO nodes"):
            manager._assert_no_unowned_nodes()

        self.assertEqual(manager._state, manager.ERROR)
        self.assertIn("/relocation_node", manager._message)


if __name__ == "__main__":
    unittest.main()
