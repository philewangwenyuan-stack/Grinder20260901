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
_stub_module("rosnode", get_node_names=lambda: [])
_stub_module(
    "rospy",
    ServiceException=RuntimeError,
    is_shutdown=lambda: False,
    logerr=lambda *_args: None,
)
_stub_module("nav_msgs")
_stub_module("nav_msgs.msg", OccupancyGrid=_Dummy, Odometry=_Dummy)
_stub_module("geometry_msgs")
_stub_module("geometry_msgs.msg", PoseWithCovarianceStamped=_Dummy)
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
    StartSuperLioLocalization=_Dummy,
    StartSuperLioLocalizationResponse=_Dummy,
)
_stub_module("super_lio")
_stub_module("super_lio.srv", GetMap=_Dummy, GetMapRequest=_Dummy)

from grinder_scheduler.super_lio_mode_manager import SuperLioModeManager  # noqa: E402


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


if __name__ == "__main__":
    unittest.main()
