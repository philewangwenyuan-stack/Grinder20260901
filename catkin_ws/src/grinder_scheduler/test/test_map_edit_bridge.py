"""Exercise actual receive/reset methods without ROS imports or a master."""
import ast
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest

import numpy as np


def bridge_class(convert):
    path = Path(__file__).resolve().parents[1] / "src/grinder_scheduler/ros_data_bridge.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    cls = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == "RosDataBridge")
    cls.body = [item for item in cls.body if isinstance(item, ast.FunctionDef) and item.name in ("reset_map", "_map_callback")]
    rospy = SimpleNamespace(Time=SimpleNamespace(now=lambda: SimpleNamespace(to_nsec=lambda: 20_000_000_000)))
    namespace = dict(rospy=rospy, occupancy_grid_array=convert)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(path), "exec"), namespace)
    return namespace["RosDataBridge"]


def frame(seconds):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(secs=seconds, nsecs=0)),
                           info=SimpleNamespace(width=1, height=1), data=[0])


class MapEditBridgeTests(unittest.TestCase):
    def node(self, converter=lambda msg: np.array([[0]], dtype=np.int8)):
        node = bridge_class(converter)()
        node._lock = threading.Lock()
        node._map_seq = 1
        node._map_msg = "previous-session"
        return node

    def test_new_scan_clears_latched_map_and_rejects_old_frames(self):
        node = self.node()
        node.reset_map()
        node._map_callback(frame(19))
        self.assertIsNone(node._map_msg)
        self.assertEqual(node._map_seq, 2)

    def test_new_session_frame_is_accepted(self):
        node = self.node()
        node.reset_map()
        message = frame(21)
        node._map_callback(message)
        self.assertIs(node._map_msg, message)
        self.assertEqual(node._map_seq, 3)

    def test_frame_validating_during_reset_cannot_overwrite_the_new_session(self):
        def convert(message):
            node.reset_map()
            return np.array([[0]], dtype=np.int8)
        node = self.node(convert)
        node._map_callback(frame(19))
        self.assertIsNone(node._map_msg)


if __name__ == "__main__": unittest.main()
