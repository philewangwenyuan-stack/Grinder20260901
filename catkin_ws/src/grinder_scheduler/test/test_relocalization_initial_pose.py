#!/usr/bin/env python3

import math
import os
import sys
import types
import unittest
from types import SimpleNamespace


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
PYTHON_SDK = os.path.join(REPO_ROOT, "third_party", "sl_linka", "sdk", "python")
if PYTHON_SDK not in sys.path:
    sys.path.insert(0, PYTHON_SDK)


def _stub_module(name, **symbols):
    module = types.ModuleType(name)
    for symbol_name, value in symbols.items():
        setattr(module, symbol_name, value)
    sys.modules[name] = module


# These generated ROS modules are not present in edit-only/test environments
# before Catkin runs. The handler under test only needs their imports to exist.
_stub_module(
    "grinder_chassis_driver.msg",
    ChassisStatus=object,
    WheelSpeedCommand=object,
    WheelSpeedState=object,
)
_stub_module("grinder_chassis_driver.srv", EnableChassis=object)
_stub_module(
    "grinder_scheduler.msg",
    MapPreviewMetadata=object,
    SchedulerStatus=object,
)
_stub_module(
    "grinder_scheduler.srv",
    GetSuperLioStatus=object,
    SaveSuperLioMap=object,
    SetSuperLioInitialPose=object,
    StartSuperLioLocalization=object,
)

from sl_link.message_gen import sl_link_pb2 as pb  # noqa: E402
from grinder_scheduler import scheduler_node as scheduler_module  # noqa: E402


SchedulerNode = scheduler_module.SchedulerNode
scheduler_module.rospy.rostime.set_rostime_initialized(True)


def _node(state="LOCALIZING", active_map_id="map-001", map_revision="a" * 64):
    node = SchedulerNode.__new__(SchedulerNode)
    node.sl_link_server = SimpleNamespace(pb=pb)
    node._initial_pose_position_variance = 0.25
    node._initial_pose_yaw_variance = math.radians(15.0) ** 2
    node._ensure_super_lio_proxies = lambda: None
    node._super_lio_status_proxy = lambda: SimpleNamespace(
        state=state,
        active_map_id=active_map_id,
        active_map_revision=map_revision,
    )
    node.set_initial_pose_calls = []

    def set_initial_pose(map_id, map_revision, pose):
        node.set_initial_pose_calls.append((map_id, map_revision, pose))
        return SimpleNamespace(
            success=True,
            message="initial_pose_published",
            state="RELOCALIZING",
            active_map_id=active_map_id,
            active_map_revision=map_revision,
        )

    node._super_lio_set_initial_pose_proxy = set_initial_pose
    node._fill_super_lio_lifecycle_fields = lambda response: setattr(
        response, "lifecycle_state", state
    )
    node.state = scheduler_module.SchedulerState.IDLE
    node._exec_active = False
    return node


def _request_with_identity(**kwargs):
    request = pb.RadarRelocalizationRequest(
        initial_pose_available=True,
        map_id="map-001",
        map_revision="a" * 64,
        **kwargs
    )
    return request


def _call(node, request):
    payload, message_id, component_id = node.handle_radar_relocalization_request(
        request.SerializeToString()
    )
    return (
        pb.RadarRelocalizationResponse.FromString(payload),
        message_id,
        component_id,
    )


class RelocalizationInitialPoseTest(unittest.TestCase):
    def test_valid_request_publishes_map_pose(self):
        node = _node()
        request = _request_with_identity()
        request.initial_pose.x = 1.2
        request.initial_pose.y = -0.5
        request.initial_pose.heading_deg = 30.0

        response, message_id, component_id = _call(node, request)

        self.assertEqual(message_id, pb.MSG_ID_RADAR_RELOCALIZATION_RESPONSE)
        self.assertEqual(component_id, pb.COMP_SYSTEM)
        self.assertEqual(response.result, pb.RESULT_SUCCESS)
        self.assertTrue(response.accepted)
        self.assertEqual(response.status, "running")
        self.assertEqual(len(node.set_initial_pose_calls), 1)
        sent_map_id, sent_revision, message = node.set_initial_pose_calls[0]
        self.assertEqual(sent_map_id, "map-001")
        self.assertEqual(sent_revision, "a" * 64)
        self.assertEqual(message.header.frame_id, "map")
        self.assertAlmostEqual(message.pose.pose.position.x, 1.2, places=5)
        self.assertAlmostEqual(message.pose.pose.position.y, -0.5, places=5)
        self.assertAlmostEqual(message.pose.pose.orientation.z, math.sin(math.radians(15.0)))
        self.assertAlmostEqual(message.pose.pose.orientation.w, math.cos(math.radians(15.0)))
        self.assertAlmostEqual(message.pose.covariance[0], 0.25)
        self.assertAlmostEqual(message.pose.covariance[7], 0.25)

    def test_missing_pose_is_rejected(self):
        node = _node()
        request = _request_with_identity()
        response, _, _ = _call(node, request)
        self.assertEqual(response.result, pb.RESULT_INVALID_PARAM)
        self.assertFalse(response.accepted)
        self.assertEqual(response.message, "initial_pose_and_map_identity_required")
        self.assertFalse(node.set_initial_pose_calls)

    def test_inactive_localization_is_rejected(self):
        node = _node(state="IDLE")
        request = _request_with_identity()
        request.initial_pose.SetInParent()
        response, _, _ = _call(node, request)
        self.assertEqual(response.result, pb.RESULT_BUSY)
        self.assertFalse(response.accepted)
        self.assertIn("not active", response.message)
        self.assertFalse(node.set_initial_pose_calls)

    def test_negative_variance_is_rejected(self):
        node = _node()
        request = _request_with_identity()
        request.initial_pose.SetInParent()
        request.initial_pose_covariance.valid = True
        request.initial_pose_covariance.x_variance = -1.0
        response, _, _ = _call(node, request)
        self.assertEqual(response.result, pb.RESULT_INVALID_PARAM)
        self.assertFalse(response.accepted)
        self.assertIn("non_negative", response.message)
        self.assertFalse(node.set_initial_pose_calls)

    def test_active_task_is_rejected(self):
        node = _node()
        node.state = scheduler_module.SchedulerState.RUNNING
        node._exec_active = True
        request = _request_with_identity()
        request.initial_pose.SetInParent()
        response, _, _ = _call(node, request)
        self.assertEqual(response.result, pb.RESULT_BUSY)
        self.assertFalse(response.accepted)
        self.assertIn("stopped", response.message)
        self.assertFalse(node.set_initial_pose_calls)

    def test_stale_map_revision_is_rejected(self):
        node = _node()
        request = _request_with_identity()
        request.map_revision = "b" * 64
        request.initial_pose.SetInParent()
        response, _, _ = _call(node, request)
        self.assertEqual(response.result, pb.RESULT_INVALID_PARAM)
        self.assertFalse(response.accepted)
        self.assertIn("map_identity_mismatch", response.message)
        self.assertFalse(node.set_initial_pose_calls)


if __name__ == "__main__":
    unittest.main()
