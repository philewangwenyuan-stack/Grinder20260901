"""Coverage continuation and production scheduler methods, without ROS/hardware."""
import ast
import copy
import json
import math
import pathlib
import sys
import time
import threading
import types
import unittest
import tempfile
from dataclasses import asdict
from unittest import mock

SOURCE = pathlib.Path(__file__).resolve().parents[1] / 'src/grinder_scheduler'
sys.path.insert(0, str(SOURCE.parent))
from grinder_scheduler.models import TaskConfigModel, SchedulerState
from grinder_scheduler.task_continuation import coverage_continuation
sys.path.insert(0, str(SOURCE.parents[2] / 'grinder_chassis_driver/src'))
from grinder_chassis_driver.motion_safety import MotionInterlock, motion_action


def points(xy, tag='A__lap_1'):
    return [dict(x=x, y=y, path_type=tag) for x, y in xy]


def scheduler_class():
    tree = ast.parse((SOURCE / 'scheduler_node.py').read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SchedulerNode')
    names = {'_continuation_identity', '_continuation_map_stamp', '_planning_continuation',
             '_capture_task_continuation', '_prepare_task_continuation_start', '_stop_execution',
             '_init_segment_execution_cursor', '_start_execution'}
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    namespace = dict(json=json, math=math, time=time, asdict=asdict, motion_action=motion_action,
                     coverage_continuation=coverage_continuation, SchedulerState=SchedulerState,
                     Bool=lambda **kw: types.SimpleNamespace(**kw),
                     UInt16=lambda **kw: types.SimpleNamespace(**kw),
                     Int16=lambda **kw: types.SimpleNamespace(**kw),
                     rospy=types.SimpleNamespace(logwarn=lambda *a: None, loginfo=lambda *a: None))
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(SOURCE / 'scheduler_node.py'), 'exec'), namespace)
    return namespace['SchedulerNode']


class CoverageContinuationTest(unittest.TestCase):
    def test_mid_lane_and_lane_end_have_different_directions(self):
        path = points([(0, 0), (10, 0), (10, 1), (0, 1), (0, 2), (10, 2)])
        region = [dict(region_id='A', global_direction='x')]
        for progress, expected in ((5, 'x'), (10, '-x'), (10.5, '-x'), (15, '-x'), (21, 'x')):
            with self.subTest(progress=progress):
                result = coverage_continuation(path, progress, region)
                self.assertEqual(expected, result['direction'])
        self.assertEqual(dict(x=5., y=0.), coverage_continuation(path, 5, region)['start_pose'])
        self.assertIsNone(coverage_continuation(path, 32, region))

    def test_all_axes_and_rotated_maps(self):
        for axis in ('x', 'y'):
            for sign in (1, -1):
                for angle in (0, 30):
                    theta = math.radians(angle + (90 if axis == 'y' else 0))
                    path = points([(0, 0), (10 * sign * math.cos(theta), 10 * sign * math.sin(theta))])
                    result = coverage_continuation(path, 5, [dict(region_id='A', global_direction=axis)], angle_deg=angle)
                    self.assertEqual(('-' if sign < 0 else '') + axis, result['direction'])

    def test_connection_does_not_set_direction_and_even_lap_retains_endpoint(self):
        path = points([(9, 0), (10, 1)], 'connection') + points([(10, 1), (0, 1), (0, 2), (10, 2)], 'A__lap_2')
        result = coverage_continuation(path, .5, [dict(region_id='A', global_direction='x')])
        self.assertEqual('-x', result['direction'])
        self.assertEqual(2, result['lap'])
        self.assertEqual(dict(x=10., y=2.), result['end_pose'])
        self.assertEqual(dict(x=10., y=2.), result['full_start_pose'])


class SchedulerContinuationTest(unittest.TestCase):
    def setUp(self):
        self.node = scheduler_class()()
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.node._motion = MotionInterlock(str(pathlib.Path(directory.name) / 'motion.json'))
        n = self.node
        n.task_config = TaskConfigModel(task_id='task', map_id='map',
            work_regions=[dict(region_id='A', global_direction='x')], selected_work_region_ids=['A'])
        n._current_map_id = lambda: 'map'
        n._alignment_yaw_deg_for_sl_link_report = lambda *a: 0.
        n._rotation_alignment_delta_deg_for_map_id = lambda *a: 0.
        self.pose = dict(x=10., y=1., heading_deg=0.)
        self.info = dict(map_version=1, frame_id='map', resolution=.05, width=100, height=100, origin_x=0., origin_y=0.)
        n.aurora_bridge = types.SimpleNamespace(get_pose=lambda: self.pose)
        n.map_service = types.SimpleNamespace(get_map_info=lambda: self.info)
        n.current_path = types.SimpleNamespace(task_id='task', points=points([(0, 0), (10, 0), (10, 1), (0, 1)]))
        n._project_robot_to_active_segment = lambda: dict(progress_s=11.)
        n._active_segment_index = 1
        n._exec_region_repeat_done = {'earlier': 1}
        n._capture_task_continuation()

    def test_stop_then_start_reuses_original_path_and_cursor(self):
        n = self.node
        original = n.current_path
        n.current_path = object()  # an intervening preview must not replace execution material
        n._rebuild_active_segments = mock.Mock()
        n._prepare_task_continuation_start()
        self.assertIs(original, n.current_path)
        self.assertEqual(11., n._continuation_checkpoint['progress_s'])
        self.assertEqual('-x', n._planning_continuation(self.pose)['direction'])

    def test_production_start_keeps_checkpoint_until_cursor_initialized(self):
        n = self.node
        original = n.current_path
        n.current_path = None
        n._chassis_settings = {'run_speed': .1}
        n._collision_imminent_lock = threading.Lock()
        n._collision_imminent_active = False
        n._super_lio_status_proxy = lambda: types.SimpleNamespace(state='READY', localization_ready=True)
        n._tf_buffer = None
        n._sorted_work_region_ids = lambda: ['A']
        for name in ('_ensure_super_lio_proxies', '_sync_task_regions_from_overlay',
                     '_set_chassis_enabled', '_reset_disc_motion_guard',
                     '_set_cmd_vel_forward_runtime_active', '_publish_path_to_navigation',
                     '_publish_active_segment_plan', '_send_active_segment_goal',
                     '_begin_task_execution_record', '_rebuild_active_segments'):
            setattr(n, name, mock.Mock())
        for name in ('work_mode_pub', 'disc_speed_pub', 'disc_enable_pub', 'light_pub'):
            setattr(n, name, types.SimpleNamespace(publish=mock.Mock()))
        n._configured_disc_speed_rpm = lambda: 1200
        n._apply_execution_speed_limit = mock.Mock(return_value=True)
        n._exec_mode = 'path'
        n._active_segments = [dict(start_s=0., end_s=10.), dict(start_s=10., end_s=21.)]
        n._project_pose_to_path_progress = mock.Mock(return_value=dict(progress_s=11., index=2))
        n._activate_segment = mock.Mock()
        n._plan_current_task = mock.Mock(side_effect=AssertionError('unchanged map should reuse path'))
        success, _ = n._start_execution()
        self.assertTrue(success)
        self.assertIs(original, n.current_path)
        self.assertEqual(1, n._activate_segment.call_args.args[0])
        self.assertEqual({'earlier': 1}, n._exec_region_repeat_done)
        self.assertIsNone(n._continuation_checkpoint)
        self.assertEqual(SchedulerState.RUNNING, n.state)
        self.assertEqual('A__lap_1', n.task_config.active_work_region_id)

    def test_map_change_requires_replan_but_preserves_direction(self):
        self.info['map_version'] = 2
        self.node._prepare_task_continuation_start()
        self.assertIsNone(self.node.current_path)
        self.assertEqual('-x', self.node._planning_continuation(self.pose)['direction'])

    def test_foreign_preview_path_cannot_be_captured_as_task_progress(self):
        self.node.current_path.task_id = 'preview'
        self.node._capture_task_continuation()
        self.assertIsNone(self.node._continuation_checkpoint)

    def test_new_identity_relocation_or_endpoint_does_not_inherit(self):
        n = self.node
        self.assertEqual({}, n._planning_continuation(self.pose, request_end=dict(x=50., y=50.)))
        self.assertEqual({}, n._planning_continuation(dict(x=0., y=0.)))
        self.info['frame_id'] = 'different_map_frame'
        self.assertEqual({}, n._planning_continuation(self.pose))
        self.info['frame_id'] = 'map'
        n.task_config.task_id = 'new_task'
        n._prepare_task_continuation_start()
        self.assertIsNone(n.current_path)
        self.assertIsNone(n._continuation_checkpoint)

    def test_stop_is_safe_before_capture_and_does_not_complete_lap(self):
        n = self.node
        events = []
        n.state = SchedulerState.RUNNING
        n._exec_active = True
        for name in ('_reset_disc_motion_guard', '_set_cmd_vel_forward_runtime_active',
                     '_safe_stop_motion', '_set_chassis_enabled', '_finalize_task_result'):
            setattr(n, name, lambda *a, _name=name, **kw: events.append(_name))
        n.disc_enable_pub = types.SimpleNamespace(publish=lambda m: events.append('disc_off'))
        n.light_pub = types.SimpleNamespace(publish=lambda m: events.append('light_off'))
        n._capture_task_continuation = lambda: events.append('capture')
        n._mark_current_region_repeat_done = mock.Mock()
        n._stop_execution()
        self.assertLess(events.index('_safe_stop_motion'), events.index('capture'))
        self.assertLess(events.index('disc_off'), events.index('capture'))
        n._mark_current_region_repeat_done.assert_not_called()
        self.assertEqual({'earlier': 1}, n._exec_region_repeat_done)
        n._stop_execution()
        self.assertEqual(1, events.count('capture'))

    def test_cursor_projection_is_restricted_to_saved_segment(self):
        n = self.node
        n._active_segments = [dict(start_s=0., end_s=10.), dict(start_s=10., end_s=21.)]
        n._project_pose_to_path_progress = mock.Mock(return_value=dict(progress_s=11., index=2))
        n._activate_segment = mock.Mock()
        n._init_segment_execution_cursor()
        kwargs = n._project_pose_to_path_progress.call_args.kwargs
        self.assertAlmostEqual(10.95, kwargs['min_s'])
        self.assertEqual(21., kwargs['max_s'])
        self.assertEqual(1, n._activate_segment.call_args.args[0])


if __name__ == '__main__':
    unittest.main()
