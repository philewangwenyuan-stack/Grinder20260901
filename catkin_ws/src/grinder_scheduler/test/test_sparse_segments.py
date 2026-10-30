"""Production arc/segment regression tests; no ROS master or machine motion."""
import ast
import bisect
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

SOURCE = Path(__file__).resolve().parents[1] / "src/grinder_scheduler/scheduler_node.py"
sys.path.insert(0, str(SOURCE.parent.parent))
from grinder_scheduler.path_geometry import compact_planner_polyline, sample_segment, segment_point_spacing
from grinder_scheduler.models import SchedulerState

METHODS = {
    "_path_cumulative_lengths", "_ensure_path_arc_lengths", "_index_at_arc_length",
    "_interpolate_path_point_at_s", "_slice_path_points_by_arc", "_project_pose_to_path_progress",
    "_build_path_direction_runs", "_resolve_straight_segment_end_index", "_segment_length",
    "_segment_yaw", "_rebuild_active_segments", "_publish_active_segment_plan",
    "_build_navigation_path_from_segment_points", "_path_tangent_quaternion_for_points",
    "_yaw_to_quaternion", "_rebuild_goal_points", "_append_goal_point", "_segment_orientation",
    "_resolve_point_orientation",
    "_task_progress",
    "_should_insert_corner_mid", "_segments_parallel_or_opposite", "_angle_abs_diff",
    "_segment_is_turn_between_rows", "_init_segment_execution_cursor", "_activate_segment",
    "_update_progress",
}


def scheduler_class(source=None):
    tree = ast.parse(source if source is not None else SOURCE.read_text(encoding="utf-8"))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "SchedulerNode")
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in METHODS]
    def pose():
        return SimpleNamespace(header=SimpleNamespace(frame_id="", stamp=None),
            pose=SimpleNamespace(position=SimpleNamespace(x=0., y=0.),
                                 orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    namespace = dict(math=math, bisect=bisect, time=__import__('time'),
        sample_segment=sample_segment, PoseStamped=pose, SchedulerState=SchedulerState,
        Path=lambda: SimpleNamespace(header=SimpleNamespace(frame_id="", stamp=None), poses=[]),
        rospy=SimpleNamespace(Time=lambda x:x, loginfo=mock.Mock(),
                              loginfo_throttle=mock.Mock(), logwarn_throttle=mock.Mock()))
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(SOURCE), "exec"), namespace)
    return namespace["SchedulerNode"]


def node(points, cls=None):
    result = (cls or scheduler_class())()
    result.current_path = SimpleNamespace(points=points, path_version=1,
        nav_path=SimpleNamespace(header=SimpleNamespace(frame_id="map")))
    result._path_arc_lengths = []
    result._goal_segment_yaw_threshold_deg = 25.
    result._segment_next_lane_leadin_m = .7
    result._should_insert_corner_mid = lambda runs, pos: False
    result._live_map_source_frame = "map"
    result._active_segment_plan_cache = None
    result._active_segment_point_spacing = .3
    result.current_path_index = 0
    result.active_segment_plan_pub = mock.Mock()
    return result


def points(coords, tag="A__lap_1"):
    return [dict(x=float(x), y=float(y), index=i, path_type=tag) for i, (x,y) in enumerate(coords)]


class SparseSegmentsTest(unittest.TestCase):
    def test_long_edge_slice_keeps_corner_and_partial_edges(self):
        n = node(points([(0,0), (30,0), (30,10)]))
        result = n._slice_path_points_by_arc(25,35)
        self.assertEqual([(25.,0.), (30.,0.), (30.,5.)], [(p['x'],p['y']) for p in result])
        dense = sample_segment(result, .3)
        self.assertIn((30.,0.), [(p['x'],p['y']) for p in dense])
        self.assertAlmostEqual(10., sum(math.hypot(b['x']-a['x'], b['y']-a['y']) for a,b in zip(dense,dense[1:])))
        self.assertTrue(all(math.hypot(b['x']-a['x'], b['y']-a['y']) <= .3 + 1e-9 for a,b in zip(dense,dense[1:])))

    def test_direction_runs_share_every_boundary_edge(self):
        for coords in ([(0,0),(30,0)], [(0,0),(30,0),(30,10),(0,10)],
                       [(0,0),(0,0),(30,0),(30,0),(30,10)]):
            n = node(points(coords))
            runs = n._build_path_direction_runs()
            covered = [i for r in runs for i in range(r['start'],r['end'])]
            self.assertEqual(list(range(len(coords)-1)), covered)
            self.assertAlmostEqual(n._ensure_path_arc_lengths()[-1],sum(r['length'] for r in runs))
            n._rebuild_active_segments()
            self.assertEqual(0., n._active_segments[0]['start_s'])
            self.assertAlmostEqual(n._path_arc_lengths[-1], n._active_segments[-1]['end_s'])

    def test_semantic_boundary_is_not_applied_early(self):
        p = points([(0,0),(30,0),(40,0)])
        p[0]['path_type'] = 'connection'
        n = node(p)
        self.assertEqual(['connection','A__lap_1'],[r['path_type'] for r in n._build_path_direction_runs()])
        for distance in (0.,15.,29.99):
            self.assertEqual(0,n._project_pose_to_path_progress(distance,.1)['index'])
            self.assertEqual('connection',n._interpolate_path_point_at_s(distance)['path_type'])
        self.assertEqual(1,n._project_pose_to_path_progress(30.,0.)['index'])
        self.assertEqual('A__lap_1',n._interpolate_path_point_at_s(30.)['path_type'])
        dense=sample_segment(n._slice_path_points_by_arc(25,35),.3)
        self.assertTrue(all(p['path_type']=='connection' for p in dense if p['x']<30))

    def test_zero_length_semantic_event_preserved(self):
        p=points([(0,0),(30,0),(30,0),(40,0)])
        p[2]['path_type']='B__lap_2'
        p[3]['path_type']='B__lap_2'
        n=node(p)
        sliced=n._slice_path_points_by_arc(25,35)
        at_corner=[p['path_type'] for p in sliced if p['x']==30]
        self.assertEqual(['A__lap_1','B__lap_2'],at_corner)
        self.assertEqual(2,n._project_pose_to_path_progress(30.,0.)['index'])
        n=node([dict(x=0.,y=0.,path_type='connection'),
                dict(x=0.,y=0.,path_type='cover'),dict(x=30.,y=0.,path_type='cover')])
        self.assertEqual('cover',n._slice_path_points_by_arc(0,10)[0]['path_type'])

    def test_projection_window_and_clipped_endpoints(self):
        n=node(points([(0,0),(30,0),(30,10)]))
        result=n._project_pose_to_path_progress(20,0,25,35)
        self.assertEqual(25.,result['progress_s'])
        result=n._project_pose_to_path_progress(30,9,25,35)
        self.assertEqual(35.,result['progress_s'])
        self.assertEqual(1,result['index'])
        self.assertEqual(40.,n._project_pose_to_path_progress(40,10,100,200)['progress_s'])
        self.assertEqual(1,len(n._slice_path_points_by_arc(35,35)))
        self.assertEqual([],node([])._slice_path_points_by_arc(0,10))
        self.assertEqual(0,node(points([(1,2)]))._project_pose_to_path_progress(1,2)['index'])

    def test_spacing_validation(self):
        for value in (0,-1,.001,float('nan'),float('inf'),True):
            with self.assertRaises(ValueError): segment_point_spacing(value)
        self.assertEqual(.3,segment_point_spacing('.3'))
        self.assertEqual([],sample_segment([],.3))
        self.assertEqual(2,len(sample_segment(points([(0,0),(.30000000000018,0)]),.3)))

    def test_progress_uses_metres_not_corner_count(self):
        n=node(points([(0,0),(30,0),(30,10)]))
        n._active_segment_last_progress_s=15.
        self.assertAlmostEqual(.375,n._task_progress())
        n.current_path_index=1
        n._active_segment_last_progress_s=35.
        self.assertAlmostEqual(.875,n._task_progress())
        n.current_path_index=2
        self.assertEqual(1.,n._task_progress())

    def test_turn_merge_preserves_both_corners_and_lane_leadin(self):
        n=node(points([(0,0),(30,0),(30,1),(0,1)]))
        del n._should_insert_corner_mid
        n._corner_mid_enabled=True
        n._corner_mid_min_length=.05
        n._corner_mid_short_ratio=.6
        n._corner_mid_parallel_threshold_deg=35.
        n._corner_mid_turn_min_angle_deg=45.
        n._rebuild_active_segments()
        self.assertEqual(2,len(n._active_segments))
        first=n._active_segments[0]
        self.assertAlmostEqual(31.7,first['end_s'])
        sliced=n._slice_path_points_by_arc(first['start_s'],first['end_s'])
        self.assertEqual([(0.,0.),(30.,0.),(30.,1.),(29.3,1.)],[(p['x'],p['y']) for p in sliced])

    def test_resume_checkpoint_projects_inside_sparse_edge(self):
        n=node(points([(0,0),(30,0),(30,10)]))
        n._rebuild_active_segments()
        n.ros_data_bridge=SimpleNamespace(get_pose=lambda:dict(x=25.,y=.1))
        n._segment_overlap_behind_m=.3
        n._publish_active_segment_plan=mock.Mock()
        n._continuation_checkpoint=dict(path=n.current_path,segment_index=0,progress_s=25.)
        n._init_segment_execution_cursor()
        self.assertAlmostEqual(25.,n._active_segment_last_progress_s)
        self.assertAlmostEqual(24.7,n._active_segment_start_s)
        self.assertEqual(0,n.current_path_index)

    def test_compaction_never_smooths_detours_reversals_or_events(self):
        def planner_points(coords,tags=None):
            return [SimpleNamespace(row=y,col=x,path_type=(tags or ['A']*len(coords))[i],point_type='intermediate',time=0.)
                    for i,(x,y) in enumerate(coords)]
        p=planner_points([(0,0),(10,0),(20,0),(20,10),(20,0),(30,0)])
        compact=compact_planner_polyline(p)
        self.assertEqual([(0,0),(20,0),(20,10),(20,0),(30,0)],[(p.col,p.row) for p in compact])
        p=planner_points([(0,0),(10,0),(20,0)],['connection','A','A'])
        self.assertEqual(3,len(compact_planner_polyline(p)))
        p=planner_points([(0,0),(10,0),(10,0),(20,0)])
        self.assertEqual(4,len(compact_planner_polyline(p)))

    def test_cached_local_sampling_invalidates_on_window_spacing_and_version(self):
        n=node(points([(0,0),(30,0),(30,10)]))
        n._active_segments=[dict()]
        window=[(25.,35.,{})]
        n._active_segment_window_s=lambda:window[0]
        with mock.patch.object(n,'_build_navigation_path_from_segment_points',wraps=n._build_navigation_path_from_segment_points) as build:
            n._publish_active_segment_plan()
            message=n.active_segment_plan_pub.publish.call_args.args[0]
            n._publish_active_segment_plan()
            self.assertIs(message,n.active_segment_plan_pub.publish.call_args.args[0])
            self.assertEqual(1,build.call_count)
            self.assertGreater(len(message.poses),3)
            window[0]=(26.,35.,{})
            n._publish_active_segment_plan()
            n._active_segment_point_spacing=.1
            n._publish_active_segment_plan()
            n.current_path.path_version+=1
            n._publish_active_segment_plan()
            self.assertEqual(4,build.call_count)

    def test_legacy_goals_include_sparse_run_endpoints(self):
        n=node(points([(0,0),(30,0),(30,10),(0,10)]))
        n._rebuild_goal_points()
        self.assertEqual([1,2,3],[p['path_index'] for p in n._goal_points])
        n.current_path.points[0]['path_type']='connection'
        n._rebuild_goal_points()
        self.assertEqual('connection',n._goal_points[0]['path_type'])

    def test_legacy_progress_cannot_finish_at_last_edge_midpoint(self):
        n=node(points([(0,0),(30,0)]))
        n.state=SchedulerState.RUNNING
        n._active_segments=[]
        n._exec_goal_reach_dist=.5
        n._finish_path_execution=mock.Mock()
        pose=dict(x=20.,y=0.)
        n.ros_data_bridge=SimpleNamespace(get_pose=lambda:pose)
        n._update_progress()
        n._finish_path_execution.assert_not_called()
        self.assertAlmostEqual(2./3,n._task_progress())
        pose['x']=30.
        n._update_progress()
        n._finish_path_execution.assert_called_once()
        n=node(points([(0,0),(30,0),(0,0)]))
        n.state=SchedulerState.RUNNING
        n._active_segments=[]
        n._exec_goal_reach_dist=.5
        n._finish_path_execution=mock.Mock()
        n.ros_data_bridge=SimpleNamespace(get_pose=lambda:dict(x=0.,y=0.))
        n._update_progress()
        n._finish_path_execution.assert_not_called()


if __name__ == '__main__':
    unittest.main()
