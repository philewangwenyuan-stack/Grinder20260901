"""Adapter integration tests with ROS message stubs; never start a ROS node."""
import copy
import importlib.util
import pathlib
import sys
import types
import unittest
from unittest import mock

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[4]
SOURCE = ROOT / 'catkin_ws/src/grinder_scheduler/src'
sys.path.insert(0,str(SOURCE))
from grinder_scheduler.models import TaskConfigModel


def load_adapter(path):
    def pose_stamped():
        return types.SimpleNamespace(
            header=types.SimpleNamespace(frame_id=''),
            pose=types.SimpleNamespace(position=types.SimpleNamespace(x=0.,y=0.,z=0.),
                orientation=types.SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
    def nav_path():
        return types.SimpleNamespace(header=types.SimpleNamespace(frame_id=''),poses=[])
    rospy = types.ModuleType('rospy')
    for name in ('loginfo','logwarn','logerr','logwarn_throttle'):
        setattr(rospy,name,lambda *args,**kwargs:None)
    geometry = types.ModuleType('geometry_msgs.msg')
    geometry.PoseStamped = pose_stamped
    nav = types.ModuleType('nav_msgs.msg')
    nav.Path = nav_path
    with mock.patch.dict(sys.modules,{'rospy':rospy,'geometry_msgs.msg':geometry,'nav_msgs.msg':nav}):
        spec = importlib.util.spec_from_file_location('adapter_preprocessing_test',str(path))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


adapter_module = load_adapter(SOURCE/'grinder_scheduler/planner_adapter.py')
planner_path = ROOT/'third_party/path_planner/mst27/mst27.py'
spec = importlib.util.spec_from_file_location('mst27_adapter_preprocessing_test',str(planner_path))
mst = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = mst
spec.loader.exec_module(mst)


def adapter(module=adapter_module, planner=mst):
    result = module.PlannerAdapter.__new__(module.PlannerAdapter)
    result._planner_module = planner
    result._hybrid_module = None
    result._path_version = 0
    result._last_runtime_backend = ''
    return result


def fixture():
    grid = np.zeros((100,200),dtype=np.int16)
    grid[0,:] = grid[-1,:] = grid[:,0] = grid[:,-1] = 100
    grid[42:47,52:57] = 100
    regions = []
    for name,x0,x1 in (('A',1.,8.),('B',10.,17.)):
        regions.append(dict(region_id=name,points=[dict(x=x0,y=1.),dict(x=x1,y=1.),
            dict(x=x1,y=8.),dict(x=x0,y=8.)],enabled=True,
            start_pose=dict(x=x0+.4,y=1.4),end_pose=dict(x=x1-.4,y=7.6)))
    config = TaskConfigModel(work_regions=regions,selected_work_region_ids=['A','B'],
        vehicle_width=.4,vehicle_length=.6,default_path_spacing=.8,
        inflation_radius=.2,aligned_obstacle_inflation=.3,aligned_obstacle_max_extent=1.2,
        endpoint_margin=.2,overlap_ratio=0.,turn_radius=.1)
    info = dict(resolution=.1,origin_x=0.,origin_y=0.,frame_id='map',width=200,height=100)
    return grid,info,config


class AdapterPreprocessingTest(unittest.TestCase):
    def _plan_with_start(self, current, region_start, request_start=None, explicit=None, blocked=False):
        grid, info, config = fixture()
        if blocked:
            grid[:,90] = 100
        config.work_regions = config.work_regions[:1]
        config.selected_work_region_ids = []
        config.region_repeat_config = {}
        config.current_pose = current
        config.start_pose = request_start or {}
        region = config.work_regions[0]
        region['start_pose'] = region_start
        if explicit is not None:
            region['start_pose_explicit'] = explicit
        starts = []
        original = mst.MultiStagePathPlanner.plan
        def record(planner, grid, boundary, stages, *args, **kwargs):
            starts.append(stages[0].start_point)
            return original(planner, grid, boundary, stages, *args, **kwargs)
        with mock.patch.object(mst.MultiStagePathPlanner, 'plan', record):
            path = adapter().plan('preview', grid, info, config)
        self.assertTrue(path.points)
        return starts[0], path

    def test_preview_uses_robot_instead_of_legacy_default_corner(self):
        start, path = self._plan_with_start(dict(x=4.,y=2.), dict(x=1.,y=8.))
        self.assertEqual((20,40), (start.row,start.col))
        self.assertAlmostEqual(4., path.points[0]['x'])
        self.assertAlmostEqual(2., path.points[0]['y'])

    def test_explicit_request_start_overrides_robot_and_region_start(self):
        start, path = self._plan_with_start(dict(x=4.,y=2.), dict(x=2.,y=3.),
                                           request_start=dict(x=3.,y=2.))
        self.assertEqual((20,30), (start.row,start.col))
        self.assertAlmostEqual(3., path.points[0]['x'])
        self.assertAlmostEqual(2., path.points[0]['y'])

    def test_explicit_region_entry_kept_but_path_starts_at_robot(self):
        start, path = self._plan_with_start(dict(x=4.,y=2.), dict(x=2.,y=3.), explicit=True)
        self.assertEqual((30,20), (start.row,start.col))
        self.assertAlmostEqual(4., path.points[0]['x'])
        self.assertAlmostEqual(2., path.points[0]['y'])
        self.assertEqual('connection', path.points[0]['path_type'])

    def test_preview_connects_robot_outside_work_region_to_coverage(self):
        _, path = self._plan_with_start(dict(x=12.,y=2.), {})
        self.assertAlmostEqual(12., path.points[0]['x'])
        self.assertAlmostEqual(2., path.points[0]['y'])
        self.assertEqual('connection', path.points[0]['path_type'])

    def test_preview_refuses_approach_through_disconnected_wall(self):
        with self.assertRaisesRegex(ValueError, 'collision-free approach'):
            self._plan_with_start(dict(x=12.,y=2.), {}, blocked=True)

    def test_multi_region_shared_context_matches_legacy_and_rebuilds_next_request(self):
        grid,info,config = fixture()
        with mock.patch.object(mst,'_mst27_cpp',None):
            with mock.patch.object(mst.MultiStagePathPlanner,'prepare_map',None):
                legacy = adapter().plan('task',grid,info,copy.deepcopy(config))
            contexts = []
            prepare = mst.MultiStagePathPlanner.prepare_map
            def record(planner,*args,**kwargs):
                prepared = prepare(planner,*args,**kwargs)
                contexts.append(prepared)
                return prepared
            with mock.patch.object(mst.MultiStagePathPlanner,'prepare_map',record):
                current_adapter = adapter()
                current = current_adapter.plan('task',grid,info,copy.deepcopy(config))
                changed = grid.copy()
                changed[20:24,120:124] = 100
                current_adapter.plan('task',changed,info,copy.deepcopy(config))
        self.assertTrue(current.points)
        self.assertEqual(legacy.points,current.points)
        self.assertEqual(legacy.length_m,current.length_m)
        self.assertEqual(2,len(contexts))
        self.assertEqual(1,contexts[0].stats()['cache_misses'])
        self.assertGreaterEqual(contexts[0].stats()['cache_hits'],1)
        self.assertFalse(np.array_equal(contexts[0].raw_grid,contexts[1].raw_grid))
        self.assertEqual(100,grid[42,52])  # caller input was not modified

    def test_cross_request_cache_keeps_complete_paths_equal(self):
        from grinder_scheduler.planning_cache import PlanningCache
        grid, info, config = fixture()
        current = adapter()
        current._preprocessing_cache = PlanningCache()
        with mock.patch.object(mst, '_mst27_cpp', None):
            baseline = adapter().plan('task', grid, info, copy.deepcopy(config))
            cold = current.plan('task', grid, info, copy.deepcopy(config))
            warm = current.plan('task', grid, info, copy.deepcopy(config))
        self.assertEqual(baseline.points, cold.points)
        self.assertEqual(cold.points, warm.points)
        self.assertEqual(cold.length_m, warm.length_m)
        self.assertEqual(1, current._preprocessing_cache.hits)
        self.assertEqual(1, current._preprocessing_cache.misses)

    def test_legacy_planner_without_prepare_map_still_receives_independent_inputs(self):
        grid,info,config = fixture()
        with mock.patch.object(mst,'_mst27_cpp',None), mock.patch.object(mst.MultiStagePathPlanner,'prepare_map',None):
            path = adapter().plan('task',grid,info,copy.deepcopy(config))
        self.assertTrue(path.points)
        self.assertEqual('map',path.nav_path.header.frame_id)

    @unittest.skipIf(mst._mst27_cpp is None,'native C++ extension unavailable')
    def test_native_multi_region_paths_match_legacy_adapter_mode(self):
        grid,info,config = fixture()
        with mock.patch.object(mst.MultiStagePathPlanner,'prepare_map',None):
            legacy = adapter().plan('task',grid,info,copy.deepcopy(config))
        current = adapter().plan('task',grid,info,copy.deepcopy(config))
        self.assertTrue(current.points)
        self.assertEqual(legacy.points,current.points)
        self.assertEqual(legacy.length_m,current.length_m)


if __name__ == '__main__':
    unittest.main()
