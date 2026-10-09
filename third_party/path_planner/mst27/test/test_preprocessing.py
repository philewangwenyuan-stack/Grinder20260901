"""Offline regression tests for request-local preprocessing (no ROS/hardware)."""
import contextlib
import copy
import importlib.util
import io
import pathlib
import sys
import unittest
from unittest import mock

import numpy as np


PLANNER_DIR = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('mst27_preprocessing_test', str(PLANNER_DIR / 'mst27.py'))
mst = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = mst
spec.loader.exec_module(mst)


def config(**kwargs):
    values = dict(width=0.4, length=0.6, path_spacing=0.8,
                  inflation_radius=0.2, aligned_obstacle_inflation=0.3,
                  aligned_obstacle_max_extent=1.2, endpoint_margin=0.2)
    values.update(kwargs)
    return mst.RobotConfig(**values)


def grid_fixture():
    grid = np.zeros((100, 120), dtype=np.uint8)
    grid[0, :] = grid[-1, :] = 1
    grid[:, 0] = grid[:, -1] = 1
    grid[42:47, 52:57] = 1  # isolated column
    grid[65:68, 20:65] = 1  # long wall: preserve original contour
    return grid


def stage_fixture(direction='x', angle=None):
    return mst.StageConfig(
        stage_id=1,
        boundary_polygon=[mst.Point(6,6),mst.Point(6,112),mst.Point(92,112),mst.Point(92,6)],
        start_point=mst.Point(10,10), end_point=mst.Point(88,108),
        direction=direction, path_spacing=8, direction_angle=angle)


class PreprocessingTest(unittest.TestCase):
    def setUp(self):
        self.console = contextlib.redirect_stdout(io.StringIO())
        self.console.__enter__()
        self.addCleanup(self.console.__exit__, None, None, None)

    def test_snapshot_is_compact_readonly_and_owns_input(self):
        source = grid_fixture()
        prepared = mst.MultiStagePathPlanner(config()).prepare_map(source, 10)
        self.assertEqual(np.uint8, prepared.raw_grid.dtype)
        self.assertEqual(np.uint8, prepared.inflated_grid.dtype)
        self.assertFalse(np.shares_memory(source, prepared.raw_grid))
        source.fill(0)
        self.assertGreater(prepared.occupied_cells, 0)
        for grid in (prepared.raw_grid, prepared.inflated_grid, prepared.stage_grid(30)):
            with self.assertRaises(ValueError):
                grid[10,10] = 1

    def test_preprocessed_grids_match_uncached_clearances_and_angles(self):
        source = grid_fixture()
        for radius, alignment, extent in ((.2,.3,1.2),(.4,.1,.5),(.0,.0,3.0)):
            planner = mst.MultiStagePathPlanner(config(inflation_radius=radius,
                aligned_obstacle_inflation=alignment, aligned_obstacle_max_extent=extent))
            prepared = planner.prepare_map(source, 10)
            expected = mst.ObstacleProcessor.inflate_obstacles(source,int(radius*10))
            np.testing.assert_array_equal(expected,prepared.inflated_grid)
            for angle in (0,90,30,30.000001,-30,210):
                expected = mst.ObstacleProcessor.build_angle_aligned_inflated(
                    source,angle,int(round(alignment*10)),int(radius*10),int(round(extent*10)))
                np.testing.assert_array_equal(expected,prepared.stage_grid(angle))

    def test_repeated_regions_preprocess_once_and_cpp_conversion_is_reused(self):
        planner = mst.MultiStagePathPlanner(config())
        prepared = planner.prepare_map(grid_fixture(),10)
        with mock.patch.object(mst.ObstacleProcessor,'build_angle_aligned_inflated',
                wraps=mst.ObstacleProcessor.build_angle_aligned_inflated) as build:
            for _ in range(10):
                grid = prepared.stage_grid(30)
                cpp_grid = prepared.cpp_stage_grid(30,grid)
                cpp_base = prepared.cpp_inflated_grid()
            self.assertEqual(1,build.call_count)
            self.assertIs(grid,prepared.stage_grid(30))
            self.assertIs(cpp_grid,prepared.cpp_stage_grid(30,grid))
            self.assertIs(cpp_base,prepared.cpp_inflated_grid())
            self.assertEqual(np.int32,cpp_grid.dtype)

    def test_cache_capacity_evicts_without_changing_grid(self):
        source = grid_fixture()
        planner = mst.MultiStagePathPlanner(config())
        prepared = planner.prepare_map(source,10,max_cache_bytes=source.nbytes*3)
        original = prepared.stage_grid(30).copy()
        for angle in range(20):
            prepared.stage_grid(angle)
            prepared.cpp_inflated_grid()  # oversize: do not retain
            stats = prepared.stats()
            self.assertLessEqual(stats['cached_bytes'],stats['cache_limit_bytes'])
        np.testing.assert_array_equal(original,prepared.stage_grid(30))

    def test_requests_and_exact_angles_do_not_share_stale_results(self):
        source = grid_fixture()
        planner = mst.MultiStagePathPlanner(config())
        first = planner.prepare_map(source,10)
        angle_grid = first.stage_grid(30)
        self.assertIsNot(angle_grid,first.stage_grid(30.000001))
        source[20:30,20:30] = 1
        second = planner.prepare_map(source,10)
        self.assertIsNot(first.raw_grid,second.raw_grid)
        self.assertFalse(np.array_equal(first.inflated_grid,second.inflated_grid))

    def test_mismatched_map_clearances_and_unapplied_obstacles_are_rejected(self):
        planner = mst.MultiStagePathPlanner(config())
        prepared = planner.prepare_map(grid_fixture(),10)
        stage = stage_fixture()
        with self.assertRaises(ValueError):
            planner.plan(prepared.raw_grid.copy(),stage.boundary_polygon,[stage],10,prepared_map=prepared)
        with self.assertRaises(ValueError):
            planner.plan(prepared.raw_grid,stage.boundary_polygon,[stage],11,prepared_map=prepared)
        planner.robot_config.inflation_radius = .4
        with self.assertRaises(ValueError):
            planner.plan(prepared.raw_grid,stage.boundary_polygon,[stage],10,prepared_map=prepared)
        with self.assertRaises(ValueError):
            planner.prepare_map(np.array([[0,100]],dtype=np.int16),10)
        planner.robot_config.inflation_radius = .2
        stage.obstacle_polygons = [[mst.Point(1,1),mst.Point(1,2),mst.Point(2,2)]]
        with self.assertRaises(ValueError):
            planner.plan(prepared.raw_grid,stage.boundary_polygon,[stage],10,prepared_map=prepared)

    def _compare_paths(self, backend):
        obstacle_grid = grid_fixture()
        free_grid = obstacle_grid.copy()
        free_grid[1:-1,1:-1] = 0
        nonempty = 0
        for source in (free_grid,obstacle_grid):
            for direction,angle in (('x',None),('-x',None),('y',None),('-y',None),('x',30),('-x',30)):
                stage = stage_fixture(direction,angle)
                planner = mst.MultiStagePathPlanner(config())
                old = planner.plan(source.copy(),stage.boundary_polygon,[copy.deepcopy(stage)],10)
                prepared = planner.prepare_map(source,10)
                new = planner.plan(prepared.raw_grid,stage.boundary_polygon,[copy.deepcopy(stage)],10,prepared_map=prepared)
                serialize = lambda path: [(p.row,p.col,p.point_type,p.path_type) for p in path]
                self.assertEqual(serialize(old),serialize(new), (backend,direction,angle))
                if old:
                    nonempty += 1
                    self.assertTrue(planner._path_collision_free(new,prepared.inflated_grid))
        # Both accepted paths and preexisting collision rejections must match.
        self.assertGreaterEqual(nonempty,6,backend)

    def test_python_paths_and_collision_checks_are_unchanged(self):
        with mock.patch.object(mst,'_mst27_cpp',None):
            self._compare_paths('python')

    @unittest.skipIf(mst._mst27_cpp is None,'native C++ extension unavailable')
    def test_native_cpp_paths_and_collision_checks_are_unchanged(self):
        self._compare_paths('cpp')

    def test_cpp_fallback_can_read_immutable_cached_grids(self):
        backend = mock.Mock()
        backend.plan_core.side_effect = RuntimeError('injected backend failure')
        with mock.patch.object(mst,'_mst27_cpp',backend):
            self._compare_paths('cpp_failure_python_fallback')

    def test_uncached_oversize_arrays_preserve_full_path(self):
        source = grid_fixture()
        planner = mst.MultiStagePathPlanner(config())
        stage = stage_fixture()
        expected = planner.plan(source.copy(),stage.boundary_polygon,[copy.deepcopy(stage)],10)
        prepared = planner.prepare_map(source,10,max_cache_bytes=0)
        actual = planner.plan(prepared.raw_grid,stage.boundary_polygon,[copy.deepcopy(stage)],10,prepared_map=prepared)
        self.assertTrue(expected)
        self.assertEqual([(p.row,p.col,p.path_type) for p in expected],
                         [(p.row,p.col,p.path_type) for p in actual])
        self.assertEqual(0,prepared.stats()['cached_bytes'])


if __name__ == '__main__':
    unittest.main()
