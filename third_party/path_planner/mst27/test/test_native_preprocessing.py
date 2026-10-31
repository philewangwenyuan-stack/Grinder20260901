"""Differential geometry and byte-boundary tests, without ROS/hardware."""
import concurrent.futures
import contextlib
import importlib.util
import io
import pathlib
import sys
import unittest
from unittest import mock

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('mst_native_preprocessing', ROOT / 'mst27.py')
mst = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = mst
spec.loader.exec_module(mst)
native = mst._native_preprocessing()


@unittest.skipIf(native is None, 'new native preprocessing extension unavailable')
class NativePreparationTest(unittest.TestCase):
    def reference(self, grid, angle, aligned, base, extent):
        with mock.patch.object(mst, '_mst27_cpp', None), contextlib.redirect_stdout(io.StringIO()):
            return mst.ObstacleProcessor.build_angle_aligned_inflated(grid, angle, aligned, base, extent)

    def assert_equivalent(self, grid, angle, aligned, base, extent):
        original = grid.copy()
        expected = self.reference(grid, angle, aligned, base, extent)
        actual = native.prepare_stage_grid(grid, angle, aligned, base, extent or 0)
        np.testing.assert_array_equal(expected, actual['grid'])
        np.testing.assert_array_equal(original, grid)
        self.assertEqual(np.uint8, actual['grid'].dtype)
        self.assertTrue(actual['grid'].flags.c_contiguous)
        for value in actual['timings_ms'].values():
            self.assertTrue(np.isfinite(value) and value >= 0)

    def test_empty_full_thin_maps_large_radii_and_angles(self):
        rng = np.random.RandomState(3401)
        for shape in [(1, 1), (1, 23), (19, 1), (17, 31)]:
            for grid in [np.zeros(shape, np.uint8), np.ones(shape, np.uint8),
                         (rng.random_sample(shape) < .15).astype(np.uint8)]:
                for angle in [0., 90., 45., -30., 30.000001, 210.]:
                    for aligned, base in [(0, 0), (2, 3), (40, 40)]:
                        self.assert_equivalent(grid, angle, aligned, base, 5)

    def test_random_components_and_angle_boundaries(self):
        rng = np.random.RandomState(9009)
        for _ in range(90):
            grid = (rng.random_sample((rng.randint(10, 90), rng.randint(10, 100))) <
                    rng.choice([.02, .12, .45])).astype(np.uint8)
            self.assert_equivalent(grid, rng.uniform(-720, 720), rng.randint(0, 8),
                                   rng.randint(0, 8), rng.choice([None, 1, 8, 30]))

    def test_columns_walls_border_and_diagonal_connectivity(self):
        grid = np.zeros((130, 170), np.uint8)
        grid[0, 5:120] = 1                      # border-connected wall
        grid[1:40, 6] = 1
        grid[50:53, 10:130] = 1                # oversized internal wall
        grid[75:83, 45:50] = 1                 # column
        grid[90:96, 100:109] = 1
        grid[20, 150] = grid[21, 151] = 1       # same 8-connected component
        for angle in [0., 1e-8, 30., 45., 89.999999, 90., -90., 180.]:
            for extent in [None, 1, 15, 100]:
                self.assert_equivalent(grid, angle, 6, 4, extent)

    def test_disk_transform_matches_scipy_including_exact_circle_boundary(self):
        rng = np.random.RandomState(709)
        for shape in [(1, 19), (29, 1), (31, 43), (300, 410)]:
            grid = (rng.random_sample(shape) < .01).astype(np.uint8)
            for radius in [0, 1, 2, 5, 17, 600]:
                with mock.patch.object(mst, '_mst27_cpp', None), contextlib.redirect_stdout(io.StringIO()):
                    expected = mst.ObstacleProcessor.inflate_obstacles(grid, radius)
                np.testing.assert_array_equal(expected, native.inflate_obstacles(grid, radius))
        grid = np.zeros((13, 15), np.uint8)
        grid[6, 7] = 1
        with mock.patch.object(mst, '_mst27_cpp', None), contextlib.redirect_stdout(io.StringIO()):
            expected = mst.ObstacleProcessor.inflate_obstacles(grid, 2.5)
        with contextlib.redirect_stdout(io.StringIO()):
            actual = mst.ObstacleProcessor.inflate_obstacles(grid, 2.5)
        np.testing.assert_array_equal(expected, actual)  # never truncate a fractional safety radius

    def test_uint8_int32_and_noncontiguous_callers_produce_identical_paths(self):
        grid = np.zeros((60, 80), np.uint8)
        grid[28:32, 35:39] = 1
        stage = dict(stage_id=1, boundary=[(5,5),(5,74),(54,74),(54,5)],
                     start_point=(8,8), end_point=(50,70), direction='x',
                     direction_angle=17., region_mask=np.ones_like(grid, dtype=bool),
                     planning_grid=grid)
        expected = native.plan_core(grid.astype(np.int32), [stage], 8., 0.)
        self.assertTrue(expected)
        for array in [grid, np.asfortranarray(grid), grid.astype(np.int32)]:
            stage['planning_grid'] = array
            self.assertEqual(expected, native.plan_core(array, [stage], 8., 0.))
        grid[0,0] = 1
        # Non-byte numeric inputs retain the historical int32 conversion,
        # never wrap occupied 256 to free 0 via an unchecked uint8 cast.
        np.testing.assert_array_equal(np.ones((1,1), np.uint8),
                                      native.inflate_obstacles(np.array([[256]], np.int32), 1))

    def test_parallel_requests_own_buffers_and_have_identical_results(self):
        grid = np.zeros((230, 310), np.uint8)
        grid[35:50, 50:67] = 1
        grid[90:94, 10:260] = 1
        grid.setflags(write=False)
        angles = [0., 30., 45., 89., -30., 90.]
        expected = [self.reference(grid, a, 5, 3, 25) for a in angles]
        with concurrent.futures.ThreadPoolExecutor(4) as pool:
            results = list(pool.map(lambda a: native.prepare_stage_grid(grid, a, 5, 3, 25)['grid'], angles))
        for a, b in zip(expected, results):
            np.testing.assert_array_equal(a, b)
        results[0].fill(0)
        self.assertTrue(results[1].any())
        self.assertTrue(grid.any())

    def test_invalid_shape_and_nonfinite_angle_rejected(self):
        for grid in [np.zeros((0, 3), np.uint8), np.zeros(5, np.uint8)]:
            with self.assertRaises(RuntimeError):
                native.prepare_stage_grid(grid, 0, 1)
        for angle in [float('nan'), float('inf')]:
            with self.assertRaises(RuntimeError):
                native.prepare_stage_grid(np.zeros((5,5), np.uint8), angle, 1)


if __name__ == '__main__':
    unittest.main()
