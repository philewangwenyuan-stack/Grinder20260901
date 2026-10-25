"""Strict sweep spacing regressions; no ROS or hardware required."""
import contextlib
import importlib.util
import io
import math
from pathlib import Path
import sys
import unittest
from unittest import mock

import numpy as np

spec = importlib.util.spec_from_file_location(
    'mst27_spacing_test', str(Path(__file__).resolve().parents[1] / 'mst27.py'))
mst = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = mst
spec.loader.exec_module(mst)


class SpacingTest(unittest.TestCase):
    def check_path(self, path, start, end, angle, spacing, resolution):
        self.assertTrue(path)
        np.testing.assert_allclose([path[0].row, path[0].col], [start.row, start.col])
        np.testing.assert_allclose([path[-1].row, path[-1].col], [end.row, end.col])
        theta = math.radians(angle)
        dx, dy = math.cos(theta), math.sin(theta)
        nx, ny = -dy, dx
        # Measure actual straight work legs, not point distances or turn connectors.
        lanes = []
        travel = []
        for a, b in zip(path, path[1:]):
            dt = (b.col-a.col)*dx + (b.row-a.row)*dy
            dn = (b.col-a.col)*nx + (b.row-a.row)*ny
            if abs(dn) < 1e-6 and abs(dt) > 30:
                s = a.col*nx + a.row*ny
                if not lanes or abs(s-lanes[-1]) > 1e-6:
                    lanes.append(s)
                    travel.append(1 if dt > 0 else -1)
        self.assertGreaterEqual(len(lanes), 3)
        for a, b in zip(travel, travel[1:]):
            self.assertEqual(a, -b)
        gaps = np.abs(np.diff(lanes)) * resolution
        np.testing.assert_allclose(gaps[:-3], spacing, atol=1e-7)
        self.assertTrue(np.all(gaps > 1e-8))
        self.assertTrue(np.all(gaps <= spacing + 1e-7))
        self.assertAlmostEqual(sum(gaps), abs((end.col-start.col)*nx +
                                              (end.row-start.row)*ny)*resolution)
        # At most the final three edge intervals may differ from the parameter.
        for gap in gaps[:-3]:
            self.assertAlmostEqual(gap, spacing)
        self.assertEqual(len(lanes), len(set(round(s, 6) for s in lanes)))

    def run_cases(self, backend):
        for resolution in (.05, .06):
            for span_m in (2., 3., 5., 7.):
                for reverse in (False, True):
                    for same_side in (False, True):
                        for angle in (0., 30., 90.):
                            with self.subTest(backend=backend, resolution=resolution,
                                             span=span_m, reverse=reverse,
                                             same_side=same_side, angle=angle):
                                theta = math.radians(angle)
                                dx, dy = math.cos(theta), math.sin(theta)
                                nx, ny = -dy, dx
                                def point(t, s):
                                    return mst.Point(int(round(300+t*dy+s*ny)),
                                                     int(round(300+t*dx+s*nx)))
                                width = int(round(span_m/resolution))
                                lo, hi = ((width, 0) if reverse else (0, width))
                                start_t = 100 if reverse else -100
                                start = point(start_t, lo)
                                end = point(start_t if same_side else -start_t, hi)
                                boundary = [point(-120,-20),point(120,-20),
                                            point(120,width+20),point(-120,width+20)]
                                config = mst.RobotConfig(path_spacing=.7, overlap_ratio=0,
                                    inflation_radius=0, aligned_obstacle_inflation=0,
                                    endpoint_margin=0, output_point_spacing=0)
                                grid = np.zeros((650,650),dtype=np.uint8)
                                with contextlib.redirect_stdout(io.StringIO()):
                                    if backend == 'core':
                                        path = mst.ObstacleAwareBoustrophedonPlanner(config).plan_subregion_oriented(
                                            grid,boundary,angle,.7/resolution,
                                            start_point=start,end_point=end,
                                            endpoint_margin=0,first_travel_sign=-1 if reverse else 1)
                                    else:
                                        planner = mst.MultiStagePathPlanner(config)
                                        stage = mst.StageConfig(stage_id=1,boundary_polygon=boundary,
                                            start_point=start,end_point=end,direction='-x' if reverse else 'x',
                                            path_spacing=.7/resolution,direction_angle=angle)
                                        path = planner.plan(grid,boundary,[stage],1/resolution)
                                self.check_path(path,start,end,angle,.7,resolution)

    def test_narrow_and_single_lane_keep_exact_endpoints(self):
        core = mst.ObstacleAwareBoustrophedonPlanner(mst.RobotConfig(path_spacing=.7))
        boundary = [mst.Point(10,10),mst.Point(10,230),
                    mst.Point(180,230),mst.Point(180,10)]
        for end_row in (20, 22, 24):
            for end_col in (20, 220):
                with self.subTest(end_row=end_row, end_col=end_col):
                    start, end = mst.Point(20,20), mst.Point(end_row,end_col)
                    with contextlib.redirect_stdout(io.StringIO()):
                        path = core.plan_subregion_oriented(
                            np.zeros((200,250),dtype=np.uint8),boundary,0,14,
                            start_point=start,end_point=end,endpoint_margin=0,
                            first_travel_sign=1)
                    self.assertTrue(path)
                    self.assertEqual((path[0].row,path[0].col),(20,20))
                    self.assertEqual((path[-1].row,path[-1].col),(end_row,end_col))
                    self.assertTrue(all(20-1e-7 <= p.row <= end_row+1e-7 for p in path))

    def test_core_spacing_and_endpoints(self):
        self.run_cases('core')

    def test_python_pipeline_preserves_fractional_cell_spacing(self):
        with mock.patch.object(mst, '_mst27_cpp', None):
            self.run_cases('python')

    @unittest.skipIf(mst._mst27_cpp is None, 'native C++ extension unavailable')
    def test_native_cpp_spacing_and_endpoints(self):
        self.run_cases('cpp')


if __name__ == '__main__':
    unittest.main()
