"""Uniform, bounded sweep spacing regressions; no ROS or hardware required."""
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
    def check_path(self, path, start, end, angle, spacing, resolution,
                   first_sign, last_sign, spacing_cells):
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
        self.assertEqual(travel[0], first_sign)
        self.assertEqual(travel[-1], last_sign)
        gaps = np.abs(np.diff(lanes)) * resolution
        span_cells = abs((end.col*nx + end.row*ny) -
                         (start.col*nx + start.row*ny))
        span = span_cells * resolution
        intervals = max(1, int(math.ceil(round(span_cells / spacing_cells, 12))))
        if (intervals % 2 == 0) != (first_sign == last_sign):
            intervals += 1
        self.assertEqual(len(gaps), intervals)
        np.testing.assert_allclose(gaps, span / intervals, rtol=0, atol=1e-7)
        self.assertTrue(np.all(gaps > 1e-8))
        self.assertTrue(np.all(gaps <= spacing + 1e-7))
        self.assertAlmostEqual(sum(gaps), span)
        self.assertEqual(len(lanes), len(set(round(s, 6) for s in lanes)))

    def run_cases(self, backend):
        for resolution in (.05, .06):
            for span_m in (2., 3., 4.9, 5., 7.):
                for reverse in (False, True):
                    for same_side in (False, True):
                        for angle in (-37., 0., 17.5, 30., 90., 137.):
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
                                first_sign = -1 if reverse else 1
                                self.check_path(path,start,end,angle,.7,resolution,
                                                first_sign,
                                                -first_sign if same_side else first_sign,
                                                .7/resolution if backend == 'core'
                                                else .7*(1/resolution))

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

    def test_span_just_above_multiple_does_not_round_interval_count_down(self):
        config = mst.RobotConfig(path_spacing=.7, endpoint_margin=0)
        core = mst.ObstacleAwareBoustrophedonPlanner(config)
        start, end = mst.Point(20, 20), mst.Point(118 + 1e-8, 20)
        boundary = [mst.Point(10,10), mst.Point(10,230),
                    mst.Point(180,230), mst.Point(180,10)]
        with contextlib.redirect_stdout(io.StringIO()):
            path = core.plan_subregion_oriented(
                np.zeros((200,250), dtype=np.uint8), boundary, 0, 14.,
                start_point=start, end_point=end, endpoint_margin=0,
                first_travel_sign=1)
        self.check_path(path, start, end, 0, .7, .05, 1, -1, 14.)

    def test_exact_multiple_does_not_add_lanes_due_to_float_roundoff(self):
        core = mst.ObstacleAwareBoustrophedonPlanner(mst.RobotConfig(path_spacing=.7))
        start, end = mst.Point(20, 20), mst.Point(118, 20)
        boundary = [mst.Point(10,10), mst.Point(10,230),
                    mst.Point(180,230), mst.Point(180,10)]
        with contextlib.redirect_stdout(io.StringIO()):
            path = core.plan_subregion_oriented(
                np.zeros((200,250), dtype=np.uint8), boundary, 0, .7/.05,
                start_point=start, end_point=end, endpoint_margin=0,
                first_travel_sign=1)
        self.check_path(path, start, end, 0, .7, .05, 1, -1, .7/.05)

    def test_python_pipeline_preserves_fractional_cell_spacing(self):
        with mock.patch.object(mst, '_mst27_cpp', None):
            self.run_cases('python')

    @unittest.skipIf(mst._mst27_cpp is None, 'native C++ extension unavailable')
    def test_native_cpp_spacing_and_endpoints(self):
        self.run_cases('cpp')


def pillar_fixture(angle, overlap=0.0):
    """Rotate both boundary and three columns; dimensions are cells at 5 cm."""
    theta = math.radians(angle)
    dx, dy = math.cos(theta), math.sin(theta)
    nx, ny = -dy, dx

    def point(t, s):
        return mst.Point(int(round(300 + t*dy + s*ny)),
                         int(round(300 + t*dx + s*nx)))

    boundary = [point(-110, -15), point(110, -15),
                point(110, 115), point(-110, 115)]
    start, end = point(-90, 10), point(90, 90)
    grid = np.zeros((650, 650), dtype=np.uint8)
    for t, s in ((-15, 30), (25, 55), (-15, 75)):
        column = [point(t-6, s-6), point(t+6, s-6),
                  point(t+6, s+6), point(t-6, s+6)]
        grid |= mst.ObstacleProcessor.polygon_to_mask(column, grid.shape)
    config = mst.RobotConfig(width=.4, length=.6, path_spacing=.7,
        overlap_ratio=overlap, inflation_radius=.2,
        aligned_obstacle_inflation=.2, aligned_obstacle_max_extent=1.5,
        endpoint_margin=.5, output_point_spacing=0)
    stage = mst.StageConfig(stage_id=1, boundary_polygon=boundary,
        start_point=start, end_point=end, direction='x', path_spacing=14.,
        direction_angle=angle, endpoint_margin=.5)
    return grid, boundary, stage, config


class PillarSpacingTest(unittest.TestCase):
    def run_cases(self, native):
        for angle in (-37., 0., 17.5, 45., 90., 137.):
            for overlap in (0., .15):
                with self.subTest(native=native, angle=angle, overlap=overlap):
                    grid, boundary, stage, config = pillar_fixture(angle, overlap)
                    backend = mst._mst27_cpp if native else None
                    with mock.patch.object(mst, '_mst27_cpp', backend):
                        with contextlib.redirect_stdout(io.StringIO()):
                            planner = mst.MultiStagePathPlanner(config)
                            prepared = planner.prepare_map(grid, 20.)
                            path = planner.plan(prepared.raw_grid, boundary, [stage], 20.,
                                                prepared_map=prepared)
                    if native:
                        self.assertIn('sweep_ms', planner.last_profile,
                                      'native test must not silently use Python fallback')
                    self.assertTrue(path, 'pillar test must produce a nonempty path')
                    np.testing.assert_allclose((path[0].row, path[0].col),
                                               (stage.start_point.row, stage.start_point.col))
                    np.testing.assert_allclose((path[-1].row, path[-1].col),
                                               (stage.end_point.row, stage.end_point.col))
                    self.assertTrue(planner._path_collision_free(
                        path, prepared.inflated_grid, sample_step=.25))
                    mask = mst.ObstacleProcessor.polygon_to_mask(boundary, grid.shape, radius=-.5)
                    for a, b in zip(path, path[1:]):
                        count = max(1, int(math.ceil(math.hypot(b.row-a.row, b.col-a.col)/.25)))
                        u = np.linspace(0, 1, count+1)
                        rows = np.rint(a.row + u*(b.row-a.row)).astype(int)
                        cols = np.rint(a.col + u*(b.col-a.col)).astype(int)
                        self.assertTrue(np.all(mask[rows[1:-1], cols[1:-1]]))

                    # In a clear cross-section before all columns, every work
                    # lane must still be present at its uniform nominal offset.
                    theta = math.radians(angle)
                    dx, dy = math.cos(theta), math.sin(theta)
                    nx, ny = -dy, dx
                    origin_t = 300*dx + 300*dy
                    gate = origin_t - 70
                    lanes = []
                    for a, b in zip(path, path[1:]):
                        at, bt = a.col*dx+a.row*dy, b.col*dx+b.row*dy
                        sa, sb = a.col*nx+a.row*ny, b.col*nx+b.row*ny
                        if abs(sa-sb) < 1e-6 and min(at, bt) <= gate <= max(at, bt):
                            lanes.append(sa)
                    lanes = np.array(sorted(set(round(s, 7) for s in lanes)))
                    s0 = stage.start_point.col*nx + stage.start_point.row*ny
                    s1 = stage.end_point.col*nx + stage.end_point.row*ny
                    maximum = config.path_spacing*(1-overlap)*20
                    intervals = int(math.ceil(abs(s1-s0)/maximum))
                    if intervals % 2:
                        intervals += 1  # both first and last travel toward +t
                    expected = np.linspace(s0, s1, intervals+1)
                    np.testing.assert_allclose(lanes, expected, rtol=0, atol=1e-6)
                    self.assertTrue(np.all(np.diff(lanes) <= maximum+1e-6))

    def test_python_rotated_pillars_and_overlap(self):
        self.run_cases(False)

    @unittest.skipIf(mst._mst27_cpp is None, 'native C++ extension unavailable')
    def test_native_rotated_pillars_and_overlap(self):
        self.run_cases(True)


if __name__ == '__main__':
    unittest.main()
