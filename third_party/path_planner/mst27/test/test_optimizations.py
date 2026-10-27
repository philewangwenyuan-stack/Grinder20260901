"""Exact inflation geometry and safe U-turn connection regressions."""
import contextlib
import importlib.util
import io
import math
from pathlib import Path
import sys
import unittest
import numpy as np
from scipy import ndimage

spec = importlib.util.spec_from_file_location('mst27_optimization_test',
    str(Path(__file__).resolve().parents[1] / 'mst27.py'))
mst = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = mst
spec.loader.exec_module(mst)


class InflationTest(unittest.TestCase):
    def test_square_dilation_identical_including_borders(self):
        rng = np.random.RandomState(721)
        for shape in ((1, 1), (7, 13), (45, 60)):
            grids = [np.zeros(shape, dtype=np.uint8), np.ones(shape, dtype=np.uint8),
                     (rng.random_sample(shape) < .04).astype(np.uint8)]
            for grid in grids:
                for radius in (0, 1, 3, 14, 30):
                    with self.subTest(shape=shape, radius=radius):
                        expected = (grid.copy() if radius == 0 else ndimage.binary_dilation(
                            grid, structure=np.ones((2*radius+1, 2*radius+1), bool)))
                        with contextlib.redirect_stdout(io.StringIO()):
                            actual = mst.ObstacleProcessor.inflate_axis_aligned_obstacles(grid, radius)
                        np.testing.assert_array_equal(expected, actual)

    def test_collision_batch_matches_scalar_sampling_and_rounding(self):
        def reference(path, grid, step=.5):
            if not path: return False
            for i,(a,b) in enumerate(zip(path,path[1:])):
                count=max(1,int(math.ceil(math.hypot(b.row-a.row,b.col-a.col)/step)))
                for q in range(1,count+1):
                    if i==len(path)-2 and q==count:continue
                    u=q/count;r=int(round(a.row+(b.row-a.row)*u));c=int(round(a.col+(b.col-a.col)*u))
                    if not (0<=r<grid.shape[0] and 0<=c<grid.shape[1]) or grid[r,c]:return False
            return True
        rng=np.random.RandomState(37)
        paths=[[],[mst.Point(2,2)], [mst.Point(0,0),mst.Point(19,19)],
               [mst.Point(.5,.5),mst.Point(18.5,18.5),mst.Point(2.5,18.5)],
               [mst.Point(-1,2),mst.Point(25,22)]]
        paths += [[mst.Point(*xy) for xy in rng.uniform(-2,22,(5,2))] for _ in range(100)]
        for _ in range(5):
            grid=(rng.random_sample((20,20))<.02).astype(np.uint8)
            for path in paths:
                self.assertEqual(reference(path,grid),mst.MultiStagePathPlanner._path_collision_free(path,grid))
        # Cross the chunk boundary with an obstacle at the far end.
        grid=np.zeros((1,40000),np.uint8);grid[0,35000]=1
        path=[mst.Point(0,0),mst.Point(0,39999)]
        self.assertFalse(mst.MultiStagePathPlanner._path_collision_free(path,grid))


@unittest.skipIf(mst._mst27_cpp is None, 'native extension unavailable')
class NativeTurnTest(unittest.TestCase):
    def plan(self, angle, blocked=False, margin=10.):
        theta = math.radians(angle)
        def point(t, s):
            return mst.Point(int(round(300+t*math.sin(theta)+s*math.cos(theta))),
                             int(round(300+t*math.cos(theta)-s*math.sin(theta))))
        boundary = [point(-120,-20), point(120,-20), point(120,180), point(-120,180)]
        start, end = point(-100,0), point(100,160)
        grid = np.zeros((650,650), dtype=np.int32)
        if blocked:
            grid[320:370,380:405] = 1
        mask = mst.ObstacleProcessor.polygon_to_mask(boundary, grid.shape, radius=-.5)
        stage = dict(stage_id=1,boundary=[(p.row,p.col) for p in boundary],
                     start_point=(start.row,start.col),end_point=(end.row,end.col),
                     direction='x',direction_angle=angle,region_mask=mask,
                     endpoint_margin_pixels=margin)
        result = mst._mst27_cpp.plan_core_profiled(grid,[stage],14.,0.)
        self.assertEqual(result['path'], mst._mst27_cpp.plan_core(grid,[stage],14.,0.))
        result['stage'] = stage
        return result, grid, mask, start, end

    def test_open_turns_have_no_retrace_and_profile_matches_plain_api(self):
        for angle in (0., 1.17857, 30., 90.):
            result, grid, mask, start, end = self.plan(angle)
            path = result['path']
            self.assertTrue(path)
            np.testing.assert_allclose(path[0][:2], (start.row,start.col))
            np.testing.assert_allclose(path[-1][:2], (end.row,end.col))
            for a,b,c in zip(path,path[1:],path[2:]):
                u=np.array(b[:2])-a[:2];v=np.array(c[:2])-b[:2]
                denominator=np.linalg.norm(u)*np.linalg.norm(v)
                if denominator>1e-8:
                    self.assertGreater(np.dot(u,v)/denominator, -.999,
                                       'open U-turn retraces a work lane')
            for value in result['timings_ms'].values():
                self.assertTrue(math.isfinite(value) and value >= 0)

    def test_tight_boundary_keeps_safe_fallback_instead_of_cutting_work(self):
        result, grid, mask, start, end = self.plan(1.17857, margin=0.)
        self.assertTrue(result['path'])
        # With no headland an outer connection can leave the region. Keeping a
        # retrace is preferable to erasing the existing complete work-lane end.
        for a,b in zip(result['path'],result['path'][1:]):
            count=max(1,int(math.ceil(math.hypot(b[0]-a[0],b[1]-a[1])/.5)))
            for f in np.linspace(0,1,count+1):
                r=int(round(a[0]+f*(b[0]-a[0])));c=int(round(a[1]+f*(b[1]-a[1])))
                self.assertTrue(mask[r,c])

    def test_obstacles_never_bypassed_by_outer_turn(self):
        result, grid, mask, start, end = self.plan(1.17857, blocked=True)
        spec = result['stage']
        stage = mst.StageConfig(stage_id=1,
            boundary_polygon=[mst.Point(*p) for p in spec['boundary']],
            start_point=start, end_point=end, direction='x', path_spacing=14.,
            direction_angle=1.17857, endpoint_margin=.5)
        planner = mst.MultiStagePathPlanner(mst.RobotConfig(
            path_spacing=.7, overlap_ratio=0, inflation_radius=0,
            aligned_obstacle_inflation=0, endpoint_margin=.5, output_point_spacing=0))
        with contextlib.redirect_stdout(io.StringIO()):
            path = planner.plan(grid,stage.boundary_polygon,[stage],20.)
        # A constrained unsafe candidate may be rejected by the final production
        # check. It must never be delivered just to eliminate a reversal.
        for a,b in zip(path,path[1:]):
            count=max(1,int(math.ceil(math.hypot(b.row-a.row,b.col-a.col)/.25)))
            for f in np.linspace(0,1,count+1):
                r=int(round(a.row+f*(b.row-a.row)));c=int(round(a.col+f*(b.col-a.col)))
                self.assertEqual(0, grid[r,c])
                self.assertTrue(mask[r,c])


if __name__ == '__main__':
    unittest.main()
