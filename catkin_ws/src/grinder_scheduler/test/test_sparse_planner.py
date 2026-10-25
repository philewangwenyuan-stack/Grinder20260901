"""Sparse/dense planning geometry equivalence on both planner backends."""
import contextlib
import copy
import io
import math
import unittest
from unittest import mock

from test_planner_preprocessing import adapter, adapter_module, fixture, mst
from test_sparse_segments import node


class SparsePlannerTest(unittest.TestCase):
    def compare(self, backend):
        grid, info, config = fixture()
        config.region_repeat_config = {'A':2,'B':1}
        config.current_pose = dict(x=1.5,y=1.5)
        original_config = mst.RobotConfig
        seen_spacing = []
        def dense_config(*args, **kwargs):
            seen_spacing.append(kwargs['output_point_spacing'])
            kwargs['output_point_spacing'] = .3
            return original_config(*args, **kwargs)
        with mock.patch.object(mst,'_mst27_cpp',backend), contextlib.redirect_stdout(io.StringIO()):
            sparse=adapter().plan('task',grid,info,copy.deepcopy(config))
            # Emulate prior whole-route sampling while retaining the identical
            # algorithm, obstacle grid, request parameters and connector logic.
            with mock.patch.object(mst,'RobotConfig',dense_config), mock.patch.object(
                    adapter_module,'compact_planner_polyline',lambda p:p):
                dense=adapter().plan('task',grid,info,copy.deepcopy(config))
        self.assertTrue(sparse.points)
        self.assertTrue(all(x==0. for x in seen_spacing))
        self.assertLess(len(sparse.points),len(dense.points))
        self.assertAlmostEqual(sparse.length_m,dense.length_m,places=5)
        a,b=node(sparse.points),node(dense.points)
        for i in range(501):
            distance=sparse.length_m*i/500
            pa,pb=a._interpolate_path_point_at_s(distance),b._interpolate_path_point_at_s(distance)
            self.assertAlmostEqual(pa['x'],pb['x'],places=5)
            self.assertAlmostEqual(pa['y'],pb['y'],places=5)
            self.assertEqual(pa['path_type'],pb['path_type'])
        # Whole edges, including long sparse ones, must remain collision free.
        collision_grid=mst.ObstacleProcessor.inflate_obstacles((grid!=0).astype('uint8'),2)
        path=[mst.PathPoint(p['row'],p['col']) for p in sparse.points]
        self.assertTrue(mst.MultiStagePathPlanner._path_collision_free(path,collision_grid))
        for tag in ('A__lap_1','A__lap_2','B__lap_1'):
            self.assertTrue(any(p['path_type']==tag for p in sparse.points))

    def test_python_sparse_geometry_and_coverage_equal_dense(self):
        self.compare(None)

    @unittest.skipIf(mst._mst27_cpp is None,'native C++ extension unavailable')
    def test_cpp_sparse_geometry_and_coverage_equal_dense(self):
        self.compare(mst._mst27_cpp)


if __name__ == '__main__':
    unittest.main()
