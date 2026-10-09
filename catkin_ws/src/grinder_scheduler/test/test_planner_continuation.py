"""Actual adapter/core integration for partial laps and subsequent full repeats."""
import copy
import math
import unittest
from unittest import mock

from test_planner_preprocessing import adapter, fixture, mst
from grinder_scheduler.task_continuation import coverage_continuation


class PlannerContinuationTest(unittest.TestCase):
    def check_backend(self, native, at_turn=False):
        grid, info, config = fixture()
        config.region_repeat_config = {'A': 3, 'B': 1}
        config.work_regions[0]['global_direction'] = 'x'
        with mock.patch.object(mst, '_mst27_cpp', native):
            full = adapter().plan('task', grid, info, config)
            arc = 0.
            for index, (a, b) in enumerate(zip(full.points, full.points[1:])):
                length = math.hypot(b['x'] - a['x'], b['y'] - a['y'])
                turn_end = (at_turn and index + 2 < len(full.points)
                    and a['path_type'] == b['path_type'] == 'A__lap_1'
                    and b['x'] > a['x'] + .05 and abs(b['y'] - a['y']) < .01
                    and abs(full.points[index + 2]['y'] - b['y']) > .05)
                mid_even = (not at_turn and a['path_type'] == b['path_type'] == 'A__lap_2'
                    and b['x'] < a['x'] - .1 and abs(b['y'] - a['y']) < .01)
                if turn_end or mid_even:
                    progress = arc + (length if turn_end else .5 * length)
                    break
                arc += length
            else:
                self.fail('Fixture has no negative-x coverage line in even lap')
            continuation = coverage_continuation(full.points, progress, config.work_regions)
            self.assertEqual('-x', continuation['direction'])
            current_lap = 1 if at_turn else 2
            self.assertEqual(current_lap, continuation['lap'])
            resumed = copy.deepcopy(config)
            resumed.continuation = continuation
            resumed.current_pose = dict(continuation['start_pose'])
            resumed.start_pose = dict(continuation['start_pose'])
            path = adapter().plan('task', grid, info, resumed)
        types = {p['path_type'] for p in path.points}
        if not at_turn:
            self.assertNotIn('A__lap_1', types)
        self.assertTrue({'A__lap_2', 'A__lap_3', 'B__lap_1'} <= types)
        lane = [p for p in path.points if p['path_type'] == 'A__lap_{}'.format(current_lap)]
        first_edge = next((a, b) for a, b in zip(lane, lane[1:])
                          if abs(b['x'] - a['x']) > .1 and abs(b['y'] - a['y']) < .01)
        self.assertLess(first_edge[1]['x'], first_edge[0]['x'])
        # Full later lap must not be generated from the truncated resume start.
        original_lap = [(p['x'], p['y']) for p in full.points if p['path_type'] == 'A__lap_3']
        later_lap = [(p['x'], p['y']) for p in path.points if p['path_type'] == 'A__lap_3']
        self.assertEqual(original_lap, later_lap)
        # Rendering labels must belong to each lap, not be overwritten through
        # shared mutable PathPoint objects when later repeats are tagged.
        self.assertIn('A__lap_1', {p['path_type'] for p in full.points})

    def test_python_partial_even_lap_keeps_negative_direction_and_full_next_lap(self):
        self.check_backend(None)

    @unittest.skipIf(mst._mst27_cpp is None, 'native C++ extension unavailable')
    def test_cpp_partial_even_lap_keeps_negative_direction_and_full_next_lap(self):
        self.check_backend(mst._mst27_cpp)

    def test_python_lane_end_replan_starts_next_lane_in_negative_direction(self):
        self.check_backend(None, at_turn=True)

    @unittest.skipIf(mst._mst27_cpp is None, 'native C++ extension unavailable')
    def test_cpp_lane_end_replan_starts_next_lane_in_negative_direction(self):
        self.check_backend(mst._mst27_cpp, at_turn=True)


if __name__ == '__main__':
    unittest.main()
