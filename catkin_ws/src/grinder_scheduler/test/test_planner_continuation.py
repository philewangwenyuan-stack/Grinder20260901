"""Actual adapter/core integration for partial laps and subsequent full repeats."""
import copy
import math
import unittest
from unittest import mock

from test_planner_preprocessing import adapter, fixture, mst
from grinder_scheduler.task_continuation import coverage_continuation
from test_sparse_segments import node as segment_node


class PlannerContinuationTest(unittest.TestCase):
    def check_same_lap_turn(self, native):
        for direction in ('x', '-x', 'y', '-y'):
            for angle in (0., 30.):
                with self.subTest(direction=direction, angle=angle):
                    grid, info, config = fixture()
                    grid[1:-1, 1:-1] = 0  # Isolate a regular lane turn from detours.
                    config.work_regions = config.work_regions[:1]
                    config.selected_work_region_ids = ['A']
                    config.region_repeat_config = {'A': 1}
                    config.work_regions[0]['global_direction'] = direction
                    config.planning_angle_deg = angle  # Production always supplies this.
                    # Rotate the region with its sweep axes so each lane has the
                    # same extent; unequal boundary lengths add valid turn extensions.
                    rotation = math.radians(angle)
                    def rotate(point):
                        x, y = .7 * (point['x'] - 4.5), .7 * (point['y'] - 4.5)
                        return dict(x=4.5 + x * math.cos(rotation) - y * math.sin(rotation),
                                    y=4.5 + x * math.sin(rotation) + y * math.cos(rotation))
                    region = config.work_regions[0]
                    region['points'] = [rotate(p) for p in region['points']]
                    region['start_pose'] = rotate(region['start_pose'])
                    region['end_pose'] = rotate(region['end_pose'])
                    theta = math.radians(angle + (90 if direction.endswith('y') else 0))
                    axis = (math.cos(theta), math.sin(theta))

                    def lanes(path):
                        result = []
                        arc = 0.
                        for a, b in zip(path.points, path.points[1:]):
                            dx, dy = b['x'] - a['x'], b['y'] - a['y']
                            length = math.hypot(dx, dy)
                            dot = dx * axis[0] + dy * axis[1]
                            if (a['path_type'] == b['path_type'] == 'A__lap_1'
                                    and length > .05 and abs(dot) / length > .98):
                                result.append((arc, length, dot, a, b))
                            arc += length
                        return result

                    with mock.patch.object(mst, '_mst27_cpp', native):
                        full = adapter().plan('task', grid, info, config)
                        full_lanes = lanes(full)
                        first = next(edge for edge in full_lanes if edge[1] > 1.)
                        second = next(edge for edge in full_lanes
                                      if edge[0] > first[0] and edge[2] * first[2] < 0.)
                        for lane, fraction in ((first, .5), (first, 1.), (second, .5)):
                            progress = lane[0] + fraction * lane[1]
                            checkpoint = coverage_continuation(
                                full.points, progress, config.work_regions, angle_deg=angle)
                            resumed = copy.deepcopy(config)
                            resumed.continuation = checkpoint
                            resumed.current_pose = dict(checkpoint['start_pose'])
                            resumed.start_pose = dict(checkpoint['start_pose'])
                            path = adapter().plan('task', grid, info, resumed)
                            edges = lanes(path)
                            self.assertGreaterEqual(len(edges), 2)
                            expected_sign = (1 if lane[2] > 0 else -1) * (1 if fraction < 1 else -1)
                            detail = 'fraction={} checkpoint={} edges={}'.format(
                                fraction, checkpoint, [(e[1], e[2], e[3]['x'], e[3]['y'],
                                                       e[4]['x'], e[4]['y']) for e in edges[:3]])
                            self.assertGreater(edges[0][2] * expected_sign, 0., detail)
                            self.assertLess(edges[0][2] * edges[1][2], 0., detail)
                            # Verify the actual ROS path builder also reverses heading.
                            n = segment_node(path.points)
                            for edge in edges[:2]:
                                msg = n._build_navigation_path_from_segment_points([edge[3], edge[4]])
                                q = msg.poses[0].pose.orientation
                                yaw = 2. * math.atan2(q.z, q.w)
                                self.assertGreater(math.cos(yaw) * (edge[4]['x'] - edge[3]['x'])
                                    + math.sin(yaw) * (edge[4]['y'] - edge[3]['y']), 0.)

    def test_python_same_lap_replan_reverses_next_lane_and_navigation_heading(self):
        self.check_same_lap_turn(None)

    @unittest.skipIf(mst._mst27_cpp is None, 'native C++ extension unavailable')
    def test_cpp_same_lap_replan_reverses_next_lane_and_navigation_heading(self):
        self.check_same_lap_turn(mst._mst27_cpp)

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
