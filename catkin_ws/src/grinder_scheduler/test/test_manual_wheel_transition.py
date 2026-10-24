"""Deterministic wheel-level transition and longitudinal-bound regressions."""
import sys
from pathlib import Path
import random
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from grinder_scheduler.manual_wheel_transition import ManualWheelTransition


class WheelTransitionTest(unittest.TestCase):
    def test_forward_steering_does_not_inject_a_whole_vehicle_stop(self):
        c = ManualWheelTransition(1.0)
        self.assertEqual((-41, -12), c.apply(-41, -12, 10, 10, 100))
        for tick in (10.1, 10.2, 10.3, 10.4, 10.5):
            left, right = c.apply(-63, 49, tick, tick, 100)
            self.assertNotEqual((0, 0), (left, right))
            self.assertEqual(-14, left + right)
        c.observe(10.51, 10.51, -14, 0, -14, 0, True)
        c.observe(10.61, 10.61, -14, 0, -14, 0, True)
        left, right = c.apply(-63, 49, 10.7, 10.7, 100)
        self.assertGreater(right, 0)
        self.assertEqual(-14, left + right)

    def test_both_steering_sides_preserve_forward_component(self):
        for side in (-1, 1):
            for direction in (-1, 1):
                c = ManualWheelTransition(1)
                start = [41 * direction, 12 * direction]
                goal = [63 * direction, -49 * direction]
                if side < 0:
                    start.reverse(); goal.reverse()
                c.apply(*start, 10, 10, 100)
                for i in range(1, 25):
                    tick = 10 + i / 10
                    output = c.apply(*goal, tick, tick, 100)
                    self.assertNotEqual((0, 0), output)
                    self.assertEqual(sum(goal), sum(output))
                self.assertEqual(tuple(goal), output)

    def test_full_steering_never_retains_straight_motion(self):
        c = ManualWheelTransition(1)
        c.apply(-33, -33, 10, 10, 100)
        for i in range(1, 25):
            tick = 10 + i / 10
            output = c.apply(-70, 70, tick, tick, 100)
            self.assertEqual(0, sum(output))
        self.assertEqual((-70, 70), output)

    def test_cached_invalid_stale_or_moving_feedback_cannot_release_a_wheel(self):
        for case in ('duplicate', 'old', 'invalid', 'moving', 'target', 'stale'):
            c = ManualWheelTransition(1)
            c.apply(-41, -12, 10, 10, 100)
            c.apply(-63, 49, 10.1, 10.1, 100)
            c.apply(-63, 49, 10.2, 10.2, 100)
            for tick in (10.3, 10.4):
                stamp = 10.3 if case == 'duplicate' else 10 if case == 'old' else tick
                c.observe(stamp, tick, -14, 1 if case == 'moving' else 0,
                          -14, 1 if case == 'target' else 0, case != 'invalid')
            now = 10.95 if case == 'stale' else 10.5
            self.assertEqual(0, c.apply(-63, 49, now, now, 100)[1])

    def test_whole_direction_reversal_keeps_full_timed_stop(self):
        c = ManualWheelTransition(1)
        c.apply(-33, -33, 10, 10, 100)
        self.assertEqual((0, 0), c.apply(33, 33, 10.1, 10.1, 100))
        c.observe(10.2, 10.2, 0, 0, 0, 0, True)
        c.observe(10.3, 10.3, 0, 0, 0, 0, True)
        self.assertEqual((0, 0), c.apply(33, 33, 10.5, 10.5, 100))
        out = c.apply(33, 33, 11.11, 11.11, 100)
        self.assertTrue(out[0] > 0 and out[1] > 0)

    def test_explicit_stop_is_immediate_and_reset_discards_all_pending_motion(self):
        c = ManualWheelTransition(1)
        c.apply(-41, -12, 10, 10, 100)
        c.apply(-63, 49, 10.1, 10.1, 100)
        self.assertEqual((0, 0), c.apply(0, 0, 10.11, 10.11, 100))
        c.reset()
        self.assertTrue(all(w.output == 0 and not w.accelerating for w in c.wheels))

    def test_reducing_steering_cannot_overspeed_other_wheel_to_preserve_translation(self):
        c = ManualWheelTransition(1)
        c.apply(-63, 49, 10, 10, 100)
        for i in range(1, 25):
            tick = 10 + i / 10
            left, right = c.apply(-41, -12, tick, tick, 100)
            self.assertNotEqual((0, 0), (left, right))
            self.assertLessEqual(abs(left), 41)
            self.assertLessEqual(abs(left + right), 53)
        self.assertEqual((-41, -12), (left, right))

    def test_renewal_rate_cannot_accelerate_transition_without_elapsed_time(self):
        c = ManualWheelTransition(1)
        c.apply(-41, -12, 10, 10, 100)
        c.apply(-63, 49, 10.1, 10.1, 100)
        c.apply(-63, 49, 10.2, 10.2, 100)
        c.observe(10.3, 10.3, -14, 0, -14, 0, True)
        c.observe(10.4, 10.4, -14, 0, -14, 0, True)
        first = c.apply(-63, 49, 10.4, 10.4, 100)
        for _ in range(100):
            self.assertEqual(first, c.apply(-63, 49, 10.4, 10.4, 100))

    def test_random_motion_never_directly_flips_a_nonzero_wheel_or_exceeds_linear_target(self):
        rng = random.Random(123)
        c = ManualWheelTransition(1)
        previous = (0, 0)
        for i in range(1000):
            steering = rng.uniform(-1, 1)
            direction = rng.choice((-1, 1))
            linear = direction * 33 * (1 - abs(steering))
            goal = (round(linear - steering * 70), round(linear + steering * 70))
            output = c.apply(*goal, i / 10, i / 10, 100)
            for old, new in zip(previous, output):
                self.assertGreaterEqual(old * new, 0)
                self.assertLessEqual(abs(new), 100)
            self.assertLessEqual(abs(sum(output)), abs(sum(goal)))
            self.assertGreaterEqual(sum(output) * sum(goal), 0)
            previous = output


if __name__ == '__main__':
    unittest.main()
