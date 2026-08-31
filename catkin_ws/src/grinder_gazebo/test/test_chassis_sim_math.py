import math
import unittest

from grinder_gazebo.chassis_sim_node import motor_rpm_to_twist, twist_to_motor_rpm


class ChassisSimMathTest(unittest.TestCase):
    def setUp(self):
        self.radius = 0.1475
        self.track = 0.70
        self.gear = 60.0

    def test_twist_round_trip(self):
        for linear, angular in ((0.0, 0.0), (0.2, 0.0), (0.0, 0.15), (-0.1, -0.1)):
            left, right = twist_to_motor_rpm(linear, angular, self.radius, self.track, self.gear)
            actual_linear, actual_angular = motor_rpm_to_twist(
                left, right, self.radius, self.track, self.gear
            )
            self.assertAlmostEqual(actual_linear, linear, places=9)
            self.assertAlmostEqual(actual_angular, angular, places=9)

    def test_straight_motor_speed(self):
        linear, angular = motor_rpm_to_twist(600, 600, self.radius, self.track, self.gear)
        expected = 600.0 * 2.0 * math.pi * self.radius / (60.0 * self.gear)
        self.assertAlmostEqual(linear, expected, places=9)
        self.assertAlmostEqual(angular, 0.0, places=9)

    def test_invalid_geometry(self):
        with self.assertRaises(ValueError):
            twist_to_motor_rpm(0.0, 0.0, 0.0, self.track, self.gear)


if __name__ == "__main__":
    unittest.main()

