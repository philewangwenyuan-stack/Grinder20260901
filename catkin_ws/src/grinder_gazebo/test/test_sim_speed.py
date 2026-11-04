"""Pure Python checks; no Gazebo instance, ROS master, build or hardware."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from grinder_gazebo.sim_speed import configure_speed


class SimSpeedTests(unittest.TestCase):
    def physics(self, step=.001):
        return SimpleNamespace(success=True, time_step=step, gravity=object(), ode_config=object())

    def test_double_speed_preserves_world_physics(self):
        physics = self.physics()
        setter = Mock(return_value=SimpleNamespace(success=True))
        self.assertEqual(configure_speed(2, lambda: physics, setter), 2000)
        setter.assert_called_once_with(time_step=.001, max_update_rate=2000,
                                       gravity=physics.gravity, ode_config=physics.ode_config)

    def test_custom_step_and_slow_motion_use_the_same_factor(self):
        for speed, step, rate in [(2, .002, 1000), (.5, .001, 500), (1, .001, 1000)]:
            with self.subTest(speed=speed, step=step):
                setter = Mock(return_value=SimpleNamespace(success=True))
                self.assertEqual(configure_speed(speed, lambda: self.physics(step), setter), rate)

    def test_invalid_speed_does_not_contact_gazebo(self):
        for speed in (0, -1, float('nan'), float('inf'), 'bad'):
            with self.subTest(speed=speed):
                getter = Mock(); setter = Mock()
                with self.assertRaises(ValueError): configure_speed(speed, getter, setter)
                getter.assert_not_called(); setter.assert_not_called()

    def test_failed_read_or_invalid_step_does_not_change_physics(self):
        for physics in (SimpleNamespace(success=False, status_message='not ready'), self.physics(0)):
            setter = Mock()
            with self.assertRaises((RuntimeError, ValueError)):
                configure_speed(2, lambda: physics, setter)
            setter.assert_not_called()

    def test_failed_write_is_reported(self):
        setter = Mock(return_value=SimpleNamespace(success=False, status_message='rejected'))
        with self.assertRaisesRegex(RuntimeError, 'rejected'):
            configure_speed(2, self.physics, setter)


if __name__ == '__main__':
    unittest.main()
