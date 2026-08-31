import unittest

from grinder_chassis_driver.register_map import (
    DISC_ENABLE_OFF,
    DISC_ENABLE_ON,
    ChassisCommand,
    RegisterSnapshot,
    clamp,
    motor_rpm_to_base_twist,
    to_int16,
    to_uint16,
)


class RegisterMapTest(unittest.TestCase):
    def test_int16_round_trip(self):
        self.assertEqual(to_uint16(-1), 0xFFFF)
        self.assertEqual(to_uint16(-32768), 0x8000)
        self.assertEqual(to_int16(0xFFFF), -1)
        self.assertEqual(to_int16(0x8000), -32768)

    def test_clamp(self):
        self.assertEqual(clamp(10, 0, 5), 5)
        self.assertEqual(clamp(-2, 0, 5), 0)
        self.assertEqual(clamp(3, 0, 5), 3)

    def test_command_encoding(self):
        command = ChassisCommand(left_wheel_speed=-100, right_wheel_speed=200, disc_enable=DISC_ENABLE_ON)
        block = command.as_register_block()
        self.assertEqual(block[0], 0xFF9C)
        self.assertEqual(block[1], 200)
        self.assertEqual(block[3], DISC_ENABLE_ON)

    def test_snapshot_decoding(self):
        snapshot = RegisterSnapshot.from_registers([0xFF9C, 200, 0x0000, DISC_ENABLE_OFF, 2, 2, 1])
        self.assertEqual(snapshot.left_wheel_speed, -100)
        self.assertEqual(snapshot.right_wheel_speed, 200)
        self.assertEqual(snapshot.work_mode, 2)

    def test_motor_rpm_to_base_twist(self):
        linear, angular = motor_rpm_to_base_twist(
            left_motor_rpm=60,
            right_motor_rpm=60,
            wheel_radius_m=0.1,
            wheel_track_m=0.5,
            gear_ratio=60,
        )
        self.assertAlmostEqual(linear, 2.0 * 3.141592653589793 * 0.1 / 60.0, places=9)
        self.assertAlmostEqual(angular, 0.0, places=9)

        _, angular = motor_rpm_to_base_twist(
            left_motor_rpm=0,
            right_motor_rpm=60,
            wheel_radius_m=0.1,
            wheel_track_m=0.5,
            gear_ratio=60,
        )
        self.assertAlmostEqual(angular, (2.0 * 3.141592653589793 * 0.1 / 60.0) / 0.5, places=9)

    def test_motor_rpm_to_base_twist_rejects_invalid_geometry(self):
        with self.assertRaises(ValueError):
            motor_rpm_to_base_twist(0, 0, 0.0, 0.5, 60.0)


if __name__ == "__main__":
    unittest.main()
