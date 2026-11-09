"""RPM remains public; only the final 4012 wire value is a control level."""
import unittest
import test_motion_safety as harness
from test_motion_safety import Message, registers
from grinder_chassis_driver.register_map import encode_disc_output_block

class DiscSpeedEncodingTest(unittest.TestCase):
    setUp = harness.SafetyTest.setUp
    driver = harness.SafetyTest.driver

    def test_scale_saturation_direction_and_monotonicity(self):
        encode = lambda rpm: encode_disc_output_block(0x4012, [rpm])[0]
        self.assertEqual([0, 7, 40, 100, 100, -40], [encode(x) for x in [0,100,600,1500,2000,-600]])
        self.assertEqual([20],encode_disc_output_block(0x4012,[600],3000))
        values = [encode(i) for i in range(1501)]
        self.assertEqual(sorted(values), values)
        self.assertTrue(all(0 <= v <= 100 for v in values))
        for bad in [0,-1,True,1500.5]:
            with self.assertRaises(ValueError): encode_disc_output_block(0x4012,[600],bad)

    def test_single_and_full_output_keep_rpm_cache_and_wheels(self):
        for lift in [True, False]:
            with self.subTest(lift=lift):
                n=self.driver(lift=lift)
                n.task_enable=True
                n._handle_disc_speed_cmd(Message(600))
                self.assertEqual(40,n._memory_serial.registers[0x4012])
                self.assertEqual(600,n.command.disc_speed)
                self.assertEqual(600,n._last_written_registers[0x4012])
                n.command.left_wheel_speed=100
                n.command.right_wheel_speed=-200
                n._submit_output('all',force=True)
                self.assertEqual(40,n._memory_serial.registers[0x4012])
                self.assertEqual(100,n._memory_serial.registers[0x4010])
                self.assertEqual((-200)&65535,n._memory_serial.registers[0x4011])
                n._issue_safe_stop()
                self.assertEqual(0,n._memory_serial.registers[0x4012])

    def test_full_block_converts_once_without_mutating_command(self):
        block=registers.ChassisCommand(left_wheel_speed=600,disc_speed=-600).as_register_block()
        wire=encode_disc_output_block(0x4010,block)
        self.assertEqual(600,wire[0]); self.assertEqual(-40,wire[2])
        self.assertEqual((-600)&65535,block[2])
        self.assertEqual([600,600],encode_disc_output_block(0x4010,[600,600]))

if __name__ == '__main__': unittest.main()
