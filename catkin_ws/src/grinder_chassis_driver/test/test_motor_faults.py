"""Motor fault readback through production polling and Modbus, no hardware."""
from types import SimpleNamespace as NS
import unittest
import test_motion_safety as safety
from test_motion_safety import load_class, DRIVER, Publisher, Stamp
from grinder_chassis_driver.modbus_transport import ModbusExceptionResponse, ModbusTransportError


class MotorFaultTest(unittest.TestCase):
    def setUp(self):
        fixture = safety.SafetyTest()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        self.node = fixture.driver()
        self.node._memory_serial.registers.update({
            0x4017: 123, 0x4018: 0xFF9C, 0x4019: 0x8000, 0x401A: 0x7FFF})
        self.node._next_diagnostics_monotonic = float('inf')

    def test_combined_signed_faults_and_speeds_then_clear(self):
        n = self.node
        n._poll_once(None)
        self.assertEqual((123, -100), (n.actual_left_motor_speed, n.actual_right_motor_speed))
        self.assertEqual((-32768, 32767), (n.snapshot.left_motor_fault_code, n.snapshot.right_motor_fault_code))
        reads = [f for f in n._memory_serial.frames if f[1] == 3]
        self.assertEqual([(0x4012, 9)], [(int.from_bytes(f[2:4], 'big'), int.from_bytes(f[4:6], 'big')) for f in reads])
        n._memory_serial.registers.update({0x4019: 0, 0x401A: 0})
        n._next_status_poll_monotonic = 0
        n._poll_once(None)
        self.assertEqual((0, 0), (n.snapshot.left_motor_fault_code, n.snapshot.right_motor_fault_code))

    def test_fallback_reads_faults_and_keeps_fast_polls_small(self):
        n = self.node
        original = n.transport.read_register_block
        calls = []
        def read(start, count, **kwargs):
            calls.append((start, count))
            if count == 9:
                raise ModbusExceptionResponse(3, 2)
            return original(start, count, **kwargs)
        n.transport.read_register_block = read
        n._poll_once(None)
        self.assertFalse(n.status_combined_read)
        self.assertEqual([(0x4012, 9), (0x4012, 5), (0x4019, 2), (0x4017, 2)], calls)
        self.assertEqual((-32768, 32767), (n.snapshot.left_motor_fault_code, n.snapshot.right_motor_fault_code))
        calls.clear()
        n._next_status_poll_monotonic = float('inf')
        n._poll_once(None)
        self.assertEqual([(0x4017, 2)], calls)

    def test_failed_fault_read_does_not_publish_fresh_status(self):
        n = self.node
        n._poll_once(None)
        n.status_combined_read = False
        n._next_status_poll_monotonic = 0
        original = n.transport.read_register_block
        def read(start, count, **kwargs):
            if start == 0x4019:
                raise ModbusTransportError('fault read failed')
            return original(start, count, **kwargs)
        n.transport.read_register_block = read
        published = []
        n._publish_state = lambda **kwargs: published.append(kwargs)
        n._poll_once(None)
        self.assertFalse(published[-1]['status_fresh'])
        self.assertEqual(-32768, n.snapshot.left_motor_fault_code)
        self.assertFalse(n.actual_speed_valid)

    def test_status_and_diagnostics_publish_raw_signed_fault_codes(self):
        n = self.node
        n._poll_once(None)
        cls, ns = load_class(DRIVER, {'_publish_state', '_publish_diagnostics'})
        ns['rospy'] = NS(Time=NS(now=lambda: Stamp(100)))
        ns['ChassisStatus'] = ns['WheelSpeedState'] = lambda: NS(header=NS())
        ns['DiagnosticArray'] = lambda: NS(header=NS())
        ns['DiagnosticStatus'] = type('DiagnosticStatus', (), {'OK': 0, 'ERROR': 2})
        n._publish_wheel_odom = n._publish_wheel_twist = lambda *args: None
        n.status_pub = Publisher()
        n.diagnostics_pub = Publisher()
        cls._publish_state(n, status_fresh=False)
        status = n.status_pub.messages[-1]
        self.assertFalse(status.connected)
        self.assertEqual((-32768, 32767), (status.left_motor_fault_code, status.right_motor_fault_code))
        cls._publish_diagnostics(n)
        values = {v.key: v.value for v in n.diagnostics_pub.messages[-1].status[0].values}
        self.assertEqual('-32768', values['left_motor_fault_code'])
        self.assertEqual('32767', values['right_motor_fault_code'])


if __name__ == '__main__':
    unittest.main()
