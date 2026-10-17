"""Production methods + real Modbus encoder/ACK parser; no ROS or hardware."""
import ast
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'catkin_ws/src/grinder_chassis_driver/src'))
sys.path.insert(0, str(ROOT / 'catkin_ws/src/grinder_scheduler/src'))
from grinder_chassis_driver.motion_safety import MotionInterlock, MotionCancelled, GuardedPublisher, motion_action
from grinder_chassis_driver.modbus_transport import ModbusTransport, ModbusTransportError, ModbusExceptionResponse
from grinder_chassis_driver import register_map as registers
from grinder_scheduler.models import SchedulerState


class Message(SimpleNamespace):
    def __init__(self, data=None, **kwargs):
        super().__init__(data=data, **kwargs)


class Publisher:
    def __init__(self, *args, **kwargs):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Stamp(float):
    def __sub__(self, other):
        return Stamp(float(self) - float(other))

    def to_sec(self):
        return float(self)


class MemorySerial:
    def __init__(self):
        self.frames, self.reply = [], b''
        self.registers = {}
        self.fail = False
        self.before_write = lambda frame: None

    def open(self):
        pass

    def close(self):
        pass

    def write(self, frame):
        self.before_write(frame)
        self.frames.append(bytes(frame))
        if self.fail:
            raise ModbusTransportError('injected serial failure')
        address = int.from_bytes(frame[2:4], 'big')
        if frame[1] == 16:
            count = int.from_bytes(frame[4:6], 'big')
            for i in range(count):
                self.registers[address + i] = int.from_bytes(frame[7+2*i:9+2*i], 'big')
            reply = frame[:6]
        elif frame[1] == 6:
            self.registers[address] = int.from_bytes(frame[4:6], 'big')
            reply = frame[:6]
        else:
            count = int.from_bytes(frame[4:6], 'big')
            reply = bytes([frame[0], 3, 2*count]) + b''.join(
                self.registers.get(address+i, 0).to_bytes(2, 'big') for i in range(count))
        self.reply = reply + ModbusTransport._crc16(reply).to_bytes(2, 'little')

    def read_exactly(self, size):
        result, self.reply = self.reply[:size], self.reply[size:]
        return result


def load_class(relative, names=None):
    path = ROOT / relative
    parsed = ast.parse(path.read_text(encoding='utf-8'))
    cls = next(n for n in parsed.body if isinstance(n, ast.ClassDef) and n.name in ('ChassisDriverNode', 'GrinderSchedulerNode', 'SchedulerNode'))
    if names is not None:
        cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    namespace = dict(vars(registers))
    namespace.update(globals())
    namespace.update({name: Message for name in ('Bool', 'Int16', 'UInt16', 'Float32', 'String', 'Twist', 'TwistWithCovarianceStamped', 'Odometry',
        'ChassisStatus', 'WheelSpeedCommand', 'WheelSpeedState', 'EnableChassis', 'EnableChassisResponse',
        'ClearFault', 'ClearFaultResponse', 'Trigger', 'TriggerResponse', 'DiagnosticStatus', 'DiagnosticArray',
        'KeyValue', 'GoalID')})
    if relative == SCHEDULER:
        from functools import wraps
        namespace['wraps'] = wraps
        decorator = next(n for n in parsed.body if isinstance(n, ast.FunctionDef) and n.name == 'localization_arming')
        exec(compile(ast.Module(body=[decorator], type_ignores=[]), str(path), 'exec'), namespace)
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace[cls.name], namespace


DRIVER = 'catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver/chassis_driver_node.py'
SCHEDULER = 'catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py'


class SafetyTest(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.directory = Path(temp.name)

    def gate(self, name='motion'):
        return MotionInterlock(str(self.directory / (name + '.json')))

    def driver(self, lift=True):
        serial = MemorySerial()
        def transport(**kwargs):
            result = ModbusTransport(**kwargs)
            result._serial_port = serial
            return result
        cls, namespace = load_class(DRIVER)
        params = {'~motion_emergency_file': str(self.directory / 'driver.json'),
                  '~indicator_emergency_file': str(self.directory / 'driver_display.json'), '~disc_lift_supported': lift}
        ros = SimpleNamespace(get_param=lambda key, default=None: params.get(key, default),
            Publisher=Publisher, Subscriber=lambda *a, **k: None, Service=lambda *a, **k: None,
            Timer=lambda *a, **k: None, on_shutdown=lambda *a: None, is_shutdown=lambda: False,
            Time=SimpleNamespace(now=lambda: Stamp(time.monotonic())),
            Duration=SimpleNamespace(from_sec=lambda value: value))
        for name in ('loginfo', 'logwarn', 'logerr', 'logwarn_throttle', 'loginfo_throttle'):
            setattr(ros, name, lambda *a, **k: None)
        namespace.update(rospy=ros, ModbusTransport=transport,
            EmergencyIndicatorSource=lambda *a: SimpleNamespace(observe=lambda: None))
        node = cls()
        node._publish_state = lambda **kwargs: None
        node._publish_diagnostics = lambda: None
        node._memory_serial = serial
        return node

    def assert_safe_frame(self, frame):
        self.assertEqual(16, frame[1])
        self.assertEqual(b'\x40\x10', frame[2:4])
        self.assertEqual(b'\x00' * 8, frame[7:15])
        self.assertEqual(ModbusTransport._crc16(frame[:-2]), int.from_bytes(frame[-2:], 'little'))

    def test_zero_cache_then_motion_then_stop_sends_zero_frame(self):
        node = self.driver()
        count = len(node._memory_serial.frames)
        node.command.left_wheel_speed = node.command.right_wheel_speed = 100
        node._write_motion_registers()
        self.assertIsNone(node._last_written_block)
        node._handle_emergency_stop_cmd(Message(True))
        self.assertGreaterEqual(len(node._memory_serial.frames), count + 2)
        self.assert_safe_frame(node._memory_serial.frames[-1])
        self.assertTrue(node._motion.latched)

    def test_disc_only_stop_and_repeated_stop_always_write(self):
        node = self.driver()
        node._handle_disc_speed_cmd(Message(1000))
        node._handle_disc_enable_cmd(Message(True))
        self.assertIsNone(node._last_written_block)
        for _ in range(2):
            count = len(node._memory_serial.frames)
            node._handle_emergency_stop_cmd(Message(True))
            self.assertGreater(len(node._memory_serial.frames), count)
            self.assert_safe_frame(node._memory_serial.frames[-1])

    def test_manual_zero_command_bypasses_ramp_and_writes_zero_immediately(self):
        node = self.driver()
        node.command.left_wheel_speed = node.command.right_wheel_speed = 100
        node._write_motion_registers()
        node.enable_cmd_ramp_limit = True
        node.max_cmd_step_rpm = 20
        before = len(node._memory_serial.frames)
        node._handle_wheel_speed_cmd(Message(left_wheel_speed=0, right_wheel_speed=0))
        self.assertGreater(len(node._memory_serial.frames), before)
        frame = node._memory_serial.frames[-1]
        self.assertEqual(b'\x00\x00\x00\x00', frame[7:11])
        self.assertEqual(0, node.last_sent_left_wheel_speed)
        self.assertEqual(0, node.last_sent_right_wheel_speed)

    def test_liftless_stop_does_not_write_lift_register(self):
        node = self.driver(False)
        self.assertEqual(5, int.from_bytes(node._memory_serial.frames[-2][4:6], 'big'))
        self.assertIn(node._memory_serial.frames[-1][1], (6, 16))
        self.assertEqual(registers.REGISTER_LIGHT, int.from_bytes(node._memory_serial.frames[-1][2:4], 'big'))

    def test_latch_rejects_all_motion_and_survives_restart(self):
        node = self.driver()
        node._handle_emergency_stop_cmd(Message(True))
        count = len(node._memory_serial.frames)
        node._handle_wheel_speed_cmd(Message(left_wheel_speed=100, right_wheel_speed=100))
        node._handle_disc_speed_cmd(Message(1000))
        node._handle_disc_enable_cmd(Message(True))
        node._handle_task_enable_cmd(Message(True))
        self.assertFalse(node._handle_enable_service(Message(enable=True)).success)
        self.assertEqual(count, len(node._memory_serial.frames))
        self.assertTrue(MotionInterlock(str(self.directory / 'driver.json')).latched)

    def test_timeout_bypasses_ramp_and_keeps_disc_policy(self):
        node = self.driver()
        node.command.disc_speed = 1000
        node.command.disc_enable = 1
        node.command.left_wheel_speed = node.command.right_wheel_speed = 100
        node.last_sent_left_wheel_speed = node.last_sent_right_wheel_speed = 100
        node.last_command_time = Stamp(time.monotonic() - 10)
        node._enforce_command_timeout()
        frame = node._memory_serial.frames[-1]
        self.assertEqual(b'\x00' * 4, frame[7:11])
        self.assertEqual(b'\x03\xe8\x00\x01', frame[11:15])

    def test_failure_invalidates_cache_and_recovery_first_writes_safe(self):
        node = self.driver()
        node._memory_serial.fail = True
        node.command.left_wheel_speed = 100
        node._write_motion_registers()
        self.assertFalse(node._outputs_ready)
        self.assertIsNone(node._last_written_block)
        self.assertEqual({}, node._last_written_registers)
        node._memory_serial.fail = False
        before = len(node._memory_serial.frames)
        node._poll_once(None)
        self.assert_safe_frame(node._memory_serial.frames[before])
        self.assertEqual(0, node.command.left_wheel_speed)
        self.assertTrue(node._outputs_ready)

    def test_old_waiting_write_cannot_run_after_stop(self):
        node = self.driver()
        entered = threading.Event()
        done = []
        def worker():
            with node._motion.action():
                entered.set()
                done.append(node._write_motion_registers())
        with node._io_lock:
            thread = threading.Thread(target=worker)
            thread.start()
            self.assertTrue(entered.wait(1))
            node._handle_emergency_stop_cmd(Message(True))
        thread.join(1)
        self.assertEqual([False], done)
        self.assert_safe_frame(node._memory_serial.frames[-1])

    def test_reset_requires_post_request_stopped_feedback(self):
        node = self.driver()
        node._handle_emergency_stop_cmd(Message(True))
        result = []
        thread = threading.Thread(target=lambda: result.append(node._reset_emergency_stop(None)))
        thread.start()
        for _ in range(8):
            time.sleep(0.04)
            node._next_status_poll_monotonic = 0
            node._poll_once(None)
            if result:
                break
        thread.join(4)
        self.assertTrue(result[0].success, result[0].message)
        self.assertFalse(node._motion.latched)
        self.assertFalse(node.task_enable)
        self.assertEqual(0, node.command.left_wheel_speed)
        self.assertFalse(MotionInterlock(str(self.directory / 'driver.json')).latched)

    def test_new_emergency_cancels_reset(self):
        node = self.driver()
        node._handle_emergency_stop_cmd(Message(True))
        result = []
        thread = threading.Thread(target=lambda: result.append(node._reset_emergency_stop(None)))
        thread.start()
        time.sleep(0.05)
        node._handle_emergency_stop_cmd(Message(True))
        thread.join(1)
        self.assertFalse(result[0].success)
        self.assertTrue(node._motion.latched)

    def test_corrupt_record_and_failed_reset_fail_closed(self):
        path = self.directory / 'broken.json'
        path.write_text('broken')
        gate = MotionInterlock(str(path))
        self.assertTrue(gate.latched)
        with patch.object(gate.record, 'flush', side_effect=OSError('disk full')):
            self.assertFalse(gate.reset(gate.generation))
        self.assertTrue(gate.latched)

    def test_old_scope_stays_invalid_after_release(self):
        gate = self.gate()
        with gate.action():
            gate.invalidate(emergency=True)
            gate.persist()
            self.assertTrue(gate.reset(gate.generation))
            with self.assertRaises(MotionCancelled):
                gate.check()

    def test_partial_write_failure_does_not_confirm_or_cache_block(self):
        node = self.driver(False)
        def fail_light(frame):
            if int.from_bytes(frame[2:4], 'big') == registers.REGISTER_LIGHT:
                raise ModbusTransportError('second transaction failed')
        node._memory_serial.before_write = fail_light
        self.assertFalse(node._issue_safe_stop())
        self.assertFalse(node._safe_write_confirmed)
        self.assertFalse(node._outputs_ready)
        self.assertIsNone(node._last_written_block)
        self.assertEqual({}, node._last_written_registers)
        node._memory_serial.before_write = lambda frame: None
        count = len(node._memory_serial.frames)
        node._poll_once(None)
        self.assert_safe_frame(node._memory_serial.frames[count])

    def test_inflight_nonzero_finishes_before_forced_stop_never_after(self):
        node = self.driver()
        entered, release = threading.Event(), threading.Event()
        def block(frame):
            if frame[7:11] != b'\x00' * 4:
                entered.set()
                release.wait(2)
        node._memory_serial.before_write = block
        worker = threading.Thread(target=lambda: node._handle_wheel_speed_cmd(Message(left_wheel_speed=100, right_wheel_speed=100)))
        worker.start()
        self.assertTrue(entered.wait(1))
        stopper = threading.Thread(target=lambda: node._handle_emergency_stop_cmd(Message(True)))
        stopper.start()
        deadline = time.monotonic() + 1
        while not node._motion.latched and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertTrue(node._motion.latched)
        release.set()
        worker.join(1)
        stopper.join(1)
        self.assertFalse(worker.is_alive() or stopper.is_alive())
        self.assert_safe_frame(node._memory_serial.frames[-1])

    def test_new_trip_during_reset_write_cannot_be_adopted_by_reset(self):
        node = self.driver()
        node._handle_emergency_stop_cmd(Message(True))
        node._memory_serial.before_write = lambda frame: node._motion.invalidate(emergency=True)
        self.assertFalse(node._reset_emergency_stop(None).success)
        self.assertTrue(node._motion.latched)

    def test_reset_rejects_disconnection_and_nonzero_feedback(self):
        node = self.driver()
        node._handle_emergency_stop_cmd(Message(True))
        node.connected = False
        self.assertFalse(node._reset_emergency_stop(None).success)
        node.connected = True
        node._memory_serial.registers[registers.REGISTER_LEFT_MOTOR_ACTUAL_SPEED] = 10
        result = []
        worker = threading.Thread(target=lambda: result.append(node._reset_emergency_stop(None)))
        worker.start()
        while worker.is_alive():
            node._next_status_poll_monotonic = 0
            node._poll_once(None)
            time.sleep(.04)
        self.assertFalse(result[0].success)
        self.assertIn('not confirmed', result[0].message)
        self.assertTrue(node._motion.latched)

    def test_safe_sync_barrier_rejects_motion_inputs(self):
        node = self.driver()
        node._outputs_ready = False
        count = len(node._memory_serial.frames)
        node._handle_wheel_speed_cmd(Message(left_wheel_speed=100, right_wheel_speed=100))
        node._handle_disc_enable_cmd(Message(True))
        self.assertEqual(count, len(node._memory_serial.frames))
        self.assertEqual(0, node.command.left_wheel_speed)

    def test_reset_persistence_does_not_block_new_trip(self):
        gate = self.gate()
        gate.invalidate(emergency=True)
        gate.persist()
        entered, release = threading.Event(), threading.Event()
        original = gate.record.flush
        def blocked():
            entered.set()
            release.wait(2)
            original()
        result = []
        with patch.object(gate.record, 'flush', side_effect=blocked):
            worker = threading.Thread(target=lambda: result.append(gate.reset(gate.generation)))
            worker.start()
            self.assertTrue(entered.wait(1))
            gate.invalidate(emergency=True)
            release.set()
            worker.join(1)
        self.assertEqual([False], result)
        self.assertTrue(gate.latched)
        self.assertTrue(MotionInterlock(gate.record.path).latched)

    def test_cancelled_reset_restores_durable_latch_after_false_write(self):
        gate = self.gate()
        gate.invalidate(emergency=True)
        gate.persist()
        original = gate.record.clear
        def interrupted(**kwargs):
            result = original(**kwargs)
            gate.invalidate()  # Stop/cancellation, without a second emergency observe.
            return result
        with patch.object(gate.record, 'clear', side_effect=interrupted):
            self.assertFalse(gate.reset(gate.generation))
        self.assertTrue(gate.latched)
        self.assertTrue(MotionInterlock(gate.record.path).latched)

    def test_first_upgrade_migrates_old_display_latch_only_once(self):
        legacy = self.directory / 'legacy.json'
        legacy.write_text(json.dumps({'emergency_observed': True}))
        path = str(self.directory / 'migrated.json')
        gate = MotionInterlock(path, str(legacy))
        self.assertTrue(gate.latched)
        self.assertTrue(MotionInterlock(path).latched)
        self.assertTrue(gate.reset(gate.generation))
        self.assertFalse(MotionInterlock(path, str(legacy)).latched)


if __name__ == '__main__':
    unittest.main()
