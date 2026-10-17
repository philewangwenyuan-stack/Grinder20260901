#!/usr/bin/env python3
"""Actual C++ ROS node against a PTY RTU; run via rostest, not unittest discovery."""
import json
import math
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import tempfile
import threading
import time
import unittest

import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Float32, Int16, String, UInt16
from std_srvs.srv import Trigger
from grinder_chassis_driver.msg import ChassisStatus, WheelSpeedCommand, WheelSpeedState
from grinder_chassis_driver.srv import EnableChassis, ClearFault


def crc(data):
    result = 0xFFFF
    for value in data:
        result ^= value
        for _ in range(8):
            result = (result >> 1) ^ (0xA001 if result & 1 else 0)
    return result.to_bytes(2, 'little')


class FakeRTU:
    def __init__(self):
        self.master, self.slave = pty.openpty()
        self.port = os.ttyname(self.slave)
        self.registers = {0x4014: 2, 0x4015: 2}
        self.writes = []
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.drop_reads = False
        self.reject_combined = False
        self.motion_delay = 0.0
        self.motion_received = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def close(self):
        self.stop.set()
        self.thread.join(1)
        os.close(self.master)
        os.close(self.slave)

    def value(self, address):
        with self.lock:
            return self.registers.get(address, 0)

    def run(self):
        data = bytearray()
        try:
            while not self.stop.is_set():
                readable, _, _ = select.select([self.master], [], [], 0.02)
                if not readable:
                    continue
                data.extend(os.read(self.master, 1024))
                while len(data) >= 8:
                    size = 9 + data[6] if data[1] == 16 else 8
                    if len(data) < size:
                        break
                    frame = bytes(data[:size])
                    del data[:size]
                    if crc(frame[:-2]) != frame[-2:]:
                        raise AssertionError('driver emitted invalid CRC')
                    address, count = int.from_bytes(frame[2:4], 'big'), int.from_bytes(frame[4:6], 'big')
                    if frame[1] in (6, 16):
                        values = [count] if frame[1] == 6 else [int.from_bytes(frame[7+i*2:9+i*2], 'big') for i in range(count)]
                        with self.lock:
                            for index, value in enumerate(values):
                                self.registers[address+index] = value
                                if address+index in (0x4010, 0x4011):
                                    self.registers[address+index+7] = value
                            self.writes.append((time.monotonic(), address, values))
                        motion = address == 0x4010 and any(values[:2])
                        if motion:
                            self.motion_received.set()
                            if self.motion_delay:
                                delay, self.motion_delay = self.motion_delay, 0.0
                                time.sleep(delay)
                        reply = frame[:6]
                    elif frame[1] == 3:
                        if self.drop_reads:
                            continue
                        if self.reject_combined and address == 0x4012 and count == 7:
                            reply = bytes([frame[0], 0x83, 2])
                        else:
                            with self.lock:
                                values = [self.registers.get(address+i, 0) for i in range(count)]
                            reply = bytes([frame[0], 3, count*2]) + b''.join(v.to_bytes(2, 'big') for v in values)
                    else:
                        raise AssertionError('unexpected function code')
                    os.write(self.master, reply + crc(reply))
        except Exception as exc:
            if not self.stop.is_set():
                self.error = exc


class DriverIntegration(unittest.TestCase):
    def wait(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            if self.rtu.error:
                self.fail(str(self.rtu.error))
            if self.process.poll() is not None:
                self.output.flush()
                self.fail('driver exited: ' + Path(self.output.name).read_text(errors='replace'))
            time.sleep(0.01)
        with self.rtu.lock:
            evidence = repr((self.rtu.registers, self.rtu.writes[-8:]))
        safety = self.messages['safety'][-1].data if self.messages['safety'] else 'no safety messages'
        self.output.flush()
        self.fail('condition timed out; safety=' + safety + '; RTU=' + evidence + '; log=' +
                  Path(self.output.name).read_text(errors='replace')[-1800:])

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.rtu = FakeRTU()
        self.addCleanup(self.rtu.close)
        self.config = Path(self.directory.name) / 'config.yaml'
        self.config.write_text('# preserve this comment\nmanual_drive_max_wheel_speed: 0.2\nother: 7\n')
        params = {
            'port': self.rtu.port, 'baudrate': 19200, 'timeout': 0.1,
            'transaction_timeout': 0.1, 'rs485_raw_log_enabled': False,
            'motion_emergency_file': self.directory.name + '/motion.json',
            'indicator_emergency_file': self.directory.name + '/indicator.json',
            'config_file': str(self.config),
        }
        if rospy.has_param('/chassis_driver'):
            rospy.delete_param('/chassis_driver')
        if self._testMethodName == 'test_slow_status_polling_does_not_report_false_disconnects':
            params['status_poll_rate_hz'] = 0.5
        for name, value in params.items():
            rospy.set_param('/chassis_driver/' + name, value)
        self.messages = {name: [] for name in ('wheel', 'twist', 'odom', 'status', 'diag', 'safety', 'indicator')}
        topics = [('wheel', '/chassis/wheel_speed_state', WheelSpeedState),
                  ('twist', '/chassis/wheel_twist', TwistWithCovarianceStamped),
                  ('odom', '/odom_wheel', Odometry), ('status', '/chassis/status', ChassisStatus),
                  ('diag', '/diagnostics', DiagnosticArray), ('safety', '/chassis/motion_safety', String),
                  ('indicator', '/chassis/indicator_emergency', Bool)]
        self.subscribers = [rospy.Subscriber(topic, cls, lambda msg, key=key: self.messages[key].append(msg))
                            for key, topic, cls in topics]
        self.addCleanup(lambda: [s.unregister() for s in self.subscribers])
        self.publishers = {}
        for topic, cls in [('wheel_speed_cmd', WheelSpeedCommand), ('task_enable', Bool),
                           ('disc_speed_cmd', Int16), ('disc_enable_cmd', Bool), ('work_mode_cmd', UInt16),
                           ('disc_lift_cmd', UInt16), ('light_cmd', Bool), ('manual_speed_limit', Float32),
                           ('emergency_stop', Bool)]:
            self.publishers[topic] = rospy.Publisher('/chassis/' + topic, cls, queue_size=1)
        self.publishers['cmd_vel'] = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.clock = rospy.Publisher('/clock', Clock, queue_size=1, latch=True)
        self.addCleanup(lambda: [p.unregister() for p in self.publishers.values()])
        self.addCleanup(self.clock.unregister)
        self.output = tempfile.NamedTemporaryFile(mode='w+', dir=self.directory.name, suffix='.log')
        self.addCleanup(self.output.close)
        # Exercise the production launch, including its executable selection.
        self.process = subprocess.Popen([
            'roslaunch', '--wait', 'grinder_chassis_driver', 'chassis_driver.launch',
            'port:=' + self.rtu.port, 'config_file:=' + str(self.config)],
            stdout=self.output, stderr=subprocess.STDOUT)
        self.addCleanup(self.stop_process)
        self.wait(lambda: all(p.get_num_connections() for p in self.publishers.values()))
        self.clock.publish(Clock(clock=rospy.Time(1234)))
        self.wait(lambda: self.messages['safety'] and json.loads(self.messages['safety'][-1].data)['outputs_ready'])
        self.enable = rospy.ServiceProxy('/chassis/enable', EnableChassis)
        self.reset = rospy.ServiceProxy('/chassis/reset_emergency_stop', Trigger)
        self.clear_indicator = rospy.ServiceProxy('/chassis/clear_emergency_indicator', Trigger)

    def stop_process(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(6)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(2)

    def send(self, name, message):
        self.publishers[name].publish(message)

    def test_feedback_interfaces_and_frozen_ros_clock(self):
        self.send('wheel_speed_cmd', WheelSpeedCommand(left_wheel_speed=200, right_wheel_speed=-200))
        self.wait(lambda: self.rtu.value(0x4010) == ((-200) & 0xFFFF))
        self.wait(lambda: any(m.feedback_valid and m.feedback_left_wheel_speed == -200 for m in self.messages['wheel']))
        self.wait(lambda: any(m.twist.twist.angular.z > 0 for m in self.messages['twist']))
        msg = next(m for m in self.messages['twist'] if m.twist.twist.angular.z > 0)
        self.assertEqual('base_link', msg.header.frame_id)
        self.assertEqual([0.04, 0.04, 1e6], [msg.twist.covariance[i] for i in (0, 35, 7)])
        self.assertGreater(msg.header.stamp.to_sec(), 1233)
        self.assertLessEqual(msg.header.stamp.to_sec(), 1234)
        # A frozen /clock must not freeze the 500ms safety watchdog.
        self.wait(lambda: self.rtu.value(0x4010) == 0 and self.rtu.value(0x4011) == 0, timeout=1.5)
        self.wait(lambda: self.messages['diag'])
        values = {v.key: v.value for v in self.messages['diag'][-1].status[0].values}
        self.assertEqual('cpp', values['driver_backend'])
        self.assertEqual(0.0, self.messages['odom'][-1].pose.pose.position.x)

    def test_navigation_and_source_arbitration(self):
        self.send('task_enable', Bool(True))
        time.sleep(0.05)
        msg = Twist(); msg.linear.x = 0.1
        self.send('cmd_vel', msg)
        self.wait(lambda: self.rtu.value(0x4010) > 0)
        before = self.rtu.value(0x4010)
        self.send('wheel_speed_cmd', WheelSpeedCommand(left_wheel_speed=600, right_wheel_speed=600))
        time.sleep(0.04)
        self.assertEqual(before, self.rtu.value(0x4010))
        self.send('wheel_speed_cmd', WheelSpeedCommand())
        self.wait(lambda: self.rtu.value(0x4010) == 0)

    def test_tool_outputs_timeout_policy_and_manual_limit_persistence(self):
        for topic, message, address, expected in [
                ('disc_speed_cmd', Int16(321), 0x4012, 321), ('disc_enable_cmd', Bool(True), 0x4013, 1),
                ('work_mode_cmd', UInt16(1), 0x4014, 1), ('disc_lift_cmd', UInt16(1), 0x4015, 1),
                ('light_cmd', Bool(True), 0x4016, 1)]:
            self.send(topic, message)
            self.wait(lambda address=address, expected=expected: self.rtu.value(address) == expected)
        self.send('wheel_speed_cmd', WheelSpeedCommand(left_wheel_speed=200, right_wheel_speed=200))
        self.wait(lambda: self.rtu.value(0x4010) != 0)
        self.wait(lambda: self.rtu.value(0x4010) == 0, timeout=1.5)
        self.assertEqual((321, 1), (self.rtu.value(0x4012), self.rtu.value(0x4013)))
        self.send('manual_speed_limit', Float32(0.05))
        self.wait(lambda: '0.05' in self.config.read_text())
        self.assertIn('# preserve this comment', self.config.read_text())
        self.assertIn('other: 7', self.config.read_text())
        self.wait(lambda: abs(rospy.get_param('/chassis_driver/manual_drive_max_wheel_speed') - 0.05) < 1e-6)
        result = rospy.ServiceProxy('/chassis/clear_fault', ClearFault)()
        self.assertFalse(result.success)

    def test_emergency_reset_indicator_and_latched_topics(self):
        self.rtu.motion_delay = 0.06
        self.send('wheel_speed_cmd', WheelSpeedCommand(left_wheel_speed=200, right_wheel_speed=200))
        self.assertTrue(self.rtu.motion_received.wait(1))
        self.send('emergency_stop', Bool(True))
        self.wait(lambda: json.loads(self.messages['safety'][-1].data)['emergency_latched'])
        self.wait(lambda: self.rtu.value(0x4010) == 0 and self.rtu.value(0x4013) == 0)
        self.wait(lambda: self.messages['indicator'] and self.messages['indicator'][-1].data)
        self.assertTrue(self.clear_indicator().success)
        self.assertTrue(json.loads(self.messages['safety'][-1].data)['emergency_latched'])
        self.send('emergency_stop', Bool(False))
        time.sleep(0.05)
        self.assertFalse(self.enable(True).success)
        late = []
        sub = rospy.Subscriber('/chassis/motion_safety', String, late.append)
        try:
            self.wait(lambda: late)
            self.assertTrue(json.loads(late[-1].data)['emergency_latched'])
        finally:
            sub.unregister()
        self.assertTrue(self.reset().success)
        self.wait(lambda: not json.loads(self.messages['safety'][-1].data)['emergency_latched'])
        self.assertEqual(0, self.rtu.value(0x4010))
        motion = json.loads(Path(self.directory.name + '/motion.json').read_text())
        self.assertIs(motion['emergency_observed'], False)

    def test_read_failure_invalidates_feedback_and_recovers_stopped(self):
        self.rtu.reject_combined = True
        self.wait(lambda: self.messages['diag'] and any(v.key == 'status_combined_read' and v.value == 'False'
                  for v in self.messages['diag'][-1].status[0].values))
        self.rtu.drop_reads = True
        self.wait(lambda: self.messages['wheel'] and not self.messages['wheel'][-1].feedback_valid)
        received = len(self.messages['twist'])
        time.sleep(0.2)
        self.assertEqual(received, len(self.messages['twist']))
        self.wait(lambda: self.messages['status'] and not self.messages['status'][-1].connected)
        status_start = len(self.messages['status'])
        time.sleep(0.6)
        self.assertTrue(all(not m.connected for m in self.messages['status'][status_start:]))
        self.rtu.drop_reads = False
        self.wait(lambda: json.loads(self.messages['safety'][-1].data)['outputs_ready'])
        self.assertEqual(0, self.rtu.value(0x4010))

    def test_slow_status_polling_does_not_report_false_disconnects(self):
        self.wait(lambda: len(self.messages['status']) >= 2, timeout=5)
        self.assertTrue(all(m.connected for m in self.messages['status']))


if __name__ == '__main__':
    rospy.init_node('cpp_chassis_driver_test')
    rostest.rosrun('grinder_chassis_driver', 'cpp_chassis_driver', DriverIntegration)
