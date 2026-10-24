"""Actual scheduler mixing/guard/status with a fake clock; no ROS or hardware."""
import json
import math
from pathlib import Path
import tempfile
import threading
from types import MethodType, SimpleNamespace
import unittest

from test_status_indicator_integration import Message, Publisher, production_method, ROOT
from grinder_chassis_driver.motion_safety import MotionInterlock
from grinder_scheduler.models import SchedulerState
from grinder_scheduler.status_indicator import LightState, StateResolver


class ManualSteeringGuardActivityTest(unittest.TestCase):
    def node(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        clock = SimpleNamespace(now=10.0)
        namespace = {
            "time": SimpleNamespace(time=lambda: 1000.0 + clock.now, monotonic=lambda: clock.now),
            "json": json,
            "math": math,
            "String": Message,
            "WheelSpeedCommand": lambda: Message(left_wheel_speed=0, right_wheel_speed=0),
            "SchedulerStatus": lambda: Message(header=Message(), pose=Message()),
            "SchedulerState": SchedulerState,
            "rospy": SimpleNamespace(Time=SimpleNamespace(now=lambda: Message(to_sec=lambda: clock.now)),
                loginfo_throttle=lambda *a: None, logwarn_throttle=lambda *a: None,
                logwarn=lambda *a: None),
        }
        node = SimpleNamespace(
            _motion=MotionInterlock(str(Path(directory.name) / "motion.json")),
            _manual_drive_lock=threading.Lock(),
            _chassis_settings={"run_speed": 0.05, "max_turn_speed_ratio": 0.7},
            _manual_speed_mps_to_rpm=lambda _: 100,
            _manual_last_left_cmd=0, _manual_last_right_cmd=0,
            _manual_reverse_guard_sec=1.0, _manual_reverse_until=0,
            _manual_command_active=False, _manual_last_command_monotonic=0,
            _manual_command_timeout_sec=0.5, _last_manual_enable_try=0,
            _wheel_speed_state_lock=threading.Lock(), last_wheel_speed_state=None,
            _manual_reverse_feedback_after=None, _manual_reverse_feedback_count=0,
            _manual_reverse_feedback_stamp=None,
            last_chassis_status=Message(enabled=True), _manual_travel_disc_active=False,
            _set_chassis_enabled=lambda _: None,
            _exec_active=False, state=SchedulerState.STOPPED,
            wheel_cmd_pub=Publisher(), status_pub=Publisher(), control_activity_pub=Publisher(),
            sl_link_server=Message(pb=Message(MANUAL_MOTION_FORWARD=1, MANUAL_MOTION_BACKWARD=2,
                MANUAL_MOTION_FORWARD_LEFT=3, MANUAL_MOTION_FORWARD_RIGHT=4,
                MANUAL_MOTION_BACKWARD_LEFT=5, MANUAL_MOTION_BACKWARD_RIGHT=6)),
            aurora_bridge=Message(get_pose=lambda: {"x": 0, "y": 0, "heading_deg": 0}),
            task_config=Message(task_id=""), _task_progress=lambda: 0,
            map_service=Message(get_map_info=lambda: None),
            media_streamer=Message(get_state=lambda: Message(online=False)),
            replan_requested=False, last_error="", current_path=None, _indicator_pause_reason="",
        )
        node._set_manual_travel_disc_active = lambda active, reason: setattr(node, "_manual_travel_disc_active", active)
        source = ROOT / "catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py"
        for name in ("_handle_manual_drive_locked", "_manual_drive_watchdog_tick", "_publish_status",
                     "_wheel_speed_state_callback"):
            setattr(node, name, MethodType(production_method(source, name, namespace), node))
        return node, clock

    def command(self, node, steering=0.0, stop=False):
        node._handle_manual_drive_locked(0, 0.0 if stop else 1.0,
            remote_x=0.0 if stop else steering,
            remote_y=0.0 if stop else -(0.05 / 0.15) * (1.0 - abs(steering)),
            max_speed_mps=0.15, max_turn_speed_ratio=0.7)

    def light(self, node, clock):
        node._publish_status()
        resolver = StateResolver()
        for name, value in {
            "scheduler": {"state": "STOPPED"},
            "chassis": {"connected": True, "disc_enabled": node._manual_travel_disc_active},
            "scheduler_emergency": False, "chassis_emergency": False,
            "activity": json.loads(node.control_activity_pub.messages[-1].data),
        }.items():
            resolver.update(name, value, clock.now)
        return resolver.resolve(clock.now)

    def wheels(self, node):
        output = node.wheel_cmd_pub.messages[-1]
        return output.left_wheel_speed, output.right_wheel_speed

    def feedback(self, node, clock, left=0, right=0, valid=True, stamp=None, target=(0, 0)):
        sample_time = clock.now if stamp is None else stamp
        node._wheel_speed_state_callback(Message(header=Message(stamp=Message(to_sec=lambda: sample_time)),
            feedback_valid=valid, feedback_left_wheel_speed=left, feedback_right_wheel_speed=right,
            target_left_wheel_speed=target[0], target_right_wheel_speed=target[1]))

    def test_single_wheel_crossing_keeps_requested_forward_component(self):
        for direction in (-1.0, 1.0):
            node, clock = self.node()
            self.command(node, direction * 0.2)
            clock.now += 0.1
            self.command(node, direction * 0.8)
            self.assertNotEqual((0, 0), self.wheels(node))
            self.assertEqual(-14, sum(self.wheels(node)))
            self.assertEqual(LightState.MANUAL, self.light(node, clock))
            for _ in range(20):
                clock.now += 0.1
                self.command(node, direction * 0.8)
                self.assertNotEqual((0, 0), self.wheels(node))
                self.assertEqual(-14, sum(self.wheels(node)))
            self.assertLess(self.wheels(node)[0] * self.wheels(node)[1], 0)

    def test_full_steering_never_has_residual_forward_command(self):
        node, clock = self.node()
        self.command(node)
        clock.now += 0.1
        self.command(node, 1.0)
        self.assertEqual((0, 0), self.wheels(node))
        for _ in range(2):
            clock.now += 0.1
            self.feedback(node, clock)
        self.command(node, 1.0)
        left, right = self.wheels(node)
        self.assertEqual(0, left + right)
        self.assertTrue(0 < right < 70)
        self.assertEqual(LightState.MANUAL, self.light(node, clock))

    def test_whole_vehicle_reverse_retains_full_guard(self):
        node, clock = self.node()
        self.command(node)
        clock.now += 0.1
        reverse = lambda: node._handle_manual_drive_locked(0, 1.0, 0.0, 1.0 / 3.0, 0.15, 0.7)
        reverse()
        for _ in range(2):
            clock.now += 0.1
            self.feedback(node, clock)
            reverse()
            self.assertEqual((0, 0), self.wheels(node))
        clock.now = 11.2
        reverse()
        self.assertTrue(all(0 < speed < 33 for speed in self.wheels(node)))

    def test_stop_and_watchdog_never_restart_from_feedback(self):
        for explicit in (True, False):
            node, clock = self.node()
            self.command(node)
            clock.now += 0.1
            self.command(node, 0.8)
            if explicit:
                self.command(node, stop=True)
            else:
                clock.now += 0.51
                node._manual_drive_watchdog_tick(None)
            self.assertEqual((0, 0), self.wheels(node))
            self.assertFalse(node._manual_command_active)
            self.assertEqual(LightState.READY, self.light(node, clock))
            count = len(node.wheel_cmd_pub.messages)
            clock.now += 2
            self.feedback(node, clock)
            node._manual_drive_watchdog_tick(None)
            self.assertEqual(count, len(node.wheel_cmd_pub.messages))

    def test_endpoint_held_does_not_restart_transition(self):
        node, clock = self.node()
        self.command(node)
        for _ in range(40):
            clock.now += 0.1
            self.command(node, 1.0)
        self.assertEqual((-70, 70), self.wheels(node))
        self.assertEqual(LightState.MANUAL, self.light(node, clock))


if __name__ == "__main__":
    unittest.main()
