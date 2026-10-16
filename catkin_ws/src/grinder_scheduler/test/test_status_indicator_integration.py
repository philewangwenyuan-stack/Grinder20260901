"""Exercise production callbacks and stop hooks without ROS or hardware."""
import ast
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from functools import wraps
from types import SimpleNamespace, MethodType
import unittest
import tempfile
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "catkin_ws/src/grinder_scheduler/src"))
sys.path.insert(0, str(ROOT / "catkin_ws/src/grinder_chassis_driver/src"))
from grinder_chassis_driver.motion_safety import MotionInterlock, motion_action
from grinder_scheduler.models import SchedulerState


class Message(SimpleNamespace):
    pass


class Publisher:
    def __init__(self, *args, **kwargs):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def production_method(path, name, namespace):
    namespace.setdefault("motion_action", motion_action)
    namespace.setdefault("localization_arming", lambda method: method)
    parsed = ast.parse(path.read_text(encoding="utf-8"))
    function = next(node for node in ast.walk(parsed) if isinstance(node, ast.FunctionDef) and node.name == name)
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), "exec"), namespace)
    return namespace[name]


class StopHookTest(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.gate = MotionInterlock(os.path.join(directory.name, "emergency.json"))
    def test_production_status_publishes_actual_activity_and_manual_expiry(self):
        namespace = {
            "time": time, "json": json, "math": math, "String": Message,
            "SchedulerState": SchedulerState,
            "SchedulerStatus": lambda: Message(header=Message(), pose=Message()),
            "rospy": SimpleNamespace(Time=SimpleNamespace(now=lambda: 0)),
        }
        method = production_method(ROOT / "catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py",
                                   "_publish_status", namespace)
        node = SimpleNamespace(
            aurora_bridge=SimpleNamespace(get_pose=lambda: {"x": 0, "y": 0, "heading_deg": 0}),
            task_config=Message(task_id="task"), state=SchedulerState.RUNNING,
            _task_progress=lambda: 0, map_service=SimpleNamespace(get_map_info=lambda: None),
            media_streamer=SimpleNamespace(get_state=lambda: Message(online=False)),
            replan_requested=False, last_error="", current_path=None,
            status_pub=Publisher(), control_activity_pub=Publisher(),
            _exec_active=True, _manual_command_active=False,
            _indicator_pause_reason="",
            _manual_last_command_monotonic=0, _manual_command_timeout_sec=0.5,
        )
        method(node)
        self.assertTrue(json.loads(node.control_activity_pub.messages[-1].data)["automatic_active"])
        node.state = SchedulerState.STOPPED
        node._exec_active = False
        node._manual_command_active = True
        node._manual_last_command_monotonic = time.monotonic()
        method(node)
        self.assertTrue(json.loads(node.control_activity_pub.messages[-1].data)["manual_active"])
        node._manual_last_command_monotonic -= 1
        method(node)
        self.assertFalse(json.loads(node.control_activity_pub.messages[-1].data)["manual_active"])
        node.state = SchedulerState.PAUSED
        node._indicator_pause_reason = "collision"
        method(node)
        self.assertEqual("collision", json.loads(node.control_activity_pub.messages[-1].data)["pause_reason"])
        node.state = SchedulerState.STOPPED
        method(node)
        self.assertEqual("", json.loads(node.control_activity_pub.messages[-1].data)["pause_reason"])

    def test_scheduler_emergency_records_event_and_keeps_stop_outputs(self):
        events = []
        namespace = {
            "Bool": Message, "WheelSpeedCommand": Message, "GoalID": Message, "SchedulerState": SchedulerState,
            "rospy": SimpleNamespace(logwarn=lambda *a: None, loginfo=lambda *a: None),
        }
        method = production_method(ROOT / "catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py",
                                   "_apply_emergency_stop", namespace)
        node = SimpleNamespace(
            _motion=self.gate, move_base_cancel_pub=Publisher(),
            _reset_emergency_stop=lambda _: Message(success=True, message="released"),
            _indicator_emergency=SimpleNamespace(observe=lambda: events.append("observe")),
            _set_cmd_vel_forward_runtime_active=lambda *a, **k: events.append("disable"),
            chassis_emergency_stop_pub=Publisher(), wheel_cmd_pub=Publisher(),
            disc_enable_pub=Publisher(), light_pub=Publisher(),
            _reset_disc_motion_guard=lambda: None,
        )
        method(node, True)
        self.assertEqual(["observe", "disable"], events)
        self.assertEqual(SchedulerState.STOPPED, node.state)
        self.assertFalse(node._exec_active)
        self.assertFalse(node.disc_enable_pub.messages[-1].data)
        self.assertFalse(node.light_pub.messages[-1].data)
        self.assertTrue(node.chassis_emergency_stop_pub.messages[-1].data)
        self.assertEqual(0, node.wheel_cmd_pub.messages[-1].left_wheel_speed)
        self.assertEqual(0, node.wheel_cmd_pub.messages[-1].right_wheel_speed)
        method(node, False)
        self.assertEqual(2, len(events))

    def test_driver_emergency_keeps_safe_stop(self):
        events = []
        namespace = {"rospy": SimpleNamespace(logwarn=lambda *a: None)}
        method = production_method(ROOT / "catkin_ws/src/grinder_chassis_driver/src/grinder_chassis_driver/chassis_driver_node.py",
                                   "_handle_emergency_stop_cmd", namespace)
        node = SimpleNamespace(
            _motion=self.gate,
            _lock=threading.Lock(), task_enable=True,
            _indicator_emergency=SimpleNamespace(observe=lambda: events.append("observe")),
            _reset_cmd_vel_filter=lambda: events.append("filter"),
            _issue_safe_stop=lambda: events.append("stop"),
        )
        method(node, Message(data=True))
        self.assertFalse(node.task_enable)
        self.assertEqual(["observe", "filter", "stop"], events)
        method(node, Message(data=False))
        self.assertEqual(3, len(events))


class NodeIntegrationTest(unittest.TestCase):
    def setUp(self):
        diagnostic = type("DiagnosticStatus", (Message,), {"OK": 0, "ERROR": 2})
        def array():
            return Message(header=Message())
        self.rospy = SimpleNamespace(
            get_param=lambda key, default: default, Publisher=Publisher, Subscriber=lambda *a, **k: None,
            Service=lambda *a, **k: None, on_shutdown=lambda *a: None,
            Time=SimpleNamespace(now=lambda: 0), is_shutdown=lambda: False,
            logwarn_throttle=lambda *a: None, ROSException=RuntimeError, ServiceException=RuntimeError,
        )
        modules = {
            "rospy": self.rospy,
            "diagnostic_msgs.msg": SimpleNamespace(DiagnosticArray=array, DiagnosticStatus=diagnostic, KeyValue=Message),
            "grinder_chassis_driver.msg": SimpleNamespace(ChassisStatus=Message),
            "grinder_scheduler.msg": SimpleNamespace(SchedulerStatus=Message),
            "std_msgs.msg": SimpleNamespace(Bool=Message, String=Message),
            "std_srvs.srv": SimpleNamespace(Trigger=Message, TriggerResponse=Message),
        }
        path = ROOT / "catkin_ws/src/grinder_scheduler/src/grinder_scheduler/status_indicator_node.py"
        spec = importlib.util.spec_from_file_location("indicator_node_test", path)
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules), patch("threading.Thread.start"):
            spec.loader.exec_module(self.module)
            self.node = self.module.StatusIndicatorNode()

    def publish_ready(self):
        self.node._scheduler(Message(state="IDLE"))
        self.node._chassis(Message(connected=True, disc_enabled=False))
        self.node._activity(Message(data='{"automatic_active":false,"manual_active":false}'))
        self.node._emergency("scheduler_emergency", Message(data=False))
        self.node._emergency("chassis_emergency", Message(data=False))

    def latest(self):
        self.node._step()
        return json.loads(self.node.status_pub.messages[-1].data)

    def test_mock_node_callbacks_and_emergency_priority(self):
        self.assertEqual("STARTUP", self.latest()["state"])
        self.publish_ready()
        self.assertEqual("READY", self.latest()["state"])
        self.node._scheduler(Message(state="RUNNING"))
        self.node._activity(Message(data='{"automatic_active":true,"manual_active":false}'))
        self.assertEqual("AUTOMATIC", self.latest()["state"])
        self.node._emergency("chassis_emergency", Message(data=True))
        self.node._scheduler(Message(state="RUNNING"))
        self.assertEqual("EMERGENCY", self.latest()["state"])
        self.node._scheduler(Message(state="PAUSED"))
        self.node._emergency("chassis_emergency", Message(data=False))
        self.assertEqual("PAUSED", self.latest()["state"])
        self.assertTrue(self.latest()["acknowledged"])

    def test_percentage_config_reaches_usb_and_legacy_raw_config_still_works(self):
        from grinder_scheduler.status_indicator import unpack
        for params, expected in (({}, 166), ({'~brightness_percent': 65}, 166),
                                 ({'~brightness_percent': 100}, 255),
                                 ({'~brightness': 102}, 102),
                                 ({'~brightness_percent': 40, '~brightness': 166}, 102)):
            with self.subTest(params=params):
                self.rospy.get_param = lambda key, default: params.get(key, default)
                with patch('threading.Thread.start'):
                    node = self.module.StatusIndicatorNode()
                node._step()
                self.assertEqual(expected, node.brightness)
                self.assertEqual(str(expected), unpack(node.transport.port.frames[-1])[4])
                self.assertEqual(expected, json.loads(node.status_pub.messages[-1].data)['brightness'])

    def test_reset_queues_explicit_controller_clear_without_motion_services(self):
        self.publish_ready()
        services = []
        self.rospy.wait_for_service = lambda *a, **k: None
        def proxy(name, _type):
            services.append(name)
            key = "scheduler_emergency" if name.startswith("/scheduler") else "chassis_emergency"
            def call():
                self.node._emergency(key, Message(data=False))
                return Message(success=True, message="cleared")
            return call
        self.rospy.ServiceProxy = proxy
        response = self.node._clear(None)
        self.assertTrue(response.success)
        self.assertEqual(["/scheduler/clear_emergency_indicator", "/chassis/clear_emergency_indicator"], services)
        self.assertTrue(self.node._clear_pending)
        self.assertFalse(self.latest()["clear_pending"])
        self.node._clear_pending = True
        self.node._emergency("chassis_emergency", Message(data=True))
        self.assertFalse(self.node._clear_pending)

    def test_bench_reset_without_chassis_source(self):
        self.node.resolver.ignore_chassis_status = True
        services = []
        def wait(name, **kwargs):
            if name.startswith('/chassis'):
                raise RuntimeError('no chassis node')
        self.rospy.wait_for_service = wait
        def proxy(name, _type):
            services.append(name)
            def clear():
                self.node._emergency('scheduler_emergency', Message(data=False))
                return Message(success=True, message='cleared')
            return clear
        self.rospy.ServiceProxy = proxy
        self.assertTrue(self.node._clear(None).success)
        self.assertEqual(['/scheduler/clear_emergency_indicator'], services)

    def test_new_emergency_cancels_display_reset(self):
        self.publish_ready()
        self.rospy.wait_for_service = lambda *a, **k: None
        def proxy(name, _type):
            def clear():
                self.node._emergency('scheduler_emergency', Message(data=True))
                return Message(success=True, message='raced')
            return clear
        self.rospy.ServiceProxy = proxy
        self.assertFalse(self.node._clear(None).success)
        self.assertFalse(self.node._clear_pending)

    def test_bench_must_reset_existing_chassis_source(self):
        self.publish_ready()
        self.node.resolver.ignore_chassis_status = True
        self.rospy.wait_for_service = lambda *a, **k: None
        def proxy(name, _type):
            if name.startswith('/chassis'):
                return lambda: Message(success=False, message='cannot persist')
            def clear():
                self.node._emergency('scheduler_emergency', Message(data=False))
                return Message(success=True, message='cleared')
            return clear
        self.rospy.ServiceProxy = proxy
        self.assertFalse(self.node._clear(None).success)
        self.assertFalse(self.node._clear_pending)

    def test_invalid_activity_does_not_refresh_freshness(self):
        self.node._activity(Message(data='{"automatic_active":"false","manual_active":false}'))
        self.assertNotIn("activity", self.node.resolver.samples)

    def test_collision_pause_reaches_the_wire(self):
        self.publish_ready()
        self.node._scheduler(Message(state="PAUSED"))
        self.node._activity(Message(data='{"automatic_active":false,"manual_active":false,"pause_reason":"collision"}'))
        self.assertEqual("COLLISION_PAUSED", self.latest()["state"])
        from grinder_scheduler.status_indicator import unpack
        self.assertEqual("8", unpack(self.node.transport.port.frames[-1])[3])
        self.node._activity(Message(data='{"automatic_active":false,"manual_active":false,"pause_reason":"manual"}'))
        self.assertEqual("PAUSED", self.latest()["state"])

    def test_localization_pause_uses_existing_fault_wire_state(self):
        self.publish_ready()
        self.node._scheduler(Message(state="PAUSED"))
        self.node._activity(Message(data='{"automatic_active":false,"manual_active":false,"pause_reason":"localization"}'))
        self.assertEqual("FAULT", self.latest()["state"])
        from grinder_scheduler.status_indicator import unpack
        self.assertEqual("6", unpack(self.node.transport.port.frames[-1])[3])


class CollisionPauseLifecycleTest(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        namespace = {
            "Bool": Message, "Int16": Message, "time": time, "SchedulerState": SchedulerState,
            "rospy": SimpleNamespace(logwarn=lambda *a: None, logerr=lambda *a: None),
        }
        path = ROOT / "catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py"
        self.node = SimpleNamespace(
            _motion=MotionInterlock(os.path.join(directory.name, "emergency.json")),
            _localization_arm_lock=threading.Lock(), _localization_arm_count=0,
            _localization_arming_cancelled=False,
            state=SchedulerState.RUNNING, _exec_active=True, _indicator_pause_reason="",
            _collision_imminent_lock=threading.Lock(), _collision_imminent_active=False,
            _collision_pause_latched=False, _collision_imminent_topic="/rpp/collision_imminent",
            _localization_motion_status=lambda: (True, "ready", {}),
            task_config=Message(task_id="task"), _chassis_settings={"run_speed": 0.1},
            current_path=Message(points=[{}]), _active_segments=[{}],
            disc_enable_pub=Publisher(), disc_speed_pub=Publisher(), light_pub=Publisher(),
            _reset_disc_motion_guard=lambda: None, _safe_stop_motion=lambda: None,
            _set_cmd_vel_forward_runtime_active=lambda *a, **k: None,
            _set_chassis_enabled=lambda *a: None, _configured_disc_speed_rpm=lambda: 1200,
            _init_segment_execution_cursor=lambda: None,
            _publish_path_to_navigation=lambda **k: None,
            _publish_active_segment_plan=lambda **k: None,
            _send_active_segment_goal=lambda **k: None,
            _capture_task_continuation=lambda: None,
            _finalize_task_result=lambda **k: None,
        )
        for name in ("_collision_imminent_callback", "_localization_watchdog_tick",
                     "_pause_execution", "_resume_execution", "_stop_execution"):
            setattr(self.node, name, MethodType(production_method(path, name, namespace), self.node))

    def test_localization_watchdog_pauses_and_requires_manual_resume(self):
        self.node._localization_motion_status = lambda: (False, "localization_heartbeat_timeout", {})
        self.node._localization_watchdog_tick(None)
        self.assertEqual(SchedulerState.PAUSED, self.node.state)
        self.assertEqual("localization", self.node._indicator_pause_reason)
        self.assertFalse(self.node.disc_enable_pub.messages[-1].data)
        self.assertFalse(self.node._resume_execution()[0])
        self.node._localization_motion_status = lambda: (True, "ready", {})
        self.assertEqual(SchedulerState.PAUSED, self.node.state)
        self.assertTrue(self.node._resume_execution()[0])
        self.assertEqual("", self.node._indicator_pause_reason)

    def test_localization_pause_has_priority_over_collision_and_manual(self):
        self.node._pause_execution(reason="collision")
        self.node._pause_execution(reason="localization")
        self.node._pause_execution(reason="manual")
        self.assertEqual("localization", self.node._indicator_pause_reason)

    def test_collision_clear_and_repeated_pause_do_not_erase_cause(self):
        self.node._collision_imminent_callback(Message(data=True))
        self.assertEqual(SchedulerState.PAUSED, self.node.state)
        self.assertEqual("collision", self.node._indicator_pause_reason)
        self.assertFalse(self.node._exec_active)
        self.assertFalse(self.node.disc_enable_pub.messages[-1].data)
        self.node._collision_imminent_callback(Message(data=False))
        self.node._pause_execution()
        self.assertEqual(SchedulerState.PAUSED, self.node.state)
        self.assertEqual("collision", self.node._indicator_pause_reason)

    def test_failed_resume_keeps_cause_successful_resume_clears_it(self):
        self.node._collision_imminent_callback(Message(data=True))
        self.assertFalse(self.node._resume_execution()[0])
        self.assertEqual("collision", self.node._indicator_pause_reason)
        self.node._collision_imminent_callback(Message(data=False))
        self.assertTrue(self.node._resume_execution()[0])
        self.assertEqual(SchedulerState.RUNNING, self.node.state)
        self.assertEqual("", self.node._indicator_pause_reason)
        self.node._pause_execution()
        self.assertEqual("manual", self.node._indicator_pause_reason)

    def test_explicit_stop_clears_collision_pause_cause(self):
        self.node._collision_imminent_callback(Message(data=True))
        self.assertTrue(self.node._stop_execution()[0])
        self.assertEqual(SchedulerState.STOPPED, self.node.state)
        self.assertEqual("", self.node._indicator_pause_reason)


if __name__ == "__main__":
    unittest.main()
