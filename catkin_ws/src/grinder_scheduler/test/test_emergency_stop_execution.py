"""Exercise actual scheduler start/reset methods against controlled barriers."""
import json
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'catkin_ws/src/grinder_chassis_driver/test'))
import test_motion_safety as harness
from grinder_chassis_driver.motion_safety import MotionInterlock, GuardedPublisher
from grinder_scheduler.models import SchedulerState

M = harness.Message


class ExecutionTest(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.directory = Path(temp.name)

    def node(self):
        names = {'_start_execution', '_resume_execution', '_localization_watchdog_tick', '_pause_execution',
                 '_apply_emergency_stop', '_handle_manual_drive', '_handle_manual_drive_locked',
                 '_set_cmd_vel_forward_runtime_active', '_stop_execution', '_reset_emergency_stop',
                 '_request_indicator_clear', '_emergency_status_tick', '_apply_control_outputs',
                 '_driver_motion_safety', '_sync_deferred_driver_emergency', '_chassis_status_callback',
                 '_restore_indicator_reset', 'capture_control_generation', 'handle_control_command', 'cancel_manual_session'}
        cls, namespace = harness.load_class(harness.SCHEDULER, names)
        class RosError(Exception):
            pass
        ros = SimpleNamespace(ROSException=RosError, ServiceException=RosError,
            get_param=lambda key, default=None: default)
        ros.Time = SimpleNamespace(now=lambda: harness.Stamp(harness.time.monotonic()))
        for name in ('loginfo', 'logwarn', 'logerr', 'loginfo_throttle'):
            setattr(ros, name, lambda *a, **k: None)
        namespace['rospy'] = ros
        node = cls()
        node._motion = MotionInterlock(str(self.directory / 'scheduler.json'))
        node._localization_arm_lock = threading.Lock()
        node._localization_arm_count = 0
        node._localization_arming_cancelled = False
        node._emergency_reset_lock = threading.Lock()
        node._indicator_clear_generation = None
        node._emergency_reset_message = ''
        node._driver_emergency_latched = False
        node._driver_emergency_deferred = False
        node._task_enable_runtime_active = False
        node._chassis_settings = {'run_speed': 0.1}
        node._collision_imminent_lock = threading.Lock()
        node._collision_imminent_active = False
        node._tf_buffer = None
        node.state = SchedulerState.STOPPED
        node.task_config = M(task_id='task', active_work_region_id='')
        node.current_path = M(points=[{}])
        node._exec_mode = 'nav'
        node._exec_active = False
        node._manual_drive_lock = threading.Lock()
        node._indicator_emergency = SimpleNamespace(observe=lambda: None)
        node._super_lio_status_proxy = lambda: M(
            state='READY', localization_ready=True,
            active_map_id='map-a', active_map_revision='revision-a')
        node._localization_motion_status = lambda: (
            True, 'ready', {'map_id': 'map-a', 'map_revision': 'revision-a'})
        for name in ('_ensure_super_lio_proxies', '_sync_task_regions_from_overlay', '_prepare_task_continuation_start',
                     '_set_chassis_enabled', '_reset_disc_motion_guard', '_init_segment_execution_cursor',
                     '_publish_path_to_navigation', '_publish_active_segment_plan', '_begin_task_execution_record',
                     '_capture_task_continuation', '_finalize_task_result', '_safe_stop_motion'):
            setattr(node, name, lambda *a, **k: None)
        node._sorted_work_region_ids = lambda: ['region']
        node._configured_disc_speed_rpm = lambda: 1000
        node._apply_execution_speed_limit = lambda: True
        for name in ('task_enable_pub', 'disc_speed_pub', 'disc_enable_pub', 'work_mode_pub', 'goal_pub', 'wheel_cmd_pub'):
            safe = (lambda msg: getattr(msg, 'left_wheel_speed', 0) == 0 and getattr(msg, 'right_wheel_speed', 0) == 0) if name == 'wheel_cmd_pub' else (lambda msg: not msg.data)
            setattr(node, name, GuardedPublisher(harness.Publisher(), node._motion, safe))
        node._send_active_segment_goal = lambda **k: node.goal_pub.publish(M(data=True))
        for name in ('light_pub', 'move_base_cancel_pub', 'chassis_emergency_stop_pub', 'motion_safety_pub'):
            setattr(node, name, harness.Publisher())
        node._plan_current_task = lambda: True
        return node, ros

    def test_start_paused_at_localization_planning_and_output_never_revives(self):
        for stage in ('localization', 'planning', 'output'):
            with self.subTest(stage=stage):
                node, _ = self.node()
                entered, release = threading.Event(), threading.Event()
                result = []
                def barrier(value):
                    entered.set()
                    self.assertTrue(release.wait(2))
                    return value
                if stage == 'localization':
                    node._super_lio_status_proxy = lambda: barrier(M(
                        state='READY', localization_ready=True,
                        active_map_id='map-a', active_map_revision='revision-a'))
                elif stage == 'planning':
                    node.current_path = None
                    def plan():
                        barrier(None)
                        node.current_path = M(points=[{}])
                        return True
                    node._plan_current_task = plan
                else:
                    node._apply_execution_speed_limit = lambda: barrier(True)
                thread = threading.Thread(target=lambda: result.append(node._start_execution()))
                thread.start()
                self.assertTrue(entered.wait(1))
                node._apply_emergency_stop(True)
                # Even release must not make the old scope valid again.
                self.assertTrue(node._motion.reset(node._motion.generation))
                release.set()
                thread.join(2)
                self.assertFalse(thread.is_alive())
                self.assertFalse(result[0][0])
                self.assertEqual(SchedulerState.STOPPED, node.state)
                self.assertFalse(node._exec_active)
                self.assertFalse(any(m.data for m in node.task_enable_pub.messages))
                self.assertFalse(any(m.data for m in node.disc_enable_pub.messages))

    def test_localization_loss_during_planning_cancels_start_before_motion(self):
        node, _ = self.node()
        entered, release = threading.Event(), threading.Event()
        node.current_path = None

        def plan():
            entered.set()
            release.wait(2)
            node.current_path = M(points=[{}])
            return True

        node._plan_current_task = plan
        result = []
        thread = threading.Thread(target=lambda: result.append(node._start_execution()))
        thread.start()
        self.assertTrue(entered.wait(1))
        node._localization_motion_status = lambda: (False, 'localization_heartbeat_timeout', {})
        node._localization_watchdog_tick(None)
        release.set()
        thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertFalse(result[0][0])
        self.assertFalse(any(message.data for message in node.task_enable_pub.messages))
        self.assertFalse(any(message.data for message in node.disc_enable_pub.messages))

    def test_old_manual_scope_cannot_publish_after_release(self):
        node, _ = self.node()
        entered, release = threading.Event(), threading.Event()
        def manual(*args):
            entered.set()
            release.wait(2)
            node.wheel_cmd_pub.publish(M(left_wheel_speed=100, right_wheel_speed=100))
        node._handle_manual_drive_locked = manual
        result = []
        thread = threading.Thread(target=lambda: result.append(node._handle_manual_drive(1, 1)))
        thread.start()
        self.assertTrue(entered.wait(1))
        node._apply_emergency_stop(True)
        node._motion.reset(node._motion.generation)
        release.set()
        thread.join(1)
        self.assertEqual([False], result)
        self.assertEqual([0], [m.left_wheel_speed for m in node.wheel_cmd_pub.messages])

    def test_reset_wait_does_not_block_emergency_and_does_not_clear_new_trip(self):
        node, ros = self.node()
        node._apply_emergency_stop(True)
        entered, release = threading.Event(), threading.Event()
        ros.wait_for_service = lambda *a, **k: None
        def service():
            entered.set()
            release.wait(2)
            return M(success=True, message='driver verified')
        ros.ServiceProxy = lambda *a: service
        result = []
        thread = threading.Thread(target=lambda: result.append(node._reset_emergency_stop(None)))
        thread.start()
        self.assertTrue(entered.wait(1))
        node._apply_emergency_stop(True)
        release.set()
        thread.join(1)
        self.assertFalse(result[0].success)
        self.assertTrue(node._motion.latched)

    def test_successful_reset_queues_lights_without_resuming(self):
        node, ros = self.node()
        node._apply_emergency_stop(True)
        ros.wait_for_service = lambda *a, **k: None
        ros.ServiceProxy = lambda *a: lambda: M(success=True, message='verified')
        result = node._reset_emergency_stop(None)
        self.assertTrue(result.success)
        self.assertFalse(node._motion.latched)
        self.assertEqual(SchedulerState.STOPPED, node.state)
        self.assertFalse(node._exec_active)
        self.assertFalse(node._task_enable_runtime_active)
        self.assertIsNotNone(node._indicator_clear_generation)
        self.assertFalse(MotionInterlock(str(self.directory / 'scheduler.json')).latched)

    def test_missing_chassis_bench_releases_scheduler_without_resuming(self):
        node, ros = self.node()
        node._apply_emergency_stop(True)
        def missing(*a, **k):
            raise ros.ROSException('not present')
        ros.wait_for_service = missing
        ros.get_param = lambda *a: True
        result = node._reset_emergency_stop(None)
        self.assertTrue(result.success)
        self.assertIn('Test mode', result.message)
        self.assertFalse(node._motion.latched)
        self.assertFalse(MotionInterlock(str(self.directory / 'scheduler.json')).latched)
        self.assertEqual(SchedulerState.STOPPED, node.state)
        self.assertFalse(node._exec_active)
        self.assertFalse(node._task_enable_runtime_active)
        self.assertIsNotNone(node._indicator_clear_generation)

    def test_driver_failure_keeps_latch_and_does_not_clear_lights(self):
        node, ros = self.node()
        node._apply_emergency_stop(True)
        ros.wait_for_service = lambda *a, **k: None
        ros.ServiceProxy = lambda *a: lambda: M(success=False, message='not stopped')
        result = node._reset_emergency_stop(None)
        self.assertFalse(result.success)
        self.assertTrue(node._motion.latched)
        self.assertIsNone(node._indicator_clear_generation)

    def test_real_manual_method_rechecks_after_enable_wait(self):
        node, _ = self.node()
        node.sl_link_server = M(pb=M(MANUAL_MOTION_FORWARD=1))
        node._manual_speed_mps_to_rpm = lambda *a: 100
        node._manual_last_left_cmd = node._manual_last_right_cmd = 0
        node._manual_reverse_guard_sec = 0
        node._last_manual_enable_try = 0
        node.last_chassis_status = M(enabled=False)
        node._set_manual_travel_disc_active = lambda *a, **k: None
        entered, release = threading.Event(), threading.Event()
        def enable(*a):
            entered.set()
            release.wait(2)
        node._set_chassis_enabled = enable
        result = []
        worker = threading.Thread(target=lambda: result.append(node._handle_manual_drive(1, 1)))
        worker.start()
        self.assertTrue(entered.wait(1))
        node._apply_emergency_stop(True)
        release.set()
        worker.join(1)
        self.assertEqual([False], result)
        self.assertEqual([0], [m.left_wheel_speed for m in node.wheel_cmd_pub.messages])

    def test_bench_clear_when_driver_present_but_disconnected(self):
        node, ros = self.node()
        node._apply_emergency_stop(True)
        node.last_chassis_status = M(connected=False)
        ros.get_param = lambda *a: True
        ros.wait_for_service = lambda *a, **k: None
        ros.ServiceProxy = lambda *a: lambda: M(success=False, message='chassis disconnected')
        result = node._reset_emergency_stop(None)
        self.assertTrue(result.success)
        self.assertIn('chassis NOT confirmed', result.message)
        self.assertFalse(node._motion.latched)
        self.assertEqual(SchedulerState.STOPPED, node.state)
        self.assertIsNotNone(node._indicator_clear_generation)

    def test_missing_chassis_normal_mode_keeps_latch(self):
        node, ros = self.node()
        node._apply_emergency_stop(True)
        def missing(*a, **k):
            raise ros.ROSException('not present')
        ros.wait_for_service = missing
        result = node._reset_emergency_stop(None)
        self.assertFalse(result.success)
        self.assertTrue(node._motion.latched)
        self.assertIsNone(node._indicator_clear_generation)

    def test_persisted_bench_reset_survives_driver_replay_then_reconnect_latches(self):
        node, ros = self.node()
        node._apply_emergency_stop(True)
        ros.get_param = lambda *a: True
        def missing(*a, **k):
            raise ros.ROSException('not present')
        ros.wait_for_service = missing
        self.assertTrue(node._reset_emergency_stop(None).success)
        # Recreate scheduler from the persisted record, then replay the driver's
        # latched topic before its first chassis status has arrived.
        restarted, ros = self.node()
        ros.get_param = lambda *a: True
        self.assertFalse(restarted._motion.latched)
        restarted._restore_indicator_reset()
        self.assertEqual(restarted._motion.generation, restarted._indicator_clear_generation)
        replay = M(data=json.dumps({'emergency_latched': True}))
        restarted._driver_motion_safety(replay)
        restarted._chassis_status_callback(M(connected=False))
        restarted._driver_motion_safety(replay)
        self.assertFalse(restarted._motion.latched)
        restarted._chassis_status_callback(M(connected=True))
        self.assertTrue(restarted._motion.latched)

    def test_startup_reset_replay_requires_explicit_durable_reset(self):
        node, _ = self.node()
        node._restore_indicator_reset()
        self.assertIsNone(node._indicator_clear_generation)
        node._apply_emergency_stop(True)
        node._restore_indicator_reset()
        self.assertIsNone(node._indicator_clear_generation)
        self.assertTrue(node._motion.reset(node._motion.generation))
        node._restore_indicator_reset()
        self.assertIsNotNone(node._indicator_clear_generation)
        node._apply_emergency_stop(True)
        self.assertIsNone(node._indicator_clear_generation)
        node._restore_indicator_reset()
        self.assertIsNone(node._indicator_clear_generation)

    def test_restored_clear_retries_services_and_clears_both_records(self):
        node, ros = self.node()
        node._apply_emergency_stop(True)
        self.assertTrue(node._motion.reset(node._motion.generation))
        restarted, ros = self.node()
        ros.get_param = lambda *a: True
        restarted._restore_indicator_reset()
        def missing(*a, **k):
            raise ros.ROSException('indicator not ready')
        ros.wait_for_service = missing
        restarted._emergency_status_tick(None)
        self.assertIsNotNone(restarted._indicator_clear_generation)
        # Exercise the real indicator clear RPC: both sources start red again,
        # as a disconnected but motion-latched driver does after restarting.
        from test_status_indicator_integration import NodeIntegrationTest, Message
        fixture = NodeIntegrationTest()
        fixture.setUp()
        indicator = fixture.node
        indicator.resolver.ignore_chassis_status = True
        indicator._emergency('scheduler_emergency', Message(data=False))
        indicator._emergency('chassis_emergency', Message(data=True))
        cleared = []
        fixture.rospy.wait_for_service = lambda *a, **k: None
        def source_proxy(name, _type):
            def clear():
                cleared.append(name)
                source = 'scheduler_emergency' if name.startswith('/scheduler') else 'chassis_emergency'
                indicator._emergency(source, Message(data=False))
                return Message(success=True, message='cleared')
            return clear
        fixture.rospy.ServiceProxy = source_proxy
        ros.wait_for_service = lambda *a, **k: None
        ros.ServiceProxy = lambda *a: lambda: indicator._clear(None)
        restarted._emergency_status_tick(None)
        self.assertEqual(['/scheduler/clear_emergency_indicator', '/chassis/clear_emergency_indicator'], cleared)
        self.assertIsNone(restarted._indicator_clear_generation)
        self.assertTrue(indicator._clear_pending)
        factory = indicator.transport.factory
        def unplugged():
            raise IOError('USB absent during boot')
        indicator.transport.factory = unplugged
        indicator.transport.retry_seconds = 0
        indicator._step()
        self.assertTrue(indicator._clear_pending)
        indicator.transport.factory = factory
        indicator._step()
        from grinder_scheduler.status_indicator import unpack
        self.assertEqual('1', unpack(indicator.transport.port.frames[-1])[-1])
        self.assertFalse(indicator._clear_pending)

    def test_deferred_driver_lock_enforced_when_test_mode_disabled(self):
        node, ros = self.node()
        ros.get_param = lambda *a: True
        node._driver_motion_safety(M(data=json.dumps({'emergency_latched': True})))
        self.assertFalse(node._motion.latched)
        ros.get_param = lambda key, default=None: default
        node._sync_deferred_driver_emergency()
        self.assertTrue(node._motion.latched)

    def test_normal_mode_driver_replay_and_new_test_emergency_still_latch(self):
        node, ros = self.node()
        node._driver_motion_safety(M(data=json.dumps({'emergency_latched': True})))
        self.assertTrue(node._motion.latched)
        self.assertTrue(node._motion.reset(node._motion.generation))
        ros.get_param = lambda *a: True
        node._apply_emergency_stop(True)
        self.assertTrue(node._motion.latched)

    def test_connected_chassis_failure_not_bypassed_in_bench_mode(self):
        node, ros = self.node()
        node._apply_emergency_stop(True)
        node.last_chassis_status = M(connected=True)
        ros.get_param = lambda *a: True
        ros.wait_for_service = lambda *a, **k: None
        ros.ServiceProxy = lambda *a: lambda: M(success=False, message='not stopped')
        result = node._reset_emergency_stop(None)
        self.assertFalse(result.success)
        self.assertTrue(node._motion.latched)
        self.assertIsNone(node._indicator_clear_generation)

    def test_new_emergency_during_bench_reset_does_not_queue_clear(self):
        node, ros = self.node()
        node._apply_emergency_stop(True)
        ros.get_param = lambda *a: True
        ros.wait_for_service = lambda *a, **k: None
        def failed_reset():
            node._apply_emergency_stop(True)
            return M(success=False, message='disconnected')
        ros.ServiceProxy = lambda *a: failed_reset
        self.assertFalse(node._reset_emergency_stop(None).success)
        self.assertIsNone(node._indicator_clear_generation)

    def test_old_light_clear_rpc_reasserts_new_emergency(self):
        for lost_response in (False, True):
            with self.subTest(lost_response=lost_response):
                node, ros = self.node()
                observations = []
                node._indicator_emergency.observe = lambda: observations.append(True)
                node._apply_emergency_stop(True)
                node._request_indicator_clear()
                ros.wait_for_service = lambda *a, **k: None
                def raced_clear():
                    node._apply_emergency_stop(True)
                    if lost_response:
                        raise ros.ServiceException('response lost after source clear')
                    return M(success=True, message='old clear completed')
                ros.ServiceProxy = lambda *a: raced_clear
                node._emergency_status_tick(None)
                self.assertTrue(node._motion.latched)
                self.assertIsNone(node._indicator_clear_generation)
                self.assertEqual(3, len(observations))


if __name__ == '__main__':
    unittest.main()
