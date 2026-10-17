"""Shared Python safety gates and scheduler AST harness; C++ driver tests live in gtest/rostest."""
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

def load_class(relative, names=None):
    path = ROOT / relative
    parsed = ast.parse(path.read_text(encoding='utf-8'))
    cls = next(n for n in parsed.body if isinstance(n, ast.ClassDef) and n.name in ('GrinderSchedulerNode', 'SchedulerNode'))
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

SCHEDULER = 'catkin_ws/src/grinder_scheduler/src/grinder_scheduler/scheduler_node.py'

class SafetyTest(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.directory = Path(temp.name)

    def gate(self, name='motion'):
        return MotionInterlock(str(self.directory / (name + '.json')))

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
