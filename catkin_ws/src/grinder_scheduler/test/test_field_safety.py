"""Offline receiver leases, wire fairness, cache ownership and readiness reasons."""
import sys
import time
import threading
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'catkin_ws/src/grinder_scheduler/src'))
sys.path.insert(0, str(ROOT/'catkin_ws/src/grinder_chassis_driver/src'))
from grinder_scheduler.manual_delivery import ManualDelivery
from grinder_scheduler.priority_wire import PriorityWire
from grinder_scheduler.planning_cache import PlanningCache
from grinder_scheduler.registration_health import explain
from grinder_chassis_driver.motion_safety import MotionInterlock, MotionCancelled, GuardedPublisher


class DeliveryTest(unittest.TestCase):
    def test_expiry_release_watchdog_and_reconnection_cancel_old_output(self):
        delivery = ManualDelivery()
        session = object()
        delivery.begin(session)
        with tempfile.TemporaryDirectory() as directory:
            interlock = MotionInterlock(str(Path(directory)/'motion.json'))
            outputs = []
            publisher = GuardedPublisher(SimpleNamespace(publish=outputs.append), interlock, lambda x: x == 0)
            for mode in ('expired', 'release', 'watchdog', 'connection', 'emergency'):
                with self.subTest(mode=mode):
                    ticket = delivery.capture(session, time.monotonic(), .5)
                    entered, release = threading.Event(), threading.Event()
                    def execute():
                        try:
                            interlock.local.delivery_validator = lambda: delivery.valid(ticket)
                            with interlock.action():
                                entered.set()
                                release.wait(1)
                                publisher.publish(100)
                        except MotionCancelled:
                            pass
                    worker = threading.Thread(target=execute)
                    worker.start()
                    self.assertTrue(entered.wait(1))
                    if mode == 'expired':
                        time.sleep(.51)
                    elif mode == 'release':
                        delivery.capture(session, time.monotonic(), .5, stop=True)
                    elif mode == 'connection':
                        session = object()
                        delivery.begin(session)
                    elif mode == 'emergency':
                        interlock.invalidate(emergency=True)
                    else:
                        delivery.invalidate()
                    publisher.publish(0)
                    release.set(); worker.join(1)
                    self.assertFalse(worker.is_alive())
                    self.assertNotIn(100, outputs)
                    if mode == 'emergency':
                        self.assertTrue(interlock.reset(interlock.generation))

    def test_old_connection_cannot_reclaim_control_or_cancel_new_connection(self):
        gate = ManualDelivery(); old, new = object(), object()
        gate.begin(old); gate.begin(new)
        ticket = gate.capture(new, time.monotonic(), .5)
        self.assertFalse(gate.valid(gate.capture(old, time.monotonic(), .5)))
        self.assertFalse(gate.invalidate(old))
        self.assertTrue(gate.valid(ticket))

    def test_waiting_ack_precedes_next_bulk_frame(self):
        wire = PriorityWire(); result = []; threads = []
        def send(control):
            with wire.hold(control): result.append(control)
        with wire.hold():
            for control in (False, True):
                worker = threading.Thread(target=send, args=(control,)); worker.start(); threads.append(worker)
            deadline = time.monotonic()+1
            while wire.controls == 0 and time.monotonic()<deadline: time.sleep(.001)
            self.assertEqual(1, wire.controls)
        for worker in threads: worker.join(1)
        self.assertEqual([True, False], result)


class CacheTest(unittest.TestCase):
    def test_immutable_bases_private_derivatives_and_invalidation(self):
        cache = PlanningCache(); config = SimpleNamespace(inflation=.3)
        grid = np.zeros((10,10), dtype=np.uint8)
        count = []
        def build():
            count.append(1)
            raw = grid.copy(); raw.setflags(write=False)
            return SimpleNamespace(raw_grid=raw, inflated_grid=raw, _cache={}, _cache_bytes=0)
        first = cache.prepare(grid, {'origin':0}, config, build)
        first._cache['mutable'] = np.ones(3)
        second = cache.prepare(grid, {'origin':0}, config, build)
        self.assertEqual(1, len(count)); self.assertEqual({}, second._cache)
        with self.assertRaises(ValueError): second.raw_grid[0,0] = 1
        grid[0,0] = 1; cache.prepare(grid, {'origin':0}, config, build)
        config.inflation=.5; cache.prepare(grid, {'origin':0}, config, build)
        self.assertLessEqual(len(cache.items), 2)
        cache.prepare(grid, {'origin':1}, config, build)
        self.assertEqual(4, len(count)); self.assertEqual(1, len(cache.items))
        tiny = PlanningCache(budget=1)
        tiny.prepare(grid, {}, config, build)
        self.assertEqual(0, tiny.bytes)

    def test_parallel_builds_keep_private_caches_and_budget(self):
        cache = PlanningCache(entries=2, budget=400)
        grid = np.zeros((10,10),dtype=np.uint8)
        barrier = threading.Barrier(2); results=[]
        def build():
            barrier.wait(timeout=2)
            raw=grid.copy(); raw.setflags(write=False)
            return SimpleNamespace(raw_grid=raw,inflated_grid=raw,_cache={},_cache_bytes=0)
        def run():
            results.append(cache.prepare(grid,{},SimpleNamespace(radius=.3),build))
        threads=[threading.Thread(target=run) for _ in range(2)]
        for worker in threads: worker.start()
        for worker in threads: worker.join(3)
        self.assertEqual(2,len(results))
        self.assertIsNot(results[0]._cache,results[1]._cache)
        self.assertEqual(200,cache.bytes)
        self.assertEqual(1,len(cache.items))


class QualityTest(unittest.TestCase):
    def test_all_failed_gates_are_reported_without_changing_thresholds(self):
        q = dict(ok=False, converged=False, fitness=.31, inlier_ratio=.19, correspondences=99)
        c = [0.0]*36; c[0]=.26
        result = explain(q,c,False,.25,.07,.3,.2,100)
        self.assertEqual(7, len(result['failure_reasons'].split(',')))
        self.assertIn('covariance_invalid', explain(q,[],False,.25,.07,.3,.2,100)['failure_reasons'])
        q.update(ok=True,converged=True,fitness=.3,inlier_ratio=.2,correspondences=100);c[0]=.25
        self.assertEqual('', explain(q,c,True,.25,.07,.3,.2,100)['failure_reasons'])

if __name__ == '__main__': unittest.main()
