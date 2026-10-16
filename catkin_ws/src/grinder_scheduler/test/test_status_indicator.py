import os
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../src")))
from grinder_scheduler.status_indicator import (
    IndicatorTransport, LightState, SimulatedPort, StateResolver,
    brightness_from_percent, envelope, state_frame, unpack, valid_ack,
)


class StatePolicyTest(unittest.TestCase):
    def ready(self, state="IDLE", automatic=False, manual=False, disc=False):
        resolver = StateResolver()
        values = {
            "scheduler": {"state": state},
            "chassis": {"connected": True, "disc_enabled": disc, "work_mode": 2},
            "activity": {"automatic_active": automatic, "manual_active": manual},
            "scheduler_emergency": False, "chassis_emergency": False,
        }
        for key, value in values.items():
            resolver.update(key, value, 10.0)
        return resolver

    def test_initial_incomplete_stays_yellow(self):
        resolver = StateResolver()
        self.assertEqual(LightState.STARTUP, resolver.resolve(100))
        resolver.update("scheduler", {"state": "IDLE"}, 100)
        self.assertEqual(LightState.STARTUP, resolver.resolve(100))
        self.assertEqual(LightState.STARTUP, resolver.resolve(1000))

    def test_localization_pause_keeps_fault_until_cleared_and_emergency_wins(self):
        resolver = self.ready("PAUSED")
        resolver.update("activity", {"automatic_active": False, "manual_active": False,
                                     "pause_reason": "localization"}, 10)
        self.assertEqual(LightState.FAULT, resolver.resolve(10))
        resolver.update("scheduler_emergency", True, 10)
        self.assertEqual(LightState.EMERGENCY, resolver.resolve(10))

    def test_all_states_and_auto_not_inferred_from_work_mode(self):
        cases = [
            ("IDLE", False, LightState.READY), ("READY", False, LightState.READY),
            ("PLANNING", False, LightState.READY), ("STOPPED", False, LightState.READY),
            ("RUNNING", True, LightState.AUTOMATIC), ("PAUSED", False, LightState.PAUSED),
            ("COMPLETED", False, LightState.COMPLETED), ("ERROR", False, LightState.FAULT),
            ("INVALID", False, LightState.FAULT), ("RUNNING", False, LightState.FAULT),
        ]
        for state, automatic, expected in cases:
            with self.subTest(state=state):
                self.assertEqual(expected, self.ready(state, automatic).resolve(10.1))

    def test_manual_activity_and_non_auto_disc(self):
        self.assertEqual(LightState.MANUAL, self.ready(manual=True).resolve(10))
        self.assertEqual(LightState.MANUAL, self.ready(disc=True).resolve(10))
        self.assertEqual(LightState.AUTOMATIC, self.ready("RUNNING", automatic=True, disc=True).resolve(10))
        self.assertEqual(LightState.PAUSED, self.ready("PAUSED", disc=True).resolve(10))

    def test_emergency_wins_over_running_error_and_stale_data(self):
        resolver = self.ready("RUNNING", automatic=True)
        resolver.update("chassis_emergency", True, 1)
        self.assertEqual(LightState.EMERGENCY, resolver.resolve(100))
        resolver.update("scheduler", {"state": "ERROR"}, 100)
        self.assertEqual(LightState.EMERGENCY, resolver.resolve(100))

    def test_ready_then_each_source_timeout_is_red(self):
        for key in StateResolver.REQUIRED:
            resolver = self.ready()
            self.assertEqual(LightState.READY, resolver.resolve(10))
            for name, (value, _) in list(resolver.samples.items()):
                if name != key:
                    resolver.update(name, value, 14)
            self.assertEqual(LightState.FAULT, resolver.resolve(14))
            resolver.update(key, resolver.samples[key][0], 14)
            self.assertEqual(LightState.READY, resolver.resolve(14))

    def test_disconnection_is_red_even_before_first_ready(self):
        resolver = StateResolver()
        resolver.update("chassis", {"connected": False}, 0)
        self.assertEqual(LightState.FAULT, resolver.resolve(0))

    def test_bench_mode_skips_missing_stale_and_disconnected_chassis(self):
        resolver = StateResolver(ignore_chassis_status=True)
        for now in (10, 20):
            resolver.update("scheduler", {"state": "IDLE"}, now)
            resolver.update("activity", {"automatic_active": False, "manual_active": False}, now)
            resolver.update("scheduler_emergency", False, now)
            self.assertEqual(LightState.READY, resolver.resolve(now))
            resolver.update("chassis", {"connected": False, "disc_enabled": True}, now)
            self.assertEqual(LightState.READY, resolver.resolve(now))
        # Other required sources still time out; received emergency still wins.
        self.assertEqual(LightState.FAULT, resolver.resolve(24))
        resolver.update("chassis_emergency", True, 1)
        self.assertEqual(LightState.EMERGENCY, resolver.resolve(24))

    def test_collision_pause_is_distinct_and_emergency_fault_still_win(self):
        resolver = self.ready("PAUSED")
        resolver.update("activity", {"automatic_active": False, "manual_active": False,
                                     "pause_reason": "collision"}, 10)
        self.assertEqual(LightState.COLLISION_PAUSED, resolver.resolve(10))
        resolver.update("activity", {"automatic_active": False, "manual_active": True,
                                     "pause_reason": "collision"}, 10)
        self.assertEqual(LightState.COLLISION_PAUSED, resolver.resolve(10))
        # The RPP clear signal is not a lamp input; the scheduler retains cause.
        resolver.update("scheduler", {"state": "PAUSED"}, 11)
        self.assertEqual(LightState.COLLISION_PAUSED, resolver.resolve(11))
        resolver.update("chassis", {"connected": False}, 11)
        self.assertEqual(LightState.FAULT, resolver.resolve(11))
        resolver.update("scheduler_emergency", True, 11)
        self.assertEqual(LightState.EMERGENCY, resolver.resolve(11))


class TransportTest(unittest.TestCase):
    SESSION = "0123456789abcdef"

    def test_crc_known_vector_and_corruption(self):
        self.assertEqual(b"123456789*29B1\n", envelope("123456789"))
        frame = state_frame(self.SESSION, 1, LightState.AUTOMATIC)
        self.assertEqual(["S1", self.SESSION, "1", "2", "166", "0"], unpack(frame))
        with self.assertRaises(ValueError):
            unpack(frame.replace(b",2,", b",3,"))
        self.assertTrue(valid_ack(envelope("A1,{},1".format(self.SESSION)), self.SESSION, 1))
        self.assertFalse(valid_ack(envelope("A1,{},2".format(self.SESSION)), self.SESSION, 1))
        self.assertFalse(valid_ack(b"noise\n", self.SESSION, 1))

    def test_bounds_and_explicit_clear(self):
        for kwargs in ({"brightness": 256}, {"brightness": -1}, {"clear": True}):
            with self.assertRaises(ValueError):
                state_frame(self.SESSION, 1, LightState.EMERGENCY, **kwargs)
        with self.assertRaises(ValueError):
            state_frame(self.SESSION, 0, LightState.READY)
        self.assertEqual("1", unpack(state_frame(self.SESSION, 1, LightState.READY, clear=True))[-1])

    def test_configurable_percentage_brightness(self):
        for percent, raw in ((1, 3), (40, 102), (65, 166), (100, 255)):
            self.assertEqual(raw, brightness_from_percent(percent))
            self.assertEqual(str(raw), unpack(state_frame(self.SESSION, 1, LightState.READY, raw))[4])
        for value in (0, -1, 101, float('nan'), float('inf'), True):
            with self.assertRaises(ValueError):
                brightness_from_percent(value)

    def test_simulated_wire_path(self):
        port = SimulatedPort()
        transport = IndicatorTransport(lambda: port)
        self.assertTrue(transport.send(LightState.READY, 0))
        self.assertTrue(transport.send(LightState.AUTOMATIC, 0.5))
        self.assertEqual("2", unpack(port.frames[-1])[2])

    def test_failed_ack_reconnect_sends_latest_only(self):
        class FailedPort(SimulatedPort):
            def readline(self, size):
                return b""
        bad, good = FailedPort(), SimulatedPort()
        devices = iter([bad, good])
        transport = IndicatorTransport(lambda: next(devices))
        self.assertFalse(transport.send(LightState.AUTOMATIC, 0))
        old_session = transport.session
        self.assertTrue(bad.closed)
        self.assertFalse(transport.send(LightState.EMERGENCY, 1))
        self.assertTrue(transport.send(LightState.EMERGENCY, 2))
        self.assertNotEqual(old_session, transport.session)
        self.assertEqual(1, len(good.frames))
        self.assertEqual("7", unpack(good.frames[0])[3])
        self.assertEqual("1", unpack(good.frames[0])[2])


if __name__ == "__main__":
    unittest.main()
