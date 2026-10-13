import json
import os
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../src")))
from grinder_chassis_driver.indicator_emergency import EmergencyRecord


class EmergencyRecordTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = os.path.join(self.directory.name, "record.json")

    def test_persistence_and_display_only_reset(self):
        record = EmergencyRecord(self.path)
        self.assertFalse(record.snapshot()[0])
        record.observe()
        self.assertFalse(os.path.exists(self.path))  # no I/O on stop path
        record.flush()
        self.assertTrue(EmergencyRecord(self.path).snapshot()[0])
        self.assertTrue(record.clear())
        self.assertFalse(EmergencyRecord(self.path).snapshot()[0])

    def test_unreadable_record_is_conservative(self):
        for value in ("broken", "[]", '{"emergency_observed": "false"}'):
            with open(self.path, "w") as target:
                target.write(value)
            self.assertTrue(EmergencyRecord(self.path).snapshot()[0])

    def test_failed_reset_keeps_emergency(self):
        record = EmergencyRecord(self.path)
        record.observe()
        record.flush()
        with patch("os.replace", side_effect=OSError("disk failure")):
            with self.assertRaises(OSError):
                record.clear()
        self.assertTrue(record.snapshot()[0])
        self.assertTrue(EmergencyRecord(self.path).snapshot()[0])

    def test_new_emergency_during_clear_wins(self):
        record = EmergencyRecord(self.path)
        record.observe()
        record.flush()
        original = record.flush
        def flush_with_event():
            record.observe()
            original()
        with patch.object(record, "flush", side_effect=flush_with_event):
            self.assertFalse(record.clear())
        self.assertTrue(record.snapshot()[0])
        self.assertTrue(EmergencyRecord(self.path).snapshot()[0])

    def test_disk_io_does_not_block_observe(self):
        record = EmergencyRecord(self.path)
        record._io_lock.acquire()
        try:
            worker = threading.Thread(target=record.observe)
            worker.start()
            worker.join(0.5)
            self.assertFalse(worker.is_alive())
        finally:
            record._io_lock.release()


if __name__ == "__main__":
    unittest.main()
