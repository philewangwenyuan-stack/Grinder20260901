"""Diagnostic disk stalls cannot block control producers or grow memory."""
import tempfile
import threading
from pathlib import Path
import unittest

from grinder_chassis_driver.raw_log import RawLogWriter


class RawLogTest(unittest.TestCase):
    def test_blocked_disk_bounds_backlog_and_reports_dropped_lines(self):
        entered,release=threading.Event(),threading.Event()
        written=[]
        writer=RawLogWriter('/unused',capacity=4)
        self.addCleanup(writer.close)
        self.addCleanup(release.set)
        def slow_write(lines):
            entered.set();release.wait(2);written.extend(lines)
        writer._write=slow_write
        writer.append('TX first')
        self.assertTrue(entered.wait(1))
        for i in range(1000):writer.append('RX '+str(i))
        self.assertEqual(4,len(writer.pending))
        self.assertGreaterEqual(writer.dropped,996)
        release.set();writer.close(1)
        self.assertFalse(writer.thread.is_alive())
        self.assertTrue(any('RAW_LOG_DROPPED' in line for line in written))
        self.assertTrue(any('RX 999' in line for line in written))

    def test_timestamp_and_payload_are_saved_after_shutdown_flush(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'raw.log'
            writer=RawLogWriter(str(path))
            writer.append('TX 01 10')
            writer.append('RX 01 10')
            writer.close(1)
            text=path.read_text()
            self.assertIn('] TX 01 10\n',text)
            self.assertIn('] RX 01 10\n',text)


if __name__=='__main__':unittest.main()
