#!/usr/bin/env python3

import json
import os
import sys
import tempfile
import threading
import unittest
from types import SimpleNamespace


SOURCE = os.path.abspath(os.path.join(os.path.dirname(__file__), "../src"))
if SOURCE not in sys.path:
    sys.path.insert(0, SOURCE)

from grinder_scheduler.metrics_exporter import MetricsExporter  # noqa: E402
from grinder_scheduler.metrics_registry import MetricsRegistry  # noqa: E402
from grinder_scheduler.sl_linka_adapter import SlLinkAServer  # noqa: E402


class MetricsTest(unittest.TestCase):
    def test_thread_safe_counts_and_bounded_events(self):
        registry = MetricsRegistry(event_capacity=3)
        counter = registry.counter("rx")
        threads = [threading.Thread(target=lambda: [counter.inc() for _ in range(1000)]) for _ in range(4)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        for index in range(5):
            registry.event("transition", number=index)
        registry.histogram("wait_ms").observe(2.0)
        registry.histogram("wait_ms").observe(6000.0)
        snapshot = registry.snapshot()
        self.assertEqual(snapshot["counters"]["rx"], 4000)
        self.assertEqual(len(snapshot["events"]), 3)
        self.assertEqual(snapshot["histograms"]["wait_ms"]["count"], 2)
        self.assertEqual(snapshot["histograms"]["wait_ms"]["over_max_ms"], 1)

    def test_export_replaces_single_json_snapshot(self):
        registry = MetricsRegistry()
        with tempfile.TemporaryDirectory() as directory:
            exporter = MetricsExporter(registry, directory, "scheduler")
            registry.counter("rx").inc()
            first_path = exporter.export_once()
            registry.counter("rx").inc()
            exporter.export_once()
            with open(first_path, "r", encoding="utf-8") as handle:
                snapshot = json.load(handle)
            self.assertEqual(snapshot["counters"]["rx"], 2)
            self.assertEqual(os.listdir(directory), [os.path.basename(first_path)])

    def test_timed_operation_records_failed_call(self):
        registry = MetricsRegistry()

        @registry.timed("map_build_ms")
        def failing_build():
            raise RuntimeError("encode failed")

        with self.assertRaisesRegex(RuntimeError, "encode failed"):
            failing_build()
        self.assertEqual(registry.snapshot()["histograms"]["map_build_ms"]["count"], 1)

    def test_map_request_records_build_and_socket_send_separately(self):
        registry = MetricsRegistry()
        server = SlLinkAServer.__new__(SlLinkAServer)
        server.pb = SimpleNamespace(
            MSG_ID_SETTINGS_READ_REQUEST=1,
            MSG_ID_SETTINGS_WRITE_REQUEST=2,
            MSG_ID_CONTROL_COMMAND=3,
            MSG_ID_TASK_CONFIG=4,
            MSG_ID_TASK_COMMAND=5,
            MSG_ID_PATH_POINT_PLAN_REQUEST=6,
            MSG_ID_CAMERA_FRAME_REQUEST=7,
            MSG_ID_MAP_REQUEST=8,
        )
        server._handler = SimpleNamespace(build_map_chunks=lambda _payload: [(b"a", 9, 1), (b"b", 9, 1)])
        server._map_chunks_sent = registry.counter("map_chunks_sent")
        server._map_send_total_ms = registry.histogram("map_send_total_ms")
        server._map_socket_send_total_ms = registry.histogram("map_socket_send_total_ms")
        server._map_request_total_ms = registry.histogram("map_request_total_ms")
        server._map_send_failures = registry.counter("map_send_failures")
        server._map_build_failures = registry.counter("map_build_failures")
        sender = SimpleNamespace(_send_payload=lambda *_args, **_kwargs: (1.0, 2.0, 10))
        server._dispatch_frame(sender, SimpleNamespace(msg_id=8, payload=b"", seq=1), log_rx=False)
        snapshot = registry.snapshot()
        self.assertEqual(snapshot["counters"]["map_chunks_sent"], 2)
        self.assertEqual(snapshot["histograms"]["map_socket_send_total_ms"]["sum"], 4.0)
        self.assertEqual(snapshot["histograms"]["map_send_total_ms"]["count"], 1)


if __name__ == "__main__":
    unittest.main()
