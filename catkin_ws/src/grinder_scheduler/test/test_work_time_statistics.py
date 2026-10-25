"""Independent counters and production history/delete integration, no ROS/hardware."""
import ast
import datetime
import json
import math
import os
import pathlib
import re
import shutil
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest import mock

PACKAGE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'src'))
sys.path.insert(0, str(PACKAGE.parents[2] / 'third_party/sl_linka/sdk/python'))
from grinder_scheduler.work_time_statistics import WorkTimeStatistics
from sl_link.message_gen import sl_link_pb2 as pb

ZONE = datetime.timezone(datetime.timedelta(hours=8))


class Clock:
    def __init__(self, wall=1792945507):
        self.wall = float(wall)
        self.mono = 10.0

    def advance(self, seconds):
        self.wall += seconds
        self.mono += seconds


def scheduler_class():
    path = PACKAGE / 'src/grinder_scheduler/scheduler_node.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'SchedulerNode')
    names = {'build_task_execution_history_chunks', 'handle_task_execution_delete_request',
             '_save_task_registry_state', '_begin_task_execution_record', '_finalize_task_result'}
    cls.body = [node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name in names]
    namespace = dict(json=json, math=math, os=os, re=re, shutil=shutil, time=time,
                     rospy=types.SimpleNamespace(loginfo=lambda *a: None, logwarn=lambda *a: None,
                                                 logwarn_throttle=lambda *a: None))
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace['SchedulerNode']


class WorkTimeStatisticsTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = os.path.join(self.directory.name, 'work_time_statistics.json')
        self.legacy = os.path.join(self.directory.name, 'task_registry.json')
        self.clock = Clock()

    def ledger(self, writer=False):
        return WorkTimeStatistics(self.path, self.legacy, wall=lambda: self.clock.wall,
                                  monotonic=lambda: self.clock.mono, timezone=ZONE, start_writer=writer)

    def history(self, records):
        pathlib.Path(self.legacy).write_text(json.dumps(dict(task_executions=records)), encoding='utf-8')

    def record(self, identity='old', start=1792845488, finish=1792846222):
        return dict(execution_id=identity, started_at=start, finished_at=finish)

    def test_import_once_survives_delete_and_restart(self):
        self.history([self.record(), self.record()])
        ledger = self.ledger()
        self.assertEqual(734, ledger.snapshot()['total_work_seconds'])
        self.history([])
        restored = self.ledger()
        self.assertEqual(734, restored.snapshot()['total_work_seconds'])
        self.history([self.record('extra')])
        self.assertEqual(734, self.ledger().snapshot()['total_work_seconds'])

    def test_field_baseline_imports_3175_seconds_and_marks_two_unknown(self):
        pairs = [(1792811902,1792812015),(1792834855,1792834891),(1792839856,1792839870),
                 (1792841582,1792841805),(1792842804,1792843012),(1792843519,1792845366),
                 (1792845488,1792846222),(1792812070,0),(1792842301,0)]
        self.history([self.record(str(i), start, end) for i,(start,end) in enumerate(pairs)])
        result = self.ledger().snapshot()
        self.assertTrue(result['available'])
        self.assertEqual(3175, result['total_work_seconds'])
        self.assertEqual(0, result['today_work_seconds'])
        self.assertEqual(2, result['incomplete_record_count'])

    def test_midnight_split_live_and_legacy(self):
        self.clock.wall = datetime.datetime(2026,10,25,23,59,50,tzinfo=ZONE).timestamp()
        ledger = self.ledger()
        ledger.start('live')
        self.clock.advance(20)
        result = ledger.snapshot()
        self.assertEqual(20, result['total_work_seconds'])
        self.assertEqual(10, result['today_work_seconds'])
        ledger.finish(); ledger.flush()
        saved = json.loads(pathlib.Path(self.path).read_text())
        self.assertEqual({'2026-10-25':10000,'2026-10-26':10000}, saved['daily_work_ms'])
        os.remove(self.path)
        start = int(self.clock.wall)-20
        self.history([self.record(start=start,finish=start+20)])
        self.assertEqual(10, self.ledger().snapshot()['today_work_seconds'])

    def test_normal_stop_repeated_reads_and_repeated_start_do_not_double_count(self):
        ledger = self.ledger(); ledger.start('live'); self.clock.advance(15)
        ledger.start('live')
        for _ in range(5): self.assertEqual(15, ledger.snapshot()['total_work_seconds'])
        self.clock.advance(5); ledger.finish(); ledger.finish(); ledger.flush()
        self.clock.advance(10000)
        self.assertEqual(20, self.ledger().snapshot()['total_work_seconds'])
        self.assertEqual(0, self.ledger().snapshot()['incomplete_record_count'])

    def test_reboot_preserves_checkpoint_excludes_downtime_and_marks_tail_once(self):
        ledger = self.ledger(); ledger.start('live'); self.clock.advance(15); ledger.flush()
        self.clock.advance(86400)
        restored = self.ledger()
        self.assertEqual(15, restored.snapshot()['total_work_seconds'])
        self.assertEqual(0, restored.snapshot()['today_work_seconds'])
        self.assertEqual(1, restored.snapshot()['incomplete_record_count'])
        self.assertEqual(1, self.ledger().snapshot()['incomplete_record_count'])
        restored.start('next'); self.clock.advance(5); restored.finish(); restored.flush()
        self.assertEqual(20, self.ledger().snapshot()['total_work_seconds'])

    def test_wall_clock_adjustment_never_adds_offline_or_negative_elapsed_time(self):
        ledger = self.ledger(); ledger.start('live')
        self.clock.wall += 3600; self.clock.mono += 10
        self.assertEqual(10, ledger.snapshot()['total_work_seconds'])
        self.clock.wall -= 7200; self.clock.mono += 5
        self.assertEqual(15, ledger.snapshot()['total_work_seconds'])

    def test_corrupt_counter_never_reimports_or_overwrites_original(self):
        self.history([self.record()])
        pathlib.Path(self.path).write_text('{broken', encoding='utf-8')
        ledger = self.ledger(); ledger.start('new'); self.clock.advance(20); ledger.flush()
        self.assertFalse(ledger.snapshot()['available'])
        self.assertEqual('{broken', pathlib.Path(self.path).read_text())

    def test_inconsistent_schema_rejected(self):
        pathlib.Path(self.path).write_text(json.dumps(dict(schema_version=1,total_work_ms=1000,daily_work_ms={})))
        self.assertFalse(self.ledger().snapshot()['available'])

    def test_corrupt_legacy_does_not_create_zero_baseline(self):
        pathlib.Path(self.legacy).write_text('{broken')
        self.assertFalse(self.ledger().snapshot()['available'])
        self.assertFalse(os.path.exists(self.path))

    def test_write_failure_retains_previous_file_and_recovers_without_reset(self):
        ledger = self.ledger(); before = pathlib.Path(self.path).read_bytes()
        ledger.start('live'); self.clock.advance(11)
        with mock.patch('grinder_scheduler.work_time_statistics.os.replace', side_effect=OSError('disk full')):
            self.assertFalse(ledger.flush())
            self.assertFalse(ledger.snapshot()['available'])
        self.assertEqual(before, pathlib.Path(self.path).read_bytes())
        self.assertTrue(ledger.flush()); ledger.finish(); ledger.flush()
        self.assertEqual(11, self.ledger().snapshot()['total_work_seconds'])

    def test_idle_does_not_rewrite_file_every_checkpoint(self):
        ledger = self.ledger()
        with mock.patch('grinder_scheduler.work_time_statistics.os.replace') as replace:
            self.clock.advance(20); ledger.flush(); ledger.flush()
            replace.assert_not_called()

    def test_disk_io_does_not_hold_accounting_lock(self):
        ledger = self.ledger(); ledger.start('live'); self.clock.advance(1)
        entered, release = threading.Event(), threading.Event()
        original_replace = os.replace
        def slow_replace(src,dst):
            entered.set(); release.wait(2); original_replace(src,dst)
        with mock.patch('grinder_scheduler.work_time_statistics.os.replace', side_effect=slow_replace):
            writer = threading.Thread(target=ledger.flush); writer.start()
            self.assertTrue(entered.wait(1))
            try:
                # Must finish while the writer is blocked, without waiting for disk I/O.
                completed = threading.Event()
                control = threading.Thread(target=lambda:(ledger.finish(),completed.set()))
                control.start()
                self.assertTrue(completed.wait(.5))
            finally:
                release.set(); writer.join(2); control.join(2)
        ledger.flush()

    def test_async_close_commits_final_counter(self):
        ledger = self.ledger(writer=True); ledger.start('live'); self.clock.advance(17); ledger.close()
        self.assertFalse(ledger._thread.is_alive())
        restored = self.ledger()
        self.assertEqual(17, restored.snapshot()['total_work_seconds'])
        self.assertEqual(0, restored.snapshot()['incomplete_record_count'])

    def node(self, ledger):
        node = scheduler_class()()
        node.sl_link_server = types.SimpleNamespace(pb=pb)
        node._work_time_statistics = ledger
        node._task_execution_records = [self.record()]
        node._active_task_execution_id = ''
        node._persist_state_dir = self.directory.name
        node._task_registry_state_file = self.legacy
        node._task_bindings = {}; node._persist_state_enabled = True
        node._task_execution_preview_base64 = lambda _: ''
        return node

    def test_production_delete_and_filtered_history_do_not_reset_counters(self):
        self.history([self.record()]); ledger = self.ledger(); node = self.node(ledger)
        request = pb.TaskExecutionDeleteRequest(execution_id='old')
        response = pb.TaskExecutionDeleteResponse()
        response.ParseFromString(node.handle_task_execution_delete_request(request.SerializeToString())[0])
        self.assertTrue(response.deleted)
        self.assertEqual([], json.loads(pathlib.Path(self.legacy).read_text())['task_executions'])
        query = pb.TaskExecutionHistoryRequest(map_id='another-map')
        chunks = node.build_task_execution_history_chunks(query.SerializeToString())
        data = []
        for raw,_,_ in chunks:
            chunk = pb.TaskExecutionHistoryChunk(); chunk.ParseFromString(raw); data.append(bytes(chunk.data))
        payload = json.loads(b''.join(data))
        self.assertEqual([], payload['records'])
        self.assertEqual(734, payload['work_time_statistics']['total_work_seconds'])
        self.assertEqual(734, self.ledger().snapshot()['total_work_seconds'])

    def test_unavailable_counter_blocks_history_deletion(self):
        pathlib.Path(self.path).write_text('{broken')
        node = self.node(self.ledger())
        request = pb.TaskExecutionDeleteRequest(execution_id='old')
        response = pb.TaskExecutionDeleteResponse()
        response.ParseFromString(node.handle_task_execution_delete_request(request.SerializeToString())[0])
        self.assertFalse(response.deleted)
        self.assertEqual(1, len(node._task_execution_records))

    def test_production_lifecycle_counts_even_when_result_image_path_fails(self):
        ledger = self.ledger(); node = self.node(ledger)
        node.task_config = types.SimpleNamespace(map_id='map', task_id='task')
        node._current_map_id = lambda:'map'
        node.current_path = None
        node._selected_task_work_area_m2 = lambda:1.0
        node._task_trajectory_relative_path = lambda identity:'task_executions/'+identity+'/trajectory.pbstream'
        node._save_task_execution_raw_map_snapshot = lambda *args:{}
        node._begin_task_execution_record()
        self.clock.advance(17)
        node._mark_current_region_repeat_done = mock.Mock(side_effect=OSError('result material unavailable'))
        node._finalize_task_result('stopped')
        self.clock.advance(1000)
        self.assertEqual(17, ledger.snapshot()['total_work_seconds'])
        ledger.flush()
        self.assertEqual(17, self.ledger().snapshot()['total_work_seconds'])


if __name__ == '__main__':
    unittest.main()
