"""Production queue/dispatch barriers; no ROS, socket or physical vehicle."""
import ast
from collections import deque
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest

import test_emergency_stop_execution as execution_fixture


ROOT = Path(__file__).resolve().parents[4]
SOURCE = ROOT / 'catkin_ws/src/grinder_scheduler/src/grinder_scheduler/sl_linka_adapter.py'
tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
classes = [n for n in tree.body if isinstance(n, ast.ClassDef)
           and n.name in ('_BoundedLatestWorker', '_DropIfBusyWorker')]
namespace = dict(threading=threading, deque=deque, rospy=None)
exec(compile(ast.Module(body=classes, type_ignores=[]), str(SOURCE), 'exec'), namespace)
Worker = namespace['_BoundedLatestWorker']


class ManualQueueTest(unittest.TestCase):
    def test_inflight_control_cannot_publish_after_connection_closes(self):
        fixture=execution_fixture.ExecutionTest()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        node,_=fixture.node()
        class Response:
            def __init__(self):self.applied_command=SimpleNamespace(CopyFrom=lambda _:None)
            def SerializeToString(self):return self.result
        node.sl_link_server=SimpleNamespace(pb=SimpleNamespace(ControlCommandResponse=Response,
            RESULT_SUCCESS=1,RESULT_FAILED=0,MSG_ID_CONTROL_COMMAND_RESPONSE=1,COMP_CONTROL=1))
        entered,release=threading.Event(),threading.Event()
        self.addCleanup(release.set)
        alive=[True];outputs=[];result=[]
        def apply(request):
            entered.set();release.wait(2)
            node._motion.check()
            outputs.append(100)
            return True,''
        node._apply_control_outputs=apply
        request=SimpleNamespace(HasField=lambda _:False)
        thread=threading.Thread(target=lambda:result.append(node.handle_control_command(
            b'',parsed_request=request,connection_valid=lambda:alive[0])[0]))
        thread.start();self.assertTrue(entered.wait(1))
        alive[0]=False;release.set();thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual([],outputs)
        self.assertEqual([0],result)

    def worker(self):
        worker = Worker(max_pending=1, name='manual-regression')
        self.addCleanup(lambda: worker.shutdown(wait=True))
        return worker

    def test_release_zero_survives_busy_worker_and_replaces_old_movement(self):
        worker = self.worker()
        entered, release, stopped = threading.Event(), threading.Event(), threading.Event()
        self.addCleanup(release.set)
        outputs = []
        def moving():
            entered.set()
            release.wait(2)
            outputs.append(100)
        def stop():
            outputs.append(0)
            stopped.set()
        worker.submit(moving)
        self.assertTrue(entered.wait(1))
        worker.submit(lambda: outputs.append(200))
        dropped = worker.submit(stop)
        self.assertIsNotNone(dropped)
        release.set()
        self.assertTrue(stopped.wait(1))
        self.assertEqual([100, 0], outputs)

    def test_many_moves_are_coalesced_without_delaying_release_behind_backlog(self):
        worker = self.worker()
        entered, release, finished = threading.Event(), threading.Event(), threading.Event()
        self.addCleanup(release.set)
        worker.submit(lambda: (entered.set(), release.wait(2)))
        self.assertTrue(entered.wait(1))
        outputs = []
        for speed in range(1000):
            worker.submit(outputs.append, speed)
        worker.submit(lambda: (outputs.append(0), finished.set()))
        self.assertEqual(1, len(worker._jobs))
        release.set()
        self.assertTrue(finished.wait(1))
        self.assertEqual([0], outputs)

    def test_queued_movement_cannot_revive_after_emergency_and_reset(self):
        fixture = execution_fixture.ExecutionTest()
        fixture.setUp()
        self.addCleanup(fixture.doCleanups)
        node, _ = fixture.node()
        worker = self.worker()
        entered, release, finished = threading.Event(), threading.Event(), threading.Event()
        self.addCleanup(release.set)
        worker.submit(lambda: (entered.set(), release.wait(2)))
        self.assertTrue(entered.wait(1))
        generation = node.capture_control_generation()
        class Response:
            def __init__(self):
                self.applied_command = SimpleNamespace(CopyFrom=lambda _: None)
            def SerializeToString(self):
                return self.result
        node.sl_link_server = SimpleNamespace(pb=SimpleNamespace(
            ControlCommandResponse=Response, RESULT_SUCCESS=1, RESULT_FAILED=0,
            MSG_ID_CONTROL_COMMAND_RESPONSE=1, COMP_CONTROL=1))
        submitted = []
        node._apply_control_outputs = lambda _: (submitted.append(100) or True, '')
        result = []
        request = SimpleNamespace(HasField=lambda _: False)
        def command():
            result.append(node.handle_control_command(b'', parsed_request=request,
                                                     control_generation=generation)[0])
            finished.set()
        worker.submit(command)
        node._apply_emergency_stop(True)
        self.assertTrue(node._motion.reset(node._motion.generation))
        release.set()
        self.assertTrue(finished.wait(1))
        self.assertEqual([0], result)
        self.assertEqual([], submitted)
        current = node.capture_control_generation()
        self.assertEqual(1, node.handle_control_command(b'', parsed_request=request,
                                                     control_generation=current)[0])
        self.assertEqual([100], submitted)


if __name__ == '__main__':
    unittest.main()
