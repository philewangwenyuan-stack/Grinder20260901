#!/usr/bin/env python3

import json
import os
import sys
import tempfile
import types
import unittest


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
PYTHON_SRC = os.path.join(REPO_ROOT, "catkin_ws", "src", "grinder_scheduler", "src")
if PYTHON_SRC not in sys.path:
    sys.path.insert(0, PYTHON_SRC)


rospy = types.ModuleType("rospy")
rospy.loginfo = lambda *_args, **_kwargs: None
rospy.logwarn = lambda *_args, **_kwargs: None
rospy.logerr = lambda *_args, **_kwargs: None
sys.modules["rospy"] = rospy

from grinder_scheduler.platform_integration import PlatformFileSync  # noqa: E402


class PlatformDeleteQueueTest(unittest.TestCase):
    def test_remote_delete_queue_is_persisted_and_retried(self):
        with tempfile.TemporaryDirectory() as state_dir:
            queue_path = os.path.join(state_dir, "pending.json")
            sync = PlatformFileSync(
                "http://platform",
                "user",
                "password",
                "project",
                "http://files",
                enabled=True,
                pending_delete_path=queue_path,
            )

            queued, message = sync.enqueue_map_delete("map-1")
            self.assertTrue(queued)
            self.assertEqual(message, "remote map delete queued")
            with open(queue_path, "r", encoding="utf-8") as handle:
                self.assertIn("map-1", json.load(handle)["jobs"])

            restored = PlatformFileSync(
                "http://platform",
                "user",
                "password",
                "project",
                "http://files",
                enabled=True,
                pending_delete_path=queue_path,
            )
            restored._load_pending_delete_jobs()
            self.assertIn("map-1", restored._pending_delete_jobs)
            job = restored._claim_pending_delete_job()
            self.assertEqual(job["map_id"], "map-1")
            restored._finish_pending_delete_job(job, "timeout")
            self.assertEqual(restored._pending_delete_jobs["map-1"]["attempts"], 1)


if __name__ == "__main__":
    unittest.main()
