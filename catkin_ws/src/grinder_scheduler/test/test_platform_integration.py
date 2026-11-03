#!/usr/bin/env python3

import json
import os
import sys
import tempfile
import types
import unittest
from unittest import mock


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


class PlatformMapUploadTest(unittest.TestCase):
    def make_sync(self):
        return PlatformFileSync(
            "http://platform", "user", "password", "project", "http://files",
            enabled=True,
        )

    def test_cloud_export_names_references_and_local_bundle(self):
        with tempfile.TemporaryDirectory() as bundle:
            originals = {
                "loc_map.pcd": b"binary localization cloud",
                "plan_map.pcd": b"local-only planning cloud",
                "map.pgm": b"P5\n1 1\n255\n\xff",
                "map.yaml": b"image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n",
                "map_info.json": json.dumps({
                    "schemaVersion": 2, "mapId": "map-1", "mapName": "一楼大厅",
                    "map": {"plan_pcd_path": "/local/plan_map.pcd",
                            "loc_pcd_path": "/local/loc_map.pcd",
                            "yaml_path": "/local/map.yaml", "image_path": "/local/map.pgm"},
                    "taskBindings": {"task-1": {"map_id": "map-1"}},
                    "files": {"planning": "plan_map.pcd"},
                }, ensure_ascii=False).encode("utf-8"),
                "extra.txt": b"not a cloud asset",
                "map_info.json.tmp": b"incomplete",
            }
            for name, data in originals.items():
                with open(os.path.join(bundle, name), "wb") as handle:
                    handle.write(data)
            sync = self.make_sync()
            requests = []

            def request_upload(request, **_kwargs):
                requests.append(request)
                response = mock.MagicMock()
                response.__enter__.return_value.read.return_value = b'{"code":200}'
                return response

            with mock.patch.object(sync, "_ensure_folder_path", return_value="folder") as folder, \
                    mock.patch("urllib.request.urlopen", side_effect=request_upload):
                sync._upload_map({"map_id": "map-1", "map_name": "一楼大厅", "local_map_dir": bundle})
            folder.assert_called_once_with(["GrinderProject", "maps", "map-1"])
            self.assertEqual(len(requests), 4)
            payloads = {}
            for request in requests:
                self.assertTrue(request.full_url.endswith("/api/app/UploadFileToFolder"))
                header, contents = request.data.split(b'name="File"; filename="', 1)[1].split(b'"\r\n', 1)
                payloads[header.decode("utf-8")] = contents.split(b"\r\n\r\n", 1)[1].rsplit(b"\r\n--", 1)[0]
            self.assertEqual(set(payloads), {
                "一楼大厅_" + name for name in ("loc_map.pcd", "map.pgm", "map.yaml", "map_info.json")
            })
            self.assertEqual(payloads["一楼大厅_loc_map.pcd"], originals["loc_map.pcd"])
            self.assertEqual(payloads["一楼大厅_map.pgm"], originals["map.pgm"])
            self.assertIn('image: "一楼大厅_map.pgm"', payloads["一楼大厅_map.yaml"].decode("utf-8"))
            manifest = json.loads(payloads["一楼大厅_map_info.json"])
            self.assertEqual(manifest["files"], {
                "localization": "一楼大厅_loc_map.pcd", "yaml": "一楼大厅_map.yaml", "image": "一楼大厅_map.pgm",
            })
            self.assertNotIn("plan_pcd_path", manifest["map"])
            self.assertEqual(manifest["map"]["yaml_path"], "一楼大厅_map.yaml")
            self.assertIn("task-1", manifest["taskBindings"])
            for name, data in originals.items():
                with open(os.path.join(bundle, name), "rb") as handle:
                    self.assertEqual(handle.read(), data)

    def test_filename_prefix_is_safe_and_has_fallback(self):
        self.assertEqual(PlatformFileSync._map_filename_prefix('大厅/东区\\一层\r\n"', "map-1"), '大厅_东区_一层___')
        self.assertEqual(PlatformFileSync._map_filename_prefix(" . ", "map-1"), "map-1")
        self.assertEqual(PlatformFileSync._map_filename_prefix("", ""), "map")

    def test_incomplete_bundle_does_not_start_upload(self):
        with tempfile.TemporaryDirectory() as bundle:
            sync = self.make_sync()
            with mock.patch.object(sync, "_multipart_upload") as upload:
                with self.assertRaisesRegex(RuntimeError, "cloud map file missing"):
                    sync._upload_map({"map_id": "map-1", "map_name": "地图", "local_map_dir": bundle})
            upload.assert_not_called()


if __name__ == "__main__":
    unittest.main()
