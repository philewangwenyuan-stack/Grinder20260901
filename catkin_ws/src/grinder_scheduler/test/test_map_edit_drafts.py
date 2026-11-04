"""Offline lifecycle tests: no ROS master, sensors, navigation or hardware."""
import json
import ast
from pathlib import Path
import os
import tempfile
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from grinder_scheduler.map_edit_drafts import MapEditDraftMixin, MapEditDraftStore
from sl_link.message_gen import sl_link_pb2 as pb


class Node(MapEditDraftMixin):
    def __init__(self, root):
        self._super_lio_map_root = root
        self._persist_state_enabled = True
        self._map_mode_delete_lock = threading.Lock()
        self._map_registry = {}
        self.map_id = "LIVE_MAP"
        self.version = 0
        self.save_calls = self.starts = self.stops = self.uploads = 0
        self.map_service = SimpleNamespace(get_map_version=lambda: self.version, save_local_state=self.save_state,
                                           load_local_state=self.load_state, reset_overlay_regions=lambda: None,
                                           bump_edit_version=self.bump_version)
        self.sl_link_server = SimpleNamespace(pb=pb)
        self._super_lio_start_mapping_proxy = self.start
        self._super_lio_stop_proxy = self.stop

    def _map_state_dir(self, map_id): return os.path.join(self._super_lio_map_root, "states", map_id)
    def _save_local_state(self, **kwargs): pass
    def save_state(self, root):
        os.makedirs(root, exist_ok=True)
        with open(os.path.join(root, "version.json"), "w") as handle: json.dump(self.version, handle)
    def load_state(self, root):
        with open(os.path.join(root, "version.json")) as handle: self.version = json.load(handle)

    def bump_version(self):
        self.version += 1
        return self.version

    def _handle_settings_write_without_draft(self, payload):
        response = pb.SettingsWriteResponse(result=pb.RESULT_SUCCESS)
        return response.SerializeToString(), pb.MSG_ID_SETTINGS_WRITE_RESPONSE, pb.COMP_SETTINGS

    def start(self):
        self.starts += 1
        return SimpleNamespace(success=True)

    def stop(self):
        self.stops += 1
        return SimpleNamespace(success=True)

    def _current_map_id(self): return self.map_id
    def _find_recorded_map_by_id(self, value): return self._map_registry.get(value)
    def _is_live_map_id(self, value): return value in ("", "LIVE_MAP")
    def _set_active_map_id(self, value, **kwargs): self.map_id = value
    def _ensure_offline_map_service_for_current_map(self, *_args): pass
    def _prepare_for_map_mode_switch(self, *_args): pass
    def _ensure_super_lio_proxies(self): pass
    def _save_map_registry_state(self): pass
    def _invalidate_raw_map_caches(self): pass

    def _super_lio_bundle_paths(self, value):
        root = os.path.join(self._super_lio_map_root, value)
        return dict(bundle_dir=root, yaml_path=os.path.join(root, "map.yaml"),
                    image_path=os.path.join(root, "map.pgm"))

    def _register_saved_map(self, root, _name, explicit_map_id):
        self._map_registry[explicit_map_id] = dict(map_id=explicit_map_id, map_revision="immutable-base")

    def _queue_saved_map_upload(self, *_args): self.uploads += 1

    def _handle_map_save_without_drawing(self, payload):
        self.save_calls += 1
        request = pb.MapSaveRequest.FromString(payload)
        if self._find_recorded_map_by_id(request.map_id) is None:
            paths = self._super_lio_bundle_paths(request.map_id)
            os.makedirs(paths["bundle_dir"])
            with open(os.path.join(paths["bundle_dir"], "map_info.json"), "w") as handle:
                json.dump({}, handle)
            self._register_saved_map(paths["bundle_dir"], "draft", request.map_id)
        response = pb.MapSaveResponse(result=pb.RESULT_SUCCESS, map_id=request.map_id, mapping_stopped=True)
        return response.SerializeToString(), pb.MSG_ID_MAP_SAVE_RESPONSE, pb.COMP_SCHEDULER

    def _handle_map_edit_without_drawing(self, payload):
        self.version += 1
        response = pb.MapEditResponse(result=pb.RESULT_SUCCESS, map_version=self.version)
        return response.SerializeToString(), pb.MSG_ID_MAP_EDIT_RESPONSE, pb.COMP_SCHEDULER


def operation(node, op, request_id="request", **kwargs):
    request = pb.MapDraftRequest(operation=op, request_id=request_id, **kwargs)
    return pb.MapDraftResponse.FromString(node.handle_map_draft_request(request.SerializeToString())[0])


class MapEditDraftTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.node = Node(self.temp.name)

    def freeze(self):
        reply = operation(self.node, pb.MAP_DRAFT_FREEZE)
        self.assertEqual(reply.result, pb.RESULT_SUCCESS, reply.message)
        return reply.binding

    def test_freeze_is_hidden_stable_and_retry_does_not_capture_again(self):
        binding = self.freeze()
        repeated = operation(self.node, pb.MAP_DRAFT_FREEZE)
        self.assertEqual(repeated.binding, binding)
        self.assertEqual(self.node.save_calls, 1)
        self.assertTrue(self.node._is_editing_draft(binding.map_id))
        self.assertEqual(self.node.map_id, binding.map_id)

    def test_restart_retains_old_draft_and_is_idempotent(self):
        old = self.freeze().map_id
        operation(self.node, pb.MAP_DRAFT_RESTART, "restart")
        operation(self.node, pb.MAP_DRAFT_RESTART, "restart")
        self.assertEqual(self.node.starts, 1)
        self.assertFalse(operation(self.node, pb.MAP_DRAFT_QUERY).exists)
        self.assertIn(old, self.node._edit_drafts().data["items"])
        new = operation(self.node, pb.MAP_DRAFT_FREEZE, "new-freeze")
        self.assertNotEqual(new.binding.map_id, old)

    def test_query_is_read_only_and_resume_recovers_from_disk(self):
        old = self.freeze()
        fresh = Node(self.temp.name)
        fresh._map_registry = self.node._map_registry
        result = operation(fresh, pb.MAP_DRAFT_QUERY)
        self.assertTrue(result.exists)
        self.assertEqual(fresh.map_id, "LIVE_MAP")
        result = operation(fresh, pb.MAP_DRAFT_RESUME, map_id=old.map_id)
        self.assertEqual(result.binding.base_revision, old.base_revision)
        self.assertEqual(fresh.map_id, old.map_id)

    def test_stale_base_or_edit_version_is_rejected(self):
        binding = self.freeze()
        request = pb.MapEditCommand(map_id=binding.map_id, edit_binding=binding)
        result = pb.MapEditResponse.FromString(self.node.handle_map_edit_command(request.SerializeToString())[0])
        self.assertEqual(result.result, pb.RESULT_SUCCESS)
        self.assertEqual(result.edit_binding.edit_version, 1)
        repeated = pb.MapEditResponse.FromString(self.node.handle_map_edit_command(request.SerializeToString())[0])
        self.assertEqual(repeated.result, pb.RESULT_SUCCESS)
        self.assertEqual(self.node.version, 1)
        request.edit_id = "different-edit"
        stale = pb.MapEditResponse.FromString(self.node.handle_map_edit_command(request.SerializeToString())[0])
        self.assertEqual(stale.result, pb.RESULT_INVALID_PARAM)
        request.edit_binding.edit_version = 1
        request.edit_binding.base_revision = "wrong-base"
        bad = pb.MapEditResponse.FromString(self.node.handle_map_edit_command(request.SerializeToString())[0])
        self.assertEqual(bad.result, pb.RESULT_INVALID_PARAM)

    def test_legacy_edit_cannot_mutate_hidden_draft(self):
        binding = self.freeze()
        request = pb.MapEditCommand(map_id=binding.map_id)
        result = pb.MapEditResponse.FromString(self.node.handle_map_edit_command(request.SerializeToString())[0])
        self.assertEqual(result.result, pb.RESULT_INVALID_PARAM)
        self.assertEqual(self.node.version, 0)

    def test_publish_is_visibility_commit_and_retries_return_same_response(self):
        binding = self.freeze()
        request = pb.MapSaveRequest(map_id=binding.map_id, map_name="factory", request_id="publish", edit_binding=binding)
        result = self.node.handle_map_save_request(request.SerializeToString())
        self.assertEqual(pb.MapSaveResponse.FromString(result[0]).result, pb.RESULT_SUCCESS)
        self.assertFalse(self.node._is_editing_draft(binding.map_id))
        self.assertFalse(operation(self.node, pb.MAP_DRAFT_QUERY).exists)
        self.assertEqual(self.node.handle_map_save_request(request.SerializeToString()), result)
        self.assertEqual(self.node.save_calls, 2)
        self.assertEqual(self.node.uploads, 1)

    def test_crash_after_bundle_commit_can_recover_before_reply(self):
        store = self.node._edit_drafts()
        store.data.update(active="draft_interrupted", items={"draft_interrupted": {"state": "capturing"}})
        store.save()
        path = self.node._super_lio_bundle_paths("draft_interrupted")["bundle_dir"]
        os.makedirs(path)
        with open(os.path.join(path, "map_info.json"), "w") as handle: handle.write("{}")
        self.assertTrue(operation(self.node, pb.MAP_DRAFT_QUERY).exists)
        result = operation(self.node, pb.MAP_DRAFT_FREEZE)
        self.assertEqual(result.result, pb.RESULT_SUCCESS, result.message)
        self.assertEqual(self.node.save_calls, 0)
        self.assertEqual(self.node.stops, 1)

    def test_failed_atomic_journal_commit_remains_hidden(self):
        binding = self.freeze()
        store = self.node._edit_drafts()
        store.data["items"][binding.map_id]["state"] = "published"
        with patch("grinder_scheduler.map_edit_drafts.os.replace", side_effect=OSError("disk full")):
            with self.assertRaises(OSError): store.save()
        self.assertTrue(self.node._is_editing_draft(binding.map_id))
        self.assertEqual(MapEditDraftStore(self.temp.name).data["items"][binding.map_id]["state"], "editing")

    def test_edit_commit_failure_rolls_back_on_retry(self):
        binding = self.freeze()
        request = pb.MapEditCommand(map_id=binding.map_id, edit_id="one", edit_binding=binding)
        real_replace = os.replace
        def fail_journal(source, target):
            if str(target).endswith(".map-edit-drafts.json"):
                raise OSError("disk full")
            return real_replace(source, target)
        with patch("grinder_scheduler.map_edit_drafts.os.replace", side_effect=fail_journal):
            result = pb.MapEditResponse.FromString(self.node.handle_map_edit_command(request.SerializeToString())[0])
        self.assertEqual(result.result, pb.RESULT_INVALID_PARAM)
        retry = pb.MapEditResponse.FromString(self.node.handle_map_edit_command(request.SerializeToString())[0])
        self.assertEqual(retry.result, pb.RESULT_SUCCESS)
        self.assertEqual(retry.map_version, 1)

    def test_full_grid_edit_snapshots_are_bounded(self):
        binding = self.freeze()
        for i in range(6):
            request = pb.MapEditCommand(map_id=binding.map_id, edit_id=str(i), edit_binding=binding)
            response = pb.MapEditResponse.FromString(self.node.handle_map_edit_command(request.SerializeToString())[0])
            self.assertEqual(response.result, pb.RESULT_SUCCESS)
            binding = response.edit_binding
        commits = os.path.join(self.node._map_state_dir(binding.map_id), "edit_commits")
        self.assertEqual(len(os.listdir(commits)), 2)

    def test_published_draft_remains_editable_as_a_formal_map(self):
        binding = self.freeze()
        publish = pb.MapSaveRequest(map_id=binding.map_id, request_id="publish", edit_binding=binding)
        self.node.handle_map_save_request(publish.SerializeToString())
        resumed = operation(self.node, pb.MAP_DRAFT_RESUME, map_id=binding.map_id)
        self.assertEqual(resumed.result, pb.RESULT_SUCCESS)
        self.assertFalse(resumed.exists)
        edit = pb.MapEditCommand(map_id=binding.map_id)
        response = pb.MapEditResponse.FromString(self.node.handle_map_edit_command(edit.SerializeToString())[0])
        self.assertEqual(response.result, pb.RESULT_SUCCESS)
        save = pb.MapSaveRequest(map_id=binding.map_id, request_id="later-edit")
        response = pb.MapSaveResponse.FromString(self.node.handle_map_save_request(save.SerializeToString())[0])
        self.assertEqual(response.result, pb.RESULT_SUCCESS)
        self.assertEqual(response.request_id, "later-edit")
        delayed = pb.MapEditCommand(map_id=binding.map_id, edit_id="late", edit_binding=binding)
        rejected = pb.MapEditResponse.FromString(self.node.handle_map_edit_command(delayed.SerializeToString())[0])
        self.assertEqual(rejected.result, pb.RESULT_INVALID_PARAM)


    def test_process_settings_returns_and_persists_the_new_edit_version(self):
        binding = self.freeze()
        request = pb.SettingsWriteRequest(process_map_id=binding.map_id, process_region_id="work", edit_binding=binding)
        request.process.run_speed = .1
        response = pb.SettingsWriteResponse.FromString(self.node.handle_settings_write_request(request.SerializeToString())[0])
        self.assertEqual(response.result, pb.RESULT_SUCCESS)
        self.assertEqual(response.edit_binding.edit_version, 1)
        fresh = Node(self.temp.name)
        fresh._map_registry = self.node._map_registry
        resumed = operation(fresh, pb.MAP_DRAFT_RESUME, map_id=binding.map_id)
        self.assertEqual(resumed.binding.edit_version, 1)

    def test_real_catalog_and_task_start_gate_hide_unpublished_maps(self):
        binding = self.freeze()
        path = Path(__file__).resolve().parents[1] / "src/grinder_scheduler/scheduler_node.py"
        tree = ast.parse(path.read_text(encoding="utf-8"))
        cls = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == "SchedulerNode")
        methods = [item for item in cls.body if isinstance(item, ast.FunctionDef) and item.name in ("_iter_recorded_maps", "_start_execution")]
        for method in methods: method.decorator_list = []
        namespace = dict(os=os)
        exec(compile(ast.fix_missing_locations(ast.Module(body=methods, type_ignores=[])), str(path), "exec"), namespace)
        record = self.node._map_registry[binding.map_id]
        record.update(bundle_dir=self.node._super_lio_bundle_paths(binding.map_id)["bundle_dir"], size_bytes=1)
        self.assertEqual(namespace["_iter_recorded_maps"](self.node), [])
        self.assertFalse(namespace["_start_execution"](self.node)[0])

    def test_corrupt_journal_fails_closed(self):
        with open(os.path.join(self.temp.name, ".map-edit-drafts.json"), "w") as handle: handle.write("{")
        with self.assertRaises(ValueError): MapEditDraftStore(self.temp.name)


if __name__ == "__main__": unittest.main()
