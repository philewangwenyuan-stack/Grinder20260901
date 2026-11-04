"""Durable editing sessions over immutable map bundles (Python 3.8)."""
import base64
from copy import deepcopy
import hashlib
import json
import os
import threading
import tempfile
import shutil
import re
import uuid

from grinder_scheduler.drawing_scheduler import DrawingObstacleMixin

_INIT_LOCK = threading.Lock()


class MapEditDraftStore:
    def __init__(self, root):
        self.path = os.path.join(root, ".map-edit-drafts.json")
        self.lock = threading.RLock()
        try:
            with open(self.path, encoding="utf-8") as handle:
                self.data = json.load(handle)
        except FileNotFoundError:
            self.data = {"active": "", "items": {}, "restarts": []}
        self.committed = deepcopy(self.data)
        # Corrupt journals must fail closed, not silently expose hidden drafts.
        if not isinstance(self.data.get("items"), dict):
            raise ValueError("invalid map editing journal")

    def save(self):
        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            temp = self.path + ".tmp"
            with open(temp, "w", encoding="utf-8") as handle:
                json.dump(self.data, handle, ensure_ascii=False)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temp, self.path)
            if os.name != "nt":
                descriptor = os.open(os.path.dirname(self.path), os.O_RDONLY)
                try:
                    os.fsync(descriptor)
                finally:
                    os.close(descriptor)
            self.committed = deepcopy(self.data)
        except Exception:
            self.data = deepcopy(self.committed)
            raise


class MapEditDraftMixin(DrawingObstacleMixin):
    def _edit_lock(self):
        with _INIT_LOCK:
            if not hasattr(self, "_map_edit_lock"):
                self._map_edit_lock = threading.RLock()
        return self._map_edit_lock

    def _edit_drafts(self):
        with _INIT_LOCK:
            if not hasattr(self, "_map_edit_draft_store"):
                self._map_edit_draft_store = MapEditDraftStore(self._super_lio_map_root)
        return self._map_edit_draft_store

    def _is_editing_draft(self, map_id):
        # Prefix is reserved, including bundles orphaned by interrupted capture.
        if not str(map_id).startswith("draft_"):
            return False
        item = self._edit_drafts().committed["items"].get(str(map_id), {})
        return item.get("state") != "published"

    def _draft_binding(self, map_id):
        record = self._find_recorded_map_by_id(map_id)
        if not record:
            raise ValueError("地图草稿资产未就绪，请重试结束扫描")
        if self._current_map_id() != map_id:
            self._set_active_map_id(map_id, reason="resume_edit_draft", migrate_bindings=False)
        self._ensure_offline_map_service_for_current_map("edit_draft")
        item = self._edit_drafts().data["items"].get(map_id, {})
        state = item.get("edit_state")
        if state and item.get("state") == "editing":
            if getattr(self.map_service, "_draft_commit", None) != (map_id, state):
                self.map_service.load_local_state(os.path.join(self._map_state_dir(map_id), "edit_commits", state))
                self.map_service._draft_commit = (map_id, state)
        elif item.get("state") == "editing":
            self._commit_draft_edits(map_id)
        return dict(map_id=map_id, base_revision=record["map_revision"],
                    edit_version=self.map_service.get_map_version())

    def _commit_draft_edits(self, map_id, response_key=None, response_bytes=None):
        store = self._edit_drafts()
        commits = os.path.join(self._map_state_dir(map_id), "edit_commits")
        os.makedirs(commits, exist_ok=True)
        temp = tempfile.mkdtemp(prefix=".pending-", dir=commits)
        self.map_service.save_local_state(temp)
        token = uuid.uuid4().hex
        os.replace(temp, os.path.join(commits, token))
        if os.name != "nt":
            descriptor = os.open(commits, os.O_RDONLY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
        item = store.data["items"][map_id]
        item["previous_edit_state"] = item.get("edit_state", "")
        item["edit_state"] = token
        if response_key is not None:
            replies = item.setdefault("edit_replies", {})
            replies[response_key] = base64.b64encode(response_bytes).decode("ascii")
            while len(replies) > 128:
                del replies[next(iter(replies))]
        store.save()  # Edit data and idempotent reply become visible together.
        self.map_service._draft_commit = (map_id, token)
        # Bound full-grid snapshots to current + previous committed edit.
        keep = {token, item.get("previous_edit_state", "")}
        for entry in os.scandir(commits):
            if entry.name in keep or not re.fullmatch(r"[0-9a-f]{32}", entry.name):
                continue
            if entry.is_dir(follow_symlinks=False) and os.path.dirname(os.path.realpath(entry.path)) == os.path.realpath(commits):
                try:
                    shutil.rmtree(entry.path)
                except OSError:
                    pass  # Cleanup failure must not undo an already committed edit.

    def _check_edit_binding(self, request):
        map_id = str(request.map_id or self._current_map_id())
        if not self._is_editing_draft(map_id) and not request.HasField("edit_binding"):
            return
        binding = request.edit_binding
        expected = self._draft_binding(map_id)
        if (binding.map_id != map_id or binding.base_revision != expected["base_revision"] or
                binding.edit_version != expected["edit_version"]):
            raise ValueError("地图编辑版本已变化，请重新打开草稿后继续")

    def handle_map_draft_request(self, payload):
        pb = self.sl_link_server.pb
        request = pb.MapDraftRequest.FromString(payload)
        response = pb.MapDraftResponse(request_id=request.request_id, feature_version=1)
        try:
            if not request.request_id:
                raise ValueError("地图草稿操作缺少请求编号")
            store = self._edit_drafts()
            with self._edit_lock(), store.lock, self._map_mode_delete_lock:
                map_id = str(request.map_id or store.data.get("active", ""))
                if request.operation == pb.MAP_DRAFT_RESTART:
                    if request.request_id not in store.data["restarts"]:
                        self._prepare_for_map_mode_switch("edit_draft_restart")
                        self._ensure_super_lio_proxies()
                        started = self._super_lio_start_mapping_proxy()
                        if not started.success:
                            raise ValueError(started.message)
                        # Keep previous drafts; never transplant their overlays.
                        store.data["active"] = ""
                        store.data["restarts"] = (store.data["restarts"] + [request.request_id])[-64:]
                        store.save()
                        self._set_active_map_id(self._live_map_id, reason="edit_draft_restart",
                                                migrate_bindings=False, save_prev_overlay=True)
                        bridge = getattr(self, "ros_data_bridge", None)
                        if bridge is not None:
                            bridge.reset_map()
                        self.map_service.reset_overlay_regions()
                        self._save_local_state(allow_empty_saved_map_overlay=True)
                    map_id = ""
                elif request.operation == pb.MAP_DRAFT_FREEZE:
                    if not getattr(self, "_persist_state_enabled", False):
                        raise ValueError("固定地图草稿需要启用 persist_state_enabled")
                    if not map_id:
                        map_id = "draft_" + hashlib.sha256(request.request_id.encode()).hexdigest()[:24]
                        store.data["items"].setdefault(map_id, {"state": "capturing"})
                        store.data["active"] = map_id
                        store.save()  # Intent journal precedes capture, enabling retry after a crash.
                    item = store.data["items"].get(map_id)
                    if not item or item["state"] == "published":
                        raise ValueError("当前草稿不可继续扫描保存")
                    if item["state"] == "capturing":
                        paths = self._super_lio_bundle_paths(map_id)
                        if os.path.isfile(os.path.join(paths["bundle_dir"], "map_info.json")):
                            # Capture may have committed before the reply or registry write was lost.
                            self._register_saved_map(paths["bundle_dir"], "未发布地图", explicit_map_id=map_id)
                            self._ensure_super_lio_proxies()
                            stopped = self._super_lio_stop_proxy()
                            if not stopped.success:
                                raise ValueError(stopped.message)
                        else:
                            save = pb.MapSaveRequest(map_id=map_id, map_name="未发布地图")
                            data, _, _ = self._handle_map_save_without_drawing(save.SerializeToString())
                            saved = pb.MapSaveResponse.FromString(data)
                            if saved.result != pb.RESULT_SUCCESS or not saved.mapping_stopped:
                                raise ValueError(saved.message or "建图停止未确认，请重试")
                        record = self._find_recorded_map_by_id(map_id)
                        record["raw_grid_yaml_path"] = paths["yaml_path"]
                        record["raw_grid_image_path"] = paths["image_path"]
                        self._invalidate_raw_map_caches()
                        if self._save_map_registry_state() is False:
                            raise ValueError("地图草稿索引写入失败，请重试")
                        item["state"] = "editing"
                        store.save()
                elif request.operation == pb.MAP_DRAFT_DISCARD:
                    if map_id not in store.data["items"]:
                        raise ValueError("地图草稿不存在")
                    self._check_edit_binding(request)
                    store.data["items"][map_id]["state"] = "discarded"
                    if store.data["active"] == map_id:
                        store.data["active"] = ""
                    store.save()
                    map_id = ""
                elif request.operation not in (pb.MAP_DRAFT_QUERY, pb.MAP_DRAFT_RESUME):
                    raise ValueError("未知地图草稿操作")
                if map_id:
                    item = store.data["items"].get(map_id, {})
                    if item.get("state") == "published" and request.operation == pb.MAP_DRAFT_RESUME:
                        response.result = pb.RESULT_SUCCESS
                        response.mapping_stopped = True
                        response.message = "published_map"
                        return response.SerializeToString(), pb.MSG_ID_MAP_DRAFT_RESPONSE, pb.COMP_SCHEDULER
                    if item.get("state") != "editing":
                        if item.get("state") == "capturing" and request.operation == pb.MAP_DRAFT_QUERY:
                            response.exists = True
                            response.binding.map_id = map_id
                            response.result = pb.RESULT_SUCCESS
                            response.message = "capture_pending"
                            return response.SerializeToString(), pb.MSG_ID_MAP_DRAFT_RESPONSE, pb.COMP_SCHEDULER
                        if item.get("state") == "capturing":
                            raise ValueError("扫描草稿尚未完成，请重试下一步")
                        raise ValueError("地图草稿已发布或放弃")
                    # QUERY is read-only; RESUME/FREEZE activate the immutable base.
                    if request.operation == pb.MAP_DRAFT_QUERY:
                        record = self._find_recorded_map_by_id(map_id)
                        if not record:
                            raise ValueError("草稿索引未就绪，请重试下一步")
                        response.binding.map_id = map_id
                        response.binding.base_revision = record["map_revision"]
                    else:
                        for key, value in self._draft_binding(map_id).items():
                            setattr(response.binding, key, value)
                    response.exists = True
                    response.mapping_stopped = True
                response.result = pb.RESULT_SUCCESS
                response.message = "ok"
        except Exception as exc:
            response.result = pb.RESULT_FAILED
            response.message = str(exc)
        return response.SerializeToString(), pb.MSG_ID_MAP_DRAFT_RESPONSE, pb.COMP_SCHEDULER

    def handle_map_edit_command(self, payload):
        pb = self.sl_link_server.pb
        with self._edit_lock():
            try:
                request = pb.MapEditCommand.FromString(payload)
                map_id = str(request.map_id or self._current_map_id())
                draft = self._is_editing_draft(map_id)
                response_key = hashlib.sha256(payload).hexdigest()
                if draft:
                    cached = self._edit_drafts().data["items"].get(map_id, {}).get("edit_replies", {}).get(response_key)
                    if cached:
                        return base64.b64decode(cached), pb.MSG_ID_MAP_EDIT_RESPONSE, pb.COMP_SCHEDULER
                self._check_edit_binding(request)
                if draft:
                    self.map_service._draft_commit = None
                result = super().handle_map_edit_command(payload)
                # The scheduler normally returns response + status frames;
                # legacy handlers and cached replies can return one frame.
                single = isinstance(result, tuple)
                frames = [result] if single else list(result)
                response_index = next((i for i, frame in enumerate(frames)
                                       if frame[1] == pb.MSG_ID_MAP_EDIT_RESPONSE), None)
                if response_index is None:
                    raise ValueError("地图编辑未返回确认应答")
                data, message_id, component = frames[response_index]
                response = pb.MapEditResponse.FromString(data)
                if request.HasField("edit_binding") and response.result == pb.RESULT_SUCCESS:
                    response.edit_binding.CopyFrom(request.edit_binding)
                    response.edit_binding.edit_version = response.map_version
                if draft and response.result == pb.RESULT_SUCCESS:
                    self._commit_draft_edits(map_id, response_key, response.SerializeToString())
                frames[response_index] = response.SerializeToString(), message_id, component
                return frames[0] if single else frames
            except Exception as exc:
                response = pb.MapEditResponse(result=pb.RESULT_INVALID_PARAM, message=str(exc))
                return response.SerializeToString(), pb.MSG_ID_MAP_EDIT_RESPONSE, pb.COMP_SCHEDULER

    def handle_settings_write_request(self, payload):
        pb = self.sl_link_server.pb
        request = pb.SettingsWriteRequest.FromString(payload)
        with self._edit_lock():
            try:
                map_id = str(request.process_map_id or self._current_map_id())
                draft = request.HasField("process") and self._is_editing_draft(map_id)
                if draft:
                    self._check_edit_binding(pb.MapEditCommand(map_id=map_id, edit_binding=request.edit_binding))
                    self.map_service._draft_commit = None
                data, message_id, component = self._handle_settings_write_without_draft(payload)
                response = pb.SettingsWriteResponse.FromString(data)
                if draft and response.result == pb.RESULT_SUCCESS:
                    version = self.map_service.bump_edit_version()
                    self._commit_draft_edits(map_id)
                    response.edit_binding.CopyFrom(request.edit_binding)
                    response.edit_binding.edit_version = version
                return response.SerializeToString(), message_id, component
            except Exception as exc:
                response = pb.SettingsWriteResponse(result=pb.RESULT_INVALID_PARAM, message=str(exc))
                return response.SerializeToString(), pb.MSG_ID_SETTINGS_WRITE_RESPONSE, pb.COMP_SETTINGS

    def handle_drawing_draft_request(self, payload):
        pb = self.sl_link_server.pb
        with self._edit_lock():
            try:
                self._check_edit_binding(pb.DrawingDraftRequest.FromString(payload))
            except ValueError as exc:
                response = pb.DrawingDraftResponse(result=pb.RESULT_INVALID_PARAM, message=str(exc))
                return response.SerializeToString(), pb.MSG_ID_DRAWING_DRAFT_RESPONSE, pb.COMP_SCHEDULER
            return super().handle_drawing_draft_request(payload)

    def _handle_path_plan_request_impl_core(self, payload, task_scoped=True, include_preview=True):
        pb = self.sl_link_server.pb
        with self._edit_lock():
            try:
                self._check_edit_binding(pb.PathPlanRequest.FromString(payload))
            except ValueError as exc:
                response = pb.PathPlanResponse(result=pb.RESULT_INVALID_PARAM, message=str(exc))
                return response.SerializeToString(), pb.MSG_ID_PATH_PLAN_RESPONSE, pb.COMP_SCHEDULER
            return super()._handle_path_plan_request_impl_core(payload, task_scoped, include_preview)

    def handle_map_save_request(self, payload):
        pb = self.sl_link_server.pb
        request = pb.MapSaveRequest.FromString(payload)
        # Legacy/new-map saves retain their existing behavior; draft publishing is explicit.
        if not str(request.map_id).startswith("draft_"):
            data, message_id, component = super().handle_map_save_request(payload)
            response = pb.MapSaveResponse.FromString(data)
            response.request_id = request.request_id
            return response.SerializeToString(), message_id, component
        store = self._edit_drafts()
        with self._edit_lock(), store.lock:
            try:
                item = store.data["items"].get(request.map_id, {})
                if item.get("publish_request") == request.request_id and item.get("reply"):
                    return base64.b64decode(item["reply"]), pb.MSG_ID_MAP_SAVE_RESPONSE, pb.COMP_SCHEDULER
                if item.get("state") == "published":
                    self._check_edit_binding(request)
                    data, message_id, component = super().handle_map_save_request(payload)
                    response = pb.MapSaveResponse.FromString(data)
                    response.request_id = request.request_id
                    return response.SerializeToString(), message_id, component
                if item.get("state") != "editing" or not request.request_id:
                    raise ValueError("地图草稿不可发布或请求编号为空")
                self._check_edit_binding(request)
                data, msg_id, component = super().handle_map_save_request(payload)
                response = pb.MapSaveResponse.FromString(data)
                response.request_id = request.request_id
                if response.result == pb.RESULT_SUCCESS:
                    self.map_service.save_local_state(self._map_state_dir(request.map_id))
                    if self._save_map_registry_state() is False:
                        raise ValueError("地图发布索引写入失败，请重试")
                    response.mapping_stopped = True
                    response.message = "map_draft_published"
                    item.update(state="published", publish_request=request.request_id,
                                reply=base64.b64encode(response.SerializeToString()).decode("ascii"))
                    if store.data["active"] == request.map_id:
                        store.data["active"] = ""
                    store.save()  # Visibility commit, after assets/overlays/drawing layer are durable.
                    self._queue_saved_map_upload(request.map_id, request.map_name)
                return response.SerializeToString(), msg_id, component
            except Exception as exc:
                response = pb.MapSaveResponse(result=pb.RESULT_FAILED, request_id=request.request_id, message=str(exc))
                return response.SerializeToString(), pb.MSG_ID_MAP_SAVE_RESPONSE, pb.COMP_SCHEDULER
