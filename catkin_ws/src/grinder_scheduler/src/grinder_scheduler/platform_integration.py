import base64
import json
import mimetypes
import os
import queue
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid

import rospy

try:
    import paho.mqtt.client as mqtt
except Exception:
    mqtt = None


def _timestamp_ms():
    return str(int(time.time() * 1000))


def _response_data(payload):
    if not isinstance(payload, dict):
        return None
    for key in ("data", "response", "result"):
        if key in payload and payload[key] is not None:
            return payload[key]
    return None


def _response_ok(payload):
    if not isinstance(payload, dict):
        return False
    if "success" in payload:
        return bool(payload.get("success"))
    code = payload.get("code", 0)
    return str(code).strip().lower() in ("0", "200", "sucess", "success", "ccflowsucess")


def _response_message(payload):
    if not isinstance(payload, dict):
        return "invalid response"
    return str(payload.get("msg", payload.get("message", "")) or "")


class PlatformFileSync:
    def __init__(
        self,
        platform_base_url,
        username,
        password,
        configured_project_id,
        file_base_url,
        remote_root_name="GrinderProject",
        enabled=False,
        timeout_sec=30.0,
    ):
        self.enabled = bool(enabled)
        self.platform_base_url = str(platform_base_url or "").rstrip("/")
        self.file_base_url = str(file_base_url or "").rstrip("/")
        self.username = str(username or "")
        self.password = str(password or "")
        self.configured_project_id = str(configured_project_id or "").strip()
        self.remote_root_name = str(remote_root_name or "GrinderProject").strip() or "GrinderProject"
        self.timeout_sec = max(3.0, float(timeout_sec))
        self.project_id = ""
        self.project_name = ""
        self._platform_token = ""
        self._file_token = ""
        self._jobs = queue.Queue(maxsize=8)
        self._stop_event = threading.Event()
        self._thread = None
        self._lock = threading.Lock()

    def start(self):
        if not self.enabled:
            return
        if not self.username or not self.password:
            rospy.logwarn("Platform file sync disabled: platform username/password is empty")
            self.enabled = False
            return
        self._thread = threading.Thread(target=self._worker, name="grinder-file-sync", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        try:
            self._jobs.put_nowait(None)
        except queue.Full:
            pass
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    def enqueue_map(self, map_id, map_name, local_map_dir, stcm_path):
        if not self.enabled:
            return False
        job = {
            "map_id": str(map_id or "").strip(),
            "map_name": str(map_name or "").strip(),
            "local_map_dir": str(local_map_dir or "").strip(),
            "stcm_path": str(stcm_path or "").strip(),
        }
        if not job["map_id"]:
            rospy.logwarn("Map file upload skipped: map_id is empty")
            return False
        try:
            self._jobs.put_nowait(job)
            rospy.loginfo("Map file upload queued: map_id=%s", job["map_id"])
            return True
        except queue.Full:
            rospy.logwarn("Map file upload queue is full: map_id=%s", job["map_id"])
            return False

    def get_project_id(self):
        with self._lock:
            return self.project_id

    def _request_json(self, base_url, path, method="GET", body=None, headers=None):
        url = base_url + path
        request_headers = {"Accept": "application/json"}
        request_headers.update(headers or {})
        data = None
        if body is not None:
            data = json.dumps(body, ensure_ascii=False).encode("utf-8")
            request_headers["Content-Type"] = "application/json"
        request = urllib.request.Request(url, data=data, headers=request_headers, method=method)
        with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
            raw = response.read().decode("utf-8", errors="replace")
        return json.loads(raw) if raw else {}

    @staticmethod
    def _jwt_user_id(token):
        try:
            segment = token.split(".")[1]
            segment += "=" * (-len(segment) % 4)
            claims = json.loads(base64.urlsafe_b64decode(segment.encode("ascii")).decode("utf-8"))
            return str(
                claims.get("http://schemas.xmlsoap.org/ws/2005/05/identity/claims/nameidentifier", "")
                or claims.get("sub", "")
            )
        except Exception:
            return ""

    def _authenticate(self):
        login = self._request_json(
            self.platform_base_url,
            "/api/app/LoginNew",
            method="POST",
            body={"userName": self.username, "password": self.password, "uuid": "", "code": ""},
        )
        if not _response_ok(login):
            raise RuntimeError("platform login failed: {}".format(_response_message(login)))
        login_data = _response_data(login)
        if not isinstance(login_data, dict):
            raise RuntimeError("platform login returned invalid data")
        platform_token = str(login_data.get("token", "") or "").strip()
        if not platform_token:
            raise RuntimeError("platform login token is empty")
        user_id = self._jwt_user_id(platform_token)
        if not user_id:
            raise RuntimeError("platform login token does not contain user id")

        query = urllib.parse.urlencode({"userId": user_id})
        projects_result = self._request_json(
            self.platform_base_url,
            "/api/app/GetProjectListByUserId?" + query,
            headers={"token": platform_token},
        )
        if not _response_ok(projects_result):
            raise RuntimeError("project query failed: {}".format(_response_message(projects_result)))
        projects = _response_data(projects_result)
        if not isinstance(projects, list) or not projects:
            raise RuntimeError("platform account has no project")
        selected = None
        if self.configured_project_id:
            selected = next(
                (item for item in projects if str(item.get("id", "")) == self.configured_project_id),
                None,
            )
            if selected is None:
                raise RuntimeError("configured projectId is not assigned to platform account")
        elif len(projects) == 1:
            selected = projects[0]
        else:
            raise RuntimeError("platform account has multiple projects; platform_project_id is required")

        with self._lock:
            self._platform_token = platform_token
            # File-management RBAC endpoints belong to the construction
            # platform API and must use the LoginNew user token. The token
            # returned by GetAuthToken is a storage-service token and is
            # rejected by AddFileFolder as having no endpoint policy.
            self._file_token = platform_token
            self.project_id = str(selected.get("id", "") or "")
            self.project_name = str(selected.get("projectName", "") or "")
        rospy.loginfo(
            "Platform authentication success: project_id=%s project_name=%s",
            self.project_id,
            self.project_name,
        )

    def _file_headers(self):
        return {"Authorization": self._file_token}

    @staticmethod
    def _flatten_nodes(value):
        output = []

        def visit(item):
            if isinstance(item, list):
                for child in item:
                    visit(child)
            elif isinstance(item, dict):
                output.append(item)
                for key in ("children", "childList", "items"):
                    if isinstance(item.get(key), list):
                        visit(item[key])

        visit(value)
        return output

    def _document_tree(self):
        result = self._request_json(
            self.file_base_url,
            "/api/app/GetAllFileDocumentTree",
            method="POST",
            body={"projectId": self.project_id},
            headers=self._file_headers(),
        )
        if not _response_ok(result):
            raise RuntimeError("file tree query failed: {}".format(_response_message(result)))
        return self._flatten_nodes(_response_data(result) or [])

    def _create_folder(self, parent_id, name, code=""):
        result = self._request_json(
            self.file_base_url,
            "/api/app/AddFileFolder",
            method="POST",
            body={
                "projectId": self.project_id,
                "parentId": parent_id or None,
                "name": name,
                "code": code or None,
                "type": 3,
                "orderNum": 0,
            },
            headers=self._file_headers(),
        )
        if not _response_ok(result):
            raise RuntimeError("create folder {} failed: {}".format(name, _response_message(result)))
        data = _response_data(result)
        if isinstance(data, str):
            return data
        if isinstance(data, dict):
            return str(data.get("id", data.get("regionId", "")) or "")
        return ""

    def _ensure_folder_path(self, names):
        nodes = self._document_tree()
        parent_id = ""
        for index, name in enumerate(names):
            found = next(
                (
                    item
                    for item in nodes
                    if str(item.get("name", "")) == name
                    and str(item.get("parentId", "") or "") == parent_id
                    and int(item.get("type", item.get("Type", 3)) or 3) == 3
                ),
                None,
            )
            if found is not None:
                parent_id = str(found.get("id", "") or "")
                continue
            code = "grinder-project-root" if index == 0 else ""
            created_id = self._create_folder(parent_id, name, code)
            if not created_id:
                nodes = self._document_tree()
                found = next(
                    (
                        item
                        for item in nodes
                        if str(item.get("name", "")) == name
                        and str(item.get("parentId", "") or "") == parent_id
                    ),
                    None,
                )
                created_id = str(found.get("id", "") or "") if found else ""
            if not created_id:
                raise RuntimeError("created folder cannot be resolved: {}".format(name))
            nodes.append({"id": created_id, "parentId": parent_id or None, "name": name, "type": 3})
            parent_id = created_id
        return parent_id

    def _multipart_upload(self, file_path, folder_id):
        boundary = "----GrinderBoundary{}".format(uuid.uuid4().hex)
        filename = os.path.basename(file_path)
        content_type = mimetypes.guess_type(filename)[0] or "application/octet-stream"
        with open(file_path, "rb") as handle:
            file_data = handle.read()
        chunks = [
            ("--{}\r\nContent-Disposition: form-data; name=\"RegionId\"\r\n\r\n{}\r\n".format(boundary, folder_id)).encode("utf-8"),
            ("--{}\r\nContent-Disposition: form-data; name=\"File\"; filename=\"{}\"\r\nContent-Type: {}\r\n\r\n".format(boundary, filename.replace('"', "_"), content_type)).encode("utf-8"),
            file_data,
            "\r\n--{}--\r\n".format(boundary).encode("utf-8"),
        ]
        headers = self._file_headers()
        headers["Content-Type"] = "multipart/form-data; boundary={}".format(boundary)
        request = urllib.request.Request(
            self.file_base_url + "/api/app/UploadFileToFolder",
            data=b"".join(chunks),
            headers=headers,
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=max(self.timeout_sec, 120.0)) as response:
            result = json.loads(response.read().decode("utf-8", errors="replace"))
        if not _response_ok(result):
            raise RuntimeError("upload {} failed: {}".format(filename, _response_message(result)))

    def _upload_directory(self, local_dir, remote_parts):
        if not os.path.isdir(local_dir):
            return 0
        uploaded = 0
        for root, directories, files in os.walk(local_dir):
            directories.sort()
            files.sort()
            relative = os.path.relpath(root, local_dir)
            parts = list(remote_parts)
            if relative != ".":
                parts.extend(relative.split(os.sep))
            folder_id = self._ensure_folder_path(parts)
            for filename in files:
                if filename.endswith(".tmp"):
                    continue
                self._multipart_upload(os.path.join(root, filename), folder_id)
                uploaded += 1
        return uploaded

    def _upload_map(self, job):
        map_parts = [self.remote_root_name, "maps", job["map_id"]]
        uploaded = self._upload_directory(job["local_map_dir"], map_parts)
        stcm_path = job["stcm_path"]
        if stcm_path and os.path.isfile(stcm_path):
            folder_id = self._ensure_folder_path(map_parts)
            self._multipart_upload(stcm_path, folder_id)
            uploaded += 1
        if uploaded <= 0:
            raise RuntimeError("no local map files found")
        rospy.loginfo(
            "Map files uploaded: project_id=%s remote=%s/maps/%s files=%d",
            self.project_id,
            self.remote_root_name,
            job["map_id"],
            uploaded,
        )

    def _worker(self):
        try:
            self._authenticate()
        except Exception as exc:
            rospy.logwarn("Platform startup authentication failed; retry on map upload: %s", exc)
        while not self._stop_event.is_set():
            try:
                job = self._jobs.get(timeout=0.5)
            except queue.Empty:
                continue
            if job is None:
                break
            try:
                if not self.get_project_id() or not self._file_token:
                    self._authenticate()
                self._upload_map(job)
            except Exception as exc:
                rospy.logerr("Map file upload failed: map_id=%s error=%s", job.get("map_id", ""), exc)
            finally:
                self._jobs.task_done()


class MqttDeviceReporter:
    def __init__(
        self,
        enabled,
        broker_host,
        broker_port,
        dev_code,
        username,
        password,
        status_provider,
        keepalive_sec=60,
        qos=1,
        status_period_sec=3.0,
    ):
        self.enabled = bool(enabled)
        self.broker_host = str(broker_host or "")
        self.broker_port = int(broker_port)
        self.dev_code = str(dev_code or "").strip()
        self.username = str(username or "")
        self.password = str(password or "")
        self.status_provider = status_provider
        self.keepalive_sec = max(10, int(keepalive_sec))
        self.qos = max(0, min(2, int(qos)))
        self.status_period_sec = max(0.5, float(status_period_sec))
        self._client = None
        self._connected = threading.Event()
        self._stop_event = threading.Event()
        self._status_thread = None
        self.data_topic = "rg/cloud/deviceData/002/{}".format(self.dev_code)
        self.online_topic = "rg/cloud/deviceState/002/{}/online".format(self.dev_code)
        self.offline_topic = "rg/cloud/deviceState/002/{}/offline".format(self.dev_code)

    def start(self):
        if not self.enabled:
            return
        if mqtt is None:
            rospy.logerr("MQTT disabled: python3-paho-mqtt is not installed")
            self.enabled = False
            return
        if not self.dev_code or not self.username or not self.password:
            rospy.logwarn("MQTT disabled: devCode or MQTT credentials is empty")
            self.enabled = False
            return
        client_id = "grinder-{}".format(self.dev_code)
        try:
            callback_version = getattr(mqtt, "CallbackAPIVersion", None)
            if callback_version is not None:
                self._client = mqtt.Client(callback_version.VERSION2, client_id=client_id)
            else:
                self._client = mqtt.Client(client_id=client_id)
        except TypeError:
            self._client = mqtt.Client(client_id=client_id)
        self._client.username_pw_set(self.username, self.password)
        self._client.will_set(
            self.offline_topic,
            json.dumps({"message": "设备因网络断开离线", "timestamp": _timestamp_ms()}, ensure_ascii=False),
            qos=self.qos,
            retain=False,
        )
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.connect_async(self.broker_host, self.broker_port, self.keepalive_sec)
        self._client.loop_start()
        self._status_thread = threading.Thread(target=self._status_loop, name="grinder-mqtt-status", daemon=True)
        self._status_thread.start()

    def stop(self):
        self._stop_event.set()
        if self._client is None:
            return
        if self._connected.is_set():
            try:
                info = self._client.publish(
                    self.offline_topic,
                    json.dumps({"message": "设备程序主动退出", "timestamp": _timestamp_ms()}, ensure_ascii=False),
                    qos=self.qos,
                    retain=False,
                )
                info.wait_for_publish(timeout=2.0)
            except Exception as exc:
                rospy.logwarn("MQTT offline publish failed: %s", exc)
        try:
            self._client.disconnect()
            self._client.loop_stop()
        except Exception:
            pass
        if self._status_thread is not None:
            self._status_thread.join(timeout=2.0)

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        try:
            code = int(reason_code)
        except Exception:
            code = getattr(reason_code, "value", -1)
        if code != 0:
            rospy.logerr("MQTT login failed: broker=%s:%d reason=%s", self.broker_host, self.broker_port, reason_code)
            return
        self._connected.set()
        client.publish(
            self.online_topic,
            json.dumps({"message": "设备在线", "timestamp": _timestamp_ms()}, ensure_ascii=False),
            qos=self.qos,
            retain=False,
        )
        rospy.loginfo("MQTT login success: broker=%s:%d client_id=grinder-%s", self.broker_host, self.broker_port, self.dev_code)

    def _on_disconnect(self, client, userdata, disconnect_flags=None, reason_code=None, properties=None):
        self._connected.clear()
        if not self._stop_event.is_set():
            rospy.logwarn("MQTT disconnected: reason=%s", reason_code if reason_code is not None else disconnect_flags)

    def _status_loop(self):
        while not self._stop_event.wait(self.status_period_sec):
            if not self._connected.is_set() or self._client is None:
                continue
            try:
                reported = self.status_provider() or {}
                payload = {"reported": reported, "timestamp": _timestamp_ms()}
                self._client.publish(
                    self.data_topic,
                    json.dumps(payload, ensure_ascii=False, separators=(",", ":")),
                    qos=self.qos,
                    retain=False,
                )
            except Exception as exc:
                rospy.logwarn_throttle(5.0, "MQTT status publish failed: %s", exc)
