import socket
import socketserver
import threading
import time
from collections import deque
from grinder_scheduler.priority_wire import PriorityWire
from grinder_scheduler.channel_sessions import ChannelSessions, BulkBudget
from concurrent.futures import ThreadPoolExecutor

from grinder_scheduler.sl_link_loader import ensure_sl_linka_sdk_on_path
try:
    import rospy  # type: ignore
except Exception:
    rospy = None


class _BoundedLatestWorker:
    """Single worker with bounded pending jobs; newest jobs replace the oldest."""

    def __init__(self, max_pending, name):
        self._max_pending = max(1, int(max_pending))
        self._jobs = deque()
        self._condition = threading.Condition()
        self._running = True
        self._thread = threading.Thread(target=self._run, name=name, daemon=True)
        self._thread.start()

    def submit(self, callback, *args):
        dropped = None
        with self._condition:
            if not self._running:
                raise RuntimeError("worker is shut down")
            if len(self._jobs) >= self._max_pending:
                dropped = self._jobs.popleft()
            self._jobs.append((callback, args))
            self._condition.notify()
        return dropped

    def clear(self):
        with self._condition:
            self._jobs.clear()

    def shutdown(self, wait=False):
        with self._condition:
            self._running = False
            self._jobs.clear()
            self._condition.notify_all()
        if wait and threading.current_thread() is not self._thread:
            self._thread.join()

    def _run(self):
        while True:
            with self._condition:
                while self._running and not self._jobs:
                    self._condition.wait()
                if not self._running:
                    return
                callback, args = self._jobs.popleft()
            try:
                callback(*args)
            except Exception as exc:
                if rospy is not None:
                    rospy.logerr("SL-LinkA bounded worker failed: %s", exc)


class _BoundedFifoWorker(_BoundedLatestWorker):
    """Bounded ordered business work; reject overload without dropping old jobs."""
    def submit(self, callback, *args):
        with self._condition:
            if not self._running or len(self._jobs) >= self._max_pending:
                return False
            self._jobs.append((callback, args))
            self._condition.notify()
            return True


class _DropIfBusyWorker:
    """Run one job at a time and reject arrivals while the job is active."""

    def __init__(self, name):
        self._condition = threading.Condition()
        self._job = None
        self._busy = False
        self._running = True
        self._thread = threading.Thread(target=self._run, name=name, daemon=True)
        self._thread.start()

    def submit(self, callback, *args):
        with self._condition:
            if not self._running:
                raise RuntimeError("worker is shut down")
            if self._busy or self._job is not None:
                return False
            self._job = (callback, args)
            self._condition.notify()
            return True

    def shutdown(self, wait=False):
        with self._condition:
            self._running = False
            self._job = None
            self._condition.notify_all()
        if wait and threading.current_thread() is not self._thread:
            self._thread.join()

    def _run(self):
        while True:
            with self._condition:
                while self._running and self._job is None:
                    self._condition.wait()
                if not self._running:
                    return
                callback, args = self._job
                self._job = None
                self._busy = True
            try:
                callback(*args)
            except Exception as exc:
                if rospy is not None:
                    rospy.logerr("SL-LinkA drop-if-busy worker failed: %s", exc)
            finally:
                with self._condition:
                    self._busy = False


class SlLinkAServer:
    def __init__(
        self,
        sdk_dir,
        host,
        port,
        callback_handler,
        rx_diagnostic_logging=False,
        rx_sample_interval_sec=1.0,
        rx_stats_interval_sec=60.0,
        metrics_registry=None,
        control_port=8003,
        session_timeout_sec=3.0,
        manual_timeout_sec=0.5,
        bulk_bytes_per_sec=1048576,
        bulk_manual_bytes_per_sec=262144,
        bulk_burst_bytes=8192,
    ):
        ensure_sl_linka_sdk_on_path(sdk_dir)
        from sl_link.frame import SL_PROTOCOL_VERSION, SlFrame, SlFrameParser
        from sl_link.message_gen import sl_link_pb2 as pb

        self.SL_PROTOCOL_VERSION = SL_PROTOCOL_VERSION
        self.SlFrame = SlFrame
        self.SlFrameParser = SlFrameParser
        self.pb = pb
        self.host = host
        self.port = port
        self.control_port = control_port
        self.sessions = ChannelSessions(timeout=session_timeout_sec)
        self.bulk_budget = BulkBudget(bulk_bytes_per_sec, bulk_manual_bytes_per_sec, bulk_burst_bytes)
        self.manual_timeout_sec = max(0.1, float(manual_timeout_sec))
        self._listeners = []
        self._peers = set()
        self._peers_lock = threading.Lock()
        self._stopping = threading.Event()
        self._handler = callback_handler
        self._server = None
        self._thread = None
        self._seq = 0
        self._seq_lock = threading.Lock()
        self._rx_diagnostic_logging = bool(rx_diagnostic_logging)
        self._rx_sample_interval_sec = max(0.0, float(rx_sample_interval_sec))
        self._rx_stats_interval_sec = max(1.0, float(rx_stats_interval_sec))
        self._rx_log_lock = threading.Lock()
        self._rx_stats_started_at = time.monotonic()
        self._rx_last_sample_at = self._rx_stats_started_at - self._rx_sample_interval_sec
        self._rx_frames = 0
        self._rx_payload_bytes = 0
        self._rx_frame_counter = metrics_registry.counter("sl_rx_frames") if metrics_registry else None
        self._control_expired = metrics_registry.counter("control_expired_drops") if metrics_registry else None
        self._control_overwritten = metrics_registry.counter("control_queue_overwrites") if metrics_registry else None
        self._control_apply = metrics_registry.histogram("control_apply_ms") if metrics_registry else None
        self._control_ack_wait = metrics_registry.histogram("control_ack_wait_ms") if metrics_registry else None
        self._control_queue_wait = metrics_registry.histogram("control_queue_wait_ms") if metrics_registry else None
        self._ordered_queue_wait = metrics_registry.histogram("ordered_queue_wait_ms") if metrics_registry else None
        self._map_chunks_sent = metrics_registry.counter("map_chunks_sent") if metrics_registry else None
        self._map_frame_pack_ms = metrics_registry.histogram("map_frame_pack_ms") if metrics_registry else None
        self._map_socket_send_ms = metrics_registry.histogram("map_socket_send_ms") if metrics_registry else None
        self._map_send_total_ms = metrics_registry.histogram("map_send_total_ms") if metrics_registry else None
        self._map_socket_send_total_ms = metrics_registry.histogram("map_socket_send_total_ms") if metrics_registry else None
        self._map_request_total_ms = metrics_registry.histogram("map_request_total_ms") if metrics_registry else None
        self._map_wire_bytes_sent = metrics_registry.counter("map_wire_bytes_sent") if metrics_registry else None
        self._map_send_failures = metrics_registry.counter("map_send_failures") if metrics_registry else None
        self._map_build_failures = metrics_registry.counter("map_build_failures") if metrics_registry else None
        self._session_busy = metrics_registry.counter("channel_busy_rejections") if metrics_registry else None
        self._session_expired = metrics_registry.counter("channel_lease_expirations") if metrics_registry else None
        self._disconnect_apply = metrics_registry.histogram("control_disconnect_apply_ms") if metrics_registry else None

    def _enum_name(self, enum_type_name, value):
        enum_type = getattr(self.pb, enum_type_name, None)
        if enum_type is None or not hasattr(enum_type, "Name"):
            return str(value)
        try:
            return enum_type.Name(int(value))
        except Exception:
            return str(value)

    def _msg_id_name(self, msg_id):
        return self._enum_name("MessageId", msg_id)

    def _message_to_log_dict(self, message, depth=0):
        if depth > 2:
            return "<nested>"
        out = {}
        try:
            fields = message.ListFields()
        except Exception:
            return str(message)
        for field, value in fields:
            if getattr(field, "type", None) == getattr(field, "TYPE_ENUM", None):
                out[field.name] = self._enum_field_value_to_log(field, value)
            else:
                out[field.name] = self._value_to_log(value, depth=depth + 1)
        return out

    def _enum_field_value_to_log(self, field, value):
        def one(enum_value):
            try:
                enum_desc = field.enum_type.values_by_number.get(int(enum_value))
                if enum_desc is not None:
                    return enum_desc.name
            except Exception:
                pass
            return int(enum_value)

        if hasattr(value, "__iter__") and not isinstance(value, (str, bytes, bytearray)):
            try:
                return [one(item) for item in value]
            except Exception:
                return str(value)
        return one(value)

    def _value_to_log(self, value, depth=0):
        if isinstance(value, (bytes, bytearray)):
            return "<bytes {}>".format(len(value))
        if isinstance(value, str):
            if len(value) > 160:
                return value[:160] + "...<truncated>"
            return value
        if isinstance(value, (int, float, bool)) or value is None:
            return value
        if isinstance(value, (list, tuple)) or hasattr(value, "__iter__") and not hasattr(value, "ListFields"):
            values = []
            try:
                for index, item in enumerate(value):
                    if index >= 8:
                        values.append("...<{} more>".format(max(0, len(value) - 8) if hasattr(value, "__len__") else "?"))
                        break
                    values.append(self._value_to_log(item, depth=depth + 1))
                return values
            except Exception:
                return str(value)
        if hasattr(value, "ListFields"):
            return self._message_to_log_dict(value, depth=depth + 1)
        return str(value)

    def _request_message_class_name(self, msg_id):
        pb = self.pb
        pairs = [
            ("MSG_ID_SETTINGS_READ_REQUEST", "SettingsReadRequest"),
            ("MSG_ID_SETTINGS_WRITE_REQUEST", "SettingsWriteRequest"),
            ("MSG_ID_CONTROL_COMMAND", "ControlCommand"),
            ("MSG_ID_TASK_CONFIG", "TaskConfig"),
            ("MSG_ID_TASK_COMMAND", "TaskCommand"),
            ("MSG_ID_TASK_RUNTIME_SETTINGS_REQUEST", "TaskRuntimeSettingsRequest"),
            ("MSG_ID_PATH_POINT_PLAN_REQUEST", "PathPointPlanRequest"),
            ("MSG_ID_CAMERA_FRAME_REQUEST", "CameraFrameRequest"),
            ("MSG_ID_MAP_REQUEST", "MapRequest"),
            ("MSG_ID_MAP_PREVIEW_REQUEST", "MapPreviewRequest"),
            ("MSG_ID_MAP_REGION_POINT_REQUEST", "MapRegionPointRequest"),
            ("MSG_ID_MAP_EDIT_COMMAND", "MapEditCommand"),
            ("MSG_ID_VIDEO_STREAM_INFO_REQUEST", "VideoStreamInfoRequest"),
            ("MSG_ID_MAP_SYNC_REQUEST", "MapSyncRequest"),
            ("MSG_ID_MAP_IMPORT_TO_RADAR_REQUEST", "MapImportToRadarRequest"),
            ("MSG_ID_MAP_MODE_REQUEST", "MapModeRequest"),
            ("MSG_ID_MAP_CATALOG_REQUEST", "MapCatalogRequest"),
            ("MSG_ID_MAP_DELETE_REQUEST", "MapDeleteRequest"),
            ("MSG_ID_MAP_SAVE_REQUEST", "MapSaveRequest"),
            ("MSG_ID_MAP_METRICS_REQUEST", "MapMetricsRequest"),
            ("MSG_ID_TASK_RESULT_REQUEST", "TaskResultRequest"),
            ("MSG_ID_TASK_EXECUTION_HISTORY_REQUEST", "TaskExecutionHistoryRequest"),
            ("MSG_ID_TASK_EXECUTION_DELETE_REQUEST", "TaskExecutionDeleteRequest"),
            ("MSG_ID_TASK_TRAJECTORY_REQUEST", "TaskTrajectoryRequest"),
            ("MSG_ID_SYSTEM_CACHE_CLEAR_REQUEST", "SystemCacheClearRequest"),
            ("MSG_ID_LIVE_MAP_CACHE_CLEAR_REQUEST", "LiveMapCacheClearRequest"),
            ("MSG_ID_RADAR_MAP_CACHE_CLEAR_REQUEST", "RadarMapCacheClearRequest"),
            ("MSG_ID_MAP_ALIGNMENT_REQUEST", "MapAlignmentRequest"),
            ("MSG_ID_RADAR_SYSTEM_STATUS_REQUEST", "RadarSystemStatusRequest"),
            ("MSG_ID_RADAR_MAP_SYNC_REQUEST", "RadarMapSyncRequest"),
            ("MSG_ID_RADAR_RELOCALIZATION_REQUEST", "RadarRelocalizationRequest"),
            (
                "MSG_ID_RADAR_RELOCALIZATION_STATUS_REQUEST",
                "RadarRelocalizationStatusRequest",
            ),
            ("MSG_ID_PATH_PLAN_REQUEST", "PathPlanRequest"),
        ]
        for msg_const_name, class_name in pairs:
            if hasattr(pb, msg_const_name) and int(msg_id) == int(getattr(pb, msg_const_name)):
                return class_name
        return ""

    def _log_rx_frame(self, frame):
        # 每帧只更新内存计数；采样日志和指标文件写入不在 TCP 接收热路径执行。
        if self._rx_frame_counter is not None:
            self._rx_frame_counter.inc()
        if rospy is None:
            return
        now = time.monotonic()
        payload_len = len(frame.payload or b"")
        with self._rx_log_lock:
            self._rx_frames += 1
            self._rx_payload_bytes += payload_len
            sample = self._rx_diagnostic_logging or (
                self._rx_sample_interval_sec > 0.0
                and now - self._rx_last_sample_at >= self._rx_sample_interval_sec
            )
            if sample:
                self._rx_last_sample_at = now
            elapsed = now - self._rx_stats_started_at
            stats = None
            if elapsed >= self._rx_stats_interval_sec:
                stats = (self._rx_frames, self._rx_payload_bytes, elapsed)
                self._rx_frames = 0
                self._rx_payload_bytes = 0
                self._rx_stats_started_at = now
        if sample:
            rospy.loginfo(
                "SL-LinkA RX %s: version=%d flags=0x%02X seq=%d ack=%d src=%d dst=%d "
                "comp=%d msg_id=0x%04X name=%s payload_len=%d",
                "frame" if self._rx_diagnostic_logging else "sample",
                int(frame.version),
                int(frame.flags),
                int(frame.seq),
                int(frame.ack_seq),
                int(frame.src_id),
                int(frame.dst_id),
                int(frame.comp_id),
                int(frame.msg_id),
                self._msg_id_name(frame.msg_id),
                payload_len,
            )
        if stats is not None:
            frames, payload_bytes, elapsed = stats
            rospy.loginfo(
                "SL-LinkA RX stats: interval_sec=%.1f frames=%d frames_per_sec=%.1f payload_bytes=%d",
                elapsed,
                frames,
                frames / elapsed,
                payload_bytes,
            )

    def start(self):
        if rospy is not None:
            rospy.loginfo(
                "SL-LinkA RX logging: diagnostic=%s sample_interval_sec=%.1f stats_interval_sec=%.1f",
                self._rx_diagnostic_logging,
                self._rx_sample_interval_sec,
                self._rx_stats_interval_sec,
            )
        outer = self

        class RequestHandler(socketserver.BaseRequestHandler):
            def setup(self):
                self.parser = outer.SlFrameParser()
                self.running = True
                self.send_lock = PriorityWire()
                self.session = object()
                self.role = self.server.channel_role
                self.admitted = False
                self.legacy = False
                self.created_at = time.monotonic()
                self.manual_received_at = None
                self.close_lock = threading.Lock()
                self.closed_once = False
                self.request.settimeout(0.1)
                self.stop_executor = _BoundedLatestWorker(1, "sl_linka_stop")
                self.control_dispatch_executor = _BoundedFifoWorker(8, "sl_linka_control")
                try:
                    self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    self.request.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF,
                                           16 * 1024 if self.role == "control" else 64 * 1024)
                except OSError as exc:
                    if rospy is not None:
                        rospy.logwarn("SL-LinkA TCP send tuning failed: %s", exc)
                # Keep command queues bounded. When producers outrun consumers,
                # stale pending requests are discarded in favor of newer input.
                self.dispatch_executor = _BoundedLatestWorker(
                    max_pending=32,
                    name="sl_linka_ordered",
                )
                # Local map deletion must not queue behind a synchronous map
                # mode transition, which may wait for Super-LIO health checks.
                self.map_delete_executor = ThreadPoolExecutor(
                    max_workers=1,
                    thread_name_prefix="sl_linka_delete",
                )
                self.map_dispatch_executor = _BoundedLatestWorker(
                    max_pending=4,
                    name="sl_linka_map",
                )
                self.manual_control_executor = _BoundedLatestWorker(
                    max_pending=1,
                    name="sl_linka_manual",
                )
                self.manual_response_executor = _DropIfBusyWorker(
                    name="sl_linka_manual_tx",
                )
                self.control_response_executor = ThreadPoolExecutor(
                    max_workers=1,
                    thread_name_prefix="sl_linka_control_tx",
                )
                self.control_response_slots = threading.BoundedSemaphore(8)
                # Large path responses must not hold the ordered request worker
                # while TCP applies backpressure. One worker preserves response
                # ordering; two slots bound queued path memory.
                self.bulk_response_executor = ThreadPoolExecutor(
                    max_workers=1,
                    thread_name_prefix="sl_linka_bulk_tx",
                )
                self.bulk_response_slots = threading.BoundedSemaphore(2)
                with outer._peers_lock:
                    outer._peers.add(self)
                self.status_thread = threading.Thread(target=self._periodic_reports, daemon=True)
                self.status_thread.start()

            def _periodic_reports(self):
                while self.running:
                    try:
                        if self.role == "data" and self._valid():
                            self._send_payload(*outer._handler.build_device_status_report())
                            self._send_payload(*outer._handler.build_task_status_report())
                    except Exception:
                        pass
                    time.sleep(1.0)

            def handle(self):
                while self.running:
                    try:
                        data = self.request.recv(32 * 1024)
                    except socket.timeout:
                        # Keep the connection alive on idle timeout.
                        continue
                    except OSError as exc:
                        if rospy is not None:
                            rospy.logwarn("SL-LinkA socket recv failed: %s", exc)
                        break

                    if not data:
                        break
                    # The historical APP sent a single NUL as its transport heartbeat.
                    # Only an admitted readonly connection may renew via that format.
                    if self.legacy and data == b"\x00":
                        outer.sessions.touch(self.role, self)
                        continue
                    for frame in self.parser.parse(data):
                        if frame.dst_id not in (outer.pb.DEVICE_LOWER, outer.pb.DEVICE_BROADCAST):
                            continue
                        received_at = time.monotonic()
                        outer._log_rx_frame(frame)
                        try:
                            if int(frame.msg_id) == int(outer.pb.MSG_ID_CHANNEL_SESSION_REQUEST):
                                self._session_request(frame)
                                continue
                            if not self.admitted:
                                if self.role == "data" and self._readonly(frame.msg_id):
                                    if not self._admit(legacy=True):
                                        self._reject(frame, "Device is occupied", outer.pb.RESULT_BUSY)
                                        self._close()
                                        return
                                else:
                                    self._reject(frame, "Channel handshake required", outer.pb.RESULT_UNSUPPORTED)
                                    self._close()
                                    return
                            if not outer.sessions.touch(self.role, self):
                                self._close()
                                return
                            if not self._allowed(frame):
                                self._reject(frame, "Request is not allowed on this channel", outer.pb.RESULT_UNSUPPORTED)
                                continue
                            if int(frame.msg_id) == int(outer.pb.MSG_ID_TASK_COMMAND) and self.role == "control":
                                # Revoke queued manual output on receipt, before any slow task cleanup.
                                outer._handler.cancel_manual_session(self.session)
                                self.manual_control_executor.clear()
                                capture = getattr(outer._handler, "capture_stop_generation", None)
                                generation = capture(self.session) if capture else outer._handler.capture_control_generation()
                                outer._stop_pending.set()
                                outer._stop_generation = generation
                                dropped = outer._task_stop_executor.submit(self._dispatch_task_stop, frame, generation)
                                if dropped is not None:
                                    self._reject(dropped[1][0], "Superseded by a newer stop", outer.pb.RESULT_BUSY)
                                continue
                            if int(frame.msg_id) == int(outer.pb.MSG_ID_CONTROL_COMMAND):
                                control_request = self._parse_control_command(frame)
                                if (
                                    control_request is not None
                                    and control_request.HasField("manual_drive")
                                    and not control_request.HasField("emergency_stop")
                                ):
                                    ticket, stop = outer._handler.capture_manual_ticket(
                                        self.session, control_request, received_at)
                                    if stop:
                                        self.manual_control_executor.clear()
                                        self.manual_received_at = None
                                    else:
                                        self.manual_received_at = received_at
                                        outer.bulk_budget.manual(outer.manual_timeout_sec)
                                    executor = self.stop_executor if stop else self.manual_control_executor
                                    dropped = executor.submit(
                                        self._dispatch_control_immediately,
                                        frame,
                                        control_request,
                                        True,
                                        received_at,
                                        outer._handler.capture_control_generation(),
                                        ticket,
                                    )
                                    self._log_dropped_job("manual control", dropped)
                                elif (control_request is not None
                                      and control_request.HasField("emergency_stop")
                                      and control_request.emergency_stop.enabled):
                                    # Latch/zero immediately; reset and ordinary
                                    # output business must never block receiving.
                                    self._dispatch_control_immediately(
                                        frame,
                                        control_request,
                                        False,
                                        received_at,
                                    )
                                else:
                                    accepted = self.control_dispatch_executor.submit(
                                        self._dispatch_control_immediately,
                                        frame, control_request, False, received_at,
                                        outer._handler.capture_control_generation())
                                    if not accepted:
                                        response = outer.pb.ControlCommandResponse(
                                            result=outer.pb.RESULT_FAILED,
                                            message="Control business queue full; request not applied")
                                        self._queue_control_response(
                                            response.SerializeToString(), outer.pb.MSG_ID_CONTROL_COMMAND_RESPONSE,
                                            outer.pb.COMP_CONTROL, int(frame.seq))
                            elif self._is_map_background_request(frame.msg_id):
                                dropped = self.map_dispatch_executor.submit(
                                    self._dispatch_ordered,
                                    frame,
                                    received_at,
                                    "map",
                                )
                                self._log_dropped_job("map request", dropped)
                            elif self._is_map_delete_request(frame.msg_id):
                                self.map_delete_executor.submit(
                                    self._dispatch_ordered,
                                    frame,
                                    received_at,
                                    "delete",
                                )
                            else:
                                dropped = self.dispatch_executor.submit(
                                    self._dispatch_ordered,
                                    frame,
                                    received_at,
                                    "ordered",
                                    outer._handler.capture_control_generation()
                                    if int(frame.msg_id) in (int(outer.pb.MSG_ID_TASK_COMMAND),
                                        int(outer.pb.MSG_ID_SETTINGS_WRITE_REQUEST),
                                        int(outer.pb.MSG_ID_TASK_RUNTIME_SETTINGS_REQUEST)) else None,
                                )
                                self._log_dropped_job("ordered request", dropped)
                        except Exception as exc:
                            if rospy is not None:
                                rospy.logerr(
                                    "SL-LinkA dispatch failed: msg_id=0x%04X seq=%d err=%s",
                                    int(frame.msg_id),
                                    int(frame.seq),
                                    exc,
                                )

            def finish(self):
                self._close()
                with outer._peers_lock:
                    outer._peers.discard(self)
                self.stop_executor.shutdown(wait=False)
                self.control_dispatch_executor.shutdown(wait=False)
                self.dispatch_executor.shutdown(wait=False)
                self.map_delete_executor.shutdown(wait=False)
                self.map_dispatch_executor.shutdown(wait=False)
                self.manual_control_executor.shutdown(wait=False)
                self.manual_response_executor.shutdown(wait=False)
                self.control_response_executor.shutdown(wait=False)
                self.bulk_response_executor.shutdown(wait=False)

            def _valid(self):
                return self.running and outer.sessions.valid(self.role, self)

            def _close(self):
                with self.close_lock:
                    if self.closed_once:
                        return
                    self.closed_once = True
                    self.running = False
                    outer.sessions.detach(self)
                try:
                    self.request.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                started = time.monotonic()
                if self.role == "control" and self.admitted:
                    outer._handler.cancel_manual_session(self.session)
                    if outer._disconnect_apply is not None:
                        outer._disconnect_apply.observe((time.monotonic() - started) * 1000.0)
                self.manual_control_executor.clear()

            def _admit(self, token="", legacy=False):
                if outer._stopping.is_set():
                    return False
                accepted, issued, retired = outer.sessions.attach(self.role, self, token, legacy)
                for peer in retired:
                    peer._close()
                if not accepted:
                    if outer._session_busy is not None:
                        outer._session_busy.inc()
                    return False
                first = not self.admitted
                self.admitted = True
                self.legacy = legacy
                if self.role == "control" and first:
                    outer._handler.begin_manual_session(self.session)
                return issued

            def _session_response(self, frame, result, message="", token=""):
                response = outer.pb.ChannelSessionResponse(
                    result=result, message=message,
                    role=outer.pb.CHANNEL_ROLE_CONTROL if self.role == "control" else outer.pb.CHANNEL_ROLE_DATA,
                    token=token, heartbeat_interval_ms=1000,
                    session_timeout_ms=int(outer.sessions.timeout * 1000),
                    control_ready=outer.sessions.control_ready(),
                    emergency_stop_latched=bool(getattr(getattr(outer._handler, "_motion", None), "latched", False)))
                self._send_payload(response.SerializeToString(), outer.pb.MSG_ID_CHANNEL_SESSION_RESPONSE,
                                   comp_id=outer.pb.COMP_CONTROL, ack_seq=frame.seq, control=True)

            def _session_request(self, frame):
                pb = outer.pb
                request = pb.ChannelSessionRequest()
                request.ParseFromString(frame.payload)
                role = pb.CHANNEL_ROLE_CONTROL if self.role == "control" else pb.CHANNEL_ROLE_DATA
                if request.role != role:
                    self._session_response(frame, pb.RESULT_INVALID_PARAM, "Wrong channel role")
                    return
                if request.action in (pb.CHANNEL_SESSION_OPEN, pb.CHANNEL_SESSION_BIND, pb.CHANNEL_SESSION_RECOVER):
                    if (self.role == "control" and not request.token) or self.legacy:
                        self._session_response(frame, pb.RESULT_INVALID_PARAM, "A valid tablet token is required")
                        self._close()
                        return
                    token = self._admit(request.token)
                    self._session_response(frame, pb.RESULT_SUCCESS if token else pb.RESULT_BUSY,
                                           "Channel ready" if token else "Device is occupied or token expired", token or "")
                    if not token:
                        self._close()
                elif request.action == pb.CHANNEL_SESSION_HEARTBEAT:
                    valid = outer.sessions.touch(self.role, self)
                    self._session_response(frame, pb.RESULT_SUCCESS if valid else pb.RESULT_FAILED,
                                           "Alive" if valid else "Session expired")
                    if not valid:
                        self._close()
                elif request.action == pb.CHANNEL_SESSION_RELEASE:
                    retired = outer.sessions.release(self.role, self)
                    self._session_response(frame, pb.RESULT_SUCCESS if retired else pb.RESULT_FAILED, "Released")
                    for peer in retired:
                        peer._close()
                else:
                    self._session_response(frame, pb.RESULT_INVALID_PARAM, "Unknown session action")

            def _readonly(self, msg_id):
                names = ("SETTINGS_READ_REQUEST", "MAP_REQUEST", "MAP_PREVIEW_REQUEST", "MAP_CATALOG_REQUEST",
                         "MAP_METRICS_REQUEST", "MAP_REGION_POINT_REQUEST", "TASK_RESULT_REQUEST",
                         "TASK_EXECUTION_HISTORY_REQUEST", "TASK_TRAJECTORY_REQUEST", "VIDEO_STREAM_INFO_REQUEST",
                         "RADAR_SYSTEM_STATUS_REQUEST", "RADAR_RELOCALIZATION_STATUS_REQUEST", "CAMERA_FRAME_REQUEST")
                return int(msg_id) in {int(getattr(outer.pb, "MSG_ID_" + name)) for name in names}

            def _allowed(self, frame):
                pb = outer.pb
                if self.legacy:
                    return self._readonly(frame.msg_id)
                if int(frame.msg_id) == int(pb.MSG_ID_TASK_COMMAND):
                    request = pb.TaskCommand()
                    request.ParseFromString(frame.payload)
                    urgent = request.command in (pb.TASK_CMD_STOP, pb.TASK_CMD_PAUSE)
                    return urgent if self.role == "control" else (
                        not urgent and outer.sessions.control_ready() and not outer._stop_pending.is_set())
                if self.role == "control":
                    if int(frame.msg_id) != int(pb.MSG_ID_CONTROL_COMMAND):
                        return False
                    request = self._parse_control_command(frame)
                    if request is not None and request.HasField("manual_drive"):
                        from grinder_scheduler.manual_delivery import is_manual_stop
                        if getattr(outer._handler, "_exec_active", False) and not is_manual_stop(request.manual_drive, pb):
                            return False
                    return True
                if int(frame.msg_id) == int(pb.MSG_ID_SETTINGS_WRITE_REQUEST):
                    return outer.sessions.control_ready() and not outer._stop_pending.is_set()
                if int(frame.msg_id) == int(pb.MSG_ID_TASK_RUNTIME_SETTINGS_REQUEST):
                    request = pb.TaskRuntimeSettingsRequest()
                    request.ParseFromString(frame.payload)
                    if request.HasField("run_speed") or request.HasField("disc_speed_rpm"):
                        return outer.sessions.control_ready() and not outer._stop_pending.is_set()
                return int(frame.msg_id) != int(pb.MSG_ID_CONTROL_COMMAND)

            def _reject(self, frame, message, result):
                pb = outer.pb
                if int(frame.msg_id) == int(pb.MSG_ID_CONTROL_COMMAND):
                    response = pb.ControlCommandResponse(result=result, message=message)
                    self._send_payload(response.SerializeToString(), pb.MSG_ID_CONTROL_COMMAND_RESPONSE,
                                       comp_id=pb.COMP_CONTROL, ack_seq=frame.seq, control=True)
                elif int(frame.msg_id) == int(pb.MSG_ID_TASK_COMMAND):
                    response = pb.TaskCommandResponse(result=result, message=message)
                    self._send_payload(response.SerializeToString(), pb.MSG_ID_TASK_COMMAND_RESPONSE,
                                       comp_id=pb.COMP_SCHEDULER, ack_seq=frame.seq, control=True)
                elif int(frame.msg_id) in (int(pb.MSG_ID_SETTINGS_WRITE_REQUEST), int(pb.MSG_ID_TASK_RUNTIME_SETTINGS_REQUEST)):
                    settings = int(frame.msg_id) == int(pb.MSG_ID_SETTINGS_WRITE_REQUEST)
                    response_type = pb.SettingsWriteResponse if settings else pb.TaskRuntimeSettingsResponse
                    response = response_type(result=result, message=message)
                    response_id = pb.MSG_ID_SETTINGS_WRITE_RESPONSE if settings else pb.MSG_ID_TASK_RUNTIME_SETTINGS_RESPONSE
                    self._send_payload(response.SerializeToString(), response_id,
                                       comp_id=pb.COMP_SETTINGS if settings else pb.COMP_SCHEDULER,
                                       ack_seq=frame.seq, control=True)
                else:
                    self._session_response(frame, result, message)

            def _dispatch_task_stop(self, frame, generation):
                try:
                    # An accepted stop must finish cleanup even if its socket closes.
                    payload, msg_id, comp_id = outer._handler.handle_task_command(
                        frame.payload, control_generation=generation)
                    if self._valid():
                        self._queue_control_response(payload, msg_id, comp_id, int(frame.seq))
                finally:
                    if outer._stop_generation == generation:
                        outer._stop_pending.clear()

            def _parse_control_command(self, frame):
                try:
                    request = outer.pb.ControlCommand()
                    request.ParseFromString(frame.payload or b"")
                    return request
                except Exception:
                    return None

            def _is_map_background_request(self, msg_id):
                names = (
                    "MSG_ID_MAP_REQUEST",
                    "MSG_ID_MAP_PREVIEW_REQUEST",
                    "MSG_ID_MAP_CATALOG_REQUEST",
                    "MSG_ID_MAP_METRICS_REQUEST",
                    "MSG_ID_MAP_REGION_POINT_REQUEST",
                )
                return int(msg_id) in {
                    int(getattr(outer.pb, name))
                    for name in names
                    if hasattr(outer.pb, name)
                }

            def _is_map_delete_request(self, msg_id):
                return (
                    hasattr(outer.pb, "MSG_ID_MAP_DELETE_REQUEST")
                    and int(msg_id) == int(outer.pb.MSG_ID_MAP_DELETE_REQUEST)
                )

            def _log_dropped_job(self, queue_name, dropped):
                if dropped is None:
                    return
                if queue_name == "manual control" and outer._control_overwritten is not None:
                    outer._control_overwritten.inc()
                _, args = dropped
                frame = args[0] if args else None
                if rospy is not None:
                    rospy.logwarn_throttle(
                        1.0,
                        "SL-LinkA %s queue full; dropped stale msg_id=0x%04X seq=%d and kept newest request.",
                        queue_name,
                        int(getattr(frame, "msg_id", 0)),
                        int(getattr(frame, "seq", 0)),
                    )
                if frame is not None and int(frame.msg_id) == int(outer.pb.MSG_ID_MAP_REQUEST):
                    try:
                        request = outer.pb.MapRequest()
                        request.ParseFromString(frame.payload)
                        result = outer.pb.MapRequestResult()
                        result.request_id = int(request.request_id)
                        result.map_id = str(request.map_id or "LIVE_MAP")
                        result.status = outer.pb.MAP_REQUEST_STATUS_NOT_READY
                        result.retry_after_ms = 500
                        result.message = "map request queue was full"
                        self._send_payload(
                            result.SerializeToString(),
                            outer.pb.MSG_ID_MAP_REQUEST_RESULT,
                            comp_id=outer.pb.COMP_MEDIA,
                            ack_seq=frame.seq,
                        )
                    except Exception:
                        pass

            def _dispatch_ordered(self, frame, received_at, queue_name="ordered", control_generation=None):
                if not self._valid():
                    return
                if control_generation is not None and control_generation != outer._handler.capture_control_generation():
                    self._reject(frame, "Request cancelled by stop/emergency", outer.pb.RESULT_FAILED)
                    return
                if not self._allowed(frame):
                    self._reject(frame, "Channel is not ready for this request", outer.pb.RESULT_BUSY)
                    return
                queue_delay_ms = (time.monotonic() - float(received_at)) * 1000.0
                if outer._ordered_queue_wait is not None:
                    outer._ordered_queue_wait.observe(queue_delay_ms)
                if rospy is not None and queue_delay_ms >= 100.0:
                    rospy.logwarn(
                        "SL-LinkA %s request queue delay: msg_id=0x%04X seq=%d delay_ms=%.1f",
                        queue_name,
                        int(frame.msg_id),
                        int(frame.seq),
                        queue_delay_ms,
                    )
                try:
                    outer._dispatch_frame(self, frame, log_rx=False, control_generation=control_generation)
                except Exception as exc:
                    if rospy is not None:
                        rospy.logerr(
                            "SL-LinkA background dispatch failed: msg_id=0x%04X seq=%d err=%s",
                            int(frame.msg_id),
                            int(frame.seq),
                            exc,
                        )

            def _dispatch_control_immediately(
                self,
                frame,
                control_request=None,
                latest_response=False,
                received_at=None,
                control_generation=None,
                manual_ticket=None,
            ):
                if not self._valid():
                    return
                if outer._stop_pending.is_set() and control_request is not None:
                    from grinder_scheduler.manual_delivery import is_manual_stop
                    urgent = (control_request.HasField("emergency_stop") and control_request.emergency_stop.enabled)
                    urgent = urgent or (control_request.HasField("manual_drive")
                                        and is_manual_stop(control_request.manual_drive, outer.pb))
                    if not urgent:
                        self._reject(frame, "Task stop is still being applied", outer.pb.RESULT_BUSY)
                        return
                started = time.monotonic()
                if received_at is not None and outer._control_queue_wait is not None:
                    # 控制等待时间从收到完整帧算到开始执行业务，便于发现排队抖动。
                    outer._control_queue_wait.observe((started - received_at) * 1000.0)
                if manual_ticket is not None and time.monotonic() > manual_ticket[2]:
                    if outer._control_expired is not None:
                        outer._control_expired.inc()
                    return
                payload, msg_id, comp_id = outer._handler.handle_control_command(
                    frame.payload,
                    parsed_request=control_request,
                    control_generation=control_generation,
                    manual_ticket=manual_ticket,
                    connection_valid=lambda: (self._valid() and outer._handler.is_control_session_current(self.session)),
                )
                applied_ms = (time.monotonic() - started) * 1000.0
                if outer._control_apply is not None:
                    outer._control_apply.observe(applied_ms)
                if rospy is not None:
                    rospy.loginfo_throttle(
                        5.0,
                        "SL-LinkA control applied immediately: rx_seq=%d apply_ms=%.1f",
                        int(frame.seq),
                        applied_ms,
                    )
                if latest_response:
                    self._queue_control_response(payload, msg_id, comp_id, int(frame.seq))
                    return
                self._queue_control_response(payload, msg_id, comp_id, int(frame.seq))

            def _queue_control_response(self, payload, msg_id, comp_id, ack_seq):
                if not self.control_response_slots.acquire(blocking=False):
                    if rospy is not None:
                        rospy.logwarn_throttle(
                            2.0,
                            "Drop SL-LinkA control response because TX queue is full.",
                        )
                    return
                self.control_response_executor.submit(
                    self._send_control_response,
                    payload,
                    msg_id,
                    comp_id,
                    ack_seq,
                    True,
                )

            def _send_control_response(self, payload, msg_id, comp_id, ack_seq, release_slot):
                try:
                    self._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=ack_seq, control=True)
                except Exception as exc:
                    if rospy is not None and self.running:
                        rospy.logwarn("SL-LinkA control response send failed: %s", exc)
                finally:
                    if release_slot:
                        self.control_response_slots.release()

            def _send_bulk_response(self, outputs, ack_seq):
                started = time.monotonic()
                frame_count = 0
                payload_bytes = 0
                wire_bytes = 0
                send_ms = 0.0
                try:
                    index = 0
                    # Yield at every complete frame so waiting control ACKs win.
                    while self.running and index < len(outputs):
                        payload, msg_id, comp_id = outputs[index]
                        sent = self._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=ack_seq)
                        frame_count += 1
                        payload_bytes += len(payload or b'')
                        if sent:
                            wire_bytes += sent[2]
                            send_ms += sent[1]
                        index += 1
                    if rospy is not None:
                        rospy.loginfo(
                            "SL-LinkA bulk response sent: ack=%d frames=%d payload_bytes=%d wire_bytes=%d total_ms=%.1f send_ms=%.1f",
                            int(ack_seq),
                            frame_count,
                            payload_bytes,
                            wire_bytes,
                            (time.monotonic() - started) * 1000.0,
                            send_ms,
                        )
                except Exception as exc:
                    if rospy is not None and self.running:
                        rospy.logwarn(
                            "SL-LinkA bulk response send failed: ack=%d sent_frames=%d total_frames=%d err=%s",
                            int(ack_seq),
                            frame_count,
                            len(outputs),
                            exc,
                        )
                finally:
                    self.bulk_response_slots.release()

            def _queue_bulk_response(self, outputs, ack_seq):
                if not outputs:
                    return True
                if not self.bulk_response_slots.acquire(blocking=False):
                    return False
                try:
                    self.bulk_response_executor.submit(
                        self._send_bulk_response,
                        outputs,
                        int(ack_seq),
                    )
                    return True
                except Exception:
                    self.bulk_response_slots.release()
                    raise

            def _send_wire(self, payload):
                if not self.running:
                    raise ConnectionError("SL-LinkA session closed")
                try:
                    self.request.sendall(payload)
                except OSError:
                    self.running = False
                    self._close()
                    raise

            def _send_payload(self, payload, msg_id, comp_id=None, ack_seq=0, control=False):
                if payload is None or msg_id is None:
                    return
                if int(msg_id) in outer._bulk_message_ids:
                    outer.bulk_budget.consume(len(payload) + 32, self._valid)
                total_started = time.monotonic()
                with self.send_lock.hold(control):
                    if control and outer._control_ack_wait is not None:
                        outer._control_ack_wait.observe((time.monotonic() - total_started) * 1000.0)
                    pack_started = time.monotonic()
                    frame = outer.SlFrame(
                        version=outer.SL_PROTOCOL_VERSION,
                        flags=0,
                        seq=outer._next_seq(),
                        ack_seq=ack_seq,
                        src_id=outer.pb.DEVICE_LOWER,
                        dst_id=outer.pb.DEVICE_APP,
                        comp_id=comp_id if comp_id is not None else outer.pb.COMP_SYSTEM,
                        msg_id=msg_id,
                        payload=payload,
                    )
                    packed = frame.pack()
                    pack_elapsed_ms = (time.monotonic() - pack_started) * 1000.0
                    send_started = time.monotonic()
                    self._send_wire(packed)
                    send_elapsed_ms = (time.monotonic() - send_started) * 1000.0
                total_elapsed_ms = (time.monotonic() - total_started) * 1000.0
                if int(msg_id) == int(outer.pb.MSG_ID_MAP_CHUNK):
                    # 帧打包与 sendall 分开计时；sendall 返回只表示数据进入本机 TCP 栈。
                    if outer._map_frame_pack_ms is not None:
                        outer._map_frame_pack_ms.observe(pack_elapsed_ms)
                        outer._map_socket_send_ms.observe(send_elapsed_ms)
                        outer._map_wire_bytes_sent.inc(len(packed))
                if rospy is not None and int(msg_id) == int(outer.pb.MSG_ID_PATH_PLAN_RESPONSE):
                    rospy.loginfo(
                        "SL-LinkA TX PathPlanResponse: seq=%d ack=%d bytes=%d total_ms=%.1f pack_ms=%.1f send_ms=%.1f",
                        int(frame.seq),
                        int(ack_seq),
                        len(packed),
                        total_elapsed_ms,
                        pack_elapsed_ms,
                        send_elapsed_ms,
                    )
                return pack_elapsed_ms, send_elapsed_ms, len(packed)

        class ThreadedServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
            allow_reuse_address = True
            daemon_threads = True
            request_queue_size = 16

        self._bulk_message_ids = {int(getattr(self.pb, "MSG_ID_" + name)) for name in (
            "MAP_CHUNK", "PATH_PLAN_RESPONSE", "PATH_POINT_PLAN_RESPONSE", "TASK_EXECUTION_HISTORY_CHUNK",
            "TASK_TRAJECTORY_CHUNK", "TASK_RESULT_RESPONSE", "CAMERA_FRAME_CHUNK")}
        self._stopping.clear()
        self._stop_pending = threading.Event()
        try:
            for role, port in (("data", self.port), ("control", self.control_port)):
                listener = ThreadedServer((self.host, port), RequestHandler)
                listener.channel_role = role
                self._listeners.append(listener)
            self._server = self._listeners[0]
            self._task_stop_executor = _BoundedLatestWorker(1, "sl_linka_task_stop")
            for listener in self._listeners:
                threading.Thread(target=listener.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True).start()
            self._thread = threading.Thread(target=self._watch_sessions, daemon=True, name="sl_linka_leases")
            self._thread.start()
        except Exception:
            for listener in self._listeners:
                listener.server_close()
            self._listeners.clear()
            self._server = None
            raise
        if rospy is not None:
            rospy.loginfo("SL-LinkA data/control listening on %s:%s/%s", self.host, self.port, self.control_port)

    def stop(self):
        self._stopping.set()
        if hasattr(self, "_task_stop_executor"):
            self._task_stop_executor.shutdown(wait=False)
        with self._peers_lock:
            peers = list(self._peers)
        for peer in peers:
            peer._close()
        for listener in self._listeners:
            listener.shutdown()
            listener.server_close()
        self._listeners.clear()
        self._server = None

    def _watch_sessions(self):
        while not self._stopping.wait(0.05):
            for peer in self.sessions.expire():
                if self._session_expired is not None:
                    self._session_expired.inc()
                peer._close()
            with self._peers_lock:
                peers = list(self._peers)
            now = time.monotonic()
            for peer in peers:
                if ((not peer.admitted and now - peer.created_at >= self.sessions.timeout)
                        or (peer.manual_received_at is not None
                            and now - peer.manual_received_at >= self.manual_timeout_sec)):
                    peer._close()

    def _next_seq(self):
        with self._seq_lock:
            value = self._seq
            self._seq = (self._seq + 1) & 0xFFFF
            return value

    def _call_data_mutation(self, peer, frame, callback, generation):
        """Every positive publish rechecks the owner and receipt-time generation."""
        gate = getattr(self._handler, "_motion", None)
        if gate is None:
            return callback(frame.payload)
        previous = getattr(gate.local, "delivery_validator", None)
        gate.local.delivery_validator = lambda: (
            peer._valid() and (generation is None or generation == gate.generation)
            and (previous is None or previous()))
        try:
            return callback(frame.payload)
        finally:
            gate.local.delivery_validator = previous

    def _dispatch_frame(self, request_handler, frame, log_rx=True, control_generation=None):
        pb = self.pb
        if log_rx:
            self._log_rx_frame(frame)
        if frame.msg_id == pb.MSG_ID_SETTINGS_READ_REQUEST:
            payload, msg_id, comp_id = self._handler.handle_settings_read_request(frame.payload)
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if frame.msg_id == pb.MSG_ID_SETTINGS_WRITE_REQUEST:
            payload, msg_id, comp_id = self._call_data_mutation(
                request_handler, frame, self._handler.handle_settings_write_request, control_generation)
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if frame.msg_id == pb.MSG_ID_CONTROL_COMMAND:
            payload, msg_id, comp_id = self._handler.handle_control_command(frame.payload)
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if frame.msg_id == pb.MSG_ID_TASK_CONFIG:
            payload, msg_id, comp_id = self._handler.handle_task_config(frame.payload)
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if frame.msg_id == pb.MSG_ID_TASK_COMMAND:
            payload, msg_id, comp_id = self._handler.handle_task_command(
                frame.payload, connection_valid=request_handler._valid,
                control_generation=control_generation)
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if frame.msg_id == pb.MSG_ID_TASK_RUNTIME_SETTINGS_REQUEST:
            payload, msg_id, comp_id = self._call_data_mutation(
                request_handler, frame, self._handler.handle_task_runtime_settings, control_generation)
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if frame.msg_id == pb.MSG_ID_PATH_POINT_PLAN_REQUEST:
            chunks = self._handler.build_path_point_plan_chunks(frame.payload)
            if request_handler._queue_bulk_response(chunks, frame.seq):
                if rospy is not None:
                    rospy.loginfo(
                        "SL-LinkA path response queued for bulk TX: ack=%d chunks=%d",
                        int(frame.seq),
                        len(chunks),
                    )
                return
            if rospy is not None:
                rospy.logwarn(
                    "SL-LinkA bulk TX queue full; sending path response synchronously: ack=%d chunks=%d",
                    int(frame.seq),
                    len(chunks),
                )
            for payload, msg_id, comp_id in chunks:
                request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if frame.msg_id == pb.MSG_ID_CAMERA_FRAME_REQUEST:
            chunks = self._handler.build_camera_frame_chunks(frame.payload)
            for payload, msg_id, comp_id in chunks:
                request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if frame.msg_id == pb.MSG_ID_MAP_REQUEST:
            request_started = time.monotonic()
            send_started = None
            socket_send_total_ms = 0.0
            try:
                result_payload, chunks = self._handler.build_map_chunks(frame.payload)
                send_started = time.monotonic()
                request_handler._send_payload(
                    result_payload,
                    pb.MSG_ID_MAP_REQUEST_RESULT,
                    comp_id=pb.COMP_MEDIA,
                    ack_seq=frame.seq,
                )
                for payload, msg_id, comp_id in chunks:
                    sent = request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
                    if sent is not None:
                        socket_send_total_ms += sent[1]
                    if self._map_chunks_sent is not None:
                        # 只有 sendall 成功后才计入已发送分片，避免把构建数量当成传输成功数量。
                        self._map_chunks_sent.inc()
                if rospy is not None:
                    rospy.loginfo_throttle(
                        5.0,
                        "SL-LinkA map transfer: chunks=%d build_ms=%.1f send_ms=%.1f socket_ms=%.1f total_ms=%.1f",
                        len(chunks),
                        (send_started - request_started) * 1000.0,
                        (time.monotonic() - send_started) * 1000.0,
                        socket_send_total_ms,
                        (time.monotonic() - request_started) * 1000.0,
                    )
            except Exception:
                if send_started is None:
                    if self._map_build_failures is not None:
                        self._map_build_failures.inc()
                    try:
                        request = pb.MapRequest()
                        request.ParseFromString(frame.payload)
                        result = pb.MapRequestResult()
                        result.request_id = int(request.request_id)
                        result.map_id = str(request.map_id or "LIVE_MAP")
                        result.status = pb.MAP_REQUEST_STATUS_ERROR
                        result.message = "map preparation failed"
                        request_handler._send_payload(
                            result.SerializeToString(),
                            pb.MSG_ID_MAP_REQUEST_RESULT,
                            comp_id=pb.COMP_MEDIA,
                            ack_seq=frame.seq,
                        )
                    except Exception:
                        pass
                elif send_started is not None and self._map_send_failures is not None:
                    self._map_send_failures.inc()
                raise
            finally:
                finished = time.monotonic()
                if send_started is not None and self._map_send_total_ms is not None:
                    # 整张地图发送阶段包含打包、锁等待和 socket 阻塞时间。
                    self._map_send_total_ms.observe((finished - send_started) * 1000.0)
                    self._map_socket_send_total_ms.observe(socket_send_total_ms)
                if self._map_request_total_ms is not None:
                    self._map_request_total_ms.observe((finished - request_started) * 1000.0)
            return
        if frame.msg_id == pb.MSG_ID_MAP_PREVIEW_REQUEST:
            payload, msg_id, comp_id = self._handler.handle_map_preview_request(frame.payload)
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if (
            hasattr(pb, "MSG_ID_MAP_REGION_POINT_REQUEST")
            and frame.msg_id == pb.MSG_ID_MAP_REGION_POINT_REQUEST
        ):
            payload, msg_id, comp_id = self._handler.handle_map_region_point_request(
                frame.payload
            )
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if frame.msg_id == pb.MSG_ID_MAP_EDIT_COMMAND:
            if rospy is not None:
                rospy.loginfo(
                    "SL-LinkA RX MapEditCommand: seq=%d ack=%d src=%d dst=%d payload_len=%d",
                    int(frame.seq),
                    int(frame.ack_seq),
                    int(frame.src_id),
                    int(frame.dst_id),
                    len(frame.payload or b""),
                )
            responses = self._handler.handle_map_edit_command(frame.payload)
            if isinstance(responses, tuple):
                responses = [responses]
            for payload, msg_id, comp_id in responses:
                request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if frame.msg_id == pb.MSG_ID_VIDEO_STREAM_INFO_REQUEST:
            payload, msg_id, comp_id = self._handler.handle_video_stream_request(frame.payload)
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if hasattr(pb, "MSG_ID_MAP_SYNC_REQUEST") and frame.msg_id == pb.MSG_ID_MAP_SYNC_REQUEST:
            payload, msg_id, comp_id = self._handler.handle_map_sync_request(frame.payload)
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if hasattr(pb, "MSG_ID_MAP_IMPORT_TO_RADAR_REQUEST") and frame.msg_id == pb.MSG_ID_MAP_IMPORT_TO_RADAR_REQUEST:
            payload, msg_id, comp_id = self._handler.handle_map_import_to_radar_request(frame.payload)
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if hasattr(pb, "MSG_ID_MAP_MODE_REQUEST") and frame.msg_id == pb.MSG_ID_MAP_MODE_REQUEST:
            payload, msg_id, comp_id = self._handler.handle_map_mode_request(frame.payload)
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if hasattr(pb, "MSG_ID_MAP_ALIGNMENT_REQUEST") and frame.msg_id == pb.MSG_ID_MAP_ALIGNMENT_REQUEST:
            payload, msg_id, comp_id = self._handler.handle_map_alignment_request(frame.payload)
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if hasattr(pb, "MSG_ID_MAP_CATALOG_REQUEST") and frame.msg_id == pb.MSG_ID_MAP_CATALOG_REQUEST:
            payload, msg_id, comp_id = self._handler.handle_map_catalog_request(frame.payload)
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if hasattr(pb, "MSG_ID_MAP_DELETE_REQUEST") and frame.msg_id == pb.MSG_ID_MAP_DELETE_REQUEST:
            payload, msg_id, comp_id = self._handler.handle_map_delete_request(frame.payload)
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if hasattr(pb, "MSG_ID_MAP_SAVE_REQUEST") and frame.msg_id == pb.MSG_ID_MAP_SAVE_REQUEST:
            payload, msg_id, comp_id = self._handler.handle_map_save_request(frame.payload)
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if hasattr(pb, "MSG_ID_MAP_METRICS_REQUEST") and frame.msg_id == pb.MSG_ID_MAP_METRICS_REQUEST:
            payload, msg_id, comp_id = self._handler.handle_map_metrics_request(frame.payload)
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if hasattr(pb, "MSG_ID_TASK_RESULT_REQUEST") and frame.msg_id == pb.MSG_ID_TASK_RESULT_REQUEST:
            payload, msg_id, comp_id = self._handler.handle_task_result_request(frame.payload)
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if (
            hasattr(pb, "MSG_ID_TASK_EXECUTION_HISTORY_REQUEST")
            and frame.msg_id == pb.MSG_ID_TASK_EXECUTION_HISTORY_REQUEST
        ):
            chunks = self._handler.build_task_execution_history_chunks(frame.payload)
            for payload, msg_id, comp_id in chunks:
                request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if (
            hasattr(pb, "MSG_ID_TASK_TRAJECTORY_REQUEST")
            and frame.msg_id == pb.MSG_ID_TASK_TRAJECTORY_REQUEST
        ):
            chunks = self._handler.build_task_trajectory_chunks(frame.payload)
            for payload, msg_id, comp_id in chunks:
                request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if (
            hasattr(pb, "MSG_ID_TASK_EXECUTION_DELETE_REQUEST")
            and frame.msg_id == pb.MSG_ID_TASK_EXECUTION_DELETE_REQUEST
        ):
            payload, msg_id, comp_id = self._handler.handle_task_execution_delete_request(
                frame.payload
            )
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if (
            hasattr(pb, "MSG_ID_SYSTEM_CACHE_CLEAR_REQUEST")
            and frame.msg_id == pb.MSG_ID_SYSTEM_CACHE_CLEAR_REQUEST
        ):
            payload, msg_id, comp_id = self._handler.handle_system_cache_clear_request(
                frame.payload
            )
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if hasattr(pb, "MSG_ID_LIVE_MAP_CACHE_CLEAR_REQUEST") and frame.msg_id == pb.MSG_ID_LIVE_MAP_CACHE_CLEAR_REQUEST:
            payload, msg_id, comp_id = self._handler.handle_live_map_cache_clear_request(frame.payload)
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if hasattr(pb, "MSG_ID_RADAR_MAP_CACHE_CLEAR_REQUEST") and frame.msg_id == pb.MSG_ID_RADAR_MAP_CACHE_CLEAR_REQUEST:
            payload, msg_id, comp_id = self._handler.handle_radar_map_cache_clear_request(frame.payload)
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if hasattr(pb, "MSG_ID_RADAR_SYSTEM_STATUS_REQUEST") and frame.msg_id == pb.MSG_ID_RADAR_SYSTEM_STATUS_REQUEST:
            payload, msg_id, comp_id = self._handler.handle_radar_system_status_request(frame.payload)
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if hasattr(pb, "MSG_ID_RADAR_MAP_SYNC_REQUEST") and frame.msg_id == pb.MSG_ID_RADAR_MAP_SYNC_REQUEST:
            payload, msg_id, comp_id = self._handler.handle_radar_map_sync_request(frame.payload)
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if hasattr(pb, "MSG_ID_RADAR_RELOCALIZATION_REQUEST") and frame.msg_id == pb.MSG_ID_RADAR_RELOCALIZATION_REQUEST:
            payload, msg_id, comp_id = self._handler.handle_radar_relocalization_request(frame.payload)
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if (
            hasattr(pb, "MSG_ID_RADAR_RELOCALIZATION_STATUS_REQUEST")
            and frame.msg_id == pb.MSG_ID_RADAR_RELOCALIZATION_STATUS_REQUEST
        ):
            payload, msg_id, comp_id = self._handler.handle_radar_relocalization_status_request(
                frame.payload
            )
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if hasattr(pb, "MSG_ID_PATH_PLAN_REQUEST") and frame.msg_id == pb.MSG_ID_PATH_PLAN_REQUEST:
            path_plan_started = time.monotonic()
            responses = self._handler.handle_path_plan_request(frame.payload)
            path_plan_elapsed_ms = (time.monotonic() - path_plan_started) * 1000.0
            if isinstance(responses, tuple):
                responses = [responses]
            if rospy is not None:
                total_payload_bytes = int(sum(len(payload or b"") for payload, _, _ in responses))
                rospy.loginfo(
                    "SL-LinkA PathPlanRequest ready-to-send: rx_seq=%d responses=%d payload_bytes=%d handle_ms=%.1f",
                    int(frame.seq),
                    len(responses),
                    total_payload_bytes,
                    path_plan_elapsed_ms,
                )
            for payload, msg_id, comp_id in responses:
                request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if rospy is not None:
            rospy.logwarn(
                "SL-LinkA RX unhandled msg_id=0x%04X seq=%d src=%d dst=%d payload_len=%d",
                int(frame.msg_id),
                int(frame.seq),
                int(frame.src_id),
                int(frame.dst_id),
                len(frame.payload or b""),
            )
