import socket
import socketserver
import threading
import time
from collections import deque
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
    def __init__(self, sdk_dir, host, port, callback_handler):
        ensure_sl_linka_sdk_on_path(sdk_dir)
        from sl_link.frame import SL_PROTOCOL_VERSION, SlFrame, SlFrameParser
        from sl_link.message_gen import sl_link_pb2 as pb

        self.SL_PROTOCOL_VERSION = SL_PROTOCOL_VERSION
        self.SlFrame = SlFrame
        self.SlFrameParser = SlFrameParser
        self.pb = pb
        self.host = host
        self.port = port
        self._handler = callback_handler
        self._server = None
        self._thread = None
        self._seq = 0
        self._seq_lock = threading.Lock()

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
        if rospy is None:
            return
        rospy.loginfo(
            "SL-LinkA RX frame: version=%d flags=0x%02X seq=%d ack=%d src=%d dst=%d "
            "comp=%d msg_id=0x%04X name=%s payload_len=%d",
            int(frame.version),
            int(frame.flags),
            int(frame.seq),
            int(frame.ack_seq),
            int(frame.src_id),
            int(frame.dst_id),
            int(frame.comp_id),
            int(frame.msg_id),
            self._msg_id_name(frame.msg_id),
            len(frame.payload or b""),
        )

    def start(self):
        if rospy is not None:
            rospy.loginfo("SL-LinkA header-only RX frame logging enabled")
        outer = self

        class RequestHandler(socketserver.BaseRequestHandler):
            def setup(self):
                self.parser = outer.SlFrameParser()
                self.running = True
                self.send_lock = threading.Lock()
                try:
                    self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    self.request.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024 * 1024)
                except OSError as exc:
                    if rospy is not None:
                        rospy.logwarn("SL-LinkA TCP send tuning failed: %s", exc)
                # Keep command queues bounded. When producers outrun consumers,
                # stale pending requests are discarded in favor of newer input.
                self.dispatch_executor = _BoundedLatestWorker(
                    max_pending=32,
                    name="sl_linka_ordered",
                )
                self.map_dispatch_executor = _BoundedLatestWorker(
                    max_pending=4,
                    name="sl_linka_map",
                )
                self.manual_control_executor = _DropIfBusyWorker(
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
                self.status_thread = threading.Thread(target=self._periodic_reports, daemon=True)
                self.status_thread.start()

            def _periodic_reports(self):
                while self.running:
                    try:
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
                    for frame in self.parser.parse(data):
                        if frame.dst_id not in (outer.pb.DEVICE_LOWER, outer.pb.DEVICE_BROADCAST):
                            continue
                        received_at = time.monotonic()
                        outer._log_rx_frame(frame)
                        try:
                            if int(frame.msg_id) == int(outer.pb.MSG_ID_CONTROL_COMMAND):
                                control_request = self._parse_control_command(frame)
                                if (
                                    control_request is not None
                                    and control_request.HasField("manual_drive")
                                ):
                                    accepted = self.manual_control_executor.submit(
                                        self._dispatch_control_immediately,
                                        frame,
                                        control_request,
                                        True,
                                    )
                                    if not accepted and rospy is not None:
                                        rospy.logwarn_throttle(
                                            1.0,
                                            "Drop incoming SL-LinkA manual control while previous command is running: seq=%d",
                                            int(frame.seq),
                                        )
                                else:
                                    self._dispatch_control_immediately(
                                        frame,
                                        control_request,
                                        False,
                                    )
                            elif self._is_map_background_request(frame.msg_id):
                                dropped = self.map_dispatch_executor.submit(
                                    self._dispatch_ordered,
                                    frame,
                                    received_at,
                                    "map",
                                )
                                self._log_dropped_job("map request", dropped)
                            else:
                                dropped = self.dispatch_executor.submit(
                                    self._dispatch_ordered,
                                    frame,
                                    received_at,
                                    "ordered",
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
                self.running = False
                self.dispatch_executor.shutdown(wait=False)
                self.map_dispatch_executor.shutdown(wait=False)
                self.manual_control_executor.shutdown(wait=False)
                self.manual_response_executor.shutdown(wait=False)
                self.control_response_executor.shutdown(wait=False)
                self.bulk_response_executor.shutdown(wait=False)

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

            def _log_dropped_job(self, queue_name, dropped):
                if dropped is None or rospy is None:
                    return
                _, args = dropped
                frame = args[0] if args else None
                rospy.logwarn_throttle(
                    1.0,
                    "SL-LinkA %s queue full; dropped stale msg_id=0x%04X seq=%d and kept newest request.",
                    queue_name,
                    int(getattr(frame, "msg_id", 0)),
                    int(getattr(frame, "seq", 0)),
                )

            def _dispatch_ordered(self, frame, received_at, queue_name="ordered"):
                queue_delay_ms = (time.monotonic() - float(received_at)) * 1000.0
                if rospy is not None and queue_delay_ms >= 100.0:
                    rospy.logwarn(
                        "SL-LinkA %s request queue delay: msg_id=0x%04X seq=%d delay_ms=%.1f",
                        queue_name,
                        int(frame.msg_id),
                        int(frame.seq),
                        queue_delay_ms,
                    )
                try:
                    outer._dispatch_frame(self, frame, log_rx=False)
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
            ):
                started = time.monotonic()
                payload, msg_id, comp_id = outer._handler.handle_control_command(
                    frame.payload,
                    parsed_request=control_request,
                )
                applied_ms = (time.monotonic() - started) * 1000.0
                if rospy is not None:
                    rospy.loginfo(
                        "SL-LinkA control applied immediately: rx_seq=%d apply_ms=%.1f",
                        int(frame.seq),
                        applied_ms,
                    )
                if latest_response:
                    accepted = self.manual_response_executor.submit(
                        self._send_control_response,
                        payload,
                        msg_id,
                        comp_id,
                        int(frame.seq),
                        False,
                    )
                    if not accepted and rospy is not None:
                        rospy.logwarn_throttle(
                            1.0,
                            "Drop incoming SL-LinkA manual response while previous response is sending: ack=%d",
                            int(frame.seq),
                        )
                    return
                if not self.control_response_slots.acquire(blocking=False):
                    if rospy is not None:
                        rospy.logwarn_throttle(
                            2.0,
                            "Drop SL-LinkA control response because TX queue is full; control was already applied.",
                        )
                    return
                self.control_response_executor.submit(
                    self._send_control_response,
                    payload,
                    msg_id,
                    comp_id,
                    int(frame.seq),
                    True,
                )

            def _send_control_response(self, payload, msg_id, comp_id, ack_seq, release_slot):
                try:
                    self._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=ack_seq)
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
                    # Coalesce adjacent protocol frames into about 128 KiB TCP
                    # writes. Frames remain individually parseable by SL-LinkA.
                    while self.running and index < len(outputs):
                        batch = bytearray()
                        with self.send_lock:
                            while index < len(outputs) and (not batch or len(batch) < 128 * 1024):
                                payload, msg_id, comp_id = outputs[index]
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
                                batch.extend(packed)
                                frame_count += 1
                                payload_bytes += len(payload or b"")
                                wire_bytes += len(packed)
                                index += 1
                            send_started = time.monotonic()
                            self.request.sendall(batch)
                            send_ms += (time.monotonic() - send_started) * 1000.0
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

            def _send_payload(self, payload, msg_id, comp_id=None, ack_seq=0):
                if payload is None or msg_id is None:
                    return
                total_started = time.monotonic()
                with self.send_lock:
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
                    self.request.sendall(packed)
                    send_elapsed_ms = (time.monotonic() - send_started) * 1000.0
                total_elapsed_ms = (time.monotonic() - total_started) * 1000.0
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

        class ThreadedServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
            allow_reuse_address = True
            daemon_threads = True
            request_queue_size = 16

        self._server = ThreadedServer((self.host, self.port), RequestHandler)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        if rospy is not None:
            rospy.loginfo("SL-LinkA listening on %s:%s", self.host, self.port)

    def stop(self):
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None

    def _next_seq(self):
        with self._seq_lock:
            value = self._seq
            self._seq = (self._seq + 1) & 0xFFFF
            return value

    def _dispatch_frame(self, request_handler, frame, log_rx=True):
        pb = self.pb
        if log_rx:
            self._log_rx_frame(frame)
        if frame.msg_id == pb.MSG_ID_SETTINGS_READ_REQUEST:
            payload, msg_id, comp_id = self._handler.handle_settings_read_request(frame.payload)
            request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
            return
        if frame.msg_id == pb.MSG_ID_SETTINGS_WRITE_REQUEST:
            payload, msg_id, comp_id = self._handler.handle_settings_write_request(frame.payload)
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
            payload, msg_id, comp_id = self._handler.handle_task_command(frame.payload)
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
            chunks = self._handler.build_map_chunks(frame.payload)
            for payload, msg_id, comp_id in chunks:
                request_handler._send_payload(payload, msg_id, comp_id=comp_id, ack_seq=frame.seq)
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
