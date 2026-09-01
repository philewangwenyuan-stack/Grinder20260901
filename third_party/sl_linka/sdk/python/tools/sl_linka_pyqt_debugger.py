#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import base64
import datetime
import json
import os
import shutil
import socket
import subprocess
import sys
import threading
import time

try:
    import cv2
    import numpy as np
except Exception:
    cv2 = None
    np = None

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
# Prefer software rendering on embedded Linux boards (e.g. RK3588)
# to avoid EGL/GLX crashes in mixed graphics stacks.
os.environ.setdefault("QT_OPENGL", "software")
os.environ.setdefault("QT_QUICK_BACKEND", "software")
os.environ.setdefault("QT_XCB_GL_INTEGRATION", "none")
# Reduce RTSP end-to-end latency when OpenCV uses FFmpeg backend.
os.environ.setdefault(
    "OPENCV_FFMPEG_CAPTURE_OPTIONS",
    "rtsp_transport;udp|fflags;nobuffer|flags;low_delay|max_delay;0|probesize;32768|analyzeduration;0|buffer_size;102400",
)

SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
SDK_PY_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "../../../../../"))
if SDK_PY_ROOT not in sys.path:
    sys.path.insert(0, SDK_PY_ROOT)

from sl_link.frame.frame import SL_PROTOCOL_VERSION, SlFrame, SlFrameParser
from sl_link.message_gen import sl_link_pb2 as pb

try:
    from PyQt5 import QtCore, QtGui, QtWidgets
except Exception as exc:  # pragma: no cover - runtime dependency check
    print("PyQt5 is required. Install with: sudo apt-get install -y python3-pyqt5")
    print(f"Import error: {exc}")
    sys.exit(1)


MSG_NAMES = {}
for _name in dir(pb):
    if _name.startswith("MSG_ID_"):
        MSG_NAMES[getattr(pb, _name)] = _name

TASK_STATE_NAMES = {}
for _name in dir(pb):
    if _name.startswith("TASK_STATE_"):
        TASK_STATE_NAMES[getattr(pb, _name)] = _name

DEFAULT_REQUEST_MAP_ID = "LIVE_MAP"


class WheelComboBox(QtWidgets.QComboBox):
    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta == 0:
            event.ignore()
            return
        step = -1 if delta > 0 else 1
        next_index = max(0, min(self.count() - 1, self.currentIndex() + step))
        if next_index != self.currentIndex():
            self.setCurrentIndex(next_index)
        event.accept()


class SlLinkClient(QtCore.QObject):
    frame_received = QtCore.pyqtSignal(object)
    state_changed = QtCore.pyqtSignal(bool, str)
    log = QtCore.pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._sock = None
        self._reader = None
        self._running = False
        self._lock = threading.Lock()
        self._parser = SlFrameParser()
        self._seq = 1

    def connect_to(self, host: str, port: int, timeout_s: float = 3.0):
        with self._lock:
            if self._running:
                return
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout_s)
            sock.connect((host, port))
            sock.settimeout(1.0)
            self._sock = sock
            self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self.state_changed.emit(True, f"Connected to {host}:{port}")

    def disconnect(self):
        with self._lock:
            self._running = False
            sock = self._sock
            self._sock = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                sock.close()
            except Exception:
                pass
        self.state_changed.emit(False, "Disconnected")

    def send(self, msg_id: int, comp_id: int, payload: bytes):
        with self._lock:
            if not self._running or self._sock is None:
                raise RuntimeError("Not connected")
            seq = self._seq
            self._seq = (self._seq + 1) & 0xFFFF
            frame = SlFrame(
                version=SL_PROTOCOL_VERSION,
                flags=0,
                seq=seq,
                ack_seq=0,
                src_id=pb.DEVICE_APP,
                dst_id=pb.DEVICE_LOWER,
                comp_id=comp_id,
                msg_id=msg_id,
                payload=payload,
            )
            self._sock.sendall(frame.pack())
            self.log.emit(f"TX seq={seq} {MSG_NAMES.get(msg_id, hex(msg_id))} len={len(payload)}")

    def _read_loop(self):
        while True:
            with self._lock:
                if not self._running or self._sock is None:
                    break
                sock = self._sock
            try:
                data = sock.recv(4096)
                if not data:
                    self.log.emit("Socket closed by peer")
                    break
                recv_mono = time.monotonic()
                recv_wall = time.time()
                for frame in self._parser.parse(data):
                    try:
                        frame._recv_monotonic = recv_mono
                        frame._recv_wall = recv_wall
                    except Exception:
                        pass
                    # Raw receive timestamp from socket reader thread.
                    # This helps distinguish wire delay from main-thread/UI delay.
                    print(
                        "[SL-LinkA RAW RX] "
                        f"{datetime.datetime.fromtimestamp(recv_wall).strftime('%H:%M:%S.%f')[:-3]} "
                        f"seq={frame.seq} {MSG_NAMES.get(frame.msg_id, hex(frame.msg_id))} "
                        f"len={len(frame.payload)}",
                        flush=True,
                    )
                    self.frame_received.emit(frame)
            except socket.timeout:
                continue
            except Exception as exc:
                self.log.emit(f"RX error: {exc}")
                break
        self.disconnect()


class RemoteJoystickWidget(QtWidgets.QWidget):
    value_changed = QtCore.pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(180, 180)
        self._x = 0.0
        self._y = 0.0
        self._pressed = False

    @property
    def x_value(self):
        return self._x

    @property
    def y_value(self):
        return self._y

    def _update_from_pos(self, pos):
        rect = self.rect()
        cx = rect.center().x()
        cy = rect.center().y()
        radius = max(1.0, min(rect.width(), rect.height()) * 0.42)
        dx = float(pos.x() - cx)
        dy = float(pos.y() - cy)
        norm = (dx * dx + dy * dy) ** 0.5
        if norm > radius:
            scale = radius / norm
            dx *= scale
            dy *= scale
        # 调试上位机摇杆约定：上 (0, -1), 下 (0, +1), 左 (-1, 0), 右 (1, 0)
        self._x = max(-1.0, min(1.0, dx / radius))
        self._y = max(-1.0, min(1.0, dy / radius))
        self.value_changed.emit(self._x, self._y)
        self.update()

    def _reset(self):
        self._x = 0.0
        self._y = 0.0
        self.value_changed.emit(self._x, self._y)
        self.update()

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self._pressed = True
            self._update_from_pos(event.pos())
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._pressed:
            self._update_from_pos(event.pos())
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self._pressed = False
            self._reset()
        super().mouseReleaseEvent(event)

    def paintEvent(self, event):
        super().paintEvent(event)
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing, True)
        rect = self.rect()
        cx = rect.center().x()
        cy = rect.center().y()
        radius = min(rect.width(), rect.height()) * 0.42
        p.setPen(QtGui.QPen(QtGui.QColor("#555"), 2))
        p.setBrush(QtGui.QColor("#1e1e1e"))
        p.drawEllipse(QtCore.QPointF(cx, cy), radius, radius)
        p.setPen(QtGui.QPen(QtGui.QColor("#2d6cdf"), 2))
        p.drawLine(int(cx - radius), int(cy), int(cx + radius), int(cy))
        p.drawLine(int(cx), int(cy - radius), int(cx), int(cy + radius))
        knob_x = cx + self._x * radius
        knob_y = cy + self._y * radius
        p.setPen(QtGui.QPen(QtGui.QColor("#8fb5ff"), 2))
        p.setBrush(QtGui.QColor("#2d6cdf"))
        p.drawEllipse(QtCore.QPointF(knob_x, knob_y), radius * 0.18, radius * 0.18)


class RtspPreviewWorker(QtCore.QObject):
    frame_ready = QtCore.pyqtSignal(str, object)
    log = QtCore.pyqtSignal(str)

    def __init__(self, stream_name: str, url: str, parent=None):
        super().__init__(parent)
        self._stream_name = str(stream_name)
        self._url = str(url)
        self._thread = None
        self._running = False
        self._lock = threading.Lock()
        self._ffmpeg = shutil.which("ffmpeg")
        self._pipe_width = 640
        self._pipe_height = 480

    def start(self):
        with self._lock:
            if self._running:
                return
            self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        with self._lock:
            self._running = False
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.2)
        self._thread = None

    def _is_running(self):
        with self._lock:
            return self._running

    def _run(self):
        if cv2 is None:
            self.log.emit(f"RTSP预览不可用: 未安装 opencv-python ({self._stream_name})")
            return
        # Prefer ffmpeg low-latency decode pipe; fallback to OpenCV backend.
        if np is not None and self._ffmpeg:
            ok = self._run_ffmpeg_pipe()
            if ok or (not self._is_running()):
                return
            self.log.emit(f"RTSP(ffmpeg)回退到OpenCV({self._stream_name})")
            time.sleep(0.1)
        self._run_opencv()

    def _run_ffmpeg_pipe(self):
        frame_bytes = int(self._pipe_width * self._pipe_height * 3)
        if frame_bytes <= 0:
            return False
        last_fail_log = 0.0
        emit_interval = 1.0 / 12.0
        last_emit_time = 0.0
        proc = None
        try:
            while self._is_running():
                if proc is None:
                    cmd = [
                        self._ffmpeg,
                        "-loglevel",
                        "error",
                        "-rtsp_transport",
                        "udp",
                        "-fflags",
                        "nobuffer",
                        "-flags",
                        "low_delay",
                        "-probesize",
                        "32",
                        "-analyzeduration",
                        "0",
                        "-max_delay",
                        "0",
                        "-i",
                        self._url,
                        "-an",
                        "-sn",
                        "-dn",
                        "-vf",
                        f"scale={self._pipe_width}:{self._pipe_height}",
                        "-pix_fmt",
                        "bgr24",
                        "-f",
                        "rawvideo",
                        "-",
                    ]
                    proc = subprocess.Popen(
                        cmd,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.DEVNULL,
                        bufsize=0,
                    )
                if proc.stdout is None:
                    return False
                buf = bytearray()
                while self._is_running() and len(buf) < frame_bytes:
                    chunk = proc.stdout.read(frame_bytes - len(buf))
                    if not chunk:
                        break
                    buf.extend(chunk)
                if not self._is_running():
                    return True
                if len(buf) != frame_bytes:
                    now = time.time()
                    if now - last_fail_log > 2.0:
                        self.log.emit(f"RTSP(ffmpeg)读取失败({self._stream_name})，重连中...")
                        last_fail_log = now
                    try:
                        proc.terminate()
                    except Exception:
                        pass
                    proc = None
                    time.sleep(0.2)
                    continue
                frame = np.frombuffer(buf, dtype=np.uint8).reshape((self._pipe_height, self._pipe_width, 3))
                now = time.time()
                if now - last_emit_time >= emit_interval:
                    self.frame_ready.emit(self._stream_name, frame.copy())
                    last_emit_time = now
            return True
        except Exception as exc:
            self.log.emit(f"RTSP(ffmpeg)异常({self._stream_name}): {exc}")
            return False
        finally:
            if proc is not None:
                try:
                    proc.terminate()
                except Exception:
                    pass

    def _run_opencv(self):
        cap = None
        last_fail_log = 0.0
        last_emit_time = 0.0
        emit_interval = 1.0 / 12.0  # Keep UI queue light to avoid lag accumulation.
        while self._is_running():
            if cap is None:
                try:
                    cap = cv2.VideoCapture(self._url, cv2.CAP_FFMPEG)
                except Exception:
                    cap = cv2.VideoCapture(self._url)
                if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
                    try:
                        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    except Exception:
                        pass
                if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
                    try:
                        cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 2000)
                    except Exception:
                        pass
                if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
                    try:
                        cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 800)
                    except Exception:
                        pass
                if not cap.isOpened():
                    now = time.time()
                    if now - last_fail_log > 2.0:
                        self.log.emit(f"RTSP连接失败({self._stream_name}): {self._url}")
                        last_fail_log = now
                    try:
                        cap.release()
                    except Exception:
                        pass
                    cap = None
                    time.sleep(0.5)
                    continue

            ok, frame = cap.read()
            if not ok or frame is None:
                now = time.time()
                if now - last_fail_log > 2.0:
                    self.log.emit(f"RTSP读取失败({self._stream_name})，正在重连...")
                    last_fail_log = now
                try:
                    cap.release()
                except Exception:
                    pass
                cap = None
                time.sleep(0.2)
                continue

            now = time.time()
            if now - last_emit_time >= emit_interval:
                self.frame_ready.emit(self._stream_name, frame)
                last_emit_time = now
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass


class MainWindow(QtWidgets.QMainWindow):
    overlay_parsed = QtCore.pyqtSignal(int, object, object)
    async_log = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("SL-LinkA PyQt Debugger")
        self.setMinimumSize(980, 680)
        self._apply_adaptive_window_size()
        self.client = SlLinkClient()
        self.client.frame_received.connect(self._on_frame)
        self.client.state_changed.connect(self._on_state_changed)
        self.client.log.connect(self._append_log)
        self._remote_x = 0.0
        self._remote_y = 0.0
        self._remote_send_enabled = False
        self._remote_sent_nonzero = False
        self._map_chunk_state = {}
        self._preview_map_meta = None
        self._preview_map_base_pixmap = None
        self._raw_map_base_pixmap = None
        self._path_preview_base_pixmap = None
        self._task_result_base_pixmap = None
        self._task_region_table_updating = False
        self._task_region_pick_order = []
        self._edit_selected_world_points = []
        self._preview_pick_mode = None  # None / "start_pose" / "end_pose"
        self._preview_overlay_payload = {}
        self._preview_overlay_mask_bytes = b""
        self._preview_overlay_parse_token = 0
        self._preview_render_token = 0
        self._map_preview_autosave = False
        self.overlay_parsed.connect(self._on_overlay_parsed)
        self.async_log.connect(self._append_log)
        self._last_preview_origin_x = 0.0
        self._last_preview_origin_y = 0.0
        self._last_preview_resolution = 0.05
        self._last_preview_width = 0
        self._last_preview_height = 0
        self._last_task_status_ts = ""
        self._rtsp_workers = {}
        self._rtsp_latest_frames = {"left": None, "right": None}
        self._rtsp_render_timer = QtCore.QTimer(self)
        self._rtsp_render_timer.setInterval(66)  # ~15fps render cadence
        self._rtsp_render_timer.timeout.connect(self._render_rtsp_latest_frames)
        self._left_cam_pixmap = None
        self._right_cam_pixmap = None
        self._build_ui()

    def _apply_adaptive_window_size(self):
        screen = QtWidgets.QApplication.primaryScreen()
        if screen is None:
            self.resize(1420, 920)
            return
        area = screen.availableGeometry()
        width = max(1280, min(2400, int(area.width() * 0.94)))
        height = max(760, min(1500, int(area.height() * 0.90)))
        self.resize(width, height)

    def _build_ui(self):
        root = QtWidgets.QWidget(self)
        self.setCentralWidget(root)
        layout = QtWidgets.QVBoxLayout(root)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        top_box = QtWidgets.QGroupBox("连接")
        top = QtWidgets.QHBoxLayout(top_box)
        top.setContentsMargins(8, 8, 8, 8)
        top.setSpacing(8)
        self.host_edit = QtWidgets.QLineEdit("127.0.0.1")
        self.host_edit.setMaximumWidth(180)
        self.port_edit = QtWidgets.QLineEdit("8002")
        self.port_edit.setMaximumWidth(100)
        self.status_label = QtWidgets.QLabel("Disconnected")
        self.robot_pose_label = QtWidgets.QLabel("机器人坐标: x=0.000  y=0.000  heading=0.0")
        self.robot_pose_label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.disconnect_btn = QtWidgets.QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)
        top.addWidget(QtWidgets.QLabel("Host"))
        top.addWidget(self.host_edit)
        top.addWidget(QtWidgets.QLabel("Port"))
        top.addWidget(self.port_edit)
        top.addWidget(self.connect_btn)
        top.addWidget(self.disconnect_btn)
        top.addWidget(self.status_label, 1)
        top.addWidget(self.robot_pose_label)
        layout.addWidget(top_box)

        # Main workspace: left controls/logs, right map previews.
        main_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        layout.addWidget(main_splitter, 1)
        control_panel = QtWidgets.QWidget()
        control_panel.setMinimumWidth(620)
        control_panel_layout = QtWidgets.QVBoxLayout(control_panel)
        control_panel_layout.setContentsMargins(0, 0, 0, 0)
        control_panel_layout.setSpacing(8)
        control_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        control_panel_layout.addWidget(control_splitter, 1)

        self.map_max_edge = QtWidgets.QSpinBox()
        self.map_max_edge.setRange(128, 640)
        self.map_max_edge.setValue(640)
        self.map_format = QtWidgets.QComboBox()
        self.map_format.addItems(["jpg", "png"])
        self.map_format.setCurrentText("jpg")
        self.map_overlay = QtWidgets.QCheckBox("include_overlay")
        self.map_overlay.setChecked(True)
        self.map_fast_mode = QtWidgets.QCheckBox("极速模式(仅原图)")
        # Keep overlay enabled by default so map edit regions are visible immediately.
        self.map_fast_mode.setChecked(False)
        self.btn_map_preview = QtWidgets.QPushButton("MapPreviewRequest")
        self.btn_map_request = QtWidgets.QPushButton("MapRequest(原始)")
        self.btn_path_plan = QtWidgets.QPushButton("PathPlanRequest")
        self.btn_task_result = QtWidgets.QPushButton("任务结果获取")
        self.btn_pick_start_pose = QtWidgets.QPushButton("选起点")
        self.btn_pick_end_pose = QtWidgets.QPushButton("选终点")
        self.btn_map_sync_get = QtWidgets.QPushButton("同步下载(雷达->本地)")
        self.btn_map_sync_set = QtWidgets.QPushButton("同步上传(本地->雷达)")
        self.btn_map_import_to_radar = QtWidgets.QPushButton("导入地图到雷达(mapId)")
        self.btn_map_save_local = QtWidgets.QPushButton("本地保存地图")
        self.btn_map_query_local = QtWidgets.QPushButton("查询本地地图")
        self.btn_map_delete_local = QtWidgets.QPushButton("删除选中地图")
        self.btn_live_map_cache_clear = QtWidgets.QPushButton("清除LIVE_MAP缓存")
        self.btn_radar_map_cache_clear = QtWidgets.QPushButton("清除雷达地图缓存")
        self.local_map_count_label = QtWidgets.QLabel("地图数量: 0")
        self.local_map_combo = WheelComboBox()
        self.local_map_combo.setMaxVisibleItems(15)
        self.local_map_combo.addItem("请先点 查询本地地图", "")
        self.btn_video_info = QtWidgets.QPushButton("VideoStreamInfoRequest")
        self.btn_task_start = QtWidgets.QPushButton("任务开始")
        self.btn_task_pause = QtWidgets.QPushButton("任务暂停")
        self.btn_task_resume = QtWidgets.QPushButton("任务继续")
        self.btn_task_stop = QtWidgets.QPushButton("任务结束")
        self.btn_task_create = QtWidgets.QPushButton("创建任务(选中区域)")
        self.btn_task_regions_refresh = QtWidgets.QPushButton("刷新区域列表")
        self.task_region_table = QtWidgets.QTableWidget(0, 4)
        self.task_region_table.setHorizontalHeaderLabels(["执行", "区域ID", "遍数", "顺序"])
        self.task_region_table.horizontalHeader().setStretchLastSection(False)
        self.task_region_table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        self.task_region_table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        self.task_region_table.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeToContents)
        self.task_region_table.horizontalHeader().setSectionResizeMode(3, QtWidgets.QHeaderView.ResizeToContents)
        self.task_region_table.verticalHeader().setVisible(False)
        self.task_region_table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.task_region_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.task_region_table.setMinimumHeight(140)
        self.task_region_table.itemChanged.connect(self._on_task_region_item_changed)
        self.btn_edit_upsert_work = QtWidgets.QPushButton("新增 工作区")
        self.btn_edit_upsert_obstacle = QtWidgets.QPushButton("新增 障碍区")
        self.btn_edit_upsert_erase = QtWidgets.QPushButton("新增 擦除区")
        self.btn_edit_upsert_crop = QtWidgets.QPushButton("新增 裁减区(长方形)")
        self.btn_edit_delete = QtWidgets.QPushButton("删除 当前region_id")
        self.btn_edit_delete_all = QtWidgets.QPushButton("一键删除所有区域")
        self.btn_edit_delete_crop = QtWidgets.QPushButton("删除 裁减区")
        self.btn_region_query = QtWidgets.QPushButton("区域查询")
        self.delete_region_combo = QtWidgets.QComboBox()
        self.delete_region_combo.addItem("请先点 区域查询", None)
        self.btn_mode_mapping_on = QtWidgets.QPushButton("雷达建图模式 开")
        self.btn_mode_mapping_off = QtWidgets.QPushButton("雷达建图模式 关")
        self.btn_mode_local_on = QtWidgets.QPushButton("雷达定位模式 开")
        self.btn_mode_local_off = QtWidgets.QPushButton("雷达定位模式 关")
        self.btn_control_up = QtWidgets.QPushButton("遥感 前")
        self.btn_control_down = QtWidgets.QPushButton("遥感 后")
        self.btn_control_left = QtWidgets.QPushButton("遥感 左")
        self.btn_control_right = QtWidgets.QPushButton("遥感 右")
        self.btn_stop = QtWidgets.QPushButton("遥感 停止")
        self.btn_emergency_stop = QtWidgets.QPushButton("紧急停止")
        self.btn_emergency_stop.setStyleSheet(
            "QPushButton { background: #c62828; color: white; font-weight: bold; }"
            "QPushButton:pressed { background: #8e0000; }"
        )
        self.remote_enable = QtWidgets.QCheckBox("启用遥感连续发送")
        self.remote_enable.setChecked(True)
        self.remote_speed = QtWidgets.QDoubleSpinBox()
        self.remote_speed.setRange(0.1, 1.0)
        self.remote_speed.setSingleStep(0.1)
        self.remote_speed.setValue(0.6)
        self.remote_xy = QtWidgets.QLabel("x=0.00  y=0.00")
        self.remote_stick = RemoteJoystickWidget()
        self.btn_chassis_enable = QtWidgets.QPushButton("底盘使能 开")
        self.btn_chassis_disable = QtWidgets.QPushButton("底盘使能 关")
        self.btn_light_on = QtWidgets.QPushButton("照明 开")
        self.btn_light_off = QtWidgets.QPushButton("照明 关")
        self.btn_lift_down = QtWidgets.QPushButton("磨盘 下降")
        self.btn_lift_up = QtWidgets.QPushButton("磨盘 升起")
        self.btn_disc_on = QtWidgets.QPushButton("磨盘 开")
        self.btn_disc_off = QtWidgets.QPushButton("磨盘 关")
        self.btn_read_chassis = QtWidgets.QPushButton("读取底盘设置")
        self.btn_apply_chassis = QtWidgets.QPushButton("应用底盘设置")
        self.disc_speed_spin = QtWidgets.QSpinBox()
        self.disc_speed_spin.setRange(0, 32767)
        self.disc_speed_spin.setValue(1200)
        self.chassis_run_speed = QtWidgets.QDoubleSpinBox()
        self.chassis_run_speed.setRange(0.01, 5.0)
        self.chassis_run_speed.setSingleStep(0.05)
        self.chassis_run_speed.setDecimals(3)
        self.chassis_run_speed.setValue(0.4)
        self.disc_enable_check = QtWidgets.QCheckBox("磨盘使能")
        self.disc_enable_check.setChecked(False)
        self.work_mode_combo = QtWidgets.QComboBox()
        self.work_mode_combo.addItem("AUTO", int(pb.WORK_MODE_AUTO))
        self.work_mode_combo.addItem("MANUAL", int(pb.WORK_MODE_MANUAL))
        self.btn_read_robot_params = QtWidgets.QPushButton("读取机器人参数")
        self.btn_apply_robot_params = QtWidgets.QPushButton("应用机器人参数")
        self.robot_vehicle_width = QtWidgets.QDoubleSpinBox()
        self.robot_vehicle_width.setRange(0.1, 10.0)
        self.robot_vehicle_width.setSingleStep(0.01)
        self.robot_vehicle_width.setDecimals(3)
        self.robot_vehicle_width.setValue(0.5)
        self.robot_vehicle_length = QtWidgets.QDoubleSpinBox()
        self.robot_vehicle_length.setRange(0.1, 20.0)
        self.robot_vehicle_length.setSingleStep(0.01)
        self.robot_vehicle_length.setDecimals(3)
        self.robot_vehicle_length.setValue(0.5)
        self.robot_path_spacing = QtWidgets.QDoubleSpinBox()
        self.robot_path_spacing.setRange(0.1, 10.0)
        self.robot_path_spacing.setSingleStep(0.01)
        self.robot_path_spacing.setDecimals(3)
        self.robot_path_spacing.setValue(0.425)
        self.robot_turn_radius = QtWidgets.QDoubleSpinBox()
        self.robot_turn_radius.setRange(0.1, 20.0)
        self.robot_turn_radius.setSingleStep(0.01)
        self.robot_turn_radius.setDecimals(3)
        self.robot_turn_radius.setValue(0.8)
        self.robot_overlap_ratio = QtWidgets.QDoubleSpinBox()
        self.robot_overlap_ratio.setRange(0.0, 0.95)
        self.robot_overlap_ratio.setSingleStep(0.01)
        self.robot_overlap_ratio.setDecimals(3)
        self.robot_overlap_ratio.setValue(0.1)
        self.robot_inflation_radius = QtWidgets.QDoubleSpinBox()
        self.robot_inflation_radius.setRange(0.0, 10.0)
        self.robot_inflation_radius.setSingleStep(0.01)
        self.robot_inflation_radius.setDecimals(3)
        self.robot_inflation_radius.setValue(0.65)

        self.task_id = QtWidgets.QLineEdit("task")
        self.task_id_combo = QtWidgets.QComboBox()
        self.task_id_combo.addItem("task", "task")
        self.request_id = QtWidgets.QLineEdit("req_" + str(int(time.time())))
        self.path_start_pose_edit = QtWidgets.QLineEdit("")
        self.path_start_pose_edit.setPlaceholderText("起点 x,y[,heading_deg] 例如 1.2,-0.8,90")
        self.path_end_pose_edit = QtWidgets.QLineEdit("")
        self.path_end_pose_edit.setPlaceholderText("终点 x,y[,heading_deg] 例如 3.0,-1.0,0")
        self.path_plan_direction_combo = QtWidgets.QComboBox()
        self.path_plan_direction_combo.addItems(["x", "-x", "y", "-y"])
        self.path_plan_direction_combo.setToolTip("规划轴与首条作业线方向")
        self.map_id_edit = QtWidgets.QLineEdit("")
        self.map_id_edit.setPlaceholderText("地图ID，例如 20260427_102229")
        self.map_name_edit = QtWidgets.QLineEdit("地图")
        self.map_name_edit.setPlaceholderText("保存地图名称（可中文）")
        self.save_dir = QtWidgets.QLineEdit(os.path.join(REPO_ROOT, "temp/sl_linka_debugger"))
        self.edit_id = QtWidgets.QLineEdit("edit_" + str(int(time.time())))
        self.edit_region_id = QtWidgets.QLineEdit("work_region_1")
        self.edit_region_name = QtWidgets.QLineEdit("work_region_1")
        self.edit_order = QtWidgets.QSpinBox()
        self.edit_order.setRange(0, 100000)
        self.edit_order.setValue(10)
        self.edit_brush_radius = QtWidgets.QDoubleSpinBox()
        self.edit_brush_radius.setRange(0.01, 10.0)
        self.edit_brush_radius.setSingleStep(0.05)
        self.edit_brush_radius.setValue(0.20)
        self.edit_target_type = QtWidgets.QComboBox()
        self.edit_target_type.addItem("WORK", int(pb.REGION_TYPE_WORK))
        self.edit_target_type.addItem("OBSTACLE", int(pb.REGION_TYPE_OBSTACLE))
        # Keep ERASE selectable even when local pb code is older.
        self.edit_target_type.addItem("ERASE", int(getattr(pb, "REGION_TYPE_ERASE", 3)))
        self.edit_target_type.addItem("CROP", 4)
        self.edit_points = QtWidgets.QPlainTextEdit(
            "0.0,0.0; 1.0,0.0; 1.0,1.0; 0.0,1.0"
        )
        self.edit_points.setPlaceholderText("点位格式: x1,y1; x2,y2; x3,y3")
        self.edit_points.setMaximumHeight(90)

        command_box = QtWidgets.QGroupBox("请求与模式控制")
        command_grid = QtWidgets.QGridLayout(command_box)
        command_grid.setContentsMargins(8, 8, 8, 8)
        command_grid.setHorizontalSpacing(8)
        command_grid.setVerticalSpacing(6)
        row = 0
        command_grid.addWidget(QtWidgets.QLabel("Map max_edge"), row, 0)
        command_grid.addWidget(self.map_max_edge, row, 1)
        command_grid.addWidget(QtWidgets.QLabel("Map format"), row, 2)
        command_grid.addWidget(self.map_format, row, 3)
        command_grid.addWidget(self.map_overlay, row, 4)
        command_grid.addWidget(self.map_fast_mode, row, 5)
        command_grid.addWidget(self.btn_map_preview, row, 6)
        command_grid.addWidget(self.btn_map_request, row, 7)
        row += 1
        command_grid.addWidget(QtWidgets.QLabel("start_pose"), row, 0)
        command_grid.addWidget(self.path_start_pose_edit, row, 1)
        command_grid.addWidget(self.btn_pick_start_pose, row, 2)
        command_grid.addWidget(QtWidgets.QLabel("end_pose"), row, 3)
        command_grid.addWidget(self.path_end_pose_edit, row, 4)
        command_grid.addWidget(self.btn_pick_end_pose, row, 5)
        command_grid.addWidget(self.path_plan_direction_combo, row, 6)
        command_grid.addWidget(self.btn_path_plan, row, 7)
        command_grid.addWidget(self.btn_task_result, row, 8)
        row += 1
        command_grid.addWidget(QtWidgets.QLabel("map_id"), row, 0)
        command_grid.addWidget(self.map_id_edit, row, 1, 1, 4)
        command_grid.addWidget(self.btn_map_sync_get, row, 5, 1, 2)
        command_grid.addWidget(self.btn_map_import_to_radar, row, 7, 1, 2)
        row += 1
        command_grid.addWidget(QtWidgets.QLabel("map_name"), row, 0)
        command_grid.addWidget(self.map_name_edit, row, 1, 1, 4)
        command_grid.addWidget(self.btn_map_sync_set, row, 5, 1, 2)
        row += 1
        command_grid.addWidget(self.btn_map_save_local, row, 5, 1, 2)
        row += 1
        command_grid.addWidget(self.btn_map_query_local, row, 0)
        command_grid.addWidget(self.local_map_count_label, row, 1)
        command_grid.addWidget(self.local_map_combo, row, 2, 1, 3)
        command_grid.addWidget(self.btn_map_delete_local, row, 5, 1, 2)
        command_grid.addWidget(self.btn_live_map_cache_clear, row, 7)
        command_grid.addWidget(self.btn_radar_map_cache_clear, row, 8)
        row += 1
        command_grid.addWidget(self.btn_video_info, row, 5, 1, 2)
        row += 1
        command_grid.addWidget(self.btn_mode_mapping_on, row, 3)
        command_grid.addWidget(self.btn_mode_mapping_off, row, 4)
        command_grid.addWidget(self.btn_mode_local_on, row, 5)
        command_grid.addWidget(self.btn_mode_local_off, row, 6)

        map_edit_box = QtWidgets.QGroupBox("地图编辑(MapEditCommand)")
        map_edit_box.setMinimumHeight(320)
        map_edit_grid = QtWidgets.QGridLayout(map_edit_box)
        map_edit_grid.setContentsMargins(8, 8, 8, 8)
        map_edit_grid.setHorizontalSpacing(8)
        map_edit_grid.setVerticalSpacing(6)
        row = 0
        map_edit_grid.addWidget(QtWidgets.QLabel("edit_id"), row, 0)
        map_edit_grid.addWidget(self.edit_id, row, 1)
        map_edit_grid.addWidget(QtWidgets.QLabel("target_type"), row, 2)
        map_edit_grid.addWidget(self.edit_target_type, row, 3)
        row += 1
        map_edit_grid.addWidget(QtWidgets.QLabel("region_id"), row, 0)
        map_edit_grid.addWidget(self.edit_region_id, row, 1)
        map_edit_grid.addWidget(QtWidgets.QLabel("region_name"), row, 2)
        map_edit_grid.addWidget(self.edit_region_name, row, 3)
        row += 1
        map_edit_grid.addWidget(QtWidgets.QLabel("order(priority)"), row, 0)
        map_edit_grid.addWidget(self.edit_order, row, 1)
        map_edit_grid.addWidget(QtWidgets.QLabel("brush_radius(m)"), row, 2)
        map_edit_grid.addWidget(self.edit_brush_radius, row, 3)
        row += 1
        map_edit_grid.addWidget(QtWidgets.QLabel("points"), row, 0, 1, 4)
        row += 1
        map_edit_grid.addWidget(self.edit_points, row, 0, 1, 4)
        row += 1
        map_edit_grid.addWidget(self.btn_edit_upsert_work, row, 0)
        map_edit_grid.addWidget(self.btn_edit_upsert_obstacle, row, 1)
        map_edit_grid.addWidget(self.btn_edit_upsert_erase, row, 2)
        map_edit_grid.addWidget(self.btn_edit_upsert_crop, row, 3)
        row += 1
        map_edit_grid.addWidget(QtWidgets.QLabel("删除区域下拉"), row, 0)
        map_edit_grid.addWidget(self.delete_region_combo, row, 1, 1, 2)
        map_edit_grid.addWidget(self.btn_region_query, row, 3)
        row += 1
        map_edit_grid.addWidget(self.btn_edit_delete, row, 0)
        map_edit_grid.addWidget(self.btn_edit_delete_crop, row, 1)
        map_edit_grid.addWidget(self.btn_edit_delete_all, row, 2, 1, 2)
        chassis_box = QtWidgets.QGroupBox("底盘功能控制")
        chassis_layout = QtWidgets.QGridLayout(chassis_box)
        chassis_layout.setContentsMargins(8, 8, 8, 8)
        chassis_layout.setHorizontalSpacing(8)
        chassis_layout.setVerticalSpacing(6)
        chassis_layout.addWidget(self.btn_chassis_enable, 0, 0)
        chassis_layout.addWidget(self.btn_chassis_disable, 0, 1)
        chassis_layout.addWidget(self.btn_light_on, 1, 0)
        chassis_layout.addWidget(self.btn_light_off, 1, 1)
        chassis_layout.addWidget(self.btn_lift_down, 2, 0)
        chassis_layout.addWidget(self.btn_lift_up, 2, 1)
        chassis_layout.addWidget(self.btn_disc_on, 3, 0)
        chassis_layout.addWidget(self.btn_disc_off, 3, 1)
        chassis_layout.addWidget(QtWidgets.QLabel("工作模式"), 4, 0)
        chassis_layout.addWidget(self.work_mode_combo, 4, 1)
        chassis_layout.addWidget(QtWidgets.QLabel("小车速度(m/s)"), 5, 0)
        chassis_layout.addWidget(self.chassis_run_speed, 5, 1)
        chassis_layout.addWidget(QtWidgets.QLabel("磨盘转速(rpm)"), 6, 0)
        chassis_layout.addWidget(self.disc_speed_spin, 6, 1)
        chassis_layout.addWidget(self.disc_enable_check, 7, 0, 1, 2)
        chassis_layout.addWidget(self.btn_read_chassis, 8, 0)
        chassis_layout.addWidget(self.btn_apply_chassis, 8, 1)
        remote_box = QtWidgets.QGroupBox("遥感控制")
        remote_layout = QtWidgets.QGridLayout(remote_box)
        remote_layout.setContentsMargins(8, 8, 8, 8)
        remote_layout.setHorizontalSpacing(8)
        remote_layout.setVerticalSpacing(6)
        remote_layout.addWidget(QtWidgets.QLabel("遥感速度"), 0, 0)
        remote_layout.addWidget(self.remote_speed, 0, 1)
        remote_layout.addWidget(self.remote_enable, 0, 2, 1, 2)
        remote_layout.addWidget(self.remote_xy, 0, 4, 1, 2)
        # Manual direction buttons are removed from UI by requirement.
        remote_layout.addWidget(self.btn_emergency_stop, 4, 0, 1, 3)
        remote_layout.addWidget(self.remote_stick, 1, 3, 4, 3)
        camera_box = QtWidgets.QGroupBox("摄像头取流预览(RTSP)")
        camera_layout = QtWidgets.QGridLayout(camera_box)
        camera_layout.setContentsMargins(8, 8, 8, 8)
        camera_layout.setHorizontalSpacing(8)
        camera_layout.setVerticalSpacing(6)
        self.left_rtsp_stream_url = "rtsp://127.0.0.1:8554/left"
        self.right_rtsp_stream_url = "rtsp://127.0.0.1:8554/right"
        self.btn_stream_start = QtWidgets.QPushButton("开始预览")
        self.btn_stream_stop = QtWidgets.QPushButton("停止预览")
        self.btn_stream_stop.setEnabled(False)
        self.camera_stream_status = QtWidgets.QLabel("未开始预览")
        self.left_cam_label = QtWidgets.QLabel("左相机画面")
        self.left_cam_label.setAlignment(QtCore.Qt.AlignCenter)
        self.left_cam_label.setMinimumSize(320, 200)
        self.left_cam_label.setStyleSheet("QLabel { background: #111; color: #ddd; border: 1px solid #444; }")
        self.right_cam_label = QtWidgets.QLabel("右相机画面")
        self.right_cam_label.setAlignment(QtCore.Qt.AlignCenter)
        self.right_cam_label.setMinimumSize(320, 200)
        self.right_cam_label.setStyleSheet("QLabel { background: #111; color: #ddd; border: 1px solid #444; }")
        row = 0
        camera_layout.addWidget(self.btn_stream_start, row, 0)
        camera_layout.addWidget(self.btn_stream_stop, row, 1)
        camera_layout.addWidget(self.camera_stream_status, row, 2, 1, 2)
        row += 1
        camera_layout.addWidget(self.left_cam_label, row, 0, 1, 2)
        camera_layout.addWidget(self.right_cam_label, row, 2, 1, 2)
        control_tabs = QtWidgets.QTabWidget()
        control_tabs.setDocumentMode(True)
        control_tabs.setElideMode(QtCore.Qt.ElideRight)
        control_tabs.setUsesScrollButtons(True)

        request_tab = QtWidgets.QWidget()
        request_layout = QtWidgets.QVBoxLayout(request_tab)
        request_layout.setContentsMargins(4, 4, 4, 4)
        request_layout.setSpacing(6)
        request_layout.addWidget(command_box)
        task_control_box = QtWidgets.QGroupBox("任务控制")
        task_control_grid = QtWidgets.QGridLayout(task_control_box)
        task_control_grid.setContentsMargins(8, 8, 8, 8)
        task_control_grid.setHorizontalSpacing(8)
        task_control_grid.setVerticalSpacing(6)
        row = 0
        task_control_grid.addWidget(QtWidgets.QLabel("task_id"), row, 0)
        task_control_grid.addWidget(self.task_id, row, 1)
        task_control_grid.addWidget(QtWidgets.QLabel("选中任务"), row, 2)
        task_control_grid.addWidget(self.task_id_combo, row, 3)
        row += 1
        task_control_grid.addWidget(QtWidgets.QLabel("request_id"), row, 0)
        task_control_grid.addWidget(self.request_id, row, 1, 1, 3)
        row += 1
        task_control_grid.addWidget(self.btn_task_create, row, 0, 1, 4)
        row += 1
        task_control_grid.addWidget(self.btn_task_regions_refresh, row, 0, 1, 4)
        row += 1
        task_control_grid.addWidget(self.task_region_table, row, 0, 1, 4)
        row += 1
        task_control_grid.addWidget(self.btn_task_start, row, 0)
        task_control_grid.addWidget(self.btn_task_pause, row, 1)
        task_control_grid.addWidget(self.btn_task_resume, row, 2)
        task_control_grid.addWidget(self.btn_task_stop, row, 3)
        task_status_box = QtWidgets.QGroupBox("任务状态(TaskStatusReport)")
        task_status_grid = QtWidgets.QGridLayout(task_status_box)
        task_status_grid.setContentsMargins(8, 8, 8, 8)
        task_status_grid.setHorizontalSpacing(8)
        task_status_grid.setVerticalSpacing(6)
        self.task_state_value = QtWidgets.QLabel("-")
        self.task_progress_value = QtWidgets.QLabel("0.0%")
        self.task_progress_bar = QtWidgets.QProgressBar()
        self.task_progress_bar.setRange(0, 1000)
        self.task_progress_bar.setValue(0)
        self.task_status_task_id_value = QtWidgets.QLabel("-")
        self.task_map_v_value = QtWidgets.QLabel("-")
        self.task_path_v_value = QtWidgets.QLabel("-")
        self.task_points_value = QtWidgets.QLabel("-")
        self.task_total_area_value = QtWidgets.QLabel("-")
        self.task_remaining_area_value = QtWidgets.QLabel("-")
        self.task_remaining_time_value = QtWidgets.QLabel("-")
        self.task_pose_value = QtWidgets.QLabel("x=0.000 y=0.000 heading=0.0")
        self.task_message_value = QtWidgets.QLabel("-")
        self.task_message_value.setWordWrap(True)
        self.task_time_value = QtWidgets.QLabel("-")
        row = 0
        task_status_grid.addWidget(QtWidgets.QLabel("state"), row, 0)
        task_status_grid.addWidget(self.task_state_value, row, 1)
        task_status_grid.addWidget(QtWidgets.QLabel("progress"), row, 2)
        task_status_grid.addWidget(self.task_progress_value, row, 3)
        row += 1
        task_status_grid.addWidget(self.task_progress_bar, row, 0, 1, 4)
        row += 1
        task_status_grid.addWidget(QtWidgets.QLabel("task_id"), row, 0)
        task_status_grid.addWidget(self.task_status_task_id_value, row, 1, 1, 3)
        row += 1
        task_status_grid.addWidget(QtWidgets.QLabel("map_version"), row, 0)
        task_status_grid.addWidget(self.task_map_v_value, row, 1)
        task_status_grid.addWidget(QtWidgets.QLabel("path_version"), row, 2)
        task_status_grid.addWidget(self.task_path_v_value, row, 3)
        row += 1
        task_status_grid.addWidget(QtWidgets.QLabel("path_points"), row, 0)
        task_status_grid.addWidget(self.task_points_value, row, 1)
        task_status_grid.addWidget(QtWidgets.QLabel("updated_at"), row, 2)
        task_status_grid.addWidget(self.task_time_value, row, 3)
        row += 1
        task_status_grid.addWidget(QtWidgets.QLabel("total_area(m²)"), row, 0)
        task_status_grid.addWidget(self.task_total_area_value, row, 1)
        task_status_grid.addWidget(QtWidgets.QLabel("remaining_area(m²)"), row, 2)
        task_status_grid.addWidget(self.task_remaining_area_value, row, 3)
        row += 1
        task_status_grid.addWidget(QtWidgets.QLabel("remaining_time"), row, 0)
        task_status_grid.addWidget(self.task_remaining_time_value, row, 1, 1, 3)
        row += 1
        task_status_grid.addWidget(QtWidgets.QLabel("pose"), row, 0)
        task_status_grid.addWidget(self.task_pose_value, row, 1, 1, 3)
        row += 1
        task_status_grid.addWidget(QtWidgets.QLabel("message"), row, 0)
        task_status_grid.addWidget(self.task_message_value, row, 1, 1, 3)
        robot_param_box = QtWidgets.QGroupBox("机器人参数设置(MapSettings)")
        robot_param_grid = QtWidgets.QGridLayout(robot_param_box)
        robot_param_grid.setContentsMargins(8, 8, 8, 8)
        robot_param_grid.setHorizontalSpacing(8)
        robot_param_grid.setVerticalSpacing(6)
        row = 0
        robot_param_grid.addWidget(QtWidgets.QLabel("vehicle_width(m)"), row, 0)
        robot_param_grid.addWidget(self.robot_vehicle_width, row, 1)
        robot_param_grid.addWidget(QtWidgets.QLabel("vehicle_length(m)"), row, 2)
        robot_param_grid.addWidget(self.robot_vehicle_length, row, 3)
        row += 1
        robot_param_grid.addWidget(QtWidgets.QLabel("default_path_spacing(m)"), row, 0)
        robot_param_grid.addWidget(self.robot_path_spacing, row, 1)
        robot_param_grid.addWidget(QtWidgets.QLabel("turn_radius(m)"), row, 2)
        robot_param_grid.addWidget(self.robot_turn_radius, row, 3)
        row += 1
        robot_param_grid.addWidget(QtWidgets.QLabel("overlap_ratio"), row, 0)
        robot_param_grid.addWidget(self.robot_overlap_ratio, row, 1)
        robot_param_grid.addWidget(QtWidgets.QLabel("inflation_radius(m)"), row, 2)
        robot_param_grid.addWidget(self.robot_inflation_radius, row, 3)
        row += 1
        robot_param_grid.addWidget(self.btn_read_robot_params, row, 0, 1, 2)
        robot_param_grid.addWidget(self.btn_apply_robot_params, row, 2, 1, 2)
        request_layout.addWidget(robot_param_box)
        request_layout.addWidget(map_edit_box, 1)
        control_tabs.addTab(request_tab, "请求控制+地图编辑")

        task_tab = QtWidgets.QWidget()
        task_layout = QtWidgets.QVBoxLayout(task_tab)
        task_layout.setContentsMargins(4, 4, 4, 4)
        task_layout.setSpacing(6)
        task_layout.addWidget(task_control_box)
        task_layout.addWidget(task_status_box)
        task_layout.addStretch(1)
        control_tabs.addTab(task_tab, "任务")

        chassis_remote_tab = QtWidgets.QWidget()
        chassis_remote_layout = QtWidgets.QVBoxLayout(chassis_remote_tab)
        chassis_remote_layout.setContentsMargins(4, 4, 4, 4)
        chassis_remote_layout.setSpacing(6)
        chassis_remote_layout.addWidget(chassis_box)
        chassis_remote_layout.addWidget(remote_box)
        chassis_remote_layout.addWidget(camera_box)
        chassis_remote_layout.addStretch(1)
        control_tabs.addTab(chassis_remote_tab, "底盘与遥感")

        control_scroll = QtWidgets.QScrollArea()
        control_scroll.setWidgetResizable(True)
        control_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        control_scroll.setWidget(control_tabs)
        control_splitter.addWidget(control_scroll)

        self.preview_tabs = QtWidgets.QTabWidget()

        preview_tab = QtWidgets.QWidget()
        preview_tab_layout = QtWidgets.QVBoxLayout(preview_tab)
        preview_tab_layout.setContentsMargins(8, 8, 8, 8)
        preview_tab_layout.setSpacing(8)

        left_box = QtWidgets.QGroupBox("叠层地图预览")
        left_box_layout = QtWidgets.QVBoxLayout(left_box)
        left_box_layout.setContentsMargins(8, 8, 8, 8)
        self.map_image_label = QtWidgets.QLabel("暂无地图图片")
        self.map_image_label.setAlignment(QtCore.Qt.AlignCenter)
        self.map_image_label.setMinimumSize(360, 240)
        self.map_image_label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.map_image_label.setStyleSheet("QLabel { background: #111; color: #ddd; border: 1px solid #444; }")
        self.map_image_label.setCursor(QtGui.QCursor(QtCore.Qt.CrossCursor))
        self.map_image_label.installEventFilter(self)
        left_box_layout.addWidget(self.map_image_label)
        preview_tab_layout.addWidget(left_box, 5)

        right_box = QtWidgets.QGroupBox("叠层地图信息")
        right_box_layout = QtWidgets.QVBoxLayout(right_box)
        right_box_layout.setContentsMargins(8, 8, 8, 8)
        self.map_info_text = QtWidgets.QPlainTextEdit()
        self.map_info_text.setReadOnly(True)
        self.map_info_text.setPlainText("等待 MapPreviewResponse ...")
        right_box_layout.addWidget(self.map_info_text)
        preview_tab_layout.addWidget(right_box, 2)
        self.preview_tabs.addTab(preview_tab, "地图预览")

        raw_tab = QtWidgets.QWidget()
        raw_tab_layout = QtWidgets.QVBoxLayout(raw_tab)
        raw_tab_layout.setContentsMargins(8, 8, 8, 8)
        raw_tab_layout.setSpacing(8)

        raw_box = QtWidgets.QGroupBox("原始地图显示(MapRequest)")
        raw_box_layout = QtWidgets.QVBoxLayout(raw_box)
        raw_box_layout.setContentsMargins(8, 8, 8, 8)
        self.raw_map_image_label = QtWidgets.QLabel("暂无原始地图图片")
        self.raw_map_image_label.setAlignment(QtCore.Qt.AlignCenter)
        self.raw_map_image_label.setMinimumSize(360, 240)
        self.raw_map_image_label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.raw_map_image_label.setStyleSheet("QLabel { background: #111; color: #ddd; border: 1px solid #444; }")
        self.raw_map_image_label.installEventFilter(self)
        raw_box_layout.addWidget(self.raw_map_image_label)
        raw_tab_layout.addWidget(raw_box, 5)

        raw_info_box = QtWidgets.QGroupBox("原始地图信息")
        raw_info_layout = QtWidgets.QVBoxLayout(raw_info_box)
        raw_info_layout.setContentsMargins(8, 8, 8, 8)
        self.raw_map_info_text = QtWidgets.QPlainTextEdit()
        self.raw_map_info_text.setReadOnly(True)
        self.raw_map_info_text.setPlainText("等待 MapRequest / MapChunk ...")
        raw_info_layout.addWidget(self.raw_map_info_text)
        raw_tab_layout.addWidget(raw_info_box, 2)
        self.preview_tabs.addTab(raw_tab, "原始地图")

        path_tab = QtWidgets.QWidget()
        path_tab_layout = QtWidgets.QVBoxLayout(path_tab)
        path_tab_layout.setContentsMargins(8, 8, 8, 8)
        path_tab_layout.setSpacing(8)

        path_box = QtWidgets.QGroupBox("路径规划图")
        path_box_layout = QtWidgets.QVBoxLayout(path_box)
        path_box_layout.setContentsMargins(8, 8, 8, 8)
        self.path_image_label = QtWidgets.QLabel("暂无路径预览图")
        self.path_image_label.setAlignment(QtCore.Qt.AlignCenter)
        self.path_image_label.setMinimumSize(360, 240)
        self.path_image_label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.path_image_label.setStyleSheet("QLabel { background: #111; color: #ddd; border: 1px solid #444; }")
        self.path_image_label.installEventFilter(self)
        path_box_layout.addWidget(self.path_image_label)
        path_tab_layout.addWidget(path_box, 5)

        path_info_box = QtWidgets.QGroupBox("路径规划信息")
        path_info_layout = QtWidgets.QVBoxLayout(path_info_box)
        path_info_layout.setContentsMargins(8, 8, 8, 8)
        self.path_info_text = QtWidgets.QPlainTextEdit()
        self.path_info_text.setReadOnly(True)
        self.path_info_text.setPlainText("等待 PathPlanRequest / PathPlanResponse ...")
        path_info_layout.addWidget(self.path_info_text)
        path_tab_layout.addWidget(path_info_box, 2)
        self.preview_tabs.addTab(path_tab, "路径规划")

        task_result_tab = QtWidgets.QWidget()
        task_result_tab_layout = QtWidgets.QVBoxLayout(task_result_tab)
        task_result_tab_layout.setContentsMargins(8, 8, 8, 8)
        task_result_tab_layout.setSpacing(8)

        task_result_box = QtWidgets.QGroupBox("任务结果图")
        task_result_box_layout = QtWidgets.QVBoxLayout(task_result_box)
        task_result_box_layout.setContentsMargins(8, 8, 8, 8)
        self.task_result_image_label = QtWidgets.QLabel("暂无任务结果图")
        self.task_result_image_label.setAlignment(QtCore.Qt.AlignCenter)
        self.task_result_image_label.setMinimumSize(360, 240)
        self.task_result_image_label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.task_result_image_label.setStyleSheet("QLabel { background: #111; color: #ddd; border: 1px solid #444; }")
        self.task_result_image_label.installEventFilter(self)
        task_result_box_layout.addWidget(self.task_result_image_label)
        task_result_tab_layout.addWidget(task_result_box, 5)

        task_result_info_box = QtWidgets.QGroupBox("任务结果信息")
        task_result_info_layout = QtWidgets.QVBoxLayout(task_result_info_box)
        task_result_info_layout.setContentsMargins(8, 8, 8, 8)
        self.task_result_info_text = QtWidgets.QPlainTextEdit()
        self.task_result_info_text.setReadOnly(True)
        self.task_result_info_text.setPlainText("等待 TaskResultRequest / TaskResultResponse ...")
        task_result_info_layout.addWidget(self.task_result_info_text)
        task_result_tab_layout.addWidget(task_result_info_box, 2)
        self.preview_tabs.addTab(task_result_tab, "任务结果")

        logs_box = QtWidgets.QGroupBox("日志")
        logs_layout = QtWidgets.QVBoxLayout(logs_box)
        logs_layout.setContentsMargins(8, 8, 8, 8)
        logs_layout.setSpacing(6)
        logs_box.setMinimumHeight(140)
        control_splitter.addWidget(logs_box)

        logs_panel = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        logs_layout.addWidget(logs_panel, 1)

        all_log_box = QtWidgets.QGroupBox("全部日志(含自动上报)")
        all_log_layout = QtWidgets.QVBoxLayout(all_log_box)
        all_log_layout.setContentsMargins(8, 8, 8, 8)
        self.log_text = QtWidgets.QPlainTextEdit()
        self.log_text.setReadOnly(True)
        all_log_layout.addWidget(self.log_text)
        logs_panel.addWidget(all_log_box)

        reqresp_log_box = QtWidgets.QGroupBox("请求/返回日志")
        reqresp_log_layout = QtWidgets.QVBoxLayout(reqresp_log_box)
        reqresp_log_layout.setContentsMargins(8, 8, 8, 8)
        self.reqresp_log_text = QtWidgets.QPlainTextEdit()
        self.reqresp_log_text.setReadOnly(True)
        reqresp_log_layout.addWidget(self.reqresp_log_text)
        logs_panel.addWidget(reqresp_log_box)

        log_btn_row = QtWidgets.QHBoxLayout()
        logs_layout.addLayout(log_btn_row)
        self.clear_all_log_btn = QtWidgets.QPushButton("清除全部日志")
        self.clear_reqresp_log_btn = QtWidgets.QPushButton("清除请求日志")
        log_btn_row.addWidget(self.clear_all_log_btn)
        log_btn_row.addWidget(self.clear_reqresp_log_btn)
        log_btn_row.addStretch(1)

        main_splitter.addWidget(self.preview_tabs)
        main_splitter.addWidget(control_panel)
        main_splitter.setStretchFactor(0, 1)
        main_splitter.setStretchFactor(1, 1)
        total_w = max(1600, self.width())
        main_splitter.setSizes([int(total_w * 0.50), int(total_w * 0.50)])
        control_splitter.setStretchFactor(0, 1)
        control_splitter.setStretchFactor(1, 1)
        total_h = max(900, self.height())
        control_splitter.setSizes([int(total_h * 0.76), int(total_h * 0.24)])
        logs_panel.setSizes([560, 560])

        self._apply_compact_min_heights()

        self.connect_btn.clicked.connect(self._on_connect)
        self.disconnect_btn.clicked.connect(self.client.disconnect)
        self.btn_map_preview.clicked.connect(self._send_map_preview)
        self.btn_map_request.clicked.connect(self._send_map_request)
        self.btn_path_plan.clicked.connect(self._send_path_plan)
        self.btn_task_result.clicked.connect(self._send_task_result)
        self.btn_pick_start_pose.clicked.connect(lambda: self._start_pose_pick_mode("start_pose"))
        self.btn_pick_end_pose.clicked.connect(lambda: self._start_pose_pick_mode("end_pose"))
        self.btn_task_start.clicked.connect(lambda: self._send_task_command(pb.TASK_CMD_START, "START"))
        self.btn_task_pause.clicked.connect(lambda: self._send_task_command(pb.TASK_CMD_PAUSE, "PAUSE"))
        self.btn_task_resume.clicked.connect(lambda: self._send_task_command(pb.TASK_CMD_RESUME, "RESUME"))
        self.btn_task_stop.clicked.connect(lambda: self._send_task_command(pb.TASK_CMD_STOP, "STOP"))
        self.btn_task_create.clicked.connect(self._send_task_config_all_regions_once)
        self.btn_task_regions_refresh.clicked.connect(self._refresh_task_region_table)
        self.btn_map_sync_get.clicked.connect(self._send_map_sync_get)
        self.btn_map_sync_set.clicked.connect(self._send_map_sync_set)
        self.btn_map_import_to_radar.clicked.connect(self._send_map_import_to_radar)
        self.btn_map_save_local.clicked.connect(self._send_map_save_local)
        self.btn_map_query_local.clicked.connect(self._query_local_maps)
        self.btn_map_delete_local.clicked.connect(self._delete_selected_local_map)
        self.btn_live_map_cache_clear.clicked.connect(self._send_live_map_cache_clear)
        self.btn_radar_map_cache_clear.clicked.connect(self._send_radar_map_cache_clear)
        self.local_map_combo.currentIndexChanged.connect(self._on_local_map_selected)
        self.btn_video_info.clicked.connect(self._send_video_info)
        self.btn_edit_upsert_work.clicked.connect(self._send_edit_upsert_work)
        self.btn_edit_upsert_obstacle.clicked.connect(self._send_edit_upsert_obstacle)
        self.btn_edit_upsert_erase.clicked.connect(self._send_edit_upsert_erase)
        self.btn_edit_upsert_crop.clicked.connect(self._send_edit_upsert_crop)
        self.btn_edit_delete.clicked.connect(self._send_edit_delete_region)
        self.btn_edit_delete_all.clicked.connect(self._send_edit_delete_all_regions)
        self.btn_edit_delete_crop.clicked.connect(self._send_edit_delete_crop_region)
        self.btn_region_query.clicked.connect(self._send_settings_read_regions)
        self.delete_region_combo.currentIndexChanged.connect(self._on_delete_region_combo_changed)
        self.btn_mode_mapping_on.clicked.connect(lambda: self._send_map_mode(pb.MAP_MODE_MAPPING, True))
        self.btn_mode_mapping_off.clicked.connect(lambda: self._send_map_mode(pb.MAP_MODE_MAPPING, False))
        self.btn_mode_local_on.clicked.connect(lambda: self._send_map_mode(pb.MAP_MODE_LOCALIZATION, True))
        self.btn_mode_local_off.clicked.connect(lambda: self._send_map_mode(pb.MAP_MODE_LOCALIZATION, False))
        # Manual direction buttons are removed from UI by requirement.
        self.btn_emergency_stop.clicked.connect(self._send_emergency_stop)
        self.btn_chassis_enable.clicked.connect(lambda: self._send_chassis_power(True))
        self.btn_chassis_disable.clicked.connect(lambda: self._send_chassis_power(False))
        self.btn_light_on.clicked.connect(lambda: self._send_light(True))
        self.btn_light_off.clicked.connect(lambda: self._send_light(False))
        self.btn_lift_down.clicked.connect(lambda: self._send_disc_lift(False))
        self.btn_lift_up.clicked.connect(lambda: self._send_disc_lift(True))
        self.btn_disc_on.clicked.connect(lambda: self._send_disc_enable(True))
        self.btn_disc_off.clicked.connect(lambda: self._send_disc_enable(False))
        self.btn_read_chassis.clicked.connect(self._send_settings_read_chassis)
        self.btn_apply_chassis.clicked.connect(self._send_settings_write_chassis)
        self.btn_read_robot_params.clicked.connect(self._send_settings_read_map_params)
        self.btn_apply_robot_params.clicked.connect(self._send_settings_write_map_params)
        self.btn_stream_start.clicked.connect(self._start_rtsp_preview)
        self.btn_stream_stop.clicked.connect(self._stop_rtsp_preview)
        self.edit_target_type.currentIndexChanged.connect(self._on_edit_target_type_changed)
        self.map_fast_mode.toggled.connect(self._on_map_fast_mode_toggled)
        self.remote_stick.value_changed.connect(self._on_joystick_value_changed)
        self.remote_enable.toggled.connect(self._on_remote_enable_changed)
        self.clear_all_log_btn.clicked.connect(self.log_text.clear)
        self.clear_reqresp_log_btn.clicked.connect(self.reqresp_log_text.clear)

        self.remote_timer = QtCore.QTimer(self)
        self.remote_timer.setInterval(120)
        self.remote_timer.timeout.connect(self._send_remote_continuous)
        self._on_edit_target_type_changed(self.edit_target_type.currentIndex())
        self._on_remote_enable_changed(self.remote_enable.isChecked())
        self._query_local_maps()
        self._refresh_task_region_table()

    def _apply_compact_min_heights(self):
        # Keep controls readable across DPI / mixed-resolution displays.
        for editor in self.findChildren(QtWidgets.QLineEdit):
            editor.setMinimumHeight(max(editor.minimumHeight(), 28))
        for combo in self.findChildren(QtWidgets.QComboBox):
            combo.setMinimumHeight(max(combo.minimumHeight(), 28))
        for spin in self.findChildren(QtWidgets.QSpinBox):
            spin.setMinimumHeight(max(spin.minimumHeight(), 28))
        for dspin in self.findChildren(QtWidgets.QDoubleSpinBox):
            dspin.setMinimumHeight(max(dspin.minimumHeight(), 28))
        for btn in self.findChildren(QtWidgets.QPushButton):
            btn.setMinimumHeight(max(btn.minimumHeight(), 28))
        for plain in self.findChildren(QtWidgets.QPlainTextEdit):
            plain.setMinimumHeight(max(plain.minimumHeight(), 64))

    def _start_rtsp_preview(self):
        self._stop_rtsp_preview(silent=True)
        self._rtsp_latest_frames = {"left": None, "right": None}
        left_url = str(self.left_rtsp_stream_url).strip()
        right_url = str(self.right_rtsp_stream_url).strip()
        if not left_url or not right_url:
            self._append_log("RTSP预览启动失败: 左右URL不能为空")
            return
        left_worker = RtspPreviewWorker("left", left_url)
        right_worker = RtspPreviewWorker("right", right_url)
        left_worker.frame_ready.connect(self._on_rtsp_frame)
        right_worker.frame_ready.connect(self._on_rtsp_frame)
        left_worker.log.connect(self._append_log)
        right_worker.log.connect(self._append_log)
        self._rtsp_workers = {"left": left_worker, "right": right_worker}
        left_worker.start()
        right_worker.start()
        self.camera_stream_status.setText("预览中")
        self.btn_stream_start.setEnabled(False)
        self.btn_stream_stop.setEnabled(True)
        self._rtsp_render_timer.start()
        self._append_log(f"RTSP预览已启动: left={left_url} right={right_url}")

    def _stop_rtsp_preview(self, silent=False):
        workers = self._rtsp_workers
        self._rtsp_workers = {}
        for _, worker in workers.items():
            try:
                worker.stop()
            except Exception:
                pass
        self._rtsp_render_timer.stop()
        self._rtsp_latest_frames = {"left": None, "right": None}
        self.btn_stream_start.setEnabled(True)
        self.btn_stream_stop.setEnabled(False)
        self.camera_stream_status.setText("未开始预览")
        if not silent:
            self._append_log("RTSP预览已停止")

    def _on_rtsp_frame(self, stream_name: str, frame):
        if cv2 is None or frame is None:
            return
        name = "left" if str(stream_name) == "left" else "right"
        # Only keep newest frame, drop stale frames immediately.
        self._rtsp_latest_frames[name] = frame

    def _render_rtsp_latest_frames(self):
        if cv2 is None:
            return
        try:
            for name, frame in list(self._rtsp_latest_frames.items()):
                if frame is None:
                    continue
                draw = frame.copy()
                now_text = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                cv2.putText(
                    draw,
                    now_text,
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 0, 0),
                    3,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    draw,
                    now_text,
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
                rgb = cv2.cvtColor(draw, cv2.COLOR_BGR2RGB)
                h, w = rgb.shape[:2]
                qimg = QtGui.QImage(rgb.data, w, h, rgb.strides[0], QtGui.QImage.Format_RGB888).copy()
                pixmap = QtGui.QPixmap.fromImage(qimg)
                if name == "left":
                    self._left_cam_pixmap = pixmap
                    self._render_camera_label(self.left_cam_label, pixmap)
                else:
                    self._right_cam_pixmap = pixmap
                    self._render_camera_label(self.right_cam_label, pixmap)
        except Exception as exc:
            self._append_log(f"RTSP帧渲染失败: {exc}")

    def _render_camera_label(self, label: QtWidgets.QLabel, pixmap: QtGui.QPixmap):
        if pixmap is None or pixmap.isNull():
            return
        target = label.size()
        if target.width() <= 8 or target.height() <= 8:
            return
        scaled = pixmap.scaled(target, QtCore.Qt.KeepAspectRatio, QtCore.Qt.FastTransformation)
        label.setPixmap(scaled)

    def _on_connect(self):
        host = self.host_edit.text().strip() or "127.0.0.1"
        port = int(self.port_edit.text().strip() or "8002")
        try:
            self.client.connect_to(host, port)
        except Exception as exc:
            self._append_log(f"Connect failed: {exc}")

    def _on_state_changed(self, online: bool, msg: str):
        self.status_label.setText(msg)
        self.connect_btn.setEnabled(not online)
        self.disconnect_btn.setEnabled(online)
        self._append_log(msg)

    def _send_map_preview(self):
        if hasattr(self, "preview_tabs"):
            self.preview_tabs.setCurrentIndex(0)
        fmt = self.map_format.currentText().strip().lower() or "png"
        include_overlay = bool(self.map_overlay.isChecked())
        if self.map_fast_mode.isChecked() and include_overlay:
            include_overlay = False
            self._append_log("极速模式已开启: MapPreviewRequest 自动关闭 include_overlay")
        req = pb.MapPreviewRequest(
            max_edge=int(self.map_max_edge.value()),
            image_format=fmt,
            include_overlay=include_overlay,
        )
        if hasattr(req, "map_id"):
            req.map_id = self._get_selected_or_manual_map_id()
        self.btn_map_preview.setEnabled(False)
        QtCore.QTimer.singleShot(2500, lambda: self.btn_map_preview.setEnabled(True))
        self._append_log(
            f"发送 MapPreviewRequest: format={fmt} include_overlay={include_overlay} "
            f"map_id={getattr(req, 'map_id', '<unsupported>')}"
        )
        self._send(pb.MSG_ID_MAP_PREVIEW_REQUEST, pb.COMP_SCHEDULER, req)

    def _send_map_request(self):
        # MapRequest(原始) should switch to raw-map tab.
        if hasattr(self, "preview_tabs"):
            self.preview_tabs.setCurrentIndex(1)
        map_id = self._get_selected_or_manual_map_id()
        req = pb.MapRequest(snapshot=True, max_chunk_size=512)
        if hasattr(req, "map_id"):
            req.map_id = str(map_id or "")
        self._append_log(
            "发送 MapRequest(0x0304): 请求原始地图分片(默认PNG) "
            f"map_id={getattr(req, 'map_id', '<unsupported>')}"
        )
        self._map_chunk_state = {}
        self.raw_map_info_text.setPlainText("MapRequest 已发送，等待 MapChunk ...")
        self.raw_map_image_label.setText("等待原始地图图片 ...")
        self.raw_map_image_label.setPixmap(QtGui.QPixmap())
        self._send(pb.MSG_ID_MAP_REQUEST, pb.COMP_MEDIA, req)

    def _send_path_plan(self):
        if hasattr(self, "preview_tabs"):
            self.preview_tabs.setCurrentIndex(2)
        req_id = self.request_id.text().strip()
        task_id = self.task_id.text().strip()
        map_id = self._get_selected_or_manual_map_id()
        selected_direction = self.path_plan_direction_combo.currentText().strip().lower() or "x"
        default_heading = 90.0 if selected_direction.lstrip("+-") == "y" else 0.0
        try:
            start_pose, start_has_heading = self._parse_pose_text(self.path_start_pose_edit.text(), "start_pose")
            end_pose, end_has_heading = self._parse_pose_text(self.path_end_pose_edit.text(), "end_pose")
        except ValueError as exc:
            QtWidgets.QMessageBox.warning(self, "PathPlan 参数错误", str(exc))
            return
        req = pb.PathPlanRequest(
            request_id=req_id,
            task_id=task_id,
            force_replan=True,
            return_path_chunks=False,
            max_chunk_size=4096,
        )
        if hasattr(req, "map_id"):
            req.map_id = str(map_id or "")
        req.global_direction = selected_direction
        if start_pose is not None:
            req.start_pose.x = float(start_pose[0])
            req.start_pose.y = float(start_pose[1])
            req.start_pose.heading_deg = float(start_pose[2] if start_has_heading else default_heading)
        if end_pose is not None:
            req.end_pose.x = float(end_pose[0])
            req.end_pose.y = float(end_pose[1])
            req.end_pose.heading_deg = float(end_pose[2] if end_has_heading else default_heading)
        self.path_info_text.setPlainText(
            "\n".join(
                [
                    "PathPlanRequest 已发送",
                    f"request_id: {req_id}",
                    f"task_id: {task_id}",
                    f"map_id: {map_id or '<empty>'}",
                    "force_replan: true",
                    "return_path_chunks: false",
                    "max_chunk_size: 4096",
                    "global_direction: {}".format(req.global_direction),
                    (
                        "start_pose: {:.3f}, {:.3f}, {:.1f}".format(
                            start_pose[0],
                            start_pose[1],
                            start_pose[2] if start_has_heading else default_heading,
                        )
                        if start_pose is not None
                        else "start_pose: <empty>"
                    ),
                    (
                        "end_pose: {:.3f}, {:.3f}, {:.1f}".format(
                            end_pose[0],
                            end_pose[1],
                            end_pose[2] if end_has_heading else default_heading,
                        )
                        if end_pose is not None
                        else "end_pose: <empty>"
                    ),
                    "axis_default: {}".format(
                        "Y(90deg)" if selected_direction.lstrip("+-") == "y" else "X(0deg)"
                    ),
                ]
            )
        )
        selected_cfg = self._collect_selected_task_regions()
        selected_text = ",".join([f"{rid}x{repeat}" for rid, repeat in selected_cfg]) or "<none>"
        self._append_log(
            "发送 PathPlanRequest: "
            f"map_id={map_id or '<empty>'} "
            f"global_direction={req.global_direction} "
            f"selected_order={selected_text}"
        )
        self._send(pb.MSG_ID_PATH_PLAN_REQUEST, pb.COMP_SCHEDULER, req)

    def _send_task_result(self):
        if hasattr(self, "preview_tabs"):
            self.preview_tabs.setCurrentIndex(2)
        req = pb.TaskResultRequest()
        map_id = self._get_selected_or_manual_map_id()
        task_id = self.task_id.text().strip()
        if map_id:
            req.map_id = map_id
        if task_id:
            req.task_id = task_id
        self.path_info_text.setPlainText(
            "\n".join(
                [
                    "TaskResultRequest 已发送",
                    f"map_id: {getattr(req, 'map_id', '') or '<empty>'}",
                    f"task_id: {getattr(req, 'task_id', '') or '<empty>'}",
                ]
            )
        )
        self._append_log(
            "发送 TaskResultRequest: "
            f"map_id={getattr(req, 'map_id', '') or '<empty>'} "
            f"task_id={getattr(req, 'task_id', '') or '<empty>'}"
        )
        self._send(pb.MSG_ID_TASK_RESULT_REQUEST, pb.COMP_SCHEDULER, req)

    def _send_task_command(self, command: int, command_name: str):
        task_id = self._get_selected_task_id()
        req = pb.TaskCommand(task_id=task_id, command=int(command))
        self._append_log(f"发送 TaskCommand: task_id={task_id} command={command_name}({int(command)})")
        self._send(pb.MSG_ID_TASK_COMMAND, pb.COMP_SCHEDULER, req)

    def _send_task_config_all_regions_once(self):
        if not hasattr(pb, "MSG_ID_TASK_CONFIG"):
            self._append_log("创建任务失败: 当前协议库不支持 TaskConfig")
            return
        task_id = self.task_id.text().strip() or "task"
        map_id = self._get_selected_or_manual_map_id()
        regions = self._preview_overlay_payload.get("regions", {}) if isinstance(self._preview_overlay_payload, dict) else {}
        work_regions = regions.get("work_regions", []) if isinstance(regions, dict) else []
        valid_regions = []
        for r in list(work_regions or []):
            if not isinstance(r, dict):
                continue
            points = list(r.get("points", []) or [])
            if len(points) < 3:
                continue
            rid = str(r.get("region_id", "") or "").strip()
            if not rid:
                continue
            valid_regions.append(r)
        if not valid_regions:
            self._append_log("创建任务失败: 当前没有可用工作区，请先查询区域或加载叠层地图")
            return

        req = pb.TaskConfig()
        req.task_id = task_id
        if hasattr(req, "map_id"):
            req.map_id = map_id

        selected_cfg = self._collect_selected_task_regions()
        selected_order = [rid for rid, _ in selected_cfg]
        selected_id_set = {rid for rid in selected_order}
        repeat_map = {rid: repeat for rid, repeat in selected_cfg}
        available_region_ids = []
        for r in valid_regions:
            region = req.work_regions.add()
            region.region_id = str(r.get("region_id", "") or "")
            region.name = str(r.get("name", "") or region.region_id)
            region.priority = int(r.get("priority", 10) or 10)
            region.enabled = bool(r.get("enabled", True))
            region.closed = True
            region.color_argb = int(r.get("color_argb", 0) or 0)
            region.region_type = int(getattr(pb, "REGION_TYPE_WORK", 1))
            for pt in list(r.get("points", []) or []):
                p = region.points.add()
                p.x = float((pt or {}).get("x", 0.0))
                p.y = float((pt or {}).get("y", 0.0))
            rid = str(region.region_id or "").strip()
            if rid:
                available_region_ids.append(rid)

        if selected_order:
            selected_ids = [rid for rid in selected_order if rid in available_region_ids]
        else:
            selected_ids = list(available_region_ids)

        if not selected_ids:
            self._append_log("创建任务失败: 未勾选任何区域")
            return

        if hasattr(req, "selected_work_region_ids"):
            req.selected_work_region_ids.extend(selected_ids)
        if hasattr(req, "region_repeats"):
            for rid in selected_ids:
                item = req.region_repeats.add()
                item.region_id = rid
                item.repeat = int(max(1, int(repeat_map.get(rid, 1))))

        selected_order_text = ",".join(selected_ids) if selected_ids else "<none>"
        repeat_order_text = ",".join(
            ["{}:{}".format(rid, int(max(1, int(repeat_map.get(rid, 1))))) for rid in selected_ids]
        ) if selected_ids else "<none>"

        self._append_log(
            "发送 TaskConfig(选中区域): "
            f"task_id={task_id} map_id={map_id} work_regions={len(selected_ids)} "
            f"region_repeats={{{', '.join([f'{rid}:{repeat_map.get(rid,1)}' for rid in selected_ids])}}}"
        )
        self._append_log(
            "TaskConfig下发顺序: "
            f"selected_work_region_ids=[{selected_order_text}] "
            f"region_repeats=[{repeat_order_text}]"
        )
        self._remember_task_id(task_id)
        self._send(pb.MSG_ID_TASK_CONFIG, pb.COMP_SCHEDULER, req)

    def _collect_selected_task_regions(self):
        selected = []
        table = getattr(self, "task_region_table", None)
        if table is None:
            return selected
        selected_map = {}
        checked_row_order = []
        for row in range(table.rowCount()):
            check_item = table.item(row, 0)
            id_item = table.item(row, 1)
            if check_item is None or id_item is None:
                continue
            rid = str(id_item.text() or "").strip()
            if not rid:
                continue
            if check_item.checkState() != QtCore.Qt.Checked:
                continue
            checked_row_order.append(rid)
            spin = table.cellWidget(row, 2)
            repeat = 1
            if isinstance(spin, QtWidgets.QSpinBox):
                repeat = int(max(1, spin.value()))
            selected_map[rid] = repeat
        ordered_ids = []
        for rid in list(self._task_region_pick_order or []):
            if rid in selected_map and rid not in ordered_ids:
                ordered_ids.append(rid)
        for rid in checked_row_order:
            if rid in selected_map and rid not in ordered_ids:
                ordered_ids.append(rid)
        for rid in ordered_ids:
            repeat = selected_map.get(rid, 1)
            selected.append((rid, repeat))
        return selected

    def _refresh_task_region_order_labels(self):
        table = getattr(self, "task_region_table", None)
        if table is None:
            return
        selected_cfg = self._collect_selected_task_regions()
        order_map = {rid: idx + 1 for idx, (rid, _repeat) in enumerate(selected_cfg)}
        self._task_region_table_updating = True
        for row in range(table.rowCount()):
            id_item = table.item(row, 1)
            rid = str(id_item.text() or "").strip() if id_item is not None else ""
            order_item = table.item(row, 3)
            if order_item is None:
                order_item = QtWidgets.QTableWidgetItem("")
                order_item.setFlags(order_item.flags() & ~QtCore.Qt.ItemIsEditable)
                table.setItem(row, 3, order_item)
            order_item.setText(str(order_map.get(rid, "")) if rid else "")
        self._task_region_table_updating = False

    def _refresh_task_region_table(self):
        table = getattr(self, "task_region_table", None)
        if table is None:
            return
        regions = []
        try:
            overlay = self._preview_overlay_payload if isinstance(self._preview_overlay_payload, dict) else {}
            region_payload = overlay.get("regions", {}) if isinstance(overlay, dict) else {}
            work_regions = region_payload.get("work_regions", []) if isinstance(region_payload, dict) else []
            for r in list(work_regions or []):
                if not isinstance(r, dict):
                    continue
                rid = str(r.get("region_id", "") or "").strip()
                points = list(r.get("points", []) or [])
                if rid and len(points) >= 3:
                    regions.append(rid)
        except Exception:
            regions = []
        prev_selected_cfg = list(self._collect_selected_task_regions() or [])
        prev_selected = {rid: repeat for rid, repeat in prev_selected_cfg}
        prev_order = [rid for rid, _ in prev_selected_cfg]
        self._task_region_table_updating = True
        table.setRowCount(0)
        for rid in regions:
            row = table.rowCount()
            table.insertRow(row)
            item_check = QtWidgets.QTableWidgetItem("")
            item_check.setFlags(item_check.flags() | QtCore.Qt.ItemIsUserCheckable)
            item_check.setCheckState(QtCore.Qt.Checked if rid in prev_selected or not prev_selected else QtCore.Qt.Unchecked)
            table.setItem(row, 0, item_check)
            item_id = QtWidgets.QTableWidgetItem(rid)
            table.setItem(row, 1, item_id)
            spin = QtWidgets.QSpinBox()
            spin.setRange(1, 100)
            spin.setValue(int(max(1, int(prev_selected.get(rid, 1)))))
            table.setCellWidget(row, 2, spin)
            item_order = QtWidgets.QTableWidgetItem("")
            item_order.setFlags(item_order.flags() & ~QtCore.Qt.ItemIsEditable)
            table.setItem(row, 3, item_order)
        self._task_region_table_updating = False
        valid_region_set = set(regions)
        self._task_region_pick_order = [rid for rid in prev_order if rid in valid_region_set]
        self._refresh_task_region_order_labels()
        if not regions:
            self._append_log("任务区域列表为空：请先获取地图预览/区域信息")

    def _on_task_region_item_changed(self, item):
        if self._task_region_table_updating:
            return
        if item is None or int(item.column()) != 0:
            return
        row = int(item.row())
        table = self.task_region_table
        id_item = table.item(row, 1)
        if id_item is None:
            return
        rid = str(id_item.text() or "").strip()
        if not rid:
            return
        checked = item.checkState() == QtCore.Qt.Checked
        if checked:
            if rid not in self._task_region_pick_order:
                self._task_region_pick_order.append(rid)
        else:
            self._task_region_pick_order = [x for x in self._task_region_pick_order if x != rid]
        self._refresh_task_region_order_labels()

    def _send_map_sync_get(self):
        map_id = self.map_id_edit.text().strip()
        self._append_log(f"执行下载: 从 Aurora 下载并注册地图 map_id={map_id or '<auto>'}")
        req = pb.MapSyncRequest(
            operation=pb.MAP_SYNC_OP_DOWNLOAD_FROM_AURORA,
            map_id=map_id,
        )
        self._send(pb.MSG_ID_MAP_SYNC_REQUEST, pb.COMP_SCHEDULER, req)

    def _send_map_sync_set(self):
        map_id = self.map_id_edit.text().strip()
        if not map_id:
            self._append_log("执行上传失败: map_id 不能为空")
            return
        self._append_log(f"执行上传: 按 map_id 上传到 Aurora -> {map_id}")
        req = pb.MapSyncRequest(
            operation=pb.MAP_SYNC_OP_UPLOAD_TO_AURORA,
            map_id=map_id,
        )
        self._send(pb.MSG_ID_MAP_SYNC_REQUEST, pb.COMP_SCHEDULER, req)

    def _send_map_import_to_radar(self):
        if not hasattr(pb, "MSG_ID_MAP_IMPORT_TO_RADAR_REQUEST"):
            self._append_log("导入地图到雷达失败: 当前协议库不支持 MapImportToRadarRequest")
            return
        map_id = self.map_id_edit.text().strip()
        if not map_id:
            self._append_log("导入地图到雷达失败: map_id 不能为空")
            return
        self._append_log(f"发送 MapImportToRadarRequest: map_id={map_id}")
        self._append_reqresp(f"发送 MapImportToRadarRequest: map_id={map_id}")
        req = pb.MapImportToRadarRequest(map_id=map_id)
        self._send(pb.MSG_ID_MAP_IMPORT_TO_RADAR_REQUEST, pb.COMP_SCHEDULER, req)

    def _send_map_save_local(self):
        if not hasattr(pb, "MSG_ID_MAP_SAVE_REQUEST"):
            self._append_log("本地保存地图失败: 当前协议库未包含 MSG_ID_MAP_SAVE_REQUEST")
            return
        map_name = self.map_name_edit.text().strip()
        map_id = self.map_id_edit.text().strip()
        req = pb.MapSaveRequest(
            map_name=map_name,
            map_id=map_id,
        )
        self._append_log(
            "执行本地保存: 从雷达保存地图到本地 -> "
            f"map_name={map_name or '<auto>'} map_id={map_id or '<auto>'}"
        )
        self._send(pb.MSG_ID_MAP_SAVE_REQUEST, pb.COMP_SCHEDULER, req)

    def _local_map_dir(self):
        raw_path = self.sync_path.text().strip()
        candidate = raw_path
        if not candidate:
            candidate = os.path.join(REPO_ROOT, "maps/raw")
        if os.path.isfile(candidate):
            candidate = os.path.dirname(candidate)
        if not os.path.isabs(candidate):
            candidate = os.path.join(REPO_ROOT, candidate)
        return os.path.abspath(candidate)

    def _query_local_maps_from_fs(self):
        map_dir = self._local_map_dir()
        self.local_map_combo.clear()
        if not os.path.isdir(map_dir):
            self.local_map_combo.addItem("目录不存在", "")
            self.local_map_count_label.setText("地图数量: 0")
            self._append_log(f"本地地图查询失败: 目录不存在 -> {map_dir}")
            return
        try:
            names = []
            for name in os.listdir(map_dir):
                path = os.path.join(map_dir, name)
                if not os.path.isfile(path):
                    continue
                lower = name.lower()
                if lower.endswith(".stcm") or lower.endswith(".yaml"):
                    names.append(name)
            names.sort()
            if not names:
                self.local_map_combo.addItem("未找到地图文件", "")
            for name in names:
                abs_path = os.path.join(map_dir, name)
                self.local_map_combo.addItem(name, abs_path)
            self.local_map_count_label.setText(f"地图数量: {len(names)}")
            self._append_log(f"本地地图查询: dir={map_dir} count={len(names)} names={names}")
        except Exception as exc:
            self.local_map_combo.addItem("查询失败", "")
            self.local_map_count_label.setText("地图数量: 0")
            self._append_log(f"本地地图查询失败: {exc}")

    def _query_local_maps(self):
        # Always query LOWER-side recorded map catalog via protocol.
        self.local_map_combo.clear()
        self.local_map_combo.addItem("LIVE_MAP", DEFAULT_REQUEST_MAP_ID)
        if not getattr(self.client, "_running", False):
            self.local_map_combo.addItem("请先连接下位机", "")
            self.local_map_count_label.setText("地图数量: --")
            self._append_log("本地地图查询未发送：当前未连接下位机")
            return
        if not hasattr(pb, "MSG_ID_MAP_CATALOG_REQUEST"):
            self.local_map_combo.addItem("协议不支持MapCatalog", "")
            self.local_map_count_label.setText("地图数量: --")
            self._append_log("本地地图查询未发送：当前协议库不支持 MapCatalogRequest")
            return
        req = pb.MapCatalogRequest()
        self.local_map_combo.addItem("查询中...", "")
        self.local_map_count_label.setText("地图数量: --")
        self._append_log("发送 MapCatalogRequest")
        self._send(pb.MSG_ID_MAP_CATALOG_REQUEST, pb.COMP_SCHEDULER, req)

    def _on_local_map_selected(self, _index: int):
        map_id = self.local_map_combo.currentData()
        if not isinstance(map_id, str) or not map_id.strip():
            return
        self.map_id_edit.setText(map_id)
        self._append_log(f"已选择本地地图 map_id: {map_id}")

    def _delete_selected_local_map(self):
        if not getattr(self.client, "_running", False):
            msg = "删除地图未发送：当前未连接下位机"
            self._append_log(msg)
            self._append_reqresp(msg)
            return
        if not hasattr(pb, "MSG_ID_MAP_DELETE_REQUEST"):
            msg = "删除地图未发送：当前协议库不支持 MapDeleteRequest"
            self._append_log(msg)
            self._append_reqresp(msg)
            return
        map_id = self.local_map_combo.currentData()
        if not isinstance(map_id, str) or not map_id.strip():
            msg = "删除地图失败: 请先选择一个地图"
            self._append_log(msg)
            self._append_reqresp(msg)
            return
        name = self.local_map_combo.currentText().strip() or map_id
        answer = QtWidgets.QMessageBox.question(
            self,
            "删除地图",
            f"确认删除地图？\n{name}\nmap_id={map_id}",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            self._append_reqresp("删除地图已取消")
            return
        req = pb.MapDeleteRequest(map_id=map_id)
        self._append_log(f"发送 MapDeleteRequest: map_id={map_id}")
        self._append_reqresp(f"发送 MapDeleteRequest: map_id={map_id}")
        self._send(pb.MSG_ID_MAP_DELETE_REQUEST, pb.COMP_SCHEDULER, req)

    def _send_video_info(self):
        req = pb.VideoStreamInfoRequest(include_debug=True)
        self._send(pb.MSG_ID_VIDEO_STREAM_INFO_REQUEST, pb.COMP_SCHEDULER, req)

    def _send_live_map_cache_clear(self):
        if not getattr(self.client, "_running", False):
            msg = "清缓存未发送：当前未连接下位机"
            self._append_log(msg)
            self._append_reqresp(msg)
            return
        if not hasattr(pb, "MSG_ID_LIVE_MAP_CACHE_CLEAR_REQUEST"):
            msg = "清缓存未发送：当前协议库不支持 LiveMapCacheClearRequest"
            self._append_log(msg)
            self._append_reqresp(msg)
            return
        req = pb.LiveMapCacheClearRequest()
        self._append_log("发送 LiveMapCacheClearRequest")
        self._append_reqresp("发送 LiveMapCacheClearRequest")
        self._send(pb.MSG_ID_LIVE_MAP_CACHE_CLEAR_REQUEST, pb.COMP_SCHEDULER, req)

    def _send_radar_map_cache_clear(self):
        if not getattr(self.client, "_running", False):
            msg = "清雷达缓存未发送：当前未连接下位机"
            self._append_log(msg)
            self._append_reqresp(msg)
            return
        if not hasattr(pb, "MSG_ID_RADAR_MAP_CACHE_CLEAR_REQUEST"):
            msg = "清雷达缓存未发送：当前协议库不支持 RadarMapCacheClearRequest"
            self._append_log(msg)
            self._append_reqresp(msg)
            return
        req = pb.RadarMapCacheClearRequest()
        self._append_log("发送 RadarMapCacheClearRequest")
        self._append_reqresp("发送 RadarMapCacheClearRequest")
        self._send(pb.MSG_ID_RADAR_MAP_CACHE_CLEAR_REQUEST, pb.COMP_SCHEDULER, req)

    def _on_edit_target_type_changed(self, _index):
        type_value = int(self.edit_target_type.currentData())
        if type_value == int(getattr(pb, "REGION_TYPE_OBSTACLE", 2)):
            prefix = "obstacle_region_"
        elif type_value == int(getattr(pb, "REGION_TYPE_ERASE", 3)):
            prefix = "erase_region_"
        elif type_value == 4:
            prefix = "crop_region_"
        else:
            prefix = "work_region_"
        old_id = self.edit_region_id.text().strip()
        digits = ""
        for ch in reversed(old_id):
            if ch.isdigit():
                digits = ch + digits
            elif digits:
                break
        if not digits:
            digits = "1"
        self.edit_region_id.setText(f"{prefix}{digits}")

    def _parse_edit_points(self):
        text = self.edit_points.toPlainText().strip()
        if not text:
            return []
        points = []
        parts = text.replace("\n", ";").split(";")
        for part in parts:
            token = part.strip()
            if not token:
                continue
            items = [x.strip() for x in token.split(",")]
            if len(items) != 2:
                raise ValueError(f"点位格式错误: '{token}'，应为 x,y")
            points.append((float(items[0]), float(items[1])))
        return points

    def _parse_pose_text(self, text: str, field_name: str):
        token = (text or "").strip()
        if not token:
            return None, False
        items = [x.strip() for x in token.split(",") if x.strip()]
        if len(items) not in (2, 3):
            raise ValueError(f"{field_name} 格式错误: '{token}'，应为 x,y 或 x,y,heading_deg")
        x_val = float(items[0])
        y_val = float(items[1])
        has_heading = len(items) == 3
        heading_deg = float(items[2]) if has_heading else 0.0
        return (x_val, y_val, heading_deg), has_heading

    def _set_pose_edit_xy(self, field_name: str, x_val: float, y_val: float):
        target = self.path_start_pose_edit if field_name == "start_pose" else self.path_end_pose_edit
        current = (target.text() or "").strip()
        heading_tail = ""
        try:
            items = [x.strip() for x in current.split(",") if x.strip()]
            if len(items) >= 3:
                heading_tail = f",{float(items[2]):.1f}"
        except Exception:
            heading_tail = ""
        target.setText(f"{x_val:.3f},{y_val:.3f}{heading_tail}")

    def _start_pose_pick_mode(self, field_name: str):
        if field_name not in ("start_pose", "end_pose"):
            return
        self._preview_pick_mode = field_name
        if field_name == "start_pose":
            self._append_log("请在预览图上左键点击选择【起点】")
        else:
            self._append_log("请在预览图上左键点击选择【终点】")

    def _remember_task_id(self, task_id: str):
        tid = str(task_id or "").strip()
        if not tid:
            return
        idx = self.task_id_combo.findData(tid)
        if idx < 0:
            self.task_id_combo.addItem(tid, tid)
            idx = self.task_id_combo.findData(tid)
        if idx >= 0:
            self.task_id_combo.setCurrentIndex(idx)
        self.task_id.setText(tid)

    def _get_selected_task_id(self):
        combo_tid = self.task_id_combo.currentData()
        if isinstance(combo_tid, str) and combo_tid.strip():
            return combo_tid.strip()
        manual_tid = self.task_id.text().strip()
        return manual_tid or "task"

    def _get_selected_or_manual_map_id(self):
        selected_map_id = ""
        try:
            combo_map_id = self.local_map_combo.currentData()
            if isinstance(combo_map_id, str):
                selected_map_id = combo_map_id.strip()
        except Exception:
            selected_map_id = ""
        manual_map_id = self.map_id_edit.text().strip()
        return selected_map_id or manual_map_id or DEFAULT_REQUEST_MAP_ID

    def _build_map_edit_request(self, operation):
        req = pb.MapEditCommand()
        req.edit_id = self.edit_id.text().strip() or f"edit_{int(time.time())}"
        req.operation = int(operation)
        req.region_name = self.edit_region_name.text().strip()
        req.target_region_id = self.edit_region_id.text().strip()
        req.target_region_type = int(self.edit_target_type.currentData())
        req.brush_radius = float(self.edit_brush_radius.value())
        req.expected_map_version = 0
        req.dry_run = False
        if hasattr(req, "map_id"):
            req.map_id = self._get_selected_or_manual_map_id()
        if hasattr(req, "start_pose"):
            start_pose, start_has_heading = self._parse_pose_text(self.path_start_pose_edit.text(), "start_pose")
            if start_pose is not None:
                req.start_pose.x = float(start_pose[0])
                req.start_pose.y = float(start_pose[1])
                req.start_pose.heading_deg = float(start_pose[2] if start_has_heading else 0.0)
        if hasattr(req, "end_pose"):
            end_pose, end_has_heading = self._parse_pose_text(self.path_end_pose_edit.text(), "end_pose")
            if end_pose is not None:
                req.end_pose.x = float(end_pose[0])
                req.end_pose.y = float(end_pose[1])
                req.end_pose.heading_deg = float(end_pose[2] if end_has_heading else 0.0)
        return req

    def _apply_region_to_request(self, req, region_type_enum):
        req.region.name = self.edit_region_name.text().strip()
        req.region.region_id = self.edit_region_id.text().strip()
        req.region.priority = int(self.edit_order.value())
        req.region.enabled = True
        req.region.closed = True
        req.region.color_argb = 0
        req.region.region_type = int(region_type_enum)
        # Bind the region to the selected axis and first-line travel direction.
        if hasattr(req.region, "global_direction"):
            req.region.global_direction = (
                self.path_plan_direction_combo.currentText().strip().lower() or "x"
            )
        points = self._parse_edit_points()
        for x, y in points:
            p = req.region.points.add()
            p.x = float(x)
            p.y = float(y)

    def _apply_polygon_to_request(self, req):
        points = self._parse_edit_points()
        for x, y in points:
            p = req.polygon.add()
            p.x = float(x)
            p.y = float(y)

    def _send_edit_upsert_work(self):
        try:
            req = self._build_map_edit_request(pb.MAP_EDIT_OP_UPSERT_WORK_REGION)
            self._apply_region_to_request(req, pb.REGION_TYPE_WORK)
            self._append_log(
                f"发送 MapEdit UPSERT_WORK id={req.region.region_id} name={req.region.name} "
                f"points={len(req.region.points)} order={req.region.priority} "
                f"dir={getattr(req.region, 'global_direction', '<none>')}"
            )
            self._send(pb.MSG_ID_MAP_EDIT_COMMAND, pb.COMP_SCHEDULER, req)
            self._optimistic_upsert_region_preview(req.region)
            self._clear_edit_points_for_next_region()
        except Exception as exc:
            self._append_log(f"MapEdit UPSERT_WORK 失败: {exc}")

    def _send_edit_upsert_obstacle(self):
        try:
            req = self._build_map_edit_request(pb.MAP_EDIT_OP_UPSERT_OBSTACLE_REGION)
            self._apply_region_to_request(req, pb.REGION_TYPE_OBSTACLE)
            self._append_log(
                f"发送 MapEdit UPSERT_OBSTACLE id={req.region.region_id} name={req.region.name} "
                f"points={len(req.region.points)} order={req.region.priority}"
            )
            self._send(pb.MSG_ID_MAP_EDIT_COMMAND, pb.COMP_SCHEDULER, req)
            self._optimistic_upsert_region_preview(req.region)
            self._clear_edit_points_for_next_region()
        except Exception as exc:
            self._append_log(f"MapEdit UPSERT_OBSTACLE 失败: {exc}")

    def _send_edit_upsert_erase(self):
        try:
            req = self._build_map_edit_request(pb.MAP_EDIT_OP_UPSERT_ERASE_REGION)
            self._apply_region_to_request(req, int(pb.REGION_TYPE_ERASE))
            self._append_log(
                f"发送 MapEdit UPSERT_ERASE id={req.region.region_id} name={req.region.name} "
                f"points={len(req.region.points)} order={req.region.priority}"
            )
            self._send(pb.MSG_ID_MAP_EDIT_COMMAND, pb.COMP_SCHEDULER, req)
            self._optimistic_upsert_region_preview(req.region)
            self._clear_edit_points_for_next_region()
        except Exception as exc:
            self._append_log(f"MapEdit UPSERT_ERASE 失败: {exc}")

    def _send_edit_upsert_crop(self):
        try:
            req = self._build_map_edit_request(pb.MAP_EDIT_OP_UPSERT_CROP_REGION)
            self._apply_region_to_request(req, int(pb.REGION_TYPE_CROP))
            # Singleton crop region by convention.
            req.region.region_id = "crop_region_1"
            if not req.region.name:
                req.region.name = "crop_region"
            self._append_log(
                f"发送 MapEdit UPSERT_CROP id={req.region.region_id} name={req.region.name} "
                f"points={len(req.region.points)} (server will normalize to rectangle/singleton)"
            )
            self._send(pb.MSG_ID_MAP_EDIT_COMMAND, pb.COMP_SCHEDULER, req)
            self._optimistic_upsert_region_preview(req.region)
            self._clear_edit_points_for_next_region()
        except Exception as exc:
            self._append_log(f"MapEdit UPSERT_CROP 失败: {exc}")

    def _clear_edit_points_for_next_region(self):
        self._edit_selected_world_points = []
        self.edit_points.setPlainText("")
        self._render_preview_map_with_selection()
        self._append_log("已清空当前点位，准备下一次区域设置")

    def _optimistic_upsert_region_preview(self, region_pb):
        if not isinstance(self._preview_overlay_payload, dict):
            self._preview_overlay_payload = {}
        regions = self._preview_overlay_payload.get("regions")
        if not isinstance(regions, dict):
            regions = {"work_regions": [], "obstacle_regions": [], "crop_region": None}
            self._preview_overlay_payload["regions"] = regions
        work_regions = regions.get("work_regions")
        if not isinstance(work_regions, list):
            work_regions = []
            regions["work_regions"] = work_regions
        obstacle_regions = regions.get("obstacle_regions")
        if not isinstance(obstacle_regions, list):
            obstacle_regions = []
            regions["obstacle_regions"] = obstacle_regions
        crop_region = regions.get("crop_region")
        points = [{"x": float(p.x), "y": float(p.y)} for p in region_pb.points]
        if not points:
            return
        region_id = str(region_pb.region_id or "").strip()
        new_item = {
            "name": str(region_pb.name or "").strip(),
            "region_id": region_id,
            "priority": int(region_pb.priority),
            "enabled": bool(region_pb.enabled),
            "closed": bool(region_pb.closed),
            "color_argb": int(region_pb.color_argb),
            "region_type": int(region_pb.region_type) if int(region_pb.region_type) != 0 else int(pb.REGION_TYPE_WORK),
            "points": points,
        }
        region_type_value = int(region_pb.region_type) if int(region_pb.region_type) != 0 else int(pb.REGION_TYPE_WORK)
        if region_type_value == int(pb.REGION_TYPE_CROP):
            regions["crop_region"] = new_item
            self._render_preview_map_with_selection()
            self._refresh_task_region_table()
            return
        target_list = work_regions if region_type_value == int(pb.REGION_TYPE_WORK) else obstacle_regions
        replaced = False
        if region_id:
            for i, old in enumerate(target_list):
                if str(old.get("region_id", "")).strip() == region_id:
                    target_list[i] = new_item
                    replaced = True
                    break
        if not replaced:
            target_list.append(new_item)
        self._render_preview_map_with_selection()
        self._refresh_task_region_table()

    def _send_edit_delete_region(self):
        try:
            req = self._build_map_edit_request(pb.MAP_EDIT_OP_DELETE_REGION)
            selected = self.delete_region_combo.currentData()
            if isinstance(selected, dict) and selected.get("region_id"):
                req.target_region_id = str(selected.get("region_id", ""))
                req.target_region_type = int(selected.get("region_type", int(pb.REGION_TYPE_WORK)))
                if not req.region_name:
                    req.region_name = str(selected.get("region_name", ""))
            self._append_log(
                f"发送 MapEdit DELETE_REGION id={req.target_region_id} "
                f"name={req.region_name} type={req.target_region_type}"
            )
            self._send(pb.MSG_ID_MAP_EDIT_COMMAND, pb.COMP_SCHEDULER, req)
        except Exception as exc:
            self._append_log(f"MapEdit DELETE_REGION 失败: {exc}")

    def _send_edit_delete_crop_region(self):
        try:
            req = self._build_map_edit_request(pb.MAP_EDIT_OP_DELETE_REGION)
            req.target_region_id = "crop_region_1"
            req.target_region_type = int(pb.REGION_TYPE_CROP)
            self._append_log(f"发送 MapEdit DELETE_CROP id=crop_region_1 type={int(pb.REGION_TYPE_CROP)}")
            self._send(pb.MSG_ID_MAP_EDIT_COMMAND, pb.COMP_SCHEDULER, req)
            # Optimistically clear local preview crop overlay.
            if isinstance(self._preview_overlay_payload, dict):
                regions = self._preview_overlay_payload.get("regions")
                if isinstance(regions, dict) and "crop_region" in regions:
                    regions["crop_region"] = None
                    self._render_preview_map_with_selection()
                    self._refresh_task_region_table()
        except Exception as exc:
            self._append_log(f"MapEdit DELETE_CROP 失败: {exc}")

    def _send_edit_delete_all_regions(self):
        try:
            targets = []
            for i in range(self.delete_region_combo.count()):
                data = self.delete_region_combo.itemData(i)
                if not isinstance(data, dict):
                    continue
                rid = str(data.get("region_id", "") or "").strip()
                if not rid:
                    continue
                rtype = int(data.get("region_type", int(pb.REGION_TYPE_WORK)))
                targets.append((rid, rtype, str(data.get("region_name", "") or "").strip()))
            # Fallback source: current preview overlay payload.
            if isinstance(self._preview_overlay_payload, dict):
                regions = self._preview_overlay_payload.get("regions")
                if isinstance(regions, dict):
                    for r in regions.get("work_regions", []) or []:
                        rid = str((r or {}).get("region_id", "") or "").strip()
                        if rid:
                            targets.append((rid, int(pb.REGION_TYPE_WORK), str((r or {}).get("name", "") or "").strip()))
                    for r in regions.get("obstacle_regions", []) or []:
                        rid = str((r or {}).get("region_id", "") or "").strip()
                        if rid:
                            rtype = int((r or {}).get("region_type", int(pb.REGION_TYPE_OBSTACLE)))
                            if rtype <= 0:
                                rtype = int(pb.REGION_TYPE_OBSTACLE)
                            targets.append((rid, rtype, str((r or {}).get("name", "") or "").strip()))
                    crop = regions.get("crop_region")
                    if isinstance(crop, dict):
                        rid = str(crop.get("region_id", "") or "crop_region_1").strip()
                        targets.append((rid, int(pb.REGION_TYPE_CROP), str(crop.get("name", "") or "crop_region").strip()))
            # de-dup while preserving order
            uniq = []
            seen = set()
            for rid, rtype, rname in targets:
                key = (rid, rtype)
                if key in seen:
                    continue
                seen.add(key)
                uniq.append((rid, rtype, rname))

            if not uniq:
                self._append_log("一键删除所有区域: 当前无可删区域，请先点 区域查询")
                return

            self._append_log(f"一键删除所有区域: 即将删除 {len(uniq)} 个区域")
            for rid, rtype, rname in uniq:
                req = self._build_map_edit_request(pb.MAP_EDIT_OP_DELETE_REGION)
                req.target_region_id = rid
                req.target_region_type = int(rtype)
                if not req.region_name:
                    req.region_name = rname
                self._append_log(
                    f"发送 MapEdit DELETE_REGION id={req.target_region_id} "
                    f"name={req.region_name} type={req.target_region_type}"
                )
                self._send(pb.MSG_ID_MAP_EDIT_COMMAND, pb.COMP_SCHEDULER, req)

            # Crop region is singleton by convention; send once as cleanup.
            self._send_edit_delete_crop_region()
            # Optimistically clear local overlay and selector for instant feedback.
            if isinstance(self._preview_overlay_payload, dict):
                regions = self._preview_overlay_payload.get("regions")
                if isinstance(regions, dict):
                    regions["work_regions"] = []
                    regions["obstacle_regions"] = []
                    regions["crop_region"] = None
            self._update_delete_region_combo_from_settings(type("S", (), {"work_regions": [], "obstacle_regions": []})())
            self._render_map_preview()
            self._refresh_task_region_table()
            # Re-query to sync with scheduler source-of-truth.
            QtCore.QTimer.singleShot(300, self._send_settings_read_regions)
        except Exception as exc:
            self._append_log(f"一键删除所有区域 失败: {exc}")

    def _send_edit_paint(self, operation, op_name):
        try:
            req = self._build_map_edit_request(operation)
            self._apply_polygon_to_request(req)
            self._append_log(
                f"发送 MapEdit {op_name} polygon_points={len(req.polygon)} "
                f"brush={req.brush_radius:.3f}"
            )
            self._send(pb.MSG_ID_MAP_EDIT_COMMAND, pb.COMP_SCHEDULER, req)
        except Exception as exc:
            self._append_log(f"MapEdit {op_name} 失败: {exc}")

    def _send_map_mode(self, mode: int, enabled: bool):
        req = pb.MapModeRequest(mode=mode, enabled=bool(enabled), map_kind=0)
        mode_name = "MAPPING" if mode == pb.MAP_MODE_MAPPING else "LOCALIZATION"
        self._append_log(f"发送 MapModeRequest: mode={mode_name} enabled={enabled}")
        self._send(pb.MSG_ID_MAP_MODE_REQUEST, pb.COMP_SCHEDULER, req)

    def _send_remote(self, x: float, y: float):
        cmd = pb.ControlCommand()
        cmd.manual_drive.remote_x = float(x)
        cmd.manual_drive.remote_y = float(y)
        cmd.manual_drive.speed_ratio = float(self.remote_speed.value())
        self._send(pb.MSG_ID_CONTROL_COMMAND, pb.COMP_CONTROL, cmd)

    def _send_manual_motion(self, motion: int):
        cmd = pb.ControlCommand()
        cmd.manual_drive.motion = int(motion)
        cmd.manual_drive.speed_ratio = float(self.remote_speed.value())
        # Clear remote XY explicitly to avoid old receiver logic being affected.
        cmd.manual_drive.remote_x = 0.0
        cmd.manual_drive.remote_y = 0.0
        self._send(pb.MSG_ID_CONTROL_COMMAND, pb.COMP_CONTROL, cmd)

    def _send_chassis_power(self, enabled: bool):
        cmd = pb.ControlCommand()
        cmd.chassis_power.enabled = bool(enabled)
        self._send(pb.MSG_ID_CONTROL_COMMAND, pb.COMP_CONTROL, cmd)

    def _send_light(self, enabled: bool):
        cmd = pb.ControlCommand()
        cmd.lighting.enabled = bool(enabled)
        self._send(pb.MSG_ID_CONTROL_COMMAND, pb.COMP_CONTROL, cmd)

    def _send_disc_lift(self, up: bool):
        cmd = pb.ControlCommand()
        cmd.disc_lift.command = pb.DISC_LIFT_CMD_UP if up else pb.DISC_LIFT_CMD_DOWN
        self._send(pb.MSG_ID_CONTROL_COMMAND, pb.COMP_CONTROL, cmd)

    def _send_disc_enable(self, enabled: bool):
        cmd = pb.ControlCommand()
        # Compatibility fallback:
        # - New protocol: ControlCommand.disc_control
        # - Old protocol: ControlCommand.disc_enable
        if hasattr(cmd, "disc_control"):
            cmd.disc_control.enabled = bool(enabled)
            cmd.disc_control.speed_rpm = int(self.disc_speed_spin.value())
        elif hasattr(cmd, "disc_enable"):
            cmd.disc_enable.enabled = bool(enabled)
            self._append_log("协议较旧：已回退为 disc_enable（不带 speed_rpm）")
        else:
            self._append_log("错误：当前 sl_link_pb2 不支持磨盘开关字段，请重新生成 proto 代码")
            return
        self._send(pb.MSG_ID_CONTROL_COMMAND, pb.COMP_CONTROL, cmd)

    def _send_settings_read_chassis(self):
        req = pb.SettingsReadRequest(read_chassis=True, read_map=False)
        self._send(pb.MSG_ID_SETTINGS_READ_REQUEST, pb.COMP_SETTINGS, req)

    def _send_settings_read_regions(self):
        map_id = self._get_selected_or_manual_map_id()
        # SettingsReadRequest 本身不带 map_id。先发一个带 map_id 的轻量请求，
        # 让调度器切换到当前选中地图，再查询区域列表。
        try:
            probe = pb.MapMetricsRequest()
            if hasattr(probe, "map_id"):
                probe.map_id = map_id
            self._send(pb.MSG_ID_MAP_METRICS_REQUEST, pb.COMP_SCHEDULER, probe)
            self._append_log(f"区域查询前切图(MapMetricsRequest): map_id={map_id or '<empty>'}")
        except Exception as exc:
            self._append_log(f"区域查询前切图请求发送失败: {exc}")
        req = pb.SettingsReadRequest(read_chassis=False, read_map=True)
        self._append_log(f"发送 区域查询(SettingsRead): read_map=true map_id={map_id or '<empty>'}")
        self._send(pb.MSG_ID_SETTINGS_READ_REQUEST, pb.COMP_SETTINGS, req)

    def _send_settings_read_map_params(self):
        map_id = self._get_selected_or_manual_map_id()
        try:
            probe = pb.MapMetricsRequest()
            if hasattr(probe, "map_id"):
                probe.map_id = map_id
            self._send(pb.MSG_ID_MAP_METRICS_REQUEST, pb.COMP_SCHEDULER, probe)
            self._append_log(f"读地图参数前切图(MapMetricsRequest): map_id={map_id or '<empty>'}")
        except Exception as exc:
            self._append_log(f"读地图参数前切图请求发送失败: {exc}")
        req = pb.SettingsReadRequest(read_chassis=False, read_map=True)
        self._append_log(f"发送 读取机器人参数(SettingsRead): read_map=true map_id={map_id or '<empty>'}")
        self._send(pb.MSG_ID_SETTINGS_READ_REQUEST, pb.COMP_SETTINGS, req)

    def _on_delete_region_combo_changed(self, _index: int):
        data = self.delete_region_combo.currentData()
        if not isinstance(data, dict):
            return
        region_id = str(data.get("region_id", "") or "")
        region_name = str(data.get("region_name", "") or "")
        if region_id:
            self.edit_region_id.setText(region_id)
        if region_name:
            self.edit_region_name.setText(region_name)

    def _send_settings_write_chassis(self):
        req = pb.SettingsWriteRequest()
        req.chassis.run_speed = float(self.chassis_run_speed.value())
        req.chassis.disc_speed_rpm = int(self.disc_speed_spin.value())
        req.chassis.disc_enabled = bool(self.disc_enable_check.isChecked())
        req.chassis.work_mode = int(self.work_mode_combo.currentData())
        self._send(pb.MSG_ID_SETTINGS_WRITE_REQUEST, pb.COMP_SETTINGS, req)

    def _send_settings_write_map_params(self):
        req = pb.SettingsWriteRequest()
        req.map.vehicle_width = float(self.robot_vehicle_width.value())
        req.map.vehicle_length = float(self.robot_vehicle_length.value())
        req.map.default_path_spacing = float(self.robot_path_spacing.value())
        req.map.turn_radius = float(self.robot_turn_radius.value())
        req.map.overlap_ratio = float(self.robot_overlap_ratio.value())
        req.map.inflation_radius = float(self.robot_inflation_radius.value())
        self._append_log(
            "发送 应用机器人参数(SettingsWrite): "
            f"w={req.map.vehicle_width:.3f} l={req.map.vehicle_length:.3f} "
            f"spacing={req.map.default_path_spacing:.3f} turn={req.map.turn_radius:.3f} "
            f"overlap={req.map.overlap_ratio:.3f} inflation={req.map.inflation_radius:.3f}"
        )
        self._send(pb.MSG_ID_SETTINGS_WRITE_REQUEST, pb.COMP_SETTINGS, req)

    def _send_emergency_stop(self):
        # Prefer dedicated EmergencyStopControl in new protocol.
        cmd = pb.ControlCommand()
        if hasattr(cmd, "emergency_stop"):
            cmd.emergency_stop.enabled = True
            self._append_log("发送 紧急停止: emergency_stop.enabled=true")
            self._remote_x = 0.0
            self._remote_y = 0.0
            self.remote_xy.setText("x=+0.00  y=+0.00")
            self._remote_sent_nonzero = False
            self._send(pb.MSG_ID_CONTROL_COMMAND, pb.COMP_CONTROL, cmd)
            return

        # Backward compatibility for old proto:
        # stop wheel command first, then cut chassis enable.
        self._append_log("发送 紧急停止(兼容旧协议): 速度归零 + 底盘断使能")
        self._remote_x = 0.0
        self._remote_y = 0.0
        self.remote_xy.setText("x=+0.00  y=+0.00")
        self._remote_sent_nonzero = False
        self._send_remote(0.0, 0.0)
        self._send_chassis_power(False)

    def _on_joystick_value_changed(self, x: float, y: float):
        self._remote_x = float(x)
        self._remote_y = float(y)
        self.remote_xy.setText(f"x={self._remote_x:+.2f}  y={self._remote_y:+.2f}")

    def _on_remote_enable_changed(self, checked: bool):
        self._remote_send_enabled = bool(checked)
        if self._remote_send_enabled:
            self.remote_timer.start()
            self._append_log("遥感连续发送已开启")
        else:
            self.remote_timer.stop()
            self._append_log("遥感连续发送已关闭")

    def _send_remote_continuous(self):
        if not self._remote_send_enabled:
            return
        if not self.client._running:
            return
        moving = abs(self._remote_x) > 0.02 or abs(self._remote_y) > 0.02
        if moving:
            self._send_remote(self._remote_x, self._remote_y)
            self._remote_sent_nonzero = True
            return
        # 松手后仅补发一次停车包，避免空闲状态持续刷包
        if self._remote_sent_nonzero:
            self._send_remote(0.0, 0.0)
            self._remote_sent_nonzero = False

    def _send(self, msg_id, comp_id, proto_msg):
        try:
            self.client.send(msg_id, comp_id, proto_msg.SerializeToString())
        except Exception as exc:
            msg = f"Send failed: {exc}"
            self._append_log(msg)
            self._append_reqresp(msg)

    def _append_reqresp(self, text: str):
        line = f"[{time.strftime('%H:%M:%S')}] {text}"
        self._append_log_line(self.reqresp_log_text, line)

    def _append_log(self, text: str):
        line = f"[{time.strftime('%H:%M:%S')}] {text}"
        self._append_log_line(self.log_text, line)
        is_reqresp = (
            text.startswith("TX ")
            or text.startswith("RX ")
            or text.startswith("发送 ")
            or "Request" in text
            or "Response" in text
            or text.startswith("Socket closed by peer")
            or text.startswith("Disconnected")
            or text.startswith("RX error:")
        )
        # Filter out periodic auto reports to keep request log focused for troubleshooting.
        if (
            "MSG_ID_DEVICE_STATUS_REPORT" in text
            or "MSG_ID_TASK_STATUS_REPORT" in text
            or text.startswith("DeviceStatus ")
            or text.startswith("TaskStatus ")
        ):
            is_reqresp = False
        if is_reqresp:
            self._append_log_line(self.reqresp_log_text, line)

    def _append_log_line(self, widget: QtWidgets.QPlainTextEdit, line: str):
        widget.appendPlainText(line)
        sb = widget.verticalScrollBar()
        sb.setValue(sb.maximum())

    def eventFilter(self, obj, event):
        map_label = getattr(self, "map_image_label", None)
        path_label = getattr(self, "path_image_label", None)
        raw_label = getattr(self, "raw_map_image_label", None)
        if obj is map_label and event is not None:
            if event.type() == QtCore.QEvent.MouseButtonPress:
                if event.button() == QtCore.Qt.LeftButton:
                    if self._preview_pick_mode in ("start_pose", "end_pose"):
                        world_point = self._extract_world_point_from_preview_click(event.pos())
                        if world_point is not None:
                            self._set_pose_edit_xy(self._preview_pick_mode, world_point["x"], world_point["y"])
                            self._append_log(
                                f"已选择{('起点' if self._preview_pick_mode == 'start_pose' else '终点')}: "
                                f"({world_point['x']:.3f},{world_point['y']:.3f})"
                            )
                            self._preview_pick_mode = None
                            return True
                    if self._append_world_point_from_preview_click(event.pos()):
                        return True
                elif event.button() == QtCore.Qt.RightButton:
                    if self._edit_selected_world_points:
                        self._edit_selected_world_points.pop()
                        self._sync_edit_points_from_selection()
                        self._render_preview_map_with_selection()
                        self._append_log("预览地图选点: 撤销最后一个点")
                        return True
            elif event.type() == QtCore.QEvent.MouseButtonDblClick:
                if event.button() == QtCore.Qt.LeftButton:
                    self._edit_selected_world_points = []
                    self._sync_edit_points_from_selection()
                    self._render_preview_map_with_selection()
                    self._append_log("预览地图选点: 已清空")
                    return True
        elif obj is path_label and event is not None:
            if event.type() == QtCore.QEvent.Resize:
                self._render_path_preview()
        elif getattr(self, "task_result_image_label", None) is not None and obj is self.task_result_image_label and event is not None:
            if event.type() == QtCore.QEvent.Resize:
                self._render_task_result_preview()
        elif obj is raw_label and event is not None:
            if event.type() == QtCore.QEvent.Resize:
                self._render_raw_map_preview()
        return super().eventFilter(obj, event)

    def _render_path_preview(self):
        if self._path_preview_base_pixmap is None or self._path_preview_base_pixmap.isNull():
            return
        self._adjust_label_aspect(
            self.path_image_label,
            max(1, self._path_preview_base_pixmap.width()),
            max(1, self._path_preview_base_pixmap.height()),
        )
        scaled = self._path_preview_base_pixmap.scaled(
            self.path_image_label.size(),
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.FastTransformation,
        )
        self.path_image_label.setPixmap(scaled)

    def _render_raw_map_preview(self):
        if self._raw_map_base_pixmap is None or self._raw_map_base_pixmap.isNull():
            return
        self._adjust_label_aspect(
            self.raw_map_image_label,
            max(1, self._raw_map_base_pixmap.width()),
            max(1, self._raw_map_base_pixmap.height()),
        )
        scaled = self._raw_map_base_pixmap.scaled(
            self.raw_map_image_label.size(),
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.FastTransformation,
        )
        self.raw_map_image_label.setPixmap(scaled)

    def _render_task_result_preview(self):
        if self._task_result_base_pixmap is None or self._task_result_base_pixmap.isNull():
            return
        self._adjust_label_aspect(
            self.task_result_image_label,
            max(1, self._task_result_base_pixmap.width()),
            max(1, self._task_result_base_pixmap.height()),
        )
        scaled = self._task_result_base_pixmap.scaled(
            self.task_result_image_label.size(),
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.FastTransformation,
        )
        self.task_result_image_label.setPixmap(scaled)

    def _append_world_point_from_preview_click(self, pos):
        world_point = self._extract_world_point_from_preview_click(pos)
        if world_point is None:
            return False
        world_x = world_point["x"]
        world_y = world_point["y"]
        col = world_point["col"]
        display_row = world_point["display_row"]
        self._edit_selected_world_points.append({"x": world_x, "y": world_y})
        self._sync_edit_points_from_selection()
        self._render_preview_map_with_selection()
        self._append_log(
            f"预览地图选点: pixel=({col},{display_row}) world=({world_x:.3f},{world_y:.3f}) total={len(self._edit_selected_world_points)}"
        )
        return True

    def _extract_world_point_from_preview_click(self, pos):
        if self._preview_map_meta is None:
            self._append_log("预览地图选点失败: 尚未收到 MapPreview 地图")
            return None
        pixmap = self.map_image_label.pixmap()
        if pixmap is None or pixmap.isNull():
            return None
        label_size = self.map_image_label.size()
        pix_w = pixmap.width()
        pix_h = pixmap.height()
        if pix_w <= 0 or pix_h <= 0:
            return None
        off_x = max(0.0, (label_size.width() - pix_w) / 2.0)
        off_y = max(0.0, (label_size.height() - pix_h) / 2.0)
        x = float(pos.x())
        y = float(pos.y())
        if x < off_x or x >= off_x + pix_w or y < off_y or y >= off_y + pix_h:
            return None

        u = (x - off_x) / float(pix_w)
        v = (y - off_y) / float(pix_h)
        width = int(self._preview_map_meta.get("width", 0))
        height = int(self._preview_map_meta.get("height", 0))
        resolution = float(self._preview_map_meta.get("resolution", 0.0))
        origin_x = float(self._preview_map_meta.get("origin_x", 0.0))
        origin_y = float(self._preview_map_meta.get("origin_y", 0.0))
        if width <= 0 or height <= 0 or resolution <= 0:
            self._append_log("预览地图选点失败: 地图元数据无效")
            return None

        col = max(0, min(width - 1, int(round(u * (width - 1)))))
        display_row = max(0, min(height - 1, int(round(v * (height - 1)))))
        row = (height - 1) - display_row
        world_x = origin_x + float(col) * resolution
        world_y = origin_y + float(row) * resolution
        return {"x": world_x, "y": world_y, "col": col, "display_row": display_row}

    def _sync_edit_points_from_selection(self):
        if not self._edit_selected_world_points:
            self.edit_points.setPlainText("")
            return
        text = "; ".join(f"{p['x']:.3f},{p['y']:.3f}" for p in self._edit_selected_world_points)
        self.edit_points.setPlainText(text)

    def _render_preview_map_with_selection(self):
        if self._preview_map_base_pixmap is None or self._preview_map_base_pixmap.isNull():
            return
        draw = QtGui.QPixmap(self._preview_map_base_pixmap)
        if self._preview_map_meta and (not self.map_fast_mode.isChecked()):
            width = int(self._preview_map_meta.get("width", 0))
            height = int(self._preview_map_meta.get("height", 0))
            resolution = float(self._preview_map_meta.get("resolution", 0.0))
            origin_x = float(self._preview_map_meta.get("origin_x", 0.0))
            origin_y = float(self._preview_map_meta.get("origin_y", 0.0))
            if width > 0 and height > 0 and resolution > 0:
                painter = QtGui.QPainter(draw)
                painter.setRenderHint(QtGui.QPainter.Antialiasing, True)

                def _world_to_screen_xy(wx, wy):
                    col = (float(wx) - origin_x) / resolution
                    row = (float(wy) - origin_y) / resolution
                    display_row = (height - 1) - row
                    px = (col / max(1, (width - 1))) * draw.width()
                    py = (display_row / max(1, (height - 1))) * draw.height()
                    return QtCore.QPointF(px, py)

                def _draw_regions(region_list, pen_color, fill_alpha=0):
                    if not region_list:
                        return
                    painter.setPen(QtGui.QPen(pen_color, 1))
                    if fill_alpha > 0:
                        brush = QtGui.QBrush(QtGui.QColor(pen_color.red(), pen_color.green(), pen_color.blue(), fill_alpha))
                    else:
                        brush = QtCore.Qt.NoBrush
                    painter.setBrush(brush)
                    for region in region_list:
                        points = region.get("points", [])
                        if len(points) < 2:
                            continue
                        poly = QtGui.QPolygonF()
                        for pt in points:
                            poly.append(_world_to_screen_xy(pt.get("x", 0.0), pt.get("y", 0.0)))
                        if len(poly) >= 3:
                            painter.drawPolygon(poly)
                        else:
                            painter.drawPolyline(poly)

                regions = self._preview_overlay_payload.get("regions", {}) if isinstance(self._preview_overlay_payload, dict) else {}
                work_regions = regions.get("work_regions", []) if isinstance(regions, dict) else []
                obstacle_regions_all = regions.get("obstacle_regions", []) if isinstance(regions, dict) else []
                crop_region = regions.get("crop_region") if isinstance(regions, dict) else None
                obstacle_regions = []
                erase_regions = []
                for region in work_regions + obstacle_regions_all:
                    try:
                        if int(region.get("region_type", 0)) == int(getattr(pb, "REGION_TYPE_ERASE", 3)):
                            erase_regions.append(region)
                        else:
                            if region in obstacle_regions_all:
                                obstacle_regions.append(region)
                    except Exception:
                        if region in obstacle_regions_all:
                            obstacle_regions.append(region)

                # Keep preview clean: draw region borders only, do not paint filled masks.
                _draw_regions(work_regions, QtGui.QColor(0, 210, 80), fill_alpha=0)
                _draw_regions(obstacle_regions, QtGui.QColor(230, 50, 50), fill_alpha=0)
                _draw_regions(erase_regions, QtGui.QColor(255, 180, 0), fill_alpha=0)
                if isinstance(crop_region, dict):
                    _draw_regions([crop_region], QtGui.QColor(0, 220, 255), fill_alpha=0)

                if self._edit_selected_world_points:
                    pen = QtGui.QPen(QtGui.QColor(0, 220, 255), 1)
                    painter.setPen(pen)
                    painter.setBrush(QtGui.QBrush(QtGui.QColor(255, 180, 0)))
                    screen_points = []
                    for p in self._edit_selected_world_points:
                        qpt = _world_to_screen_xy(p["x"], p["y"])
                        screen_points.append(qpt)
                        painter.drawEllipse(qpt, 2, 2)
                    if len(screen_points) >= 2:
                        for i in range(len(screen_points) - 1):
                            painter.drawLine(screen_points[i], screen_points[i + 1])
                painter.end()

        self._adjust_label_aspect(
            self.map_image_label,
            max(1, draw.width()),
            max(1, draw.height()),
        )
        scaled = draw.scaled(
            self.map_image_label.size(),
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.FastTransformation,
        )
        self.map_image_label.setPixmap(scaled)

    def _adjust_label_aspect(self, label: QtWidgets.QLabel, image_width: int, image_height: int):
        if image_width <= 0 or image_height <= 0:
            return
        target_ratio = float(image_height) / float(image_width)
        current_width = max(1, label.width())
        target_height = int(round(current_width * target_ratio))
        target_height = max(260, min(620, target_height))
        # Keep adaptive but avoid forcing overly large minimum heights on small/HiDPI screens.
        label.setMinimumHeight(220)
        label.setMaximumHeight(target_height)

    def _show_map_preview(self, image_data: bytes, map_resp):
        self._preview_render_token += 1
        token = int(self._preview_render_token)
        self.btn_map_preview.setEnabled(True)
        pixmap = QtGui.QPixmap()
        if not pixmap.loadFromData(image_data):
            self._append_log("MapPreview 图片解码失败")
            return
        self._preview_map_base_pixmap = pixmap
        self._preview_map_meta = {
            "width": int(map_resp.width),
            "height": int(map_resp.height),
            "resolution": float(map_resp.resolution),
            "origin_x": float(map_resp.origin.x),
            "origin_y": float(map_resp.origin.y),
        }
        self._preview_overlay_payload = {}
        self._preview_overlay_mask_bytes = b""
        if self.map_fast_mode.isChecked():
            pass
        elif map_resp.overlay_json:
            overlay_len = len(map_resp.overlay_json.encode("utf-8"))
            if overlay_len > 450000:
                self._append_log(f"overlay_json较大({overlay_len} bytes)，跳过解析避免界面卡顿")
            else:
                self._preview_overlay_parse_token += 1
                overlay_token = int(self._preview_overlay_parse_token)
                threading.Thread(
                    target=self._parse_overlay_async,
                    args=(overlay_token, token, map_resp.overlay_json),
                    daemon=True,
                ).start()
        self._render_preview_map_with_selection()
        overlay_size = len(map_resp.overlay_json.encode("utf-8")) if map_resp.overlay_json else 0
        info_lines = [
            f"result: {map_resp.result}",
            f"message: {map_resp.message}",
            f"map_version: {map_resp.map_version}",
            f"width x height: {map_resp.width} x {map_resp.height}",
            f"resolution: {map_resp.resolution:.6f} m/cell",
            f"origin: ({map_resp.origin.x:.6f}, {map_resp.origin.y:.6f}, {map_resp.origin.heading_deg:.3f})",
            f"frame_id: {map_resp.frame_id}",
            f"preview_scale: ({float(getattr(map_resp, 'preview_scale_x', 0.0)):.6f}, "
            f"{float(getattr(map_resp, 'preview_scale_y', 0.0)):.6f})",
            f"image_bytes: {len(image_data)}",
            f"overlay_json_bytes: {overlay_size}",
        ]
        self.map_info_text.setPlainText("\n".join(info_lines))
        self._last_preview_origin_x = float(map_resp.origin.x)
        self._last_preview_origin_y = float(map_resp.origin.y)
        self._last_preview_resolution = float(map_resp.resolution)
        self._last_preview_width = int(map_resp.width)
        self._last_preview_height = int(map_resp.height)

    def _on_map_fast_mode_toggled(self, checked: bool):
        if checked:
            self._append_log("极速模式已开启: 仅显示地图原图，跳过叠层渲染")
        else:
            self._append_log("极速模式已关闭: 恢复叠层渲染")
        self._render_preview_map_with_selection()

    def _parse_overlay_async(self, token: int, render_token: int, overlay_json_text: str):
        payload = {}
        mask_bytes = b""
        try:
            decoded = json.loads(overlay_json_text)
            if isinstance(decoded, dict):
                payload = decoded
                mask_b64 = decoded.get("overlay_mask_png_base64", "")
                if mask_b64 and len(mask_b64) <= 180000:
                    try:
                        mask_bytes = base64.b64decode(mask_b64)
                    except Exception:
                        mask_bytes = b""
        except Exception:
            payload = {}
            mask_bytes = b""
        self.overlay_parsed.emit((token << 16) | (render_token & 0xFFFF), payload, mask_bytes)

    def _on_overlay_parsed(self, token: int, payload, mask_bytes):
        parse_token = int(token) >> 16
        render_token = int(token) & 0xFFFF
        if parse_token != int(self._preview_overlay_parse_token):
            return
        if render_token != int(self._preview_render_token):
            return
        self._preview_overlay_payload = payload if isinstance(payload, dict) else {}
        self._preview_overlay_mask_bytes = mask_bytes if isinstance(mask_bytes, (bytes, bytearray)) else b""
        self._render_preview_map_with_selection()
        self._refresh_task_region_table()

    def _show_raw_map_chunk(self, map_chunk, full_bytes: bytes):
        preview_bytes = None
        preview_fmt = "jpg"
        if map_chunk.encoding == pb.MAP_ENCODING_OCCUPANCY_GRID:
            if cv2 is None or np is None:
                self.raw_map_info_text.setPlainText("MapRequest 失败: 缺少 numpy/cv2 依赖")
                self._append_log("MapRequest OccupancyGrid decode skipped: numpy/cv2 not installed")
                return
            expected = int(map_chunk.width) * int(map_chunk.height)
            if len(full_bytes) < expected:
                self._append_log(f"MapChunk 数据长度不足: got={len(full_bytes)} expected={expected}")
                self.raw_map_info_text.setPlainText(
                    "\n".join(
                        [
                            "MapRequest 失败",
                            f"encoding: {map_chunk.encoding}",
                            f"width x height: {map_chunk.width} x {map_chunk.height}",
                            f"expected_bytes: {expected}",
                            f"got_bytes: {len(full_bytes)}",
                        ]
                    )
                )
                return
            # Compatibility decode:
            # - Free: 0
            # - Occupied: 100 (or >=100 and <255)
            # - Unknown: -1 (int8) or 255 (uint8)
            raw = np.frombuffer(full_bytes[:expected], dtype=np.uint8).reshape((int(map_chunk.height), int(map_chunk.width)))
            image = np.zeros((raw.shape[0], raw.shape[1], 3), dtype=np.uint8)
            image[:, :] = (180, 180, 180)  # unknown default
            image[raw == 0] = (245, 245, 245)
            image[(raw >= 100) & (raw < 255)] = (45, 45, 45)
            image[raw == 255] = (180, 180, 180)
            image = cv2.flip(image, 0)
            ok, buf = cv2.imencode(".jpg", image)
            if not ok:
                self.raw_map_info_text.setPlainText("MapRequest 失败: OccupancyGrid 转图片失败")
                return
            preview_bytes = buf.tobytes()
            preview_fmt = "jpg"
        elif map_chunk.encoding == pb.MAP_ENCODING_PNG:
            preview_bytes = full_bytes
            preview_fmt = "png"
        else:
            # JSON or unknown encoding: only display info
            self.raw_map_info_text.setPlainText(
                "\n".join(
                    [
                        "MapRequest 已完成",
                        f"encoding: {map_chunk.encoding}",
                        f"map_id: {map_chunk.map_id}",
                        f"width x height: {map_chunk.width} x {map_chunk.height}",
                        f"resolution: {map_chunk.resolution:.6f}",
                        f"origin: ({float(getattr(map_chunk, 'origin', pb.Pose2D()).x):.6f}, "
                        f"{float(getattr(map_chunk, 'origin', pb.Pose2D()).y):.6f}, "
                        f"{float(getattr(map_chunk, 'origin', pb.Pose2D()).heading_deg):.3f})",
                        f"frame_id: {getattr(map_chunk, 'frame_id', '')}",
                        f"preview_scale: ({float(getattr(map_chunk, 'preview_scale_x', 0.0)):.6f}, "
                        f"{float(getattr(map_chunk, 'preview_scale_y', 0.0)):.6f})",
                        f"data_bytes: {len(full_bytes)}",
                    ]
                )
            )
            return

        pixmap = QtGui.QPixmap()
        if pixmap.loadFromData(preview_bytes):
            self._raw_map_base_pixmap = pixmap
            self._render_raw_map_preview()

        self.raw_map_info_text.setPlainText(
            "\n".join(
                [
                    "MapRequest 已完成",
                    f"encoding: {map_chunk.encoding}",
                    f"map_id: {map_chunk.map_id}",
                    f"utc_time: {map_chunk.utc_time}",
                    f"width x height: {map_chunk.width} x {map_chunk.height}",
                    f"resolution: {map_chunk.resolution:.6f}",
                    f"origin: ({float(getattr(map_chunk, 'origin', pb.Pose2D()).x):.6f}, "
                    f"{float(getattr(map_chunk, 'origin', pb.Pose2D()).y):.6f}, "
                    f"{float(getattr(map_chunk, 'origin', pb.Pose2D()).heading_deg):.3f})",
                    f"frame_id: {getattr(map_chunk, 'frame_id', '')}",
                    f"preview_scale: ({float(getattr(map_chunk, 'preview_scale_x', 0.0)):.6f}, "
                    f"{float(getattr(map_chunk, 'preview_scale_y', 0.0)):.6f})",
                    f"data_bytes: {len(full_bytes)}",
                ]
            )
        )
        self._save_blob("map_request", preview_fmt, preview_bytes)

    def _show_path_preview(self, image_data: bytes, path_resp):
        started = time.monotonic()
        pixmap = QtGui.QPixmap()
        if not pixmap.loadFromData(image_data):
            self._append_log("PathPreview 图片解码失败")
            return
        decode_ms = (time.monotonic() - started) * 1000.0
        render_started = time.monotonic()
        self._path_preview_base_pixmap = pixmap
        self._render_path_preview()
        render_ms = (time.monotonic() - render_started) * 1000.0
        info_started = time.monotonic()
        info_lines = [
            f"result: {getattr(path_resp, 'result', 0)}",
            f"message: {getattr(path_resp, 'message', '')}",
            f"request_id: {getattr(path_resp, 'request_id', '')}",
            f"task_id: {getattr(path_resp, 'task_id', '')}",
            f"planned: {getattr(path_resp, 'planned', False)}",
            f"map_version: {getattr(path_resp, 'map_version', 0)}",
            f"width x height: {getattr(path_resp, 'width', 0)} x {getattr(path_resp, 'height', 0)}",
            f"resolution: {float(getattr(path_resp, 'resolution', 0.0)):.6f} m/cell",
            f"origin: ({float(getattr(getattr(path_resp, 'origin', pb.Pose2D()), 'x', 0.0)):.6f}, "
            f"{float(getattr(getattr(path_resp, 'origin', pb.Pose2D()), 'y', 0.0)):.6f}, "
            f"{float(getattr(getattr(path_resp, 'origin', pb.Pose2D()), 'heading_deg', 0.0)):.3f})",
            f"frame_id: {getattr(path_resp, 'frame_id', '')}",
            f"preview_scale: ({float(getattr(path_resp, 'preview_scale_x', 0.0)):.6f}, "
            f"{float(getattr(path_resp, 'preview_scale_y', 0.0)):.6f})",
            f"path_version: {getattr(path_resp, 'path_version', 0)}",
            f"path_point_count: {getattr(path_resp, 'path_point_count', 0)}",
            f"path_length_m: {float(getattr(path_resp, 'path_length_m', 0.0)):.6f}",
            f"total_work_area_m2: {float(getattr(path_resp, 'total_work_area_m2', 0.0)):.6f}",
            (
                f"estimated_time_s: {float(getattr(path_resp, 'estimated_time_s', -1.0)):.3f}"
                if float(getattr(path_resp, "estimated_time_s", -1.0)) >= 0.0
                else "estimated_time_s: -1"
            ),
            f"preview_format: {path_resp.preview_format}",
            f"preview_bytes: {len(image_data)}",
        ]
        self.path_info_text.setPlainText("\n".join(info_lines))
        info_ms = (time.monotonic() - info_started) * 1000.0
        total_ms = (time.monotonic() - started) * 1000.0
        self._append_log(
            f"PathPreview UI perf: total={total_ms:.1f}ms decode={decode_ms:.1f}ms "
            f"render={render_ms:.1f}ms info={info_ms:.1f}ms bytes={len(image_data)}"
        )

    def _show_task_result_preview(self, image_data: bytes, task_resp):
        pixmap = QtGui.QPixmap()
        if not pixmap.loadFromData(image_data):
            self._append_log("TaskResult 图片解码失败")
            return
        self._task_result_base_pixmap = pixmap
        self._render_task_result_preview()
        lines = [
            f"result: {task_resp.result}",
            f"message: {task_resp.message}",
            f"map_id: {task_resp.map_id}",
            f"task_id: {task_resp.task_id}",
            f"final_state: {int(getattr(task_resp, 'final_state', 0))}",
            f"all_completed: {bool(getattr(task_resp, 'all_completed', False))}",
            f"stop_reason: {getattr(task_resp, 'stop_reason', '')}",
            f"path_version: {int(getattr(task_resp, 'path_version', 0))}",
            f"finished_at: {int(getattr(task_resp, 'finished_at', 0))}",
            f"image_format: {getattr(task_resp, 'image_format', '')}",
            f"image_size: {int(getattr(task_resp, 'image_width', 0))}x{int(getattr(task_resp, 'image_height', 0))}",
            f"selected_work_region_ids: {', '.join(list(getattr(task_resp, 'selected_work_region_ids', []) or []))}",
            "region_results:",
        ]
        for item in list(getattr(task_resp, "region_results", []) or []):
            lines.append(
                f"- {item.region_id}({item.region_name}) target={int(item.target_repeat)} "
                f"executed={int(item.executed_repeat)} completed={bool(item.completed)} "
                f"reason={item.unfinished_reason}"
            )
        self.task_result_info_text.setPlainText("\n".join(lines))

    def _save_blob(self, prefix: str, ext: str, data: bytes):
        self._save_blob_async(prefix, ext, data)

    def _save_blob_async(self, prefix: str, ext: str, data: bytes):
        folder = self.save_dir.text().strip() or "/tmp"
        payload = bytes(data)
        filename = f"{prefix}_latest.{ext}"

        def _worker():
            started = time.monotonic()
            try:
                os.makedirs(folder, exist_ok=True)
                path = os.path.join(folder, filename)
                with open(path, "wb") as f:
                    f.write(payload)
                elapsed_ms = (time.monotonic() - started) * 1000.0
                self.async_log.emit(f"Saved {len(payload)} bytes -> {path} save_ms={elapsed_ms:.1f}")
            except Exception as exc:
                self.async_log.emit(f"Save failed: prefix={prefix} ext={ext} err={exc}")

        threading.Thread(target=_worker, daemon=True).start()

    def _format_region_lines(self, regions, title: str):
        lines = [f"{title}: {len(regions)}"]
        for idx, r in enumerate(regions):
            rid = str(getattr(r, "region_id", "") or "")
            name = str(getattr(r, "name", "") or "")
            rtype = int(getattr(r, "region_type", 0))
            prio = int(getattr(r, "priority", 0))
            enabled = bool(getattr(r, "enabled", False))
            closed = bool(getattr(r, "closed", False))
            points_cnt = len(getattr(r, "points", []))
            lines.append(
                f"  [{idx}] id={rid} name={name} type={rtype} "
                f"priority={prio} enabled={enabled} closed={closed} points={points_cnt}"
            )
        return lines

    def _update_delete_region_combo_from_settings(self, map_settings):
        items = []
        for r in list(getattr(map_settings, "work_regions", [])):
            region_id = str(getattr(r, "region_id", "") or "")
            if not region_id:
                continue
            items.append(
                {
                    "region_id": region_id,
                    "region_name": str(getattr(r, "name", "") or ""),
                    "region_type": int(pb.REGION_TYPE_WORK),
                    "points": len(getattr(r, "points", [])),
                }
            )
        for r in list(getattr(map_settings, "obstacle_regions", [])):
            region_id = str(getattr(r, "region_id", "") or "")
            if not region_id:
                continue
            region_type = int(getattr(r, "region_type", int(pb.REGION_TYPE_OBSTACLE)))
            if region_type <= 0:
                region_type = int(pb.REGION_TYPE_OBSTACLE)
            items.append(
                {
                    "region_id": region_id,
                    "region_name": str(getattr(r, "name", "") or ""),
                    "region_type": region_type,
                    "points": len(getattr(r, "points", [])),
                }
            )

        type_name = {
            int(pb.REGION_TYPE_WORK): "WORK",
            int(pb.REGION_TYPE_OBSTACLE): "OBSTACLE",
            int(getattr(pb, "REGION_TYPE_ERASE", 3)): "ERASE",
            4: "CROP",
        }
        self.delete_region_combo.blockSignals(True)
        self.delete_region_combo.clear()
        if not items:
            self.delete_region_combo.addItem("查询结果为空", None)
        else:
            self.delete_region_combo.addItem("请选择要删除的区域", None)
            for item in items:
                label = (
                    f"[{type_name.get(int(item['region_type']), str(item['region_type']))}] "
                    f"{item['region_id']} ({item['region_name']}) pts={item['points']}"
                )
                self.delete_region_combo.addItem(label, item)
        self.delete_region_combo.blockSignals(False)

    def _on_frame(self, frame):
        name = MSG_NAMES.get(frame.msg_id, hex(frame.msg_id))
        recv_mono = getattr(frame, "_recv_monotonic", None)
        recv_wall = getattr(frame, "_recv_wall", None)
        if recv_mono is not None:
            ui_delay_ms = max(0.0, (time.monotonic() - float(recv_mono)) * 1000.0)
            recv_time_text = (
                datetime.datetime.fromtimestamp(float(recv_wall)).strftime("%H:%M:%S.%f")[:-3]
                if recv_wall is not None
                else "--:--:--.---"
            )
            self._append_log(
                f"RX seq={frame.seq} {name} len={len(frame.payload)} "
                f"recv_at={recv_time_text} ui_delay={ui_delay_ms:.1f}ms"
            )
        else:
            self._append_log(f"RX seq={frame.seq} {name} len={len(frame.payload)}")
        try:
            if frame.msg_id == pb.MSG_ID_DEVICE_STATUS_REPORT:
                m = pb.DeviceStatusReport()
                m.ParseFromString(frame.payload)
                self._set_robot_pose_display(
                    float(getattr(m.position, "x", 0.0)),
                    float(getattr(m.position, "y", 0.0)),
                    float(getattr(m.position, "heading_deg", 0.0)),
                )
                self._append_log(
                    f"DeviceStatus pos=({m.position.x:.3f},{m.position.y:.3f},{m.position.heading_deg:.1f}) "
                    f"disc={m.disc_enabled} light={m.light_enabled} chassis={m.chassis_enabled}"
                )
            elif frame.msg_id == pb.MSG_ID_TASK_STATUS_REPORT:
                m = pb.TaskStatusReport()
                m.ParseFromString(frame.payload)
                self._remember_task_id(getattr(m, "task_id", ""))
                self._update_task_status_panel(m)
                total_area = float(getattr(m, "total_work_area_m2", 0.0))
                remaining_area = float(getattr(m, "remaining_work_area_m2", 0.0))
                remaining_time_s = float(getattr(m, "remaining_time_s", -1.0))
                if remaining_time_s >= 0.0:
                    eta_text = f"{remaining_time_s:.1f}s"
                else:
                    eta_text = "--"
                self._append_log(
                    f"TaskStatus state={m.state} progress={m.progress:.3f} map_v={m.map_version} "
                    f"path_v={m.path_version} points={m.path_point_count} "
                    f"area={total_area:.2f}/{remaining_area:.2f} eta={eta_text} msg={m.message}"
                )
            elif hasattr(pb, "MSG_ID_TASK_COMMAND_RESPONSE") and frame.msg_id == pb.MSG_ID_TASK_COMMAND_RESPONSE:
                m = pb.TaskCommandResponse()
                m.ParseFromString(frame.payload)
                self._remember_task_id(getattr(m, "task_id", ""))
                self._append_log(
                    f"TaskCommandResponse result={m.result} task_id={m.task_id} msg={m.message}"
                )
            elif hasattr(pb, "MSG_ID_TASK_CONFIG_RESPONSE") and frame.msg_id == pb.MSG_ID_TASK_CONFIG_RESPONSE:
                m = pb.TaskConfigResponse()
                m.ParseFromString(frame.payload)
                self._remember_task_id(getattr(m, "task_id", ""))
                self._append_log(
                    f"TaskConfigResponse result={m.result} task_id={m.task_id} msg={m.message}"
                )
            elif frame.msg_id == pb.MSG_ID_SETTINGS_READ_RESPONSE:
                m = pb.SettingsReadResponse()
                m.ParseFromString(frame.payload)
                self._append_log(
                    f"SettingsRead result={m.result} msg={m.message} "
                    f"run_speed={float(m.chassis.run_speed):.3f}m/s "
                    f"work_mode={m.chassis.work_mode} disc_rpm={m.chassis.disc_speed_rpm} "
                    f"disc_enabled={m.chassis.disc_enabled}"
                )
                if float(m.chassis.run_speed) > 0:
                    self.chassis_run_speed.setValue(float(m.chassis.run_speed))
                self.disc_speed_spin.setValue(int(m.chassis.disc_speed_rpm))
                self.disc_enable_check.setChecked(bool(m.chassis.disc_enabled))
                mode_value = int(m.chassis.work_mode)
                idx = self.work_mode_combo.findData(mode_value)
                if idx >= 0:
                    self.work_mode_combo.setCurrentIndex(idx)
                self.robot_vehicle_width.setValue(float(m.map.vehicle_width) if float(m.map.vehicle_width) > 0 else self.robot_vehicle_width.value())
                self.robot_vehicle_length.setValue(float(m.map.vehicle_length) if float(m.map.vehicle_length) > 0 else self.robot_vehicle_length.value())
                self.robot_path_spacing.setValue(float(m.map.default_path_spacing) if float(m.map.default_path_spacing) > 0 else self.robot_path_spacing.value())
                self.robot_turn_radius.setValue(float(m.map.turn_radius) if float(m.map.turn_radius) > 0 else self.robot_turn_radius.value())
                self.robot_overlap_ratio.setValue(float(m.map.overlap_ratio))
                self.robot_inflation_radius.setValue(float(m.map.inflation_radius) if float(m.map.inflation_radius) > 0 else self.robot_inflation_radius.value())
                work_lines = self._format_region_lines(list(m.map.work_regions), "work_regions")
                obstacle_lines = self._format_region_lines(list(m.map.obstacle_regions), "obstacle_regions")
                map_lines = [
                    f"map.vehicle_width={float(m.map.vehicle_width):.3f}",
                    f"map.vehicle_length={float(m.map.vehicle_length):.3f}",
                    f"map.default_path_spacing={float(m.map.default_path_spacing):.3f}",
                    f"map.turn_radius={float(m.map.turn_radius):.3f}",
                    f"map.overlap_ratio={float(m.map.overlap_ratio):.3f}",
                    f"map.inflation_radius={float(m.map.inflation_radius):.3f}",
                ] + work_lines + obstacle_lines
                self._append_log("区域查询返回:\n" + "\n".join(map_lines))
                self.map_info_text.setPlainText("\n".join(["[区域查询(SettingsReadResponse)]"] + map_lines))
                self._update_delete_region_combo_from_settings(m.map)
            elif frame.msg_id == pb.MSG_ID_SETTINGS_WRITE_RESPONSE:
                m = pb.SettingsWriteResponse()
                m.ParseFromString(frame.payload)
                self._append_log(
                    f"SettingsWrite result={m.result} msg={m.message} "
                    f"run_speed={float(m.chassis.run_speed):.3f}m/s "
                    f"work_mode={m.chassis.work_mode} disc_rpm={m.chassis.disc_speed_rpm} "
                    f"disc_enabled={m.chassis.disc_enabled}"
                )
            elif frame.msg_id == pb.MSG_ID_MAP_PREVIEW_RESPONSE:
                if hasattr(self, "preview_tabs"):
                    self.preview_tabs.setCurrentIndex(0)
                self.btn_map_preview.setEnabled(True)
                m = pb.MapPreviewResponse()
                m.ParseFromString(frame.payload)
                self._append_log(
                    f"MapPreview result={m.result} map_v={m.map_version} "
                    f"size={m.width}x{m.height} res={m.resolution:.3f} img={len(m.image_data)}"
                )
                if m.image_data:
                    fmt = "png" if m.image_data[:8] == b"\x89PNG\r\n\x1a\n" else "jpg"
                    self._save_blob("map_preview", fmt, m.image_data)
                    self._show_map_preview(m.image_data, m)
                if m.overlay_json:
                    self._append_log(f"overlay_json(head)={m.overlay_json[:160]}")
            elif hasattr(pb, "MSG_ID_MAP_EDIT_RESPONSE") and frame.msg_id == pb.MSG_ID_MAP_EDIT_RESPONSE:
                m = pb.MapEditResponse()
                m.ParseFromString(frame.payload)
                self._append_log(
                    f"MapEditResponse result={m.result} map_version={m.map_version} msg={m.message}"
                )
                self.map_info_text.appendPlainText(
                    f"\n[MapEditResponse] result={m.result} map_version={m.map_version} msg={m.message}"
                )
            elif hasattr(pb, "MSG_ID_MAP_EDIT_STATUS_REPORT") and frame.msg_id == pb.MSG_ID_MAP_EDIT_STATUS_REPORT:
                m = pb.MapEditStatusReport()
                m.ParseFromString(frame.payload)
                self._append_log(
                    f"MapEditStatus map_version={m.map_version} applied_to_planner={m.applied_to_planner} msg={m.message}"
                )
                self.map_info_text.appendPlainText(
                    f"\n[MapEditStatus] map_version={m.map_version} applied={m.applied_to_planner} msg={m.message}"
                )
            elif frame.msg_id == pb.MSG_ID_PATH_PLAN_RESPONSE:
                if hasattr(self, "preview_tabs"):
                    self.preview_tabs.setCurrentIndex(2)
                m = pb.PathPlanResponse()
                m.ParseFromString(frame.payload)
                eta_s = float(getattr(m, "estimated_time_s", -1.0))
                eta_text = f"{eta_s:.1f}s" if eta_s >= 0.0 else "--"
                self._append_log(
                    f"PathPlan result={m.result} planned={m.planned} path_v={m.path_version} "
                    f"points={m.path_point_count} len={m.path_length_m:.3f} "
                    f"area={float(getattr(m, 'total_work_area_m2', 0.0)):.2f} eta={eta_text}"
                )
                if m.preview_image:
                    ext = (m.preview_format or "jpg").lower()
                    self._save_blob("path_preview", ext, m.preview_image)
                    self._show_path_preview(m.preview_image, m)
                else:
                    self.path_info_text.setPlainText(
                        "\n".join(
                            [
                                f"result: {m.result}",
                                f"message: {m.message}",
                                f"request_id: {m.request_id}",
                                f"task_id: {m.task_id}",
                                f"planned: {m.planned}",
                                f"map_version: {m.map_version}",
                                f"path_version: {m.path_version}",
                                f"path_point_count: {m.path_point_count}",
                                f"path_length_m: {m.path_length_m:.6f}",
                                f"total_work_area_m2: {float(getattr(m, 'total_work_area_m2', 0.0)):.6f}",
                                (
                                    f"estimated_time_s: {float(getattr(m, 'estimated_time_s', -1.0)):.3f}"
                                    if float(getattr(m, "estimated_time_s", -1.0)) >= 0.0
                                    else "estimated_time_s: -1"
                                ),
                                "preview_image: <empty>",
                            ]
                        )
                    )
            elif hasattr(pb, "MSG_ID_TASK_RESULT_RESPONSE") and frame.msg_id == pb.MSG_ID_TASK_RESULT_RESPONSE:
                if hasattr(self, "preview_tabs"):
                    self.preview_tabs.setCurrentIndex(3)
                m = pb.TaskResultResponse()
                m.ParseFromString(frame.payload)
                self._append_log(
                    f"TaskResult result={m.result} map_id={m.map_id} task_id={m.task_id} "
                    f"all_completed={m.all_completed} regions={len(m.region_results)} "
                    f"img={len(getattr(m, 'image_data', b''))}"
                )
                if getattr(m, "image_data", b""):
                    ext = (m.image_format or "jpg").lower()
                    self._save_blob("task_result", ext, m.image_data)
                    self._show_task_result_preview(m.image_data, m)
                lines = [
                    f"result: {m.result}",
                    f"message: {m.message}",
                    f"map_id: {m.map_id}",
                    f"task_id: {m.task_id}",
                    f"final_state: {int(getattr(m, 'final_state', 0))}",
                    f"all_completed: {bool(getattr(m, 'all_completed', False))}",
                    f"stop_reason: {getattr(m, 'stop_reason', '')}",
                    f"path_version: {int(getattr(m, 'path_version', 0))}",
                    f"finished_at: {int(getattr(m, 'finished_at', 0))}",
                    f"image_format: {getattr(m, 'image_format', '')}",
                    f"image_size: {int(getattr(m, 'image_width', 0))}x{int(getattr(m, 'image_height', 0))}",
                    f"selected_work_region_ids: {', '.join(list(getattr(m, 'selected_work_region_ids', []) or []))}",
                    "region_results:",
                ]
                for item in list(getattr(m, "region_results", []) or []):
                    lines.append(
                        f"- {item.region_id}({item.region_name}) target={int(item.target_repeat)} "
                        f"executed={int(item.executed_repeat)} completed={bool(item.completed)} "
                        f"reason={item.unfinished_reason}"
                    )
                self.task_result_info_text.setPlainText("\n".join(lines))
            elif frame.msg_id == pb.MSG_ID_MAP_SYNC_RESPONSE:
                m = pb.MapSyncResponse()
                m.ParseFromString(frame.payload)
                self._append_log(
                    f"MapSync result={m.result} op={m.operation} reloaded={m.navigation_map_reloaded} "
                    f"map_id={getattr(m, 'map_id', '')} map_name={getattr(m, 'map_name', '')} "
                    f"yaml={m.map_yaml_path}"
                )
                if m.operation == pb.MAP_SYNC_OP_DOWNLOAD_FROM_AURORA:
                    self._append_log("MapSync 语义: 已执行“下载到本地”")
                elif m.operation == pb.MAP_SYNC_OP_UPLOAD_TO_AURORA:
                    self._append_log("MapSync 语义: 已执行“上传到雷达”")
            elif hasattr(pb, "MSG_ID_MAP_IMPORT_TO_RADAR_RESPONSE") and frame.msg_id == pb.MSG_ID_MAP_IMPORT_TO_RADAR_RESPONSE:
                m = pb.MapImportToRadarResponse()
                m.ParseFromString(frame.payload)
                msg = (
                    f"MapImportToRadar result={m.result} imported={m.imported} "
                    f"map_id={m.map_id} map_name={m.map_name} msg={m.message}"
                )
                self._append_log(msg)
                self._append_reqresp(msg)
            elif hasattr(pb, "MSG_ID_MAP_CATALOG_RESPONSE") and frame.msg_id == pb.MSG_ID_MAP_CATALOG_RESPONSE:
                m = pb.MapCatalogResponse()
                m.ParseFromString(frame.payload)
                self.local_map_combo.clear()
                self.local_map_combo.addItem("LIVE_MAP", DEFAULT_REQUEST_MAP_ID)
                if int(m.result) != int(pb.RESULT_SUCCESS):
                    self.local_map_combo.addItem("查询失败", "")
                    self.local_map_count_label.setText("地图数量: 0")
                    self._append_log(f"MapCatalog result={m.result} msg={m.message}")
                else:
                    names = []
                    thumb_infos = []
                    seen_ids = {DEFAULT_REQUEST_MAP_ID}
                    for item in m.items:
                        name = (item.name or "").strip() or "未命名地图"
                        map_id = str(getattr(item, "map_id", "")).strip()
                        if map_id in seen_ids:
                            continue
                        seen_ids.add(map_id)
                        label = f"{name}({map_id})" if map_id else name
                        names.append(label)
                        self.local_map_combo.addItem(label, map_id)
                        thumb_b64 = str(getattr(item, "thumbnail_image_b64", "") or "")
                        thumb_infos.append(
                            "{}:{} {}x{} b64={}".format(
                                map_id or name,
                                str(getattr(item, "thumbnail_format", "") or ""),
                                int(getattr(item, "thumbnail_width", 0) or 0),
                                int(getattr(item, "thumbnail_height", 0) or 0),
                                len(thumb_b64),
                            )
                        )
                    if not names:
                        self.local_map_combo.addItem("无记录地图", "")
                    count = int(m.total_count) if int(m.total_count) > 0 else len(names)
                    self.local_map_count_label.setText(f"地图数量: {count} + LIVE_MAP")
                    self._append_log(
                        f"MapCatalog result={m.result} count={count} names={names} thumbnails={thumb_infos}"
                    )
            elif hasattr(pb, "MSG_ID_MAP_SAVE_RESPONSE") and frame.msg_id == pb.MSG_ID_MAP_SAVE_RESPONSE:
                m = pb.MapSaveResponse()
                m.ParseFromString(frame.payload)
                self._append_log(
                    f"MapSave result={m.result} reloaded={m.navigation_map_reloaded} "
                    f"map_id={getattr(m, 'map_id', '')} map_name={getattr(m, 'map_name', '')} "
                    f"yaml={m.map_yaml_path} msg={m.message}"
                )
            elif hasattr(pb, "MSG_ID_MAP_DELETE_RESPONSE") and frame.msg_id == pb.MSG_ID_MAP_DELETE_RESPONSE:
                m = pb.MapDeleteResponse()
                m.ParseFromString(frame.payload)
                msg = (
                    f"MapDelete result={m.result} deleted={m.deleted} "
                    f"map_id={getattr(m, 'map_id', '')} map_name={getattr(m, 'map_name', '')} msg={m.message}"
                )
                self._append_log(msg)
                self._append_reqresp(msg)
                self._query_local_maps()
            elif hasattr(pb, "MSG_ID_MAP_MODE_RESPONSE") and frame.msg_id == pb.MSG_ID_MAP_MODE_RESPONSE:
                m = pb.MapModeResponse()
                m.ParseFromString(frame.payload)
                self._append_log(
                    f"MapMode result={m.result} mode={m.mode} enabled={m.enabled} "
                    f"map_kind={m.map_kind} msg={m.message}"
                )
            elif hasattr(pb, "MSG_ID_LIVE_MAP_CACHE_CLEAR_RESPONSE") and frame.msg_id == pb.MSG_ID_LIVE_MAP_CACHE_CLEAR_RESPONSE:
                m = pb.LiveMapCacheClearResponse()
                m.ParseFromString(frame.payload)
                msg = f"LiveMapCacheClear result={m.result} msg={m.message}"
                self._append_log(msg)
                self._append_reqresp(msg)
            elif hasattr(pb, "MSG_ID_RADAR_MAP_CACHE_CLEAR_RESPONSE") and frame.msg_id == pb.MSG_ID_RADAR_MAP_CACHE_CLEAR_RESPONSE:
                m = pb.RadarMapCacheClearResponse()
                m.ParseFromString(frame.payload)
                msg = f"RadarMapCacheClear result={m.result} msg={m.message}"
                self._append_log(msg)
                self._append_reqresp(msg)
            elif frame.msg_id == pb.MSG_ID_VIDEO_STREAM_INFO_RESPONSE:
                m = pb.VideoStreamInfoResponse()
                m.ParseFromString(frame.payload)
                self._append_log(
                    f"VideoInfo result={m.result} online={m.online} codec={m.codec} "
                    f"url={m.stream_url} {m.width}x{m.height}"
                )
            elif frame.msg_id == pb.MSG_ID_CONTROL_COMMAND_RESPONSE:
                m = pb.ControlCommandResponse()
                m.ParseFromString(frame.payload)
                self._append_log(f"ControlResp result={m.result} msg={m.message}")
            elif frame.msg_id == pb.MSG_ID_CAMERA_FRAME_CHUNK:
                m = pb.CameraFrameChunk()
                m.ParseFromString(frame.payload)
                self._append_log(
                    f"CameraChunk frame={m.frame_id} {m.chunk_index+1}/{m.total_chunks} bytes={len(m.data)}"
                )
            elif frame.msg_id == pb.MSG_ID_MAP_CHUNK:
                if hasattr(self, "preview_tabs"):
                    self.preview_tabs.setCurrentIndex(1)
                m = pb.MapChunk()
                m.ParseFromString(frame.payload)
                key = (int(m.map_id), int(m.total_chunks), int(m.width), int(m.height), int(m.encoding))
                st = self._map_chunk_state.get(key)
                if st is None:
                    st = {"chunks": [None] * max(1, int(m.total_chunks)), "received": 0, "meta": m}
                    self._map_chunk_state = {key: st}
                idx = int(m.chunk_index)
                if 0 <= idx < len(st["chunks"]) and st["chunks"][idx] is None:
                    st["chunks"][idx] = bytes(m.data)
                    st["received"] += 1
                self._append_log(
                    f"MapChunk map_id={m.map_id} {m.chunk_index+1}/{m.total_chunks} bytes={len(m.data)}"
                )
                if st["received"] == len(st["chunks"]) and all(c is not None for c in st["chunks"]):
                    full_bytes = b"".join(st["chunks"])
                    self._append_log(f"MapChunk 重组完成: map_id={m.map_id} total_bytes={len(full_bytes)}")
                    self._show_raw_map_chunk(m, full_bytes)
            else:
                b64 = base64.b64encode(frame.payload[:24]).decode("ascii")
                self._append_log(f"Unhandled payload(head b64): {b64}...")
        except Exception as exc:
            self._append_log(f"Decode error for {name}: {exc}")

    def _strip_path_preview_magenta_annotations(self, image_bytes: bytes) -> bytes:
        """
        Remove colored annotations (magenta labels/arrows + orange segment-end points)
        from path preview image.
        This is a UI-side safeguard when backend preview still contains labels.
        """
        if not image_bytes or cv2 is None or np is None:
            return image_bytes
        try:
            arr = np.frombuffer(image_bytes, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                return image_bytes

            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # Magenta/purple (labels/arrows)
            magenta_lower = np.array([135, 50, 50], dtype=np.uint8)
            magenta_upper = np.array([179, 255, 255], dtype=np.uint8)
            mask_magenta = cv2.inRange(hsv, magenta_lower, magenta_upper)

            # Orange (segment-end markers)
            orange_lower = np.array([8, 70, 70], dtype=np.uint8)
            orange_upper = np.array([28, 255, 255], dtype=np.uint8)
            mask_orange = cv2.inRange(hsv, orange_lower, orange_upper)

            mask = cv2.bitwise_or(mask_magenta, mask_orange)
            if int(mask.max()) == 0:
                return image_bytes

            # Slightly dilate so text strokes and marker edges are fully removed.
            kernel = np.ones((2, 2), dtype=np.uint8)
            mask = cv2.dilate(mask, kernel, iterations=1)

            # Inpaint masked annotations to nearby background/path colors.
            cleaned = cv2.inpaint(img, mask, 2, cv2.INPAINT_TELEA)
            ok, enc = cv2.imencode(".jpg", cleaned, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            if not ok:
                return image_bytes
            return enc.tobytes()
        except Exception:
            return image_bytes

    def _update_task_status_panel(self, report):
        state_id = int(getattr(report, "state", 0))
        state_name = TASK_STATE_NAMES.get(state_id, str(state_id))
        state_name = state_name.replace("TASK_STATE_", "")
        progress = max(0.0, min(1.0, float(getattr(report, "progress", 0.0))))
        percent = progress * 100.0
        report_task_id = str(getattr(report, "task_id", "") or "").strip()
        self.task_status_task_id_value.setText(report_task_id or "-")
        self.task_state_value.setText(state_name)
        self.task_progress_value.setText(f"{percent:.1f}%")
        self.task_progress_bar.setValue(int(round(progress * 1000.0)))
        self.task_map_v_value.setText(str(int(getattr(report, "map_version", 0))))
        self.task_path_v_value.setText(str(int(getattr(report, "path_version", 0))))
        self.task_points_value.setText(str(int(getattr(report, "path_point_count", 0))))
        total_area = float(getattr(report, "total_work_area_m2", 0.0))
        remaining_area = float(getattr(report, "remaining_work_area_m2", 0.0))
        remaining_time_s = float(getattr(report, "remaining_time_s", -1.0))
        self.task_total_area_value.setText(f"{total_area:.2f}")
        self.task_remaining_area_value.setText(f"{remaining_area:.2f}")
        if remaining_time_s >= 0.0:
            self.task_remaining_time_value.setText(f"{remaining_time_s:.1f} s")
        else:
            self.task_remaining_time_value.setText("--")
        pos = getattr(report, "position", None)
        if pos is not None:
            px = float(getattr(pos, "x", 0.0))
            py = float(getattr(pos, "y", 0.0))
            ph = float(getattr(pos, "heading_deg", 0.0))
            self.task_pose_value.setText(f"x={px:.3f} y={py:.3f} heading={ph:.1f}")
            self._set_robot_pose_display(px, py, ph)
        else:
            self.task_pose_value.setText("x=0.000 y=0.000 heading=0.0")
        self.task_message_value.setText(str(getattr(report, "message", "") or "-"))
        self._last_task_status_ts = time.strftime("%H:%M:%S", time.localtime())
        self.task_time_value.setText(self._last_task_status_ts)

    def _set_robot_pose_display(self, x: float, y: float, heading_deg: float):
        self.robot_pose_label.setText(
            f"机器人坐标: x={float(x):.3f}  y={float(y):.3f}  heading={float(heading_deg):.1f}"
        )

    def closeEvent(self, event):
        self._stop_rtsp_preview(silent=True)
        self.client.disconnect()
        super().closeEvent(event)


def main():
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_UseSoftwareOpenGL, True)
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
