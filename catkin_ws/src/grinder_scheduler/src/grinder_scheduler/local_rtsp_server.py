import os
import platform
import shutil
import socket
import subprocess
import threading
import time
import textwrap

import cv2
import rospy

from grinder_scheduler.models import VideoStreamState


class _RtspPublisherWorker:
    def __init__(
        self,
        ffmpeg_path,
        push_url,
        hardware_push_url,
        public_url,
        fps,
        width,
        enabled,
        name,
        log_dir,
        preferred_encoder="auto",
        bitrate_kbps=800,
        keyframe_interval=15,
    ):
        self._ffmpeg_path = ffmpeg_path
        self._gstreamer_path = shutil.which("gst-launch-1.0")
        self._gst_inspect_path = shutil.which("gst-inspect-1.0")
        self._push_url = push_url
        self._hardware_push_url = hardware_push_url
        self._public_url = public_url
        self._fps = max(1, int(fps))
        self._target_width = max(320, int(width))
        self._enabled = bool(enabled and (ffmpeg_path or self._gstreamer_path))
        self._name = name
        self._log_dir = log_dir
        self._process = None
        self._log_fp = None
        self._lock = threading.Lock()
        self._last_frame_time = 0.0
        self._state = VideoStreamState(stream_url=self._public_url, codec="h264")
        self._preferred_encoder = str(preferred_encoder or "auto").strip().lower()
        self._bitrate_kbps = max(100, int(bitrate_kbps))
        self._keyframe_interval = max(1, int(keyframe_interval))
        self._encoder_candidates = self._detect_encoder_candidates()
        self._encoder_index = 0
        self._active_encoder = self._encoder_candidates[0] if self._encoder_candidates else "libx264"
        self._encoder_runtime_failures = {}
        self._encoder_fail_threshold = 3

    def push_frame(self, frame):
        if not self._enabled or frame is None:
            return
        now = time.time()
        if now - self._last_frame_time < 1.0 / float(self._fps):
            return
        self._last_frame_time = now
        scale = self._target_width / float(frame.shape[1])
        target_height = max(2, int(round(frame.shape[0] * scale)))
        if target_height % 2 == 1:
            target_height += 1
        target_width = self._target_width if self._target_width % 2 == 0 else self._target_width + 1
        frame = cv2.resize(frame, (target_width, target_height), interpolation=cv2.INTER_AREA)
        with self._lock:
            self._ensure_process(frame.shape[1], frame.shape[0])
            if self._process is None:
                return
            try:
                self._process.stdin.write(frame.tobytes())
                self._process.stdin.flush()
                self._state.width = frame.shape[1]
                self._state.height = frame.shape[0]
                self._state.online = True
                self._state.last_update_utc = int(now)
                self._encoder_runtime_failures[self._active_encoder] = 0
            except Exception:
                self._mark_runtime_encoder_failure_locked()
                self._stop_locked()

    def stop(self):
        with self._lock:
            self._stop_locked()

    def get_state(self):
        with self._lock:
            return VideoStreamState(**self._state.__dict__)

    def _ensure_process(self, width, height):
        if self._process is not None:
            if self._process.poll() is None:
                return
            # Process died unexpectedly while running; count this as encoder runtime failure.
            self._mark_runtime_encoder_failure_locked()
            self._process = None
        if self._ffmpeg_path is None and self._gstreamer_path is None:
            return
        os.makedirs(self._log_dir, exist_ok=True)
        log_path = os.path.join(self._log_dir, "ffmpeg_{}.log".format(self._name))
        self._log_fp = open(log_path, "ab")
        if not self._encoder_candidates:
            self._encoder_candidates = ["libx264"]
        attempts = 0
        max_attempts = len(self._encoder_candidates)
        while attempts < max_attempts:
            encoder = self._encoder_candidates[self._encoder_index]
            command = self._build_ffmpeg_command(width, height, encoder)
            target_url = self._hardware_push_url if encoder == "mpph264enc" else self._push_url
            rospy.loginfo(
                "Starting RTSP publisher %s -> %s (encoder=%s)",
                self._name,
                target_url,
                encoder,
            )
            self._process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=self._log_fp, stderr=self._log_fp)
            time.sleep(0.15)
            if self._process.poll() is None:
                self._active_encoder = encoder
                self._state.codec = str(encoder)
                return
            rospy.logwarn(
                "ffmpeg RTSP publisher %s failed to start with encoder=%s, fallback to next candidate.",
                self._name,
                encoder,
            )
            self._process = None
            self._encoder_index = (self._encoder_index + 1) % max_attempts
            attempts += 1

    def _mark_runtime_encoder_failure_locked(self):
        encoder = str(self._active_encoder or "")
        if not encoder:
            return
        failures = int(self._encoder_runtime_failures.get(encoder, 0)) + 1
        self._encoder_runtime_failures[encoder] = failures
        if failures >= int(self._encoder_fail_threshold):
            if self._encoder_candidates:
                old = self._encoder_index
                self._encoder_index = (self._encoder_index + 1) % len(self._encoder_candidates)
                rospy.logwarn(
                    "RTSP publisher %s encoder=%s runtime failures=%d, switching candidate index %d -> %d",
                    self._name,
                    encoder,
                    failures,
                    old,
                    self._encoder_index,
                )
            self._encoder_runtime_failures[encoder] = 0

    def _detect_encoder_candidates(self):
        if self._ffmpeg_path is None:
            return ["mpph264enc"] if self._mpp_backend_available() else []
        available = set()
        try:
            output = subprocess.check_output(
                [self._ffmpeg_path, "-hide_banner", "-encoders"],
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                timeout=3.0,
            )
            for line in output.splitlines():
                line = line.strip()
                if not line or line.startswith("--"):
                    continue
                parts = line.split()
                if len(parts) >= 2 and parts[0].startswith("V"):
                    available.add(parts[1].strip())
        except Exception:
            return ["libx264"]

        machine = platform.machine().lower()
        # Stability-first ordering:
        # - On x86/VM, prefer libx264 first (hardware wrappers may exist but be unstable).
        # - On ARM boards, prefer hardware first.
        if machine in ("x86_64", "amd64", "i386", "i686"):
            preferred = [
                "libx264",
                "h264_nvenc",
                "h264_vaapi",
                "h264_v4l2m2m",
                "h264_omx",
            ]
        else:
            preferred = [
                "h264_v4l2m2m",
                "h264_vaapi",
                "h264_omx",
                "h264_nvenc",
                "libx264",
            ]

        if self._preferred_encoder and self._preferred_encoder != "auto":
            custom = [self._preferred_encoder]
            for item in preferred:
                if item not in custom:
                    custom.append(item)
            preferred = custom

        candidates = [name for name in preferred if name in available]
        if self._preferred_encoder == "mpph264enc" and self._mpp_backend_available():
            candidates.insert(0, "mpph264enc")
        if self._ffmpeg_path is not None and "libx264" not in candidates:
            candidates.append("libx264")
        # Keep only runtime-usable encoders; some wrappers are listed but fail at runtime.
        verified = []
        for enc in candidates:
            if self._probe_encoder(enc):
                verified.append(enc)
            else:
                rospy.logwarn("RTSP encoder probe failed, skip encoder=%s for stream %s", enc, self._name)
        if self._ffmpeg_path is not None and "libx264" not in verified:
            verified.append("libx264")
        return verified

    def _probe_encoder(self, encoder):
        if str(encoder) == "mpph264enc":
            return self._mpp_backend_available()
        if self._ffmpeg_path is None:
            return False
        try:
            # Fast sanity probe: create a tiny synthetic stream and encode a handful of frames.
            cmd = [
                self._ffmpeg_path,
                "-loglevel",
                "error",
                "-f",
                "lavfi",
                "-i",
                "testsrc=size=64x64:rate=5",
                "-frames:v",
                "5",
                "-an",
                "-c:v",
                str(encoder),
            ]
            if str(encoder) == "libx264":
                cmd.extend(["-preset", "ultrafast", "-tune", "zerolatency", "-pix_fmt", "yuv420p"])
            else:
                cmd.extend(["-pix_fmt", "nv12"])
            cmd.extend(["-f", "null", "-"])
            subprocess.check_call(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=3.0,
            )
            return True
        except Exception:
            return False

    def _mpp_backend_available(self):
        if self._gstreamer_path is None or self._gst_inspect_path is None:
            return False
        for element in ("mpph264enc", "videoconvert", "h264parse", "flvmux", "rtmpsink"):
            try:
                subprocess.check_call(
                    [self._gst_inspect_path, element],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=2.0,
                )
            except Exception:
                return False
        return True

    def _build_ffmpeg_command(self, width, height, encoder):
        if str(encoder) == "mpph264enc":
            return [
                self._gstreamer_path,
                "-q",
                "fdsrc",
                "fd=0",
                "do-timestamp=true",
                "!",
                "rawvideoparse",
                "format=bgr",
                "width={}".format(width),
                "height={}".format(height),
                "framerate={}/1".format(self._fps),
                "!",
                "videoconvert",
                "!",
                "video/x-raw,format=NV12",
                "!",
                "mpph264enc",
                "bps={}".format(self._bitrate_kbps * 1000),
                "gop={}".format(self._keyframe_interval),
                "profile=baseline",
                "rc-mode=cbr",
                "!",
                "h264parse",
                "config-interval=-1",
                "!",
                "flvmux",
                "streamable=true",
                "!",
                "rtmpsink",
                "location={}".format(self._hardware_push_url),
                "sync=false",
                "async=false",
            ]
        command = [
            self._ffmpeg_path,
            "-loglevel",
            "warning",
            "-fflags",
            "nobuffer",
            "-flags",
            "low_delay",
            "-avioflags",
            "direct",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "bgr24",
            "-s",
            "{}x{}".format(width, height),
            "-r",
            str(self._fps),
            "-i",
            "-",
            "-an",
            "-c:v",
            str(encoder),
        ]
        if str(encoder) == "libx264":
            command.extend(
                [
                    "-preset",
                    "ultrafast",
                    "-tune",
                    "zerolatency",
                    "-x264-params",
                    "keyint=5:min-keyint=5:scenecut=0:rc-lookahead=0:sync-lookahead=0",
                    "-pix_fmt",
                    "yuv420p",
                ]
            )
        else:
            # Hardware encoder path (best-effort generic options).
            command.extend(
                [
                    "-pix_fmt",
                    "nv12",
                ]
            )
        command.extend(
            [
                "-bf",
                "0",
                "-g",
                "5",
                "-keyint_min",
                "5",
                "-sc_threshold",
                "0",
                "-muxpreload",
                "0",
                "-muxdelay",
                "0",
                "-max_delay",
                "0",
                "-flush_packets",
                "1",
                "-f",
                "rtsp",
                "-rtsp_transport",
                "udp",
                self._push_url,
            ]
        )
        return command

    def _stop_locked(self):
        if self._process is None:
            self._state.online = False
            return
        try:
            if self._process.stdin:
                self._process.stdin.close()
        except Exception:
            pass
        try:
            self._process.terminate()
        except Exception:
            pass
        self._process = None
        if self._log_fp is not None:
            try:
                self._log_fp.close()
            except Exception:
                pass
            self._log_fp = None
        self._state.online = False


class LocalRtspStreamServer:
    def __init__(
        self,
        host="0.0.0.0",
        public_host="auto",
        port=8554,
        fps=8,
        width=1280,
        enabled=True,
        start_server=True,
        mediamtx_path="",
        log_dir="temp",
        preferred_encoder="auto",
        bitrate_kbps=800,
        keyframe_interval=15,
    ):
        self._host = host
        self._public_host = self._resolve_public_host(public_host)
        self._port = int(port)
        self._enabled = bool(enabled)
        self._start_server = bool(start_server)
        self._log_dir = log_dir
        self._ffmpeg_path = shutil.which("ffmpeg")
        self._mediamtx_path = self._resolve_mediamtx_path(
            mediamtx_path.strip() or shutil.which("mediamtx") or shutil.which("rtsp-simple-server")
        )
        self._server_process = None
        self._server_log_fp = None
        self._config_path = os.path.join(self._log_dir, "mediamtx.generated.yml")
        self._left_url_push = "rtsp://127.0.0.1:{}/left".format(self._port)
        self._right_url_push = "rtsp://127.0.0.1:{}/right".format(self._port)
        self._left_url_hardware_push = "rtmp://127.0.0.1:1935/left"
        self._right_url_hardware_push = "rtmp://127.0.0.1:1935/right"
        self._left_url_public = "rtsp://{}:{}/left".format(self._public_host, self._port)
        self._right_url_public = "rtsp://{}:{}/right".format(self._public_host, self._port)
        self._left = _RtspPublisherWorker(
            ffmpeg_path=self._ffmpeg_path,
            push_url=self._left_url_push,
            hardware_push_url=self._left_url_hardware_push,
            public_url=self._left_url_public,
            fps=fps,
            width=width,
            enabled=enabled,
            name="left",
            log_dir=self._log_dir,
            preferred_encoder=preferred_encoder,
            bitrate_kbps=bitrate_kbps,
            keyframe_interval=keyframe_interval,
        )
        self._right = _RtspPublisherWorker(
            ffmpeg_path=self._ffmpeg_path,
            push_url=self._right_url_push,
            hardware_push_url=self._right_url_hardware_push,
            public_url=self._right_url_public,
            fps=fps,
            width=width,
            enabled=enabled,
            name="right",
            log_dir=self._log_dir,
            preferred_encoder=preferred_encoder,
            bitrate_kbps=bitrate_kbps,
            keyframe_interval=keyframe_interval,
        )
        self._rtsp_ready = False

    def start(self):
        if not self._enabled or not self._start_server or self._mediamtx_path is None:
            return
        if self._server_process is not None and self._server_process.poll() is None:
            return
        env = os.environ.copy()
        os.makedirs(self._log_dir, exist_ok=True)
        self._write_config_file()
        log_path = os.path.join(self._log_dir, "mediamtx.log")
        self._server_log_fp = open(log_path, "ab")
        rospy.loginfo("Starting mediamtx on rtsp://%s:%s using %s", self._public_host, self._port, self._mediamtx_path)
        self._server_process = subprocess.Popen(
            [self._mediamtx_path, self._config_path],
            stdout=self._server_log_fp,
            stderr=self._server_log_fp,
            env=env,
        )
        self._rtsp_ready = self._wait_rtsp_listener_ready(timeout_s=5.0)
        if self._rtsp_ready:
            rospy.loginfo("mediamtx rtsp listener is ready on 127.0.0.1:%s", self._port)
        else:
            rospy.logwarn("mediamtx rtsp listener not ready within timeout on 127.0.0.1:%s", self._port)

    def stop(self):
        self._left.stop()
        self._right.stop()
        self._rtsp_ready = False
        if self._server_process is None:
            return
        try:
            self._server_process.terminate()
        except Exception:
            pass
        self._server_process = None
        if self._server_log_fp is not None:
            try:
                self._server_log_fp.close()
            except Exception:
                pass
            self._server_log_fp = None

    def push_frame(self, stream_name, frame):
        if stream_name == "left":
            self._left.push_frame(frame)
        elif stream_name == "right":
            self._right.push_frame(frame)

    def get_state(self):
        left_state = self._left.get_state()
        right_state = self._right.get_state()
        if left_state.online:
            return left_state
        if right_state.online:
            return right_state
        state = left_state
        state.stream_url = self.get_stream_url("left")
        state.online = False
        return state

    def get_stream_url(self, stream_name):
        if stream_name == "right":
            return self._right_url_public
        return self._left_url_public

    def get_stream_urls(self):
        return {"left": self._left_url_public, "right": self._right_url_public}

    def ffmpeg_available(self):
        return self._ffmpeg_path is not None

    def mediamtx_available(self):
        return self._mediamtx_path is not None and os.path.isfile(self._mediamtx_path) and os.access(
            self._mediamtx_path, os.X_OK
        )

    def server_running(self):
        return self._server_process is not None and self._server_process.poll() is None

    def _wait_rtsp_listener_ready(self, timeout_s=5.0):
        deadline = time.time() + max(0.2, float(timeout_s))
        while time.time() < deadline:
            if self._server_process is None or self._server_process.poll() is not None:
                return False
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(0.2)
            try:
                sock.connect(("127.0.0.1", int(self._port)))
                return True
            except OSError:
                time.sleep(0.08)
            finally:
                sock.close()
        return False

    def _write_config_file(self):
        payload = textwrap.dedent(
            """
            logLevel: info
            rtspAddress: :{port}
            # Lower internal buffering to reduce end-to-end latency.
            readBufferCount: 64
            # Allow both UDP and TCP for player compatibility.
            rtspTransports: [udp, tcp]
            # Keep server-side output queue small to avoid stale-frame buildup.
            writeQueueSize: 128
            rtmpAddress: :1935
            hls: no
            webrtc: no
            srt: no
            paths:
              left:
                source: publisher
              right:
                source: publisher
              live/grinder_main:
                source: publisher
            """
        ).strip().format(port=self._port)
        with open(self._config_path, "w", encoding="utf-8") as handle:
            handle.write(payload)
            handle.write("\n")

    def _resolve_public_host(self, public_host):
        if public_host and public_host not in ("auto", "127.0.0.1", "localhost"):
            return public_host
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.connect(("10.255.255.255", 1))
            address = sock.getsockname()[0]
            return address or "127.0.0.1"
        except OSError:
            return "127.0.0.1"
        finally:
            sock.close()

    def _resolve_mediamtx_path(self, input_path):
        if not input_path:
            return None
        base_path = os.path.abspath(input_path)
        base_name = os.path.basename(base_path)
        base_dir = os.path.dirname(base_path)
        machine = platform.machine().lower()
        preferred = None
        if machine in ("aarch64", "arm64"):
            preferred = "mediamtx_aarch64"
        elif machine in ("x86_64", "amd64"):
            preferred = "mediamtx_x86_64"

        candidates = []
        if preferred and base_name == "mediamtx":
            candidates.append(os.path.join(base_dir, preferred))
        candidates.append(base_path)

        for candidate in candidates:
            if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
                if candidate != base_path:
                    rospy.loginfo(
                        "Auto-selected mediamtx for %s: %s (fallback was %s)",
                        machine,
                        candidate,
                        base_path,
                    )
                return candidate
        return base_path
