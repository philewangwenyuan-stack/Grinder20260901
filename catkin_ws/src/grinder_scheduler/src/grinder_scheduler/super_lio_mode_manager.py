#!/usr/bin/env python3

import json
import math
import os
import queue
import re
import socket
import shutil
import tempfile
import threading
import time
from collections import deque
from xmlrpc.client import ServerProxy

import roslaunch
import rosnode
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse

from grinder_scheduler.srv import (
    GetSuperLioStatus,
    GetSuperLioStatusResponse,
    SaveSuperLioMap,
    SaveSuperLioMapResponse,
    SetSuperLioInitialPose,
    SetSuperLioInitialPoseResponse,
    StartSuperLioLocalization,
    StartSuperLioLocalizationResponse,
)
from super_lio.srv import GetMap, GetMapRequest
from grinder_scheduler.map_asset_revision import compute_map_asset_revision, get_or_compute_map_asset_revision
from grinder_scheduler.metrics_registry import MetricsRegistry
from grinder_scheduler.metrics_exporter import MetricsExporter

try:
    import tf2_ros
except Exception:
    tf2_ros = None


metrics = MetricsRegistry()


def _record_operation_failure(operation, error):
    metrics.counter("super_lio_{}_failed".format(operation)).inc()
    if "timeout" in str(error).lower() or "timed out" in str(error).lower():
        metrics.counter("super_lio_timeout").inc()
    # 事件环仅保留操作名与结果；异常原文可能包含目录或设备信息，不写入指标文件。
    metrics.event("super_lio_operation", operation=operation, result="failed")


class SuperLioShutdownTimeout(RuntimeError):
    def __init__(self, residual_nodes):
        self.residual_nodes = list(residual_nodes or [])
        super().__init__("Super-LIO shutdown timed out; residual nodes: {}".format(", ".join(self.residual_nodes)))


class SuperLioModeManager:
    IDLE = "IDLE"
    STARTING = "STARTING"
    MAPPING = "MAPPING"
    SAVING = "SAVING"
    STOPPING = "STOPPING"
    LOCALIZING = "LOCALIZING"
    RELOCALIZING = "RELOCALIZING"
    READY = "READY"
    TIMEOUT = "TIMEOUT"
    ERROR = "ERROR"

    _MAPPING_NODES = {
        "/super_lio_node", "/cloud_to_occupancy_grid", "/super_lio_loop",
        "/map_to_odom_identity", "/base_to_laser",
    }
    _LOCALIZATION_NODES = {"/relocation_node", "/map_server", "/base_to_laser"}
    _MANAGED_NODES = _MAPPING_NODES | _LOCALIZATION_NODES

    def __init__(self):
        self._main_thread_id = threading.get_ident()
        self._operation_queue = queue.Queue()
        self._lock = threading.RLock()
        self._state = self.IDLE
        self._message = "idle"
        self._active_map_id = ""
        self._active_map_revision = ""
        self._bundle_dir = ""
        self._localization_ready = False
        self._initial_pose_received = False
        self._localization_good_frames = 0
        self._registration_quality_valid = False
        self._registration_fitness = 0.0
        self._registration_inlier_ratio = 0.0
        self._residual_nodes = []
        self._odom_event = threading.Event()
        self._map_event = threading.Event()
        self._launch_parent = None
        self._owned_mode = ""
        self._owned_node_uris = {}
        self._expected_owned_node_names = set()
        self._accept_new_owned_node_uris = False
        self._health_lock = threading.RLock()
        self._stream_samples = {
            "/livox/lidar": deque(maxlen=256),
            "/livox/imu": deque(maxlen=512),
            "/lio/odom": deque(maxlen=256),
        }
        self._pending_odom = deque(maxlen=32)
        self._quality_samples = deque(maxlen=32)
        self._last_quality_stamp = 0.0
        self._last_good_pose = None
        self._last_good_pose_stamp = 0.0
        self._map_metadata = None
        self._map_root = os.path.abspath(os.path.expanduser(rospy.get_param(
            "~super_lio_map_root", "/home/neardi/work/Grinder/maps"
        )))
        if self._map_root in (os.path.abspath(os.sep), os.path.abspath(os.path.expanduser("~"))):
            raise RuntimeError("super_lio_map_root must be a dedicated map directory")
        self._startup_timeout = max(
            2.0, float(rospy.get_param("~super_lio_startup_timeout_sec", 60.0))
        )
        self._shutdown_timeout = max(
            1.0, float(rospy.get_param("~super_lio_shutdown_timeout_sec", 8.0))
        )
        self._cleanup_timeout = max(
            1.0, float(rospy.get_param("~super_lio_cleanup_timeout_sec", 4.0))
        )
        self._health_window_sec = max(
            0.5, float(rospy.get_param("~super_lio_health_window_sec", 1.0))
        )
        self._message_max_age_sec = max(
            0.1, float(rospy.get_param("~super_lio_message_max_age_sec", 0.5))
        )
        self._map_max_age_sec = max(
            self._message_max_age_sec,
            float(rospy.get_param("~super_lio_map_max_age_sec", 5.0)),
        )
        self._min_lidar_hz = max(0.1, float(rospy.get_param("~super_lio_min_lidar_hz", 5.0)))
        self._min_imu_hz = max(0.1, float(rospy.get_param("~super_lio_min_imu_hz", 20.0)))
        self._min_odom_hz = max(0.1, float(rospy.get_param("~super_lio_min_odom_hz", 5.0)))
        self._required_good_frames = max(
            1, int(rospy.get_param("~super_lio_localization_stable_frames", 8))
        )
        self._fitness_max = max(0.0, float(rospy.get_param("~super_lio_fitness_max_m2", 0.3)))
        self._inlier_ratio_min = min(
            1.0, max(0.0, float(rospy.get_param("~super_lio_inlier_ratio_min", 0.2)))
        )
        self._min_correspondences = max(
            1, int(rospy.get_param("~super_lio_min_correspondences", 100))
        )
        self._max_xy_variance = max(
            0.0, float(rospy.get_param("~super_lio_max_xy_variance", 0.25))
        )
        self._max_yaw_variance = max(
            0.0, float(rospy.get_param("~super_lio_max_yaw_variance", 0.06853892))
        )
        self._max_pose_jump_m = max(
            0.01, float(rospy.get_param("~super_lio_max_pose_jump_m", 0.75))
        )
        self._max_yaw_jump_rad = max(
            0.01, float(rospy.get_param("~super_lio_max_yaw_jump_rad", 0.52359878))
        )
        self._tf_base_frame = str(rospy.get_param("~super_lio_tf_base_frame", "base_link"))
        self._preflight_ports = list(rospy.get_param("~super_lio_preflight_ports", []) or [])
        self._tf_buffer = tf2_ros.Buffer() if tf2_ros is not None else None
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer) if self._tf_buffer is not None else None
        self._loop_closure = bool(rospy.get_param("~super_lio_loop_closure", False))
        self._autosave = bool(rospy.get_param("~super_lio_autosave", False))
        self._mapping_launch = roslaunch.rlutil.resolve_launch_arguments(
            ["grinder_scheduler", "super_lio_mapping_managed.launch"]
        )[0]
        self._localization_launch = roslaunch.rlutil.resolve_launch_arguments(
            ["grinder_scheduler", "super_lio_localization_managed.launch"]
        )[0]
        self._task_enable_pub = rospy.Publisher(
            "/chassis/task_enable", Bool, queue_size=1, latch=True
        )
        self._initial_pose_pub = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=1
        )
        rospy.Subscriber("/livox/lidar", PointCloud2, self._lidar_callback, queue_size=10)
        rospy.Subscriber("/livox/imu", Imu, self._imu_callback, queue_size=100)
        rospy.Subscriber("/lio/odom", Odometry, self._localization_odom_callback, queue_size=1)
        rospy.Subscriber("/map", OccupancyGrid, self._map_callback, queue_size=1)
        rospy.Subscriber(
            "/super_lio/registration_diagnostics",
            DiagnosticArray,
            self._diagnostics_callback,
            queue_size=10,
        )
        os.makedirs(self._map_root, exist_ok=True)
        os.makedirs(self._staging_dir(), exist_ok=True)
        self._metrics_exporter = None
        if bool(rospy.get_param("~metrics_export_enabled", True)):
            metrics_dir = rospy.get_param(
                "~metrics_export_dir",
                os.path.join(os.path.dirname(self._map_root), "temp", "grinder_metrics"),
            )
            self._metrics_exporter = MetricsExporter(
                metrics, metrics_dir, "super_lio_mode_manager",
                interval_sec=rospy.get_param("~metrics_export_interval_sec", 10.0),
                logwarn=rospy.logwarn,
            )
            self._metrics_exporter.start()
        self._detect_unowned_conflicts()

        rospy.Service("/super_lio_mode/start_mapping", Trigger, self._service_start_mapping)
        rospy.Service("/super_lio_mode/save_map", SaveSuperLioMap, self._service_save_map)
        rospy.Service(
            "/super_lio_mode/start_localization",
            StartSuperLioLocalization,
            self._service_start_localization,
        )
        rospy.Service("/super_lio_mode/stop", Trigger, self._service_stop)
        rospy.Service("/super_lio_mode/get_status", GetSuperLioStatus, self._handle_status)
        rospy.Service(
            "/super_lio_mode/set_initial_pose",
            SetSuperLioInitialPose,
            self._handle_set_initial_pose,
        )
        rospy.on_shutdown(self.shutdown)
        self._publish_safe()
        rospy.loginfo(
            "Super-LIO mode manager ready: state=%s map_root=%s", self._state, self._map_root
        )

    def _call_on_main_thread(self, handler, request):
        """Run roslaunch operations on the Python main thread.

        rospy services execute in worker threads. roslaunch installs Unix signal
        handlers while creating its process monitor, and Python only permits
        signal registration from the main thread. Queue the complete service
        operation and keep the service callback blocked until the main loop has
        produced its response.
        """
        if threading.get_ident() == self._main_thread_id:
            return handler(request)
        operation = {
            "handler": handler,
            "request": request,
            "event": threading.Event(),
            "response": None,
            "error": None,
        }
        self._operation_queue.put(operation)
        while not operation["event"].wait(0.1):
            if rospy.is_shutdown():
                raise rospy.ServiceException("Super-LIO mode manager is shutting down")
        if operation["error"] is not None:
            raise rospy.ServiceException(str(operation["error"]))
        return operation["response"]

    def _service_start_mapping(self, request):
        return self._call_on_main_thread(self._handle_start_mapping, request)

    def _service_save_map(self, request):
        return self._call_on_main_thread(self._handle_save_map, request)

    def _service_start_localization(self, request):
        return self._call_on_main_thread(self._handle_start_localization, request)

    def _service_stop(self, request):
        return self._call_on_main_thread(self._handle_stop, request)

    def _process_next_operation(self, timeout=0.2):
        try:
            operation = self._operation_queue.get(timeout=timeout)
        except queue.Empty:
            return False
        try:
            operation["response"] = operation["handler"](operation["request"])
        except Exception as exc:
            operation["error"] = exc
            rospy.logerr("Super-LIO main-thread operation failed: %s", exc)
        finally:
            operation["event"].set()
            self._operation_queue.task_done()
        return True

    def spin(self):
        while not rospy.is_shutdown():
            self._process_next_operation()

    def _staging_dir(self):
        return os.path.join(self._map_root, ".staging", "current")

    def _publish_safe(self):
        self._task_enable_pub.publish(Bool(data=False))

    @staticmethod
    def _stamp_sec(message):
        try:
            return float(message.header.stamp.to_sec())
        except Exception:
            return 0.0

    def _record_stream_message(self, topic, message):
        stamp = self._stamp_sec(message)
        with self._health_lock:
            samples = self._stream_samples.get(topic)
            if samples is not None:
                samples.append((time.monotonic(), stamp))

    def _lidar_callback(self, message):
        self._record_stream_message("/livox/lidar", message)

    def _imu_callback(self, message):
        self._record_stream_message("/livox/imu", message)

    def _localization_odom_callback(self, message):
        # Set the event before taking the state lock because startup waits while
        # holding it. Health records are protected separately from lifecycle state.
        self._odom_event.set()
        self._record_stream_message("/lio/odom", message)
        stamp = self._stamp_sec(message)
        matched = []
        with self._health_lock:
            if stamp > 0.0:
                self._pending_odom.append((stamp, time.monotonic(), message))
            matched = self._take_quality_pairs_locked()
        for odom, quality in matched:
            self._evaluate_localization_sample(odom, quality)

    def _diagnostics_callback(self, message):
        arrival = time.monotonic()
        matched = []
        with self._health_lock:
            for status in message.status:
                if str(status.name).strip("/") != "super_lio_registration":
                    continue
                values = {str(item.key): str(item.value) for item in status.values}
                try:
                    quality = {
                        "stamp": self._stamp_sec(message),
                        "arrival": arrival,
                        "ok": int(status.level) == 0,
                        "fitness": float(values.get("fitness_m2", "nan")),
                        "inlier_ratio": float(values.get("inlier_ratio", "nan")),
                        "correspondences": int(values.get("valid_correspondences", "0")),
                        "converged": values.get("converged", "false").lower() == "true",
                    }
                except (TypeError, ValueError):
                    continue
                if quality["stamp"] > 0.0:
                    self._quality_samples.append(quality)
                    matched.extend(self._take_quality_pairs_locked())
        for odom, quality in matched:
            self._evaluate_localization_sample(odom, quality)

    def _take_quality_pairs_locked(self):
        matched = []
        remaining_odom = deque(maxlen=self._pending_odom.maxlen)
        while self._pending_odom:
            sample = self._pending_odom.popleft()
            quality_index = None
            for index, quality in enumerate(self._quality_samples):
                if abs(float(quality["stamp"]) - float(sample[0])) <= 0.025:
                    quality_index = index
                    break
            if quality_index is None:
                if time.monotonic() - sample[1] <= self._message_max_age_sec:
                    remaining_odom.append(sample)
                continue
            quality = self._quality_samples[quality_index]
            del self._quality_samples[quality_index]
            if float(sample[0]) > self._last_quality_stamp:
                self._last_quality_stamp = float(sample[0])
                matched.append((sample[2], quality))
        self._pending_odom.extend(remaining_odom)
        return matched

    def _map_callback(self, message):
        self._map_event.set()
        with self._health_lock:
            self._map_metadata = {
                "arrival": time.monotonic(),
                "stamp": self._stamp_sec(message),
                "frame_id": str(message.header.frame_id or ""),
                "width": int(message.info.width),
                "height": int(message.info.height),
            }

    @staticmethod
    def _quaternion_is_valid(quaternion, tolerance=0.02):
        values = (
            float(quaternion.x), float(quaternion.y),
            float(quaternion.z), float(quaternion.w),
        )
        if not all(math.isfinite(value) for value in values):
            return False
        norm = math.sqrt(sum(value * value for value in values))
        return math.isfinite(norm) and abs(norm - 1.0) <= tolerance

    def _stamp_is_fresh(self, stamp_sec, max_age=None):
        now = rospy.Time.now().to_sec()
        if now <= 0.0 or float(stamp_sec) <= 0.0:
            return False
        age = now - float(stamp_sec)
        return -0.1 <= age <= float(max_age or self._message_max_age_sec)

    def _stream_is_healthy(self, topic, minimum_hz):
        now = time.monotonic()
        with self._health_lock:
            samples = [
                sample for sample in self._stream_samples.get(topic, ())
                if now - sample[0] <= self._health_window_sec
            ]
        if len(samples) < 3 or now - samples[-1][0] > self._message_max_age_sec:
            return False
        stamps = [sample[1] for sample in samples]
        if any(stamp <= 0.0 for stamp in stamps):
            return False
        if any(right <= left for left, right in zip(stamps, stamps[1:])):
            return False
        if not self._stamp_is_fresh(stamps[-1]):
            return False
        elapsed = samples[-1][0] - samples[0][0]
        rate = (len(samples) - 1) / elapsed if elapsed > 0.0 else 0.0
        return rate >= float(minimum_hz)

    def _map_is_fresh(self):
        with self._health_lock:
            metadata = dict(self._map_metadata or {})
        return bool(
            metadata
            and metadata.get("width", 0) > 0
            and metadata.get("height", 0) > 0
            and metadata.get("frame_id") == "map"
            and time.monotonic() - metadata.get("arrival", 0.0) <= self._map_max_age_sec
            and self._stamp_is_fresh(metadata.get("stamp", 0.0), self._map_max_age_sec)
        )

    def _tf_is_valid(self):
        if self._tf_buffer is None:
            return False
        try:
            transform = self._tf_buffer.lookup_transform(
                "map", self._tf_base_frame, rospy.Time(0), rospy.Duration(0.25)
            )
            if not self._stamp_is_fresh(self._stamp_sec(transform)):
                return False
            translation = transform.transform.translation
            if not all(math.isfinite(float(value)) for value in (translation.x, translation.y, translation.z)):
                return False
            return self._quaternion_is_valid(transform.transform.rotation)
        except Exception:
            return False

    @staticmethod
    def _yaw_from_quaternion(quaternion):
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
        )

    def _evaluate_localization_sample(self, odom, quality):
        stamp = self._stamp_sec(odom)
        pose = odom.pose.pose
        covariance = odom.pose.covariance
        position = pose.position
        valid = (
            self._stamp_is_fresh(stamp)
            and str(odom.header.frame_id) == "map"
            and all(math.isfinite(float(value)) for value in (position.x, position.y, position.z))
            and self._quaternion_is_valid(pose.orientation)
            and len(covariance) >= 36
            and all(math.isfinite(float(covariance[index])) for index in (0, 7, 35))
            and 0.0 <= float(covariance[0]) <= self._max_xy_variance
            and 0.0 <= float(covariance[7]) <= self._max_xy_variance
            and 0.0 <= float(covariance[35]) <= self._max_yaw_variance
            and bool(quality.get("ok"))
            and bool(quality.get("converged"))
            and math.isfinite(float(quality.get("fitness", float("nan"))))
            and float(quality.get("fitness", float("inf"))) <= self._fitness_max
            and math.isfinite(float(quality.get("inlier_ratio", float("nan"))))
            and float(quality.get("inlier_ratio", 0.0)) >= self._inlier_ratio_min
            and int(quality.get("correspondences", 0)) >= self._min_correspondences
            and abs(float(quality.get("stamp", 0.0)) - stamp) <= 0.025
            and time.monotonic() - float(quality.get("arrival", 0.0)) <= self._message_max_age_sec
            and self._stream_is_healthy("/lio/odom", self._min_odom_hz)
            and self._tf_is_valid()
        )
        x = float(position.x)
        y = float(position.y)
        yaw = self._yaw_from_quaternion(pose.orientation)
        with self._lock:
            if (
                self._owned_mode != "localization"
                or not self._initial_pose_received
                or self._state not in (self.RELOCALIZING, self.READY)
            ):
                return
            self._registration_quality_valid = bool(valid)
            self._registration_fitness = float(quality.get("fitness", 0.0) or 0.0)
            self._registration_inlier_ratio = float(quality.get("inlier_ratio", 0.0) or 0.0)
            if not valid:
                self._localization_good_frames = 0
                self._localization_ready = False
                self._state = self.RELOCALIZING
                self._message = "registration quality below readiness thresholds"
                self._last_good_pose = None
                self._last_good_pose_stamp = 0.0
                return
            if self._last_good_pose is not None:
                dt = stamp - self._last_good_pose_stamp
                distance = math.hypot(x - self._last_good_pose[0], y - self._last_good_pose[1])
                yaw_delta = abs(math.atan2(
                    math.sin(yaw - self._last_good_pose[2]),
                    math.cos(yaw - self._last_good_pose[2]),
                ))
                if dt <= 0.0 or distance > self._max_pose_jump_m or yaw_delta > self._max_yaw_jump_rad:
                    self._localization_good_frames = 0
                    self._localization_ready = False
                    self._state = self.RELOCALIZING
                    self._message = "pose jump exceeded readiness thresholds"
                    self._last_good_pose = (x, y, yaw)
                    self._last_good_pose_stamp = stamp
                    return
            self._last_good_pose = (x, y, yaw)
            self._last_good_pose_stamp = stamp
            self._localization_good_frames += 1
            if self._localization_good_frames >= self._required_good_frames:
                if not self._localization_ready:
                    metrics.counter("super_lio_localization_ready").inc()
                    metrics.event("super_lio_operation", operation="localization", result="ready")
                self._state = self.READY
                self._localization_ready = True
                self._message = "localization ready"

    def _handle_set_initial_pose(self, request):
        response = SetSuperLioInitialPoseResponse()
        with self._lock:
            requested_map_id = self._safe_map_id(request.map_id)
            requested_revision = str(request.map_revision or "").strip().lower()
            pose = request.pose
            stamp = self._stamp_sec(pose)
            quaternion = pose.pose.pose.orientation
            message = ""
            if self._state not in (self.LOCALIZING, self.RELOCALIZING, self.READY) or self._owned_mode != "localization":
                message = "Super-LIO localization is not active"
            elif requested_map_id != self._active_map_id or requested_revision != self._active_map_revision:
                message = "initial_pose_map_identity_mismatch"
            elif not self._bundle_dir or not os.path.isdir(self._bundle_dir):
                message = "active_map_bundle_unavailable"
            elif str(pose.header.frame_id) != "map" or not self._stamp_is_fresh(stamp):
                message = "initial_pose_frame_or_timestamp_invalid"
            elif not self._quaternion_is_valid(quaternion):
                message = "initial_pose_quaternion_invalid"
            elif not all(math.isfinite(float(value)) for value in (
                pose.pose.pose.position.x,
                pose.pose.pose.position.y,
                pose.pose.pose.position.z,
            )):
                message = "initial_pose_position_invalid"
            else:
                try:
                    current_revision = get_or_compute_map_asset_revision(self._bundle_dir)
                except Exception:
                    current_revision = ""
                if current_revision != self._active_map_revision:
                    self._state = self.ERROR
                    self._localization_ready = False
                    message = "active_map_asset_revision_changed"
                covariance = pose.pose.covariance
                if message:
                    pass
                elif len(covariance) < 36 or not all(math.isfinite(float(covariance[index])) for index in (0, 7, 35)):
                    message = "initial_pose_covariance_invalid"
                elif any(float(covariance[index]) < 0.0 for index in (0, 7, 35)):
                    message = "initial_pose_covariance_negative"
            if message:
                response.success = False
                response.message = message
            else:
                self._initial_pose_received = True
                self._localization_ready = False
                self._localization_good_frames = 0
                self._registration_quality_valid = False
                self._last_good_pose = None
                self._last_good_pose_stamp = 0.0
                self._state = self.RELOCALIZING
                self._message = "initial pose accepted; waiting for stable registration"
                self._initial_pose_pub.publish(pose)
                response.success = True
                response.message = "initial_pose_published"
            response.state = self._state
            response.active_map_id = self._active_map_id
            response.active_map_revision = self._active_map_revision
        return response

    @staticmethod
    def _safe_map_id(value):
        text = re.sub(r"[^0-9A-Za-z._-]", "_", str(value or "").strip()).strip("._")
        return text[:96]

    def _bundle_path(self, map_id):
        safe_id = self._safe_map_id(map_id)
        if not safe_id:
            raise RuntimeError("map_id is empty or invalid")
        path = os.path.abspath(os.path.join(self._map_root, safe_id))
        if os.path.commonpath([path, self._map_root]) != self._map_root:
            raise RuntimeError("map bundle escapes configured map root")
        return path

    def _safe_rmtree(self, path):
        target = os.path.realpath(os.path.abspath(path))
        root = os.path.realpath(self._map_root)
        if target == root or os.path.commonpath([target, root]) != root:
            raise RuntimeError("refuse to remove path outside map root: {}".format(target))
        if os.path.isdir(target):
            shutil.rmtree(target)

    @staticmethod
    def _node_names():
        try:
            return set(rosnode.get_node_names())
        except Exception:
            return set()

    @staticmethod
    def _lookup_node_uri(node_name):
        try:
            code, _message, uri = rospy.get_master().lookupNode(rospy.get_name(), node_name)
            return str(uri) if int(code) == 1 and uri else ""
        except Exception:
            return ""

    @staticmethod
    def _node_uri_is_reachable(uri):
        if not uri:
            return False
        try:
            with ServerProxy(uri, allow_none=True) as node:
                code, _message, _pid = node.getPid(rospy.get_name())
            return int(code) == 1
        except Exception:
            return False

    def _live_managed_nodes(self):
        """Return reachable nodes registered under names reserved by the manager."""
        live = set()
        for node_name in sorted(self._node_names() & self._MANAGED_NODES):
            if self._node_uri_is_reachable(self._lookup_node_uri(node_name)):
                live.add(node_name)
        return live

    def _refresh_owned_node_uris(self):
        if not self._accept_new_owned_node_uris:
            return
        for node_name in sorted(self._node_names() & self._expected_owned_node_names):
            if node_name in self._owned_node_uris:
                continue
            uri = self._lookup_node_uri(node_name)
            if uri and self._node_uri_is_reachable(uri):
                self._owned_node_uris[node_name] = uri

    def _owned_residual_nodes(self):
        self._refresh_owned_node_uris()
        residual = []
        current_names = self._node_names()
        for node_name, owned_uri in sorted(self._owned_node_uris.items()):
            if self._node_uri_is_reachable(owned_uri):
                residual.append(node_name)
                continue
            # A different process can register the same ROS name while an old
            # launch child is shutting down. Report it but never signal it.
            if node_name in current_names:
                current_uri = self._lookup_node_uri(node_name)
                if current_uri and current_uri != owned_uri and self._node_uri_is_reachable(current_uri):
                    residual.append(node_name)
        return sorted(set(residual))

    def _detect_unowned_conflicts(self):
        conflicts = sorted(self._live_managed_nodes())
        if conflicts:
            self._state = self.ERROR
            self._message = "unowned Super-LIO nodes already running: {}".format(", ".join(conflicts))
            rospy.logerr(self._message)

    def _assert_no_unowned_nodes(self):
        if self._launch_parent is not None:
            return
        conflicts = sorted(self._live_managed_nodes())
        if conflicts:
            raise RuntimeError("unowned Super-LIO nodes already running: {}".format(", ".join(conflicts)))

    def _assert_livox_single_owner(self):
        try:
            code, _message, system_state = rospy.get_master().getSystemState()
            if int(code) != 1:
                raise RuntimeError("ROS master did not return its publisher state")
            publishers = dict((str(topic), set(nodes)) for topic, nodes in system_state[0])
        except Exception as exc:
            raise RuntimeError("unable to inspect Livox ROS publishers: {}".format(exc))
        lidar_nodes = publishers.get("/livox/lidar", set())
        imu_nodes = publishers.get("/livox/imu", set())
        if len(lidar_nodes) != 1 or lidar_nodes != imu_nodes:
            raise RuntimeError(
                "Livox driver must be running once and publish both /livox/lidar and /livox/imu; "
                "lidar_publishers={} imu_publishers={}".format(
                    ",".join(sorted(lidar_nodes)) or "<none>",
                    ",".join(sorted(imu_nodes)) or "<none>",
                )
            )

    def _assert_configured_ports_free(self):
        for item in self._preflight_ports:
            if not isinstance(item, dict):
                raise RuntimeError("super_lio_preflight_ports entries must be maps")
            host = str(item.get("host", "0.0.0.0"))
            port = int(item.get("port", 0))
            protocol = str(item.get("protocol", "udp")).strip().lower()
            if not (1 <= port <= 65535) or protocol not in ("tcp", "udp"):
                raise RuntimeError("invalid Super-LIO preflight port entry: {}".format(item))
            sock_type = socket.SOCK_STREAM if protocol == "tcp" else socket.SOCK_DGRAM
            sock = socket.socket(socket.AF_INET, sock_type)
            try:
                sock.bind((host, port))
            except OSError as exc:
                raise RuntimeError(
                    "Super-LIO managed {} port is already in use: {}:{} ({})".format(
                        protocol, host, port, exc,
                    )
                )
            finally:
                sock.close()

    def _start_launch(self, launch_file, arguments, owned_mode):
        self._assert_no_unowned_nodes()
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        parent = roslaunch.parent.ROSLaunchParent(uuid, [(launch_file, list(arguments))])
        parent.start()
        self._launch_parent = parent
        self._owned_mode = owned_mode
        self._expected_owned_node_names = set(
            self._MAPPING_NODES if owned_mode == "mapping" else self._LOCALIZATION_NODES
        )
        self._owned_node_uris = {}
        self._accept_new_owned_node_uris = True
        self._refresh_owned_node_uris()

    def _stop_owned_launch(self):
        parent = self._launch_parent
        self._state = self.STOPPING
        self._refresh_owned_node_uris()
        self._accept_new_owned_node_uris = False
        if parent is not None:
            try:
                parent.shutdown()
            except Exception as exc:
                rospy.logwarn("roslaunch parent shutdown raised: %s", exc)

        deadline = time.monotonic() + self._shutdown_timeout
        while time.monotonic() < deadline:
            self._refresh_owned_node_uris()
            if not self._owned_residual_nodes():
                break
            rospy.sleep(0.1)

        residual = self._owned_residual_nodes()
        if residual:
            # Signal only the exact XML-RPC URIs captured from this launch.
            # Never call rosnode.kill_nodes(name), which can target a new,
            # unowned process that reused a ROS node name.
            for node_name in sorted(residual):
                uri = self._owned_node_uris.get(node_name, "")
                if not uri or not self._node_uri_is_reachable(uri):
                    continue
                try:
                    with ServerProxy(uri, allow_none=True) as node:
                        node.shutdown(rospy.get_name(), "managed Super-LIO launch cleanup")
                except Exception as exc:
                    rospy.logwarn("Failed to request shutdown of owned node %s: %s", node_name, exc)

            deadline = time.monotonic() + self._cleanup_timeout
            while time.monotonic() < deadline:
                if not self._owned_residual_nodes():
                    break
                rospy.sleep(0.1)

        residual = self._owned_residual_nodes()
        if residual:
            self._residual_nodes = residual
            raise SuperLioShutdownTimeout(residual)

        self._launch_parent = None
        self._owned_mode = ""
        self._owned_node_uris = {}
        self._expected_owned_node_names = set()
        self._accept_new_owned_node_uris = False
        self._residual_nodes = []

    def _wait_service(self, name):
        rospy.wait_for_service(name, timeout=self._startup_timeout)

    def _wait_topic_event(self, event, topic):
        deadline = time.monotonic() + self._startup_timeout
        while not rospy.is_shutdown():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise RuntimeError(
                    "timeout exceeded while waiting for message on topic {}".format(topic)
                )
            if event.wait(min(0.2, remaining)):
                return
        raise RuntimeError("shutdown while waiting for message on topic {}".format(topic))

    def _clear_health_samples(self):
        with self._health_lock:
            for samples in self._stream_samples.values():
                samples.clear()
            self._pending_odom.clear()
            self._quality_samples.clear()
            self._last_quality_stamp = 0.0
            self._map_metadata = None

    def _wait_for_health(self, predicate, description):
        deadline = time.monotonic() + self._startup_timeout
        while not rospy.is_shutdown():
            self._refresh_owned_node_uris()
            if predicate():
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise RuntimeError("startup health timeout: {}".format(description))
            rospy.sleep(min(0.1, remaining))
        raise RuntimeError("shutdown while waiting for startup health: {}".format(description))

    def _assert_livox_stream_health(self):
        return (
            self._stream_is_healthy("/livox/lidar", self._min_lidar_hz)
            and self._stream_is_healthy("/livox/imu", self._min_imu_hz)
        )

    def _clear_localization_quality(self):
        self._initial_pose_received = False
        self._localization_ready = False
        self._localization_good_frames = 0
        self._registration_quality_valid = False
        self._registration_fitness = 0.0
        self._registration_inlier_ratio = 0.0
        self._last_good_pose = None
        self._last_good_pose_stamp = 0.0

    def _start_mapping_locked(self):
        self._publish_safe()
        if self._state == self.MAPPING and self._owned_mode == "mapping":
            return "mapping already active"
        self._stop_owned_launch()
        self._assert_no_unowned_nodes()
        self._assert_livox_single_owner()
        self._assert_configured_ports_free()
        self._state = self.STARTING
        self._message = "starting mapping; waiting for fresh topics and valid TF"
        self._odom_event.clear()
        self._map_event.clear()
        self._clear_health_samples()
        staging = self._staging_dir()
        if os.path.isdir(staging):
            self._safe_rmtree(staging)
        os.makedirs(staging, exist_ok=True)
        args = [
            "staging_dir:={}".format(staging),
            "loop_closure:={}".format(str(self._loop_closure).lower()),
            "autosave:={}".format(str(self._autosave).lower()),
        ]
        try:
            rospy.loginfo("Starting managed Super-LIO mapping launch")
            self._start_launch(self._mapping_launch, args, "mapping")
            self._wait_service("/lio/get_3dmap")
            self._wait_service("/cloud_to_occupancy_grid/save_map")
            self._wait_service("/cloud_to_occupancy_grid/reset_map")
            rospy.loginfo("Super-LIO mapping services ready; waiting for fresh Livox/odometry streams")
            self._wait_topic_event(self._odom_event, "/lio/odom")
            self._wait_topic_event(self._map_event, "/map")
            self._wait_for_health(
                lambda: (
                    self._assert_livox_stream_health()
                    and self._stream_is_healthy("/lio/odom", self._min_odom_hz)
                    and self._map_is_fresh()
                    and self._tf_is_valid()
                ),
                "fresh /livox/lidar, /livox/imu, /lio/odom, /map and valid map->{} TF".format(self._tf_base_frame),
            )
        except Exception as startup_error:
            try:
                self._stop_owned_launch()
            except SuperLioShutdownTimeout:
                raise
            raise startup_error
        self._residual_nodes = []
        self._state = self.MAPPING
        self._message = "mapping ready"
        self._active_map_id = "LIVE_MAP"
        self._active_map_revision = ""
        self._bundle_dir = ""
        self._clear_localization_quality()
        return self._message

    @staticmethod
    def _validate_pcd(path):
        if not os.path.isfile(path) or os.path.getsize(path) <= 0:
            raise RuntimeError("PCD is missing or empty: {}".format(path))
        points = 0
        with open(path, "rb") as handle:
            for _ in range(80):
                line = handle.readline()
                if not line:
                    break
                text = line.decode("ascii", errors="ignore").strip()
                if text.upper().startswith("POINTS "):
                    try:
                        points = int(text.split(None, 1)[1])
                    except Exception:
                        points = 0
                if text.upper().startswith("DATA "):
                    break
        if points <= 0:
            raise RuntimeError("PCD contains no valid points: {}".format(path))

    @staticmethod
    def _validate_grid(yaml_path, image_path):
        if not os.path.isfile(yaml_path) or os.path.getsize(yaml_path) <= 0:
            raise RuntimeError("map YAML is missing or empty: {}".format(yaml_path))
        if not os.path.isfile(image_path) or os.path.getsize(image_path) <= 0:
            raise RuntimeError("map image is missing or empty: {}".format(image_path))
        with open(yaml_path, "r", encoding="utf-8") as handle:
            yaml_text = handle.read()
        if not re.search(r"(?m)^\s*image\s*:\s*map\.pgm\s*$", yaml_text):
            raise RuntimeError("map YAML must reference map.pgm")

    def _validate_bundle(self, bundle_dir):
        paths = {
            "loc": os.path.join(bundle_dir, "loc_map.pcd"),
            "plan": os.path.join(bundle_dir, "plan_map.pcd"),
            "yaml": os.path.join(bundle_dir, "map.yaml"),
            "image": os.path.join(bundle_dir, "map.pgm"),
        }
        self._validate_pcd(paths["loc"])
        self._validate_pcd(paths["plan"])
        self._validate_grid(paths["yaml"], paths["image"])
        return paths

    @staticmethod
    def _fsync_file(path):
        with open(path, "rb") as handle:
            os.fsync(handle.fileno())

    @staticmethod
    def _fsync_directory(path):
        flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
        descriptor = os.open(path, flags)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)

    def _fsync_bundle(self, bundle_dir):
        for filename in (
            "loc_map.pcd", "plan_map.pcd", "map.yaml", "map.pgm", "map_info.json",
        ):
            self._fsync_file(os.path.join(bundle_dir, filename))
        self._fsync_directory(bundle_dir)

    def _start_localization_locked(self, map_id):
        self._publish_safe()
        bundle = self._bundle_path(map_id)
        self._validate_bundle(bundle)
        map_revision = get_or_compute_map_asset_revision(bundle)
        if (
            self._state in (self.LOCALIZING, self.RELOCALIZING, self.READY)
            and self._owned_mode == "localization"
            and self._active_map_id == map_id
            and self._active_map_revision == map_revision
        ):
            return bundle, "localization already active"
        self._stop_owned_launch()
        self._assert_no_unowned_nodes()
        self._assert_livox_single_owner()
        self._assert_configured_ports_free()
        self._state = self.STARTING
        self._message = "starting localization; waiting for fresh map and Livox streams"
        self._odom_event.clear()
        self._map_event.clear()
        self._clear_health_samples()
        args = ["bundle_dir:={}".format(bundle)]
        for key in (
            "base_to_laser_x", "base_to_laser_y", "base_to_laser_z",
            "base_to_laser_roll", "base_to_laser_pitch", "base_to_laser_yaw",
        ):
            args.append("{}:={}".format(key, rospy.get_param("~" + key, 0.0)))
        try:
            self._start_launch(self._localization_launch, args, "localization")
            self._wait_topic_event(self._map_event, "/map")
            self._wait_for_health(
                lambda: self._assert_livox_stream_health() and self._map_is_fresh(),
                "fresh /livox/lidar, /livox/imu and map_server /map",
            )
        except Exception as startup_error:
            try:
                self._stop_owned_launch()
            except SuperLioShutdownTimeout:
                raise
            raise startup_error
        self._residual_nodes = []
        self._state = self.LOCALIZING
        self._message = "localization starting; map loaded; initial pose required"
        self._active_map_id = map_id
        self._active_map_revision = map_revision
        self._bundle_dir = bundle
        self._clear_localization_quality()
        return bundle, self._message

    @metrics.timed("super_lio_start_mapping_ms")
    def _handle_start_mapping(self, _request):
        # 生命周期埋点只记录操作边界和结果，不记录地图文件内容。
        metrics.counter("super_lio_start_mapping_attempts").inc()
        with self._lock:
            try:
                message = self._start_mapping_locked()
                metrics.counter("super_lio_start_mapping_succeeded").inc()
                metrics.event("super_lio_operation", operation="start_mapping", result="succeeded")
                return TriggerResponse(success=True, message=message)
            except Exception as exc:
                _record_operation_failure("start_mapping", exc)
                self._state = self.TIMEOUT if isinstance(exc, SuperLioShutdownTimeout) else self.ERROR
                self._message = str(exc)
                self._residual_nodes = list(getattr(exc, "residual_nodes", []))
                self._publish_safe()
                return TriggerResponse(success=False, message=self._message)

    @metrics.timed("super_lio_save_map_ms")
    def _handle_save_map(self, request):
        metrics.counter("super_lio_save_map_attempts").inc()
        response = SaveSuperLioMapResponse()
        with self._lock:
            temp_dir = ""
            bundle_saved = False
            map_revision = ""
            try:
                if self._state != self.MAPPING or self._owned_mode != "mapping":
                    raise RuntimeError("map save requires active MAPPING state")
                map_id = self._safe_map_id(request.map_id)
                if not map_id:
                    raise RuntimeError("map_id is empty or invalid")
                final_dir = self._bundle_path(map_id)
                temp_dir = tempfile.mkdtemp(prefix=".tmp-{}-".format(map_id), dir=self._map_root)
                self._state = self.SAVING
                self._message = "saving map"
                self._publish_safe()

                self._wait_service("/lio/get_3dmap")
                get_map = rospy.ServiceProxy("/lio/get_3dmap", GetMap)
                map_request = GetMapRequest()
                map_request.cmd_id = 1
                map_request.enable_filter = True
                map_request.map_dir = temp_dir
                map_request.map_name = "map"
                map_response = get_map(map_request)
                if not bool(map_response.success):
                    raise RuntimeError("/lio/get_3dmap failed")

                self._wait_service("/cloud_to_occupancy_grid/save_map")
                save_grid = rospy.ServiceProxy("/cloud_to_occupancy_grid/save_map", Trigger)
                grid_response = save_grid()
                if not bool(grid_response.success):
                    raise RuntimeError(grid_response.message or "2D map save failed")
                for filename in ("map.yaml", "map.pgm"):
                    source = os.path.join(self._staging_dir(), filename)
                    if not os.path.isfile(source):
                        raise RuntimeError("staged map file missing: {}".format(source))
                    shutil.copy2(source, os.path.join(temp_dir, filename))

                paths = self._validate_bundle(temp_dir)
                map_revision = compute_map_asset_revision(temp_dir)
                manifest = {
                    "schemaVersion": 3,
                    "mapId": map_id,
                    "mapName": str(request.map_name or ""),
                    "assetRevision": map_revision,
                    "savedAt": int(time.time()),
                    "files": {
                        "localization": "loc_map.pcd",
                        "planning": "plan_map.pcd",
                        "yaml": "map.yaml",
                        "image": "map.pgm",
                    },
                }
                manifest_path = os.path.join(temp_dir, "map_info.json")
                with open(manifest_path, "w", encoding="utf-8") as handle:
                    json.dump(manifest, handle, ensure_ascii=False, indent=2)
                    handle.flush()
                    os.fsync(handle.fileno())
                self._fsync_bundle(temp_dir)
                if os.path.exists(final_dir):
                    raise RuntimeError("map bundle already exists: {}".format(final_dir))
                if os.stat(temp_dir).st_dev != os.stat(self._map_root).st_dev:
                    raise RuntimeError("temporary map bundle must be on the destination filesystem")
                os.replace(temp_dir, final_dir)
                self._fsync_directory(self._map_root)
                temp_dir = ""
                paths = {
                    "loc": os.path.join(final_dir, "loc_map.pcd"),
                    "plan": os.path.join(final_dir, "plan_map.pcd"),
                    "yaml": os.path.join(final_dir, "map.yaml"),
                    "image": os.path.join(final_dir, "map.pgm"),
                }
                bundle_saved = True

                try:
                    self._stop_owned_launch()
                    response.mapping_stopped = True
                    metrics.counter("super_lio_stop_mapping").inc()
                except SuperLioShutdownTimeout as stop_error:
                    self._state = self.TIMEOUT
                    self._message = "map_saved_but_mapping_stop_timeout: {}".format(stop_error)
                    self._residual_nodes = list(stop_error.residual_nodes)
                    response.mapping_stopped = False
                    response.residual_nodes = list(self._residual_nodes)
                # Saving a map is the end of the mapping session.  Do not
                # immediately start localization here: the task-start path
                # owns that transition so an idle robot does not keep the
                # localization stack alive after an APP save operation.
                if self._state != self.TIMEOUT:
                    self._state = self.IDLE
                    self._message = "map saved; mapping stopped; localization pending"
                self._active_map_id = map_id
                self._active_map_revision = map_revision
                self._bundle_dir = final_dir
                self._clear_localization_quality()

                response.success = True
                response.localization_started = False
                response.message = (
                    "map_saved_mapping_stop_timeout"
                    if self._state == self.TIMEOUT
                    else "map_saved_localization_pending"
                )
                response.state = self._state
                response.bundle_dir = final_dir
                response.loc_pcd_path = paths["loc"]
                response.plan_pcd_path = paths["plan"]
                response.yaml_path = paths["yaml"]
                response.image_path = paths["image"]
                response.asset_revision = map_revision
                if bundle_saved:
                    metrics.counter("super_lio_save_map_succeeded").inc()
                metrics.event("super_lio_operation", operation="save_map", result="succeeded")
                return response
            except Exception as exc:
                _record_operation_failure("save_map", exc)
                if temp_dir and os.path.isdir(temp_dir):
                    try:
                        self._safe_rmtree(temp_dir)
                    except Exception as cleanup_exc:
                        rospy.logwarn("Failed to remove temporary map bundle: %s", cleanup_exc)
                # A failed save must leave the running mapper available for retry.
                if self._owned_mode == "mapping" and self._launch_parent is not None:
                    self._state = self.MAPPING
                else:
                    self._state = self.ERROR
                self._message = str(exc)
                self._publish_safe()
                response.success = False
                response.localization_started = False
                response.message = self._message
                response.state = self._state
                response.mapping_stopped = False
                response.residual_nodes = list(self._residual_nodes)
                return response

    @metrics.timed("super_lio_start_localization_ms")
    def _handle_start_localization(self, request):
        metrics.counter("super_lio_start_localization_attempts").inc()
        response = StartSuperLioLocalizationResponse()
        with self._lock:
            try:
                bundle, message = self._start_localization_locked(self._safe_map_id(request.map_id))
                response.success = True
                response.message = message
                response.state = self._state
                response.bundle_dir = bundle
                response.yaml_path = os.path.join(bundle, "map.yaml")
                response.image_path = os.path.join(bundle, "map.pgm")
                response.asset_revision = self._active_map_revision
                metrics.counter("super_lio_start_localization_succeeded").inc()
                metrics.event("super_lio_operation", operation="start_localization", result="succeeded")
            except Exception as exc:
                _record_operation_failure("start_localization", exc)
                self._state = self.TIMEOUT if isinstance(exc, SuperLioShutdownTimeout) else self.ERROR
                self._message = str(exc)
                self._residual_nodes = list(getattr(exc, "residual_nodes", []))
                self._publish_safe()
                response.success = False
                response.message = self._message
                response.state = self._state
            return response

    @metrics.timed("super_lio_stop_ms")
    def _handle_stop(self, _request):
        metrics.counter("super_lio_stop_attempts").inc()
        with self._lock:
            try:
                stopped_mapping = self._state == self.MAPPING
                self._publish_safe()
                self._stop_owned_launch()
                if stopped_mapping:
                    metrics.counter("super_lio_stop_mapping").inc()
                self._state = self.IDLE
                self._message = "stopped"
                self._active_map_id = ""
                self._active_map_revision = ""
                self._bundle_dir = ""
                self._clear_localization_quality()
                self._residual_nodes = []
                metrics.counter("super_lio_stop_succeeded").inc()
                metrics.event("super_lio_operation", operation="stop", result="succeeded")
                return TriggerResponse(success=True, message=self._message)
            except Exception as exc:
                _record_operation_failure("stop", exc)
                self._state = self.TIMEOUT if isinstance(exc, SuperLioShutdownTimeout) else self.ERROR
                self._message = str(exc)
                self._residual_nodes = list(getattr(exc, "residual_nodes", []))
                return TriggerResponse(success=False, message=self._message)

    def _handle_status(self, _request):
        with self._lock:
            return GetSuperLioStatusResponse(
                success=self._state not in (self.ERROR, self.TIMEOUT),
                message=self._message,
                state=self._state,
                active_map_id=self._active_map_id,
                bundle_dir=self._bundle_dir,
                localization_ready=self._localization_ready,
                active_map_revision=self._active_map_revision,
                localization_good_frames=self._localization_good_frames,
                localization_required_frames=self._required_good_frames,
                registration_quality_valid=self._registration_quality_valid,
                registration_fitness=self._registration_fitness,
                registration_inlier_ratio=self._registration_inlier_ratio,
                residual_nodes=list(self._residual_nodes),
            )

    def shutdown(self):
        if getattr(self, "_metrics_exporter", None) is not None:
            self._metrics_exporter.stop()
        with self._lock:
            self._publish_safe()
            try:
                self._stop_owned_launch()
            except SuperLioShutdownTimeout as exc:
                self._state = self.TIMEOUT
                self._message = str(exc)
                self._residual_nodes = list(exc.residual_nodes)
                rospy.logerr(self._message)


def main():
    rospy.init_node("super_lio_mode_manager")
    manager = SuperLioModeManager()
    manager.spin()
