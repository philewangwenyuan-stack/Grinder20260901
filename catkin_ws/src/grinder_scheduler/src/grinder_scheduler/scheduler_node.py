#!/usr/bin/env python3

import json
import math
import os
import re
import shutil
import struct
import threading
import time
import zlib
import base64
import bisect
from datetime import datetime
from collections import deque
from copy import deepcopy
from dataclasses import asdict

import cv2
import numpy as np
import rospy
try:
    from dynamic_reconfigure.client import Client as DynamicReconfigureClient
except Exception:
    DynamicReconfigureClient = None
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Pose, PoseStamped, Twist
from grinder_chassis_driver.msg import ChassisStatus, WheelSpeedCommand, WheelSpeedState
from grinder_chassis_driver.srv import EnableChassis
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav_msgs.srv import LoadMap
from std_msgs.msg import Bool, Int16, UInt16
from std_srvs.srv import Trigger, TriggerResponse

try:
    import tf2_ros
    from tf.transformations import euler_from_quaternion
except Exception:
    tf2_ros = None
    euler_from_quaternion = None

from grinder_scheduler.aurora_bridge import AuroraBridge
from grinder_scheduler.local_rtsp_server import LocalRtspStreamServer
from grinder_scheduler.map_service import MapService
from grinder_scheduler.media_streamer import FFmpegMediaStreamer
from grinder_scheduler.map_catalog_response import fill_map_catalog_response
from grinder_scheduler.models import (
    PlannerPath,
    SchedulerState,
    TaskConfigModel,
    VideoStreamState,
    is_planning_direction,
    normalize_planning_direction,
)
from grinder_scheduler.msg import MapPreviewMetadata, SchedulerStatus
from grinder_scheduler.planner_adapter import PlannerAdapter
from grinder_scheduler.platform_integration import MqttDeviceReporter, PlatformFileSync
from grinder_scheduler.sl_linka_adapter import SlLinkAServer

try:
    from slamware_ros_sdk.srv import (
        RelocalizationRequest as RadarRelocalizationService,
        SyncGetStcm,
        SyncSetStcm,
    )
    from slamware_ros_sdk.msg import (
        ClearMapRequest,
        MapKind,
        SetMapLocalizationRequest,
        SetMapUpdateRequest,
        SyncMapRequest,
        RelocalizationStatus as RadarRelocalizationStatus,
        SystemStatus as RadarSystemStatus,
    )
except Exception:
    SyncGetStcm = None
    SyncSetStcm = None
    RadarRelocalizationService = None
    ClearMapRequest = None
    MapKind = None
    SetMapLocalizationRequest = None
    SetMapUpdateRequest = None
    SyncMapRequest = None
    RadarRelocalizationStatus = None
    RadarSystemStatus = None

PREVIEW_MAX_EDGE_CAP = 640
DEFAULT_LIVE_MAP_ID = "LIVE_MAP"


def _format_ts_s(ts_value):
    try:
        ts = int(float(ts_value))
    except Exception:
        return ""
    if ts <= 0:
        return ""
    return datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M:%S")


def _detect_runtime_base_dir():
    env_base = os.environ.get("GRINDER_BASE_DIR", "").strip()
    if env_base:
        return os.path.abspath(os.path.expanduser(os.path.expandvars(env_base)))
    # scheduler_node.py -> grinder_scheduler/src/grinder_scheduler/scheduler_node.py
    # repo root (Grinder) is 5 levels up.
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../../"))


def _resolve_runtime_path(path_value, base_dir):
    if path_value is None:
        return ""
    normalized = os.path.expanduser(os.path.expandvars(str(path_value).strip()))
    if not normalized:
        return ""
    if os.path.isabs(normalized):
        return os.path.normpath(normalized)
    return os.path.normpath(os.path.join(base_dir, normalized))


class SchedulerNode:
    @staticmethod
    def _sanitize_preview_edge(value, default_edge=PREVIEW_MAX_EDGE_CAP, cap_edge=PREVIEW_MAX_EDGE_CAP):
        if value is None:
            edge = int(default_edge)
        else:
            try:
                edge = int(value)
            except Exception:
                edge = int(default_edge)
        edge = max(64, edge)
        return min(int(cap_edge), edge)

    def __init__(self):
        self.state = SchedulerState.IDLE
        self._collision_imminent_topic = rospy.get_param(
            "~collision_imminent_topic", "/rpp/collision_imminent"
        )
        self._collision_imminent_lock = threading.Lock()
        self._collision_imminent_active = False
        self._collision_pause_latched = False
        self.task_config = TaskConfigModel()
        self._load_robot_config_from_params()
        self._progress_history = deque(maxlen=180)
        self.last_error = ""
        self.replan_requested = False
        self.current_path = None
        self.current_path_index = 0
        self._path_arc_lengths = []
        self._active_segments = []
        self._active_segment_index = 0
        self._active_segment_start_s = 0.0
        self._active_segment_last_progress_s = 0.0
        self._active_segment_stall_progress_s = 0.0
        self._active_segment_last_progress_update_time = time.time()
        self._active_segment_last_switch_time = 0.0
        self._active_segment_goal_sent_index = -1
        self._goal_points = []
        self._goal_segment_yaw_threshold_deg = max(
            1.0, float(rospy.get_param("~path_goal_segment_yaw_threshold_deg", 25.0))
        )
        self._corner_mid_enabled = bool(rospy.get_param("~path_corner_mid_enabled", True))
        self._corner_mid_min_length = max(0.05, float(rospy.get_param("~path_corner_mid_min_length", 0.05)))
        self._corner_mid_short_ratio = max(0.1, float(rospy.get_param("~path_corner_mid_short_ratio", 0.6)))
        self._corner_mid_parallel_threshold_deg = max(
            1.0, float(rospy.get_param("~path_corner_mid_parallel_threshold_deg", 35.0))
        )
        self._corner_mid_turn_min_angle_deg = max(
            1.0, float(rospy.get_param("~path_corner_mid_turn_min_angle_deg", 45.0))
        )
        self.last_chassis_status = None
        self.last_wheel_speed_state = None
        self._wheel_speed_state_lock = threading.Lock()
        self._wheel_odom_lock = threading.Lock()
        self._wheel_odom_topic = str(
            rospy.get_param("~wheel_odom_topic", "/odom_wheel")
        ).strip() or "/odom_wheel"
        self._wheel_odom_timeout_sec = max(
            0.1, float(rospy.get_param("~wheel_odom_timeout_sec", 1.0))
        )
        self._wheel_odom_linear_speed_mps = 0.0
        self._wheel_odom_angular_speed_radps = 0.0
        self._wheel_odom_received_monotonic = 0.0
        self._wheel_speed_feedback_radius_m = max(
            1.0e-6, float(rospy.get_param("~wheel_speed_feedback_radius_m", 0.1475))
        )
        self._wheel_speed_feedback_gear_ratio = max(
            1.0e-6, float(rospy.get_param("~wheel_speed_feedback_gear_ratio", 60.0))
        )
        self._exec_mode = str(rospy.get_param("~path_execution_mode", "move_base_goal")).strip().lower()
        if self._exec_mode != "move_base_goal":
            rospy.logwarn(
                "path_execution_mode=%s is deprecated in scheduler; force to move_base_goal (cmd_vel now handled by chassis_driver)",
                self._exec_mode,
            )
            self._exec_mode = "move_base_goal"
        # Segment execution uses this only to trigger move_base with the active segment endpoint.
        self._exec_goal_topic = rospy.get_param("~path_goal_topic", "/move_base_simple/goal")
        self._task_enable_topic = rospy.get_param("~task_enable_topic", "/chassis/task_enable")
        self._task_enable_runtime_active = False
        self._path_plan_request_use_all_regions = bool(rospy.get_param("~path_plan_request_use_all_regions", True))
        self._exec_goal_reach_dist = max(0.05, float(rospy.get_param("~path_goal_reach_dist", 0.12)))
        self._exec_goal_interval = max(0.1, float(rospy.get_param("~path_goal_interval", 1.0)))
        self._exec_segment_timeout = max(2.0, float(rospy.get_param("~path_segment_timeout", 10.0)))
        self._active_segment_plan_topic = rospy.get_param(
            "~active_segment_plan_topic", "/grinder/navigation/active_segment_plan"
        )
        self._active_segment_publish_hz = max(
            0.2, float(rospy.get_param("~active_segment_publish_hz", 3.0))
        )
        self._segment_switch_distance_m = max(
            0.05, float(rospy.get_param("~path_segment_switch_distance_m", 2.8))
        )
        self._segment_short_progress_ratio = min(
            0.99,
            max(0.1, float(rospy.get_param("~path_segment_short_progress_ratio", 0.85))),
        )
        self._segment_next_lane_leadin_m = max(
            0.0, float(rospy.get_param("~path_segment_next_lane_leadin_m", 0.7))
        )
        self._segment_overlap_behind_m = max(
            0.0, float(rospy.get_param("~path_segment_overlap_behind_m", 0.3))
        )
        self._active_segment_goal_refresh_s = max(
            0.0, float(rospy.get_param("~active_segment_goal_refresh_s", 2.0))
        )
        self._active_segment_progress_epsilon_m = max(
            0.001, float(rospy.get_param("~active_segment_progress_epsilon_m", 0.1))
        )
        self._exec_max_linear = max(0.05, float(rospy.get_param("~path_exec_max_linear", 0.35)))
        self._exec_max_angular = max(0.1, float(rospy.get_param("~path_exec_max_angular", 0.9)))
        self._exec_k_linear = max(0.05, float(rospy.get_param("~path_exec_k_linear", 0.8)))
        self._exec_k_angular = max(0.05, float(rospy.get_param("~path_exec_k_angular", 1.5)))
        self._exec_active = False
        self._exec_goal_index = 0
        self._exec_last_send_time = 0.0
        self._exec_last_force_send_time = 0.0
        self._exec_goal_start_time = 0.0
        self._exec_publish_start_pose_once = False
        self._exec_region_order = []
        self._exec_region_index = -1
        self._exec_region_repeat_done = {}
        self._disc_follow_path_type = bool(rospy.get_param("~disc_follow_path_type", True))
        self._disc_travel_spin_enabled = bool(
            rospy.get_param("~disc_travel_spin_enabled", False)
        )
        self._disc_travel_speed_rpm = max(
            0,
            min(32767, int(rospy.get_param("~disc_travel_speed_rpm", 300))),
        )
        self._disc_lift_supported = bool(rospy.get_param("~disc_lift_supported", True))
        self._manual_travel_disc_active = False
        self._disc_last_mode = ""  # "cover" | "transition" | ""
        self._disc_auto_cover_desired = False
        self._disc_motion_guard_enabled = bool(rospy.get_param("~disc_motion_guard_enabled", True))
        self._disc_stop_linear_threshold = max(
            0.0, float(rospy.get_param("~disc_stop_linear_threshold", 0.01))
        )
        self._disc_stop_angular_threshold = max(
            0.0, float(rospy.get_param("~disc_stop_angular_threshold", 0.01))
        )
        self._disc_resume_linear_threshold = max(
            self._disc_stop_linear_threshold,
            float(rospy.get_param("~disc_resume_linear_threshold", 0.02)),
        )
        self._disc_resume_angular_threshold = max(
            self._disc_stop_angular_threshold,
            float(rospy.get_param("~disc_resume_angular_threshold", 0.03)),
        )
        self._disc_stop_hold_sec = max(0.1, float(rospy.get_param("~disc_stop_hold_sec", 1.0)))
        self._disc_switch_min_interval = max(
            0.0, float(rospy.get_param("~disc_switch_min_interval", 2.0))
        )
        self._disc_cmd_vel_stale_sec = max(
            0.1, float(rospy.get_param("~disc_cmd_vel_stale_sec", 1.0))
        )
        self._last_cmd_vel_time = 0.0
        self._last_cmd_vel_linear = 0.0
        self._last_cmd_vel_angular = 0.0
        self._disc_stationary_since = 0.0
        self._disc_motion_guard_stopped = False
        self._disc_last_switch_time = 0.0
        self._task_stop_reason = ""
        self._last_task_result = {}
        # Task result color palette (repeat index -> color, BGR).
        self._task_result_palette_bgr = [
            (74, 201, 245),   # 1
            (90, 210, 120),   # 2
            (180, 170, 70),   # 3
            (220, 120, 70),   # 4
            (200, 90, 160),   # 5
            (160, 70, 210),   # 6
            (80, 80, 230),    # 7
        ]
        self._current_plan_scope = "single"
        self._task_bindings = {}
        self._task_execution_records = []
        self._active_task_execution_id = ""
        self._task_trajectory_lock = threading.RLock()
        self._task_trajectory_sample_interval_sec = max(
            0.1,
            float(rospy.get_param("~task_trajectory_sample_interval_sec", 1.0)),
        )
        self._task_trajectory_default_max_points = max(
            1,
            int(rospy.get_param("~task_trajectory_default_max_points", 3600)),
        )
        self._task_trajectory_max_duration_sec = max(
            self._task_trajectory_sample_interval_sec,
            float(rospy.get_param("~task_trajectory_max_duration_sec", 14400.0)),
        )
        self._task_trajectory_max_samples = max(
            1,
            int(
                math.ceil(
                    self._task_trajectory_max_duration_sec
                    / self._task_trajectory_sample_interval_sec
                )
            ),
        )
        self._task_trajectory_max_points = max(
            self._task_trajectory_default_max_points,
            min(
                self._task_trajectory_max_samples,
                int(rospy.get_param("~task_trajectory_max_points", 14400)),
            ),
        )
        self._task_trajectory_next_sample_monotonic = 0.0
        self._task_obstacle_regions = {}
        self._task_obstacle_regions_lock = threading.RLock()
        self._map_registry = {}
        self._max_chassis_run_speed = max(
            0.01,
            float(rospy.get_param("~max_chassis_run_speed", 0.2)),
        )
        default_run_speed = float(rospy.get_param("~default_chassis_run_speed", 0.2))
        self._navigation_speed_reconfigure_namespace = str(
            rospy.get_param(
                "~navigation_speed_reconfigure_namespace",
                "/move_base/TebLocalPlannerROS",
            )
        ).strip() or "/move_base/TebLocalPlannerROS"

        self._navigation_speed_reconfigure_parameter = str(
            rospy.get_param(
                "~navigation_speed_reconfigure_parameter",
                "max_vel_x",
            )
        ).strip() or "max_vel_x"
        
        self._navigation_speed_reconfigure_timeout = max(
            0.1,
            float(rospy.get_param("~navigation_speed_reconfigure_timeout", 1.0)),
        )
        self._navigation_speed_client = None
        self._navigation_speed_lock = threading.Lock()
        self._chassis_settings = {
            "work_mode": 1,  # 1:auto, 2:manual
            "disc_speed_rpm": 1200,
            "run_speed": max(0.0, min(self._max_chassis_run_speed, default_run_speed)),
            "max_turn_speed_ratio": 1.0,
        }
        self._live_map_id = str(rospy.get_param("~live_map_id", DEFAULT_LIVE_MAP_ID)).strip() or DEFAULT_LIVE_MAP_ID
        self._active_map_id = self._live_map_id
        self._last_seen_map_id = ""
        runtime_base_param = rospy.get_param("~runtime_base_dir", "").strip()
        self._runtime_base_dir = (
            _resolve_runtime_path(runtime_base_param, os.getcwd())
            if runtime_base_param
            else _detect_runtime_base_dir()
        )
        self._preview_max_edge_cap = max(64, int(rospy.get_param("~preview_max_edge_cap", PREVIEW_MAX_EDGE_CAP)))
        self._robot_config_yaml_path = _resolve_runtime_path(
            rospy.get_param("~robot_config_yaml_path", "catkin_ws/src/grinder_scheduler/config/scheduler.yaml"),
            self._runtime_base_dir,
        )
        self._initial_map_preview_enabled = bool(rospy.get_param("~initial_map_preview_enabled", False))
        self._initial_map_preview_saved = False
        self._initial_map_preview_dir = _resolve_runtime_path(
            rospy.get_param("~initial_map_preview_dir", "temp"),
            self._runtime_base_dir,
        )
        self._initial_map_preview_max_edge = self._sanitize_preview_edge(
            rospy.get_param("~initial_map_preview_max_edge", PREVIEW_MAX_EDGE_CAP),
            cap_edge=self._preview_max_edge_cap,
        )
        self._initial_map_preview_format = rospy.get_param("~initial_map_preview_format", "jpg")
        self._planned_path_preview_dir = _resolve_runtime_path(
            rospy.get_param("~planned_path_preview_dir", "temp"),
            self._runtime_base_dir,
        )
        self._planned_path_debug_dir = _resolve_runtime_path(
            rospy.get_param("~planned_path_debug_dir", "temp/path_debug"),
            self._runtime_base_dir,
        )
        self._planned_path_debug_enabled = bool(rospy.get_param("~planned_path_debug_enabled", False))
        self._planned_path_preview_max_edge = self._sanitize_preview_edge(
            rospy.get_param("~planned_path_preview_max_edge", PREVIEW_MAX_EDGE_CAP),
            cap_edge=self._preview_max_edge_cap,
        )
        self._planned_path_preview_format = rospy.get_param("~planned_path_preview_format", "jpg")
        self._planned_path_preview_save_on_plan = bool(
            rospy.get_param("~planned_path_preview_save_on_plan", False)
        )
        self._planned_path_preview_save_response_file = bool(
            rospy.get_param("~planned_path_preview_save_response_file", False)
        )
        self._preview_rga_enabled = bool(rospy.get_param("~preview_rga_enabled", False))
        self._preview_rga_backend = str(rospy.get_param("~preview_rga_backend", "auto") or "auto").strip().lower()
        if self._preview_rga_backend not in ("auto", "librga"):
            rospy.logwarn(
                "Invalid preview_rga_backend=%s, fallback to auto",
                self._preview_rga_backend,
            )
            self._preview_rga_backend = "auto"
        self._preview_rga_available = self._detect_preview_rga_available()
        if self._preview_rga_enabled:
            if self._preview_rga_available:
                rospy.logwarn(
                    "preview_rga_enabled=true and /dev/rga exists, but Python RGA render path is not wired yet; fallback to OpenCV."
                )
            else:
                rospy.logwarn(
                    "preview_rga_enabled=true but RGA device/backend is unavailable; fallback to OpenCV."
                )
        self._planned_path_preview_include_overlay = bool(rospy.get_param("~planned_path_preview_include_overlay", True))
        # Path preview annotation policy:
        # disable direction arrows / target index overlays to keep the preview clean.
        self._planned_path_preview_show_direction = False
        self._planned_path_preview_arrow_step = max(
            1, int(rospy.get_param("~planned_path_preview_arrow_step", 10))
        )
        self._planned_path_preview_arrow_len_px = max(
            4, int(rospy.get_param("~planned_path_preview_arrow_len_px", 12))
        )
        self._preview_snapshot_cache_key = None
        self._preview_snapshot_cache = None
        self._path_preview_payload_cache_key = None
        self._path_preview_payload_cache = None
        self._path_plan_request_cache_key = None
        self._path_plan_request_cache_path_version = 0
        self._path_preview_crop_cache_key = None
        self._path_preview_crop_cache_bbox = None
        self._path_preview_overlay_base_cache_key = None
        self._path_preview_overlay_base_cache = None
        self._task_config_auto_plan_enabled = bool(rospy.get_param("~task_config_auto_plan_enabled", False))
        requested_use_all = bool(rospy.get_param("~plan_use_all_work_regions", False))
        # Single-region planning policy:
        # even if param is set, enforce one work region per planning request.
        self._plan_use_all_work_regions = False
        if requested_use_all:
            rospy.logwarn("plan_use_all_work_regions is ignored: single-region planning mode is enforced.")
        self._reload_navigation_map_on_plan = bool(rospy.get_param("~reload_navigation_map_on_plan", True))
        self._persist_state_enabled = bool(rospy.get_param("~persist_state_enabled", True))
        # Runtime-task persistence switch:
        # False => task is realtime-only and must be re-sent by APP every launch.
        self._persist_runtime_task_state = bool(rospy.get_param("~persist_runtime_task_state", False))
        self._persist_state_dir = _resolve_runtime_path(
            rospy.get_param("~persist_state_dir", "temp/grinder_scheduler_state"),
            self._runtime_base_dir,
        )
        self._save_state_on_plan_success = bool(rospy.get_param("~save_state_on_plan_success", False))
        self._map_registry_state_file = os.path.join(self._persist_state_dir, "map_registry.json")
        self._task_registry_state_file = os.path.join(self._persist_state_dir, "task_registry.json")
        self._task_obstacle_regions_state_file = os.path.join(
            self._persist_state_dir,
            "task_obstacle_regions.json",
        )
        self._live_preview_enabled = bool(rospy.get_param("~live_preview_enabled", True))
        self._live_preview_hz = max(0.1, float(rospy.get_param("~live_preview_hz", 1.0)))
        self._live_preview_max_edge = self._sanitize_preview_edge(
            rospy.get_param("~live_preview_max_edge", PREVIEW_MAX_EDGE_CAP),
            cap_edge=self._preview_max_edge_cap,
        )
        self._live_preview_format = rospy.get_param("~live_preview_format", "jpg")
        self._live_preview_file = _resolve_runtime_path(
            rospy.get_param("~live_preview_file", "temp/aurora_map_preview_latest.jpg"),
            self._runtime_base_dir,
        )
        self._live_preview_next_time = 0.0
        self._live_map_enabled = bool(rospy.get_param("~live_map_enabled", True))
        self._live_map_hz = max(0.1, float(rospy.get_param("~live_map_hz", 1.0)))
        self._live_map_dir = _resolve_runtime_path(
            rospy.get_param("~live_map_dir", "temp/live_map"),
            self._runtime_base_dir,
        )
        self._live_map_yaml_name = rospy.get_param("~live_map_yaml_name", "map.yaml")
        self._live_map_image_name = rospy.get_param("~live_map_image_name", "map1.pgm")
        self._live_map_next_time = 0.0
        self._live_map_crop_to_free_space = bool(rospy.get_param("~live_map_crop_to_free_space", True))
        self._live_map_crop_margin_m = max(0.0, float(rospy.get_param("~live_map_crop_margin_m", 0.5)))
        self._live_map_cache_clear_mode = str(
            rospy.get_param("~live_map_cache_clear_mode", "all")
        ).strip().lower()
        if self._live_map_cache_clear_mode not in ("all", "memory_only"):
            self._live_map_cache_clear_mode = "all"
        self._live_map_align_to_initial_yaw = bool(
            rospy.get_param("~live_map_align_to_initial_yaw", True)
        )
        self._live_map_source_frame = str(rospy.get_param("~live_map_source_frame", "map")).strip() or "map"
        self._live_map_aligned_frame = str(rospy.get_param("~live_map_aligned_frame", "map_aligned")).strip() or "map_aligned"
        self._sl_link_input_points_frame = str(
            rospy.get_param("~sl_link_input_points_frame", self._live_map_source_frame)
        ).strip() or self._live_map_source_frame
        if self._sl_link_input_points_frame not in (self._live_map_source_frame, self._live_map_aligned_frame):
            rospy.logwarn(
                "Invalid sl_link_input_points_frame=%s, fallback to %s",
                self._sl_link_input_points_frame,
                self._live_map_source_frame,
            )
            self._sl_link_input_points_frame = self._live_map_source_frame
        self._live_map_align_yaw_timeout = max(
            0.01, float(rospy.get_param("~live_map_align_yaw_timeout", 0.05))
        )
        self._live_map_last_rotated_version = None
        self._live_map_last_rotated_yaw = None
        self._live_map_last_rotated_grid = None
        self._live_map_last_rotated_origin = None
        self._initial_pose_alignment_yaw = None
        self._live_map_app_rotation_deg = None
        self._live_map_rotation_alignment_delta_deg = None
        self._tf_buffer = None
        self._tf_listener = None
        if self._live_map_align_to_initial_yaw and tf2_ros is not None:
            try:
                self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
                self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
            except Exception as exc:
                rospy.logwarn("Failed to init tf2 listener for live map alignment: %s", exc)
                self._tf_buffer = None
                self._tf_listener = None
        self._saved_map_thumb_max_edge = max(64, int(rospy.get_param("~saved_map_thumb_max_edge", 192)))
        self._saved_map_thumb_jpeg_quality = max(30, min(95, int(rospy.get_param("~saved_map_thumb_jpeg_quality", 70))))
        self._map_catalog_max_items = max(1, int(rospy.get_param("~map_catalog_max_items", 200)))
        self._map_catalog_include_thumbnails = bool(rospy.get_param("~map_catalog_include_thumbnails", True))
        self._map_catalog_max_thumbnail_b64_total = max(
            0, int(rospy.get_param("~map_catalog_max_thumbnail_b64_total", 512000))
        )
        self._last_manual_enable_try = 0.0
        self._manual_wheel_radius_m = max(
            1e-6,
            float(rospy.get_param("/chassis_driver/cmd_vel_wheel_radius_m", 0.1)),
        )
        self._manual_gear_ratio = max(
            1e-6,
            float(rospy.get_param("/chassis_driver/cmd_vel_gear_ratio", 60.0)),
        )
        self._manual_speed_scale = max(
            1e-6,
            float(rospy.get_param("/chassis_driver/cmd_vel_scale", 1.0)),
        )
        self._manual_max_abs_wheel_rpm = max(
            1,
            int(abs(float(rospy.get_param("/chassis_driver/cmd_vel_max_abs_wheel_rpm", 1500.0)))),
        )
        self._manual_reverse_guard_sec = max(0.0, float(rospy.get_param("~manual_reverse_guard_sec", 0.25)))
        self._manual_reverse_until = 0.0
        self._manual_last_left_cmd = 0
        self._manual_last_right_cmd = 0
        self._manual_drive_lock = threading.Lock()
        self._manual_command_timeout_sec = max(
            0.1,
            float(rospy.get_param("~manual_command_timeout_sec", 0.5)),
        )
        self._manual_last_command_monotonic = 0.0
        self._manual_command_active = False
        self._map_request_encoding = str(rospy.get_param("~map_request_encoding", "png")).strip().lower()
        self._nav_map_yaml_path = _resolve_runtime_path(
            rospy.get_param(
                "~navigation_map_yaml_path",
                "catkin_ws/src/2-dnavigation-package/2dnavigation/teb_local_planner_tutorials/maps/map.yaml",
            ),
            self._runtime_base_dir,
        )
        self._stcm_local_dir = _resolve_runtime_path(
            rospy.get_param("~stcm_local_dir", "maps/raw"),
            self._runtime_base_dir,
        )
        self._sync_get_stcm_service = rospy.get_param(
            "~sync_get_stcm_service", "/slamware_ros_sdk_server_node/sync_get_stcm"
        )
        self._sync_set_stcm_service = rospy.get_param(
            "~sync_set_stcm_service", "/slamware_ros_sdk_server_node/sync_set_stcm"
        )
        self._radar_relocalization_service = rospy.get_param(
            "~radar_relocalization_service",
            "/slamware_ros_sdk_server_node/relocalization",
        )
        self._change_map_service = rospy.get_param("~change_map_service", "/change_map")
        self._set_map_update_topic = rospy.get_param(
            "~set_map_update_topic", "/slamware_ros_sdk_server_node/set_map_update"
        )
        self._set_map_localization_topic = rospy.get_param(
            "~set_map_localization_topic", "/slamware_ros_sdk_server_node/set_map_localization"
        )
        self._clear_map_topic = rospy.get_param("~clear_map_topic", "/slamware_ros_sdk_server_node/clear_map")
        self._sync_map_topic = rospy.get_param("~sync_map_topic", "/slamware_ros_sdk_server_node/sync_map")
        self._radar_system_status_topic = rospy.get_param(
            "~radar_system_status_topic",
            "/slamware_ros_sdk_server_node/system_status",
        )
        self._radar_relocalization_status_topic = rospy.get_param(
            "~radar_relocalization_status_topic",
            "/slamware_ros_sdk_server_node/relocalization_status",
        )
        self._radar_system_status_lock = threading.Lock()
        self._radar_system_status_available = False
        self._radar_system_status = ""
        self._radar_system_status_timestamp_ns = 0
        self._radar_relocalization_status_lock = threading.Lock()
        self._radar_relocalization_raw_available = False
        self._radar_relocalization_raw_status = ""
        self._radar_relocalization_raw_timestamp_ns = 0
        self._radar_relocalization_aggregate_status = "idle"
        self._radar_relocalization_aggregate_timestamp_ns = 0
        self._radar_mapping_mode_default_on_startup = bool(
            rospy.get_param("~radar_mapping_mode_default_on_startup", True)
        )
        self._radar_mapping_sync_period_sec = max(
            0.5,
            float(rospy.get_param("~radar_mapping_sync_period_sec", 3.0)),
        )
        self._radar_mapping_sync_burst_count = max(
            0,
            int(rospy.get_param("~radar_mapping_sync_burst_count", 3)),
        )
        self._radar_mapping_mode_active = False
        self._radar_mapping_sync_remaining = 0
        self._radar_clear_before_import_delay_sec = max(
            0.0,
            float(rospy.get_param("~radar_clear_before_import_delay_sec", 0.2)),
        )
        self._radar_relocalization_after_import_delay_sec = max(
            0.0,
            float(rospy.get_param("~radar_relocalization_after_import_delay_sec", 1.0)),
        )
        self._radar_relocalization_service_wait_sec = max(
            0.1,
            float(rospy.get_param("~radar_relocalization_service_wait_sec", 3.0)),
        )
        self._sync_get_proxy = None
        self._sync_set_proxy = None
        self._radar_relocalization_proxy = None
        self._change_map_proxy = None

        self.map_service = MapService()
        self._draw_region_id_on_preview = bool(rospy.get_param("~draw_region_id_on_preview", True))
        self._draw_region_label_on_preview = bool(rospy.get_param("~draw_region_label_on_preview", True))
        self.map_service.set_draw_region_id_on_preview(self._draw_region_id_on_preview)
        self.map_service.set_draw_region_label_on_preview(self._draw_region_label_on_preview)
        self.map_service.configure_preview_rga(
            self._preview_rga_enabled,
            self._preview_rga_backend,
            self._preview_rga_available,
        )
        self._load_local_state()
        self.aurora_bridge = AuroraBridge(
            map_topic=rospy.get_param("~map_topic", "/slamware_ros_sdk_server_node/map"),
            odom_topic=rospy.get_param("~odom_topic", "/slamware_ros_sdk_server_node/odom"),
            left_image_topic=rospy.get_param("~left_image_topic", "/slamware_ros_sdk_server_node/left_image_raw"),
            right_image_topic=rospy.get_param("~right_image_topic", "/slamware_ros_sdk_server_node/right_image_raw"),
            first_frame_save_dir=_resolve_runtime_path(
                rospy.get_param("~first_frame_save_dir", "temp"),
                self._runtime_base_dir,
            ),
            save_first_frame_on_startup=bool(rospy.get_param("~save_first_frame_on_startup", False)),
            use_depth_colorized_image=rospy.get_param("~use_depth_colorized_image", True),
            depth_image_colorized_topic=rospy.get_param("~depth_image_colorized_topic", "/slamware_ros_sdk_server_node/depth_image_colorized"),
            localization_quality_position_reference_m=rospy.get_param(
                "~localization_quality_position_reference_m", 0.1
            ),
            localization_quality_heading_reference_deg=rospy.get_param(
                "~localization_quality_heading_reference_deg", 10.0
            ),
            localization_quality_unavailable_variance=rospy.get_param(
                "~localization_quality_unavailable_variance", 1.0e6
            ),
        )
        self.planner = PlannerAdapter(
            _resolve_runtime_path(
                rospy.get_param("~planner_script_path", "third_party/path_planner/mst27/mst27.py"),
                self._runtime_base_dir,
            )
        )
        stream_name = str(rospy.get_param("~stream_name", "auto") or "").strip()
        if not stream_name or stream_name.lower() == "auto":
            stream_name = str(rospy.get_param("~mqtt_dev_code", "grinder_main") or "grinder_main").strip()
        stream_push_url = str(rospy.get_param("~stream_url", "") or "").replace("{stream}", stream_name)
        stream_play_url = str(rospy.get_param("~stream_play_url", "") or "").replace("{stream}", stream_name)
        self.media_streamer = FFmpegMediaStreamer(
            stream_url=stream_push_url,
            play_url=stream_play_url,
            fps=rospy.get_param("~stream_fps", 10),
            width=rospy.get_param("~stream_width", 640),
            bitrate_kbps=rospy.get_param("~stream_bitrate_kbps", 800),
            keyframe_interval=rospy.get_param("~stream_keyframe_interval", 20),
            enabled=rospy.get_param("~stream_enabled", True),
        )
        self.local_stream_server = LocalRtspStreamServer(
            host=rospy.get_param("~local_rtsp_host", "0.0.0.0"),
            public_host=rospy.get_param("~local_rtsp_public_host", "auto"),
            port=rospy.get_param("~local_rtsp_port", 8554),
            fps=rospy.get_param("~local_rtsp_fps", 12),
            width=rospy.get_param("~local_rtsp_width", 960),
            enabled=rospy.get_param("~local_rtsp_enabled", True),
            start_server=rospy.get_param("~local_rtsp_start_server", True),
            preferred_encoder=str(rospy.get_param("~local_rtsp_encoder", "auto")).strip().lower(),
            bitrate_kbps=rospy.get_param("~local_rtsp_bitrate_kbps", 800),
            keyframe_interval=rospy.get_param("~local_rtsp_keyframe_interval", 15),
            mediamtx_path=_resolve_runtime_path(
                rospy.get_param("~local_rtsp_mediamtx_path", "tools/mediamtx/mediamtx"),
                self._runtime_base_dir,
            ),
            log_dir=_resolve_runtime_path(
                rospy.get_param("~local_rtsp_log_dir", "temp"),
                self._runtime_base_dir,
            ),
        )
        self._local_rtsp_max_frame_age = max(0.1, float(rospy.get_param("~local_rtsp_max_frame_age", 0.4)))
        self.local_stream_server.start()

        self._global_plan_topic = rospy.get_param("~global_plan_topic", "/grinder/GlobalPlanner/plan")
        self.global_plan_pub = rospy.Publisher(self._global_plan_topic, Path, queue_size=1, latch=True)
        self.active_segment_plan_pub = rospy.Publisher(
            self._active_segment_plan_topic, Path, queue_size=1, latch=True
        )
        self.goal_pub = rospy.Publisher(self._exec_goal_topic, PoseStamped, queue_size=10)
        self.task_enable_pub = rospy.Publisher(self._task_enable_topic, Bool, queue_size=10, latch=True)
        self.status_pub = rospy.Publisher("/scheduler/status", SchedulerStatus, queue_size=10)
        self.preview_meta_pub = rospy.Publisher("/scheduler/map_preview_metadata", MapPreviewMetadata, queue_size=10, latch=True)
        self.diagnostics_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=10)
        self._set_map_update_pub = None
        self._set_map_localization_pub = None
        self._clear_map_pub = None
        self._sync_map_pub = None
        if SetMapUpdateRequest is not None:
            self._set_map_update_pub = rospy.Publisher(
                self._set_map_update_topic, SetMapUpdateRequest, queue_size=2
            )
        if SetMapLocalizationRequest is not None:
            self._set_map_localization_pub = rospy.Publisher(
                self._set_map_localization_topic, SetMapLocalizationRequest, queue_size=2
            )
        if ClearMapRequest is not None:
            self._clear_map_pub = rospy.Publisher(self._clear_map_topic, ClearMapRequest, queue_size=2)
        if SyncMapRequest is not None:
            self._sync_map_pub = rospy.Publisher(self._sync_map_topic, SyncMapRequest, queue_size=2)

        self.wheel_cmd_pub = rospy.Publisher("/chassis/wheel_speed_cmd", WheelSpeedCommand, queue_size=10)
        self.disc_speed_pub = rospy.Publisher("/chassis/disc_speed_cmd", Int16, queue_size=10)
        self.disc_enable_pub = rospy.Publisher("/chassis/disc_enable_cmd", Bool, queue_size=10)
        self.work_mode_pub = rospy.Publisher("/chassis/work_mode_cmd", UInt16, queue_size=10)
        self.disc_lift_pub = rospy.Publisher("/chassis/disc_lift_cmd", UInt16, queue_size=10)
        self.light_pub = rospy.Publisher("/chassis/light_cmd", Bool, queue_size=10)
        self.chassis_emergency_stop_pub = rospy.Publisher("/chassis/emergency_stop", Bool, queue_size=1)

        rospy.Subscriber("/chassis/status", ChassisStatus, self._chassis_status_callback, queue_size=10)
        rospy.Subscriber("/chassis/wheel_speed_state", WheelSpeedState, self._wheel_speed_state_callback, queue_size=10)
        rospy.Subscriber(
            self._wheel_odom_topic,
            Odometry,
            self._wheel_odom_callback,
            queue_size=10,
            tcp_nodelay=True,
        )
        rospy.Subscriber("/cmd_vel", Twist, self._cmd_vel_callback, queue_size=10)
        rospy.Subscriber(
            self._collision_imminent_topic,
            Bool,
            self._collision_imminent_callback,
            queue_size=10,
        )
        if RadarSystemStatus is not None:
            rospy.Subscriber(
                self._radar_system_status_topic,
                RadarSystemStatus,
                self._radar_system_status_callback,
                queue_size=20,
            )
        else:
            rospy.logwarn("slamware_ros_sdk/SystemStatus is unavailable; radar status query will report unavailable")
        if RadarRelocalizationStatus is not None:
            rospy.Subscriber(
                self._radar_relocalization_status_topic,
                RadarRelocalizationStatus,
                self._radar_relocalization_status_callback,
                queue_size=20,
            )
        else:
            rospy.logwarn(
                "slamware_ros_sdk/RelocalizationStatus is unavailable; "
                "radar relocalization raw status will be unavailable"
            )
        rospy.Service("~plan_now", Trigger, self._handle_plan_now)

        self.enable_service = rospy.ServiceProxy("/chassis/enable", EnableChassis)

        self.sl_link_server = SlLinkAServer(
            sdk_dir=_resolve_runtime_path(
                rospy.get_param("~sl_linka_python_sdk_dir", "third_party/sl_linka/sdk/python"),
                self._runtime_base_dir,
            ),
            host=rospy.get_param("~sl_linka_host", "0.0.0.0"),
            port=rospy.get_param("~sl_linka_port", 8002),
            callback_handler=self,
        )
        self.sl_link_server.start()

        platform_password = str(rospy.get_param("~platform_password", "") or "").strip()
        if not platform_password:
            platform_password = os.environ.get("GRINDER_PLATFORM_PASSWORD", "").strip()
        self.platform_file_sync = PlatformFileSync(
            platform_base_url=rospy.get_param("~platform_base_url", "http://14.18.103.194:19001"),
            username=rospy.get_param("~platform_username", "WangYaLing"),
            password=platform_password,
            configured_project_id=rospy.get_param("~platform_project_id", ""),
            file_base_url=rospy.get_param("~file_server_base_url", "http://14.18.103.194:19001"),
            remote_root_name=rospy.get_param("~file_remote_root_name", "GrinderProject"),
            enabled=rospy.get_param("~file_upload_on_map_save", True),
            timeout_sec=rospy.get_param("~platform_http_timeout_sec", 30.0),
        )
        self.platform_file_sync.start()

        mqtt_username = str(rospy.get_param("~mqtt_username", "") or "").strip()
        mqtt_password = str(rospy.get_param("~mqtt_password", "") or "").strip()
        if not mqtt_username:
            mqtt_username = os.environ.get("MQTT_USERNAME", os.environ.get("MQTT_ACCESS_KEY", "")).strip()
        if not mqtt_password:
            mqtt_password = os.environ.get("MQTT_PASSWORD", os.environ.get("MQTT_ACCESS_SECRET", "")).strip()
        self.mqtt_reporter = MqttDeviceReporter(
            enabled=rospy.get_param("~mqtt_enabled", True),
            broker_host=rospy.get_param("~mqtt_broker_host", "14.18.91.10"),
            broker_port=rospy.get_param("~mqtt_broker_port", 1883),
            dev_code=rospy.get_param("~mqtt_dev_code", ""),
            username=mqtt_username,
            password=mqtt_password,
            status_provider=self._mqtt_status_snapshot,
            keepalive_sec=rospy.get_param("~mqtt_keepalive_sec", 60),
            qos=rospy.get_param("~mqtt_qos", 1),
            status_period_sec=rospy.get_param("~mqtt_status_period_sec", 3.0),
            network_latency_enabled=rospy.get_param("~mqtt_network_latency_enabled", True),
            network_latency_period_sec=rospy.get_param("~mqtt_network_latency_period_sec", 5.0),
            network_latency_timeout_ms=rospy.get_param("~mqtt_network_latency_timeout_ms", 1000),
        )
        self.mqtt_reporter.start()

        tick_hz = max(1.0, float(rospy.get_param("~scheduler_tick_hz", 5.0)))
        stream_push_hz = max(5.0, float(rospy.get_param("~stream_push_hz", 20.0)))
        self.timer = rospy.Timer(rospy.Duration(1.0 / tick_hz), self._tick)
        self.stream_timer = rospy.Timer(rospy.Duration(1.0 / stream_push_hz), self._stream_tick)
        self.active_segment_timer = rospy.Timer(
            rospy.Duration(1.0 / self._active_segment_publish_hz),
            self._publish_active_segment_tick,
        )
        self.manual_drive_watchdog_timer = rospy.Timer(
            rospy.Duration(0.05),
            self._manual_drive_watchdog_tick,
        )
        self._radar_mapping_sync_timer = rospy.Timer(
            rospy.Duration(self._radar_mapping_sync_period_sec),
            self._radar_mapping_sync_tick,
        )
        self._radar_mapping_startup_timer = None
        if self._radar_mapping_mode_default_on_startup:
            self._radar_mapping_startup_timer = rospy.Timer(
                rospy.Duration(self._radar_mapping_sync_period_sec),
                self._radar_mapping_startup_tick,
            )
        # Default: do not allow chassis to consume /cmd_vel until task starts.
        self.task_enable_pub.publish(Bool(data=False))

    def _set_cmd_vel_forward_runtime_active(self, enabled, publish_zero=False, reason=""):
        # Compatibility wrapper: runtime task control now uses /chassis/task_enable.
        enabled = bool(enabled)
        changed = (self._task_enable_runtime_active != enabled)
        if changed:
            self._task_enable_runtime_active = enabled
            self.task_enable_pub.publish(Bool(data=enabled))
            rospy.loginfo(
                "task_enable runtime %s%s",
                "enabled" if enabled else "disabled",
                (" reason={}".format(reason) if reason else ""),
            )

    def _stream_tick(self, _event):
        left, right, left_stamp, right_stamp = self.aurora_bridge.get_latest_frames()
        now = rospy.Time.now()
        max_age = rospy.Duration.from_sec(self._local_rtsp_max_frame_age)
        if left is not None and left_stamp and left_stamp != rospy.Time(0):
            if now > left_stamp and (now - left_stamp) > max_age:
                rospy.logwarn_throttle(
                    2.0,
                    "Drop stale left frame for RTSP: age=%.3fs (max=%.3fs)",
                    (now - left_stamp).to_sec(),
                    self._local_rtsp_max_frame_age,
                )
                left = None
        if right is not None and right_stamp and right_stamp != rospy.Time(0):
            if now > right_stamp and (now - right_stamp) > max_age:
                rospy.logwarn_throttle(
                    2.0,
                    "Drop stale right frame for RTSP: age=%.3fs (max=%.3fs)",
                    (now - right_stamp).to_sec(),
                    self._local_rtsp_max_frame_age,
                )
                right = None
        if left is not None:
            self.local_stream_server.push_frame("left", left)
            # SRS uses the same left camera source as TileCarrier.
            self.media_streamer.push_frames(left, None)
        if right is not None:
            self.local_stream_server.push_frame("right", right)

    def _publish_path_to_navigation(self, publish_goal=False, reason=""):
        try:
            if self.current_path is None or self.current_path.nav_path is None or not self.current_path.nav_path.poses:
                return
            msg = self._build_navigation_path_for_move_base()
            self.global_plan_pub.publish(msg)
            if publish_goal:
                self._publish_path_endpoint_goal(reason=reason or "sync_with_global_plan")
            rospy.loginfo_throttle(2.0, "Published planned path to navigation: points=%d", len(msg.poses))
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Failed to publish path to navigation: %s", exc)

    def _publish_active_segment_tick(self, _event):
        if self.state != SchedulerState.RUNNING or not self._exec_active:
            return
        self._publish_active_segment_plan(reason="timer")

    def _radar_mapping_startup_tick(self, _event):
        if self._radar_mapping_mode_active:
            if self._radar_mapping_startup_timer is not None:
                self._radar_mapping_startup_timer.shutdown()
                self._radar_mapping_startup_timer = None
            return
        try:
            conn, kind_value = self._switch_to_mapping_mode(set_live_map_active=True)
            rospy.loginfo(
                "Radar default startup mapping mode entered: subscribers=%d map_kind=%d",
                conn,
                kind_value,
            )
            if self._radar_mapping_startup_timer is not None:
                self._radar_mapping_startup_timer.shutdown()
                self._radar_mapping_startup_timer = None
        except Exception as exc:
            rospy.logwarn_throttle(
                10.0,
                "Radar default startup mapping mode pending: %s",
                exc,
            )

    def _radar_mapping_sync_tick(self, _event):
        if not self._radar_mapping_mode_active:
            return
        if self._radar_mapping_sync_remaining <= 0:
            return
        try:
            if self._send_radar_map_sync():
                self._radar_mapping_sync_remaining -= 1
                rospy.loginfo(
                    "Radar mapping sync burst progress: remaining=%d",
                    self._radar_mapping_sync_remaining,
                )
        except Exception as exc:
            rospy.logwarn_throttle(10.0, "Radar mapping sync failed: %s", exc)

    def _reset_active_segment_state(self):
        self._path_arc_lengths = []
        self._active_segments = []
        self._active_segment_index = 0
        self._active_segment_start_s = 0.0
        self._active_segment_last_progress_s = 0.0
        self._active_segment_stall_progress_s = 0.0
        self._active_segment_last_progress_update_time = time.time()
        self._active_segment_last_switch_time = 0.0
        self._active_segment_goal_sent_index = -1

    def _path_cumulative_lengths(self, points):
        lengths = []
        total = 0.0
        prev = None
        for point in list(points or []):
            x = float(point.get("x", 0.0))
            y = float(point.get("y", 0.0))
            if prev is not None:
                total += math.hypot(x - prev[0], y - prev[1])
            lengths.append(float(total))
            prev = (x, y)
        return lengths

    def _ensure_path_arc_lengths(self):
        if self.current_path is None or not self.current_path.points:
            self._path_arc_lengths = []
            return []
        if len(self._path_arc_lengths) != len(self.current_path.points):
            self._path_arc_lengths = self._path_cumulative_lengths(self.current_path.points)
        return self._path_arc_lengths

    def _index_at_arc_length(self, target_s):
        path_s = self._ensure_path_arc_lengths()
        if not path_s:
            return 0
        target_s = float(target_s)
        return max(0, min(len(path_s) - 1, int(bisect.bisect_left(path_s, target_s))))

    def _build_path_direction_runs(self):
        runs = []
        if self.current_path is None or not self.current_path.points:
            return runs
        points = self.current_path.points
        path_s = self._ensure_path_arc_lengths()
        n = len(points)
        cursor = 0
        cached_xy = [(float(p.get("x", 0.0)), float(p.get("y", 0.0))) for p in points]
        while cursor < n:
            end_idx = self._resolve_straight_segment_end_index(cursor)
            end_idx = max(cursor, min(n - 1, int(end_idx)))
            runs.append(
                {
                    "start": int(cursor),
                    "end": int(end_idx),
                    "goal": int(end_idx),
                    "path_type": str(points[end_idx].get("path_type", "") or ""),
                    "length": self._segment_length(cached_xy, cursor, end_idx),
                    "yaw": self._segment_yaw(cached_xy, cursor, end_idx),
                    "start_s": float(path_s[cursor]) if path_s else 0.0,
                    "end_s": float(path_s[end_idx]) if path_s else 0.0,
                }
            )
            if end_idx >= n - 1:
                break
            cursor = end_idx + 1
        return runs

    def _rebuild_active_segments(self):
        self._active_segments = []
        self._active_segment_index = 0
        self._active_segment_start_s = 0.0
        self._active_segment_last_progress_s = 0.0
        self._active_segment_stall_progress_s = 0.0
        self._active_segment_last_progress_update_time = time.time()
        self._active_segment_last_switch_time = 0.0
        self._active_segment_goal_sent_index = -1
        if self.current_path is None or not self.current_path.points:
            self._path_arc_lengths = []
            return
        points = self.current_path.points
        path_s = self._path_cumulative_lengths(points)
        self._path_arc_lengths = path_s
        if len(points) == 1:
            self._active_segments = [
                {
                    "run_pos": 0,
                    "start_index": 0,
                    "end_index": 0,
                    "start_s": 0.0,
                    "end_s": 0.0,
                    "length": 0.0,
                }
            ]
            return

        runs = self._build_path_direction_runs()
        if not runs:
            self._active_segments = [
                {
                    "run_pos": 0,
                    "start_index": 0,
                    "end_index": len(points) - 1,
                    "start_s": 0.0,
                    "end_s": float(path_s[-1]),
                    "length": float(path_s[-1]),
                }
            ]
            return

        turn_positions = set()
        for pos in range(len(runs)):
            try:
                if self._should_insert_corner_mid(runs, pos):
                    turn_positions.add(pos)
            except Exception:
                pass

        for pos, run in enumerate(runs):
            if pos in turn_positions:
                continue
            start_s = float(run.get("start_s", 0.0))
            end_s = float(run.get("end_s", start_s))
            end_index = int(run.get("end", 0))
            if pos + 1 < len(runs):
                next_run = runs[pos + 1]
                if pos + 1 in turn_positions:
                    end_s = float(next_run.get("end_s", end_s))
                    end_index = int(next_run.get("end", end_index))
                    if pos + 2 < len(runs):
                        lead_run = runs[pos + 2]
                        lead_start_s = float(lead_run.get("start_s", end_s))
                        lead_end_s = float(lead_run.get("end_s", lead_start_s))
                        end_s = min(lead_end_s, lead_start_s + float(self._segment_next_lane_leadin_m))
                        end_index = self._index_at_arc_length(end_s)
                else:
                    lead_start_s = float(next_run.get("start_s", end_s))
                    lead_end_s = float(next_run.get("end_s", lead_start_s))
                    if lead_end_s > lead_start_s:
                        end_s = min(lead_end_s, lead_start_s + float(self._segment_next_lane_leadin_m))
                        end_index = self._index_at_arc_length(end_s)
            if end_s <= start_s + 1e-6 and int(run.get("end", 0)) < len(points) - 1:
                continue
            self._active_segments.append(
                {
                    "run_pos": int(pos),
                    "start_index": int(run.get("start", 0)),
                    "end_index": int(end_index),
                    "start_s": float(start_s),
                    "end_s": float(end_s),
                    "length": max(0.0, float(end_s) - float(start_s)),
                }
            )

        if not self._active_segments:
            self._active_segments = [
                {
                    "run_pos": 0,
                    "start_index": 0,
                    "end_index": len(points) - 1,
                    "start_s": 0.0,
                    "end_s": float(path_s[-1]),
                    "length": float(path_s[-1]),
                }
            ]
        rospy.loginfo("Active path segments rebuilt: count=%d", len(self._active_segments))

    def _interpolate_path_point_at_s(self, target_s):
        points = list(self.current_path.points if self.current_path is not None else [])
        path_s = self._ensure_path_arc_lengths()
        if not points:
            return {}
        if len(points) == 1 or not path_s:
            return dict(points[0])
        target_s = max(0.0, min(float(target_s), float(path_s[-1])))
        if target_s <= float(path_s[0]) + 1e-9:
            return dict(points[0])
        idx = max(0, min(len(points) - 2, int(bisect.bisect_left(path_s, target_s) - 1)))
        while idx + 1 < len(path_s) - 1 and target_s > float(path_s[idx + 1]) + 1e-9:
            idx += 1
        s0 = float(path_s[idx])
        s1 = float(path_s[idx + 1])
        denom = max(1e-9, s1 - s0)
        ratio = max(0.0, min(1.0, (target_s - s0) / denom))
        base = dict(points[idx + 1] if ratio >= 0.5 else points[idx])
        x0 = float(points[idx].get("x", 0.0))
        y0 = float(points[idx].get("y", 0.0))
        x1 = float(points[idx + 1].get("x", 0.0))
        y1 = float(points[idx + 1].get("y", 0.0))
        base["x"] = x0 + (x1 - x0) * ratio
        base["y"] = y0 + (y1 - y0) * ratio
        if "row" in points[idx] and "row" in points[idx + 1]:
            base["row"] = float(points[idx].get("row", 0.0)) + (
                float(points[idx + 1].get("row", 0.0)) - float(points[idx].get("row", 0.0))
            ) * ratio
        if "col" in points[idx] and "col" in points[idx + 1]:
            base["col"] = float(points[idx].get("col", 0.0)) + (
                float(points[idx + 1].get("col", 0.0)) - float(points[idx].get("col", 0.0))
            ) * ratio
        base["point_type"] = "segment_boundary"
        base["index"] = self._index_at_arc_length(target_s)
        return base

    def _slice_path_points_by_arc(self, start_s, end_s):
        points = list(self.current_path.points if self.current_path is not None else [])
        path_s = self._ensure_path_arc_lengths()
        if not points:
            return []
        if not path_s:
            return [dict(p) for p in points]
        start_s = max(0.0, min(float(start_s), float(path_s[-1])))
        end_s = max(start_s, min(float(end_s), float(path_s[-1])))
        sliced = [self._interpolate_path_point_at_s(start_s)]
        for idx, value in enumerate(path_s):
            value = float(value)
            if start_s + 1e-5 < value < end_s - 1e-5:
                sliced.append(dict(points[idx]))
        if end_s > start_s + 1e-5:
            sliced.append(self._interpolate_path_point_at_s(end_s))

        deduped = []
        for point in sliced:
            if not point:
                continue
            if deduped:
                prev = deduped[-1]
                if math.hypot(
                    float(point.get("x", 0.0)) - float(prev.get("x", 0.0)),
                    float(point.get("y", 0.0)) - float(prev.get("y", 0.0)),
                ) < 1e-4:
                    continue
            deduped.append(point)
        return deduped

    def _path_tangent_quaternion_for_points(self, points, index):
        if not points:
            return self._yaw_to_quaternion(0.0)
        index = max(0, min(len(points) - 1, int(index)))
        cx = float(points[index].get("x", 0.0))
        cy = float(points[index].get("y", 0.0))
        for next_idx in range(index + 1, len(points)):
            nx = float(points[next_idx].get("x", 0.0))
            ny = float(points[next_idx].get("y", 0.0))
            if math.hypot(nx - cx, ny - cy) > 1e-6:
                return self._yaw_to_quaternion(math.atan2(ny - cy, nx - cx))
        for prev_idx in range(index - 1, -1, -1):
            px = float(points[prev_idx].get("x", 0.0))
            py = float(points[prev_idx].get("y", 0.0))
            if math.hypot(cx - px, cy - py) > 1e-6:
                return self._yaw_to_quaternion(math.atan2(cy - py, cx - px))
        return self._yaw_to_quaternion(0.0)

    def _path_tangent_quaternion_at_s(self, target_s):
        points = list(self.current_path.points if self.current_path is not None else [])
        path_s = self._ensure_path_arc_lengths()
        if len(points) < 2 or not path_s:
            return self._yaw_to_quaternion(0.0)
        target_s = max(0.0, min(float(target_s), float(path_s[-1])))
        chosen = 0
        for idx in range(len(points) - 1):
            if float(path_s[idx]) - 1e-6 <= target_s <= float(path_s[idx + 1]) + 1e-6:
                chosen = idx
                break
        if chosen >= len(points) - 1:
            chosen = len(points) - 2
        x0 = float(points[chosen].get("x", 0.0))
        y0 = float(points[chosen].get("y", 0.0))
        x1 = float(points[chosen + 1].get("x", 0.0))
        y1 = float(points[chosen + 1].get("y", 0.0))
        if math.hypot(x1 - x0, y1 - y0) <= 1e-6:
            return self._path_tangent_quaternion_for_points(points, chosen)
        return self._yaw_to_quaternion(math.atan2(y1 - y0, x1 - x0))

    def _build_navigation_path_from_segment_points(self, segment_points):
        nav_path = Path()
        source_frame = (
            self.current_path.nav_path.header.frame_id
            if self.current_path is not None
            and self.current_path.nav_path is not None
            and self.current_path.nav_path.header.frame_id
            else self._live_map_source_frame
        )
        nav_path.header.frame_id = source_frame
        nav_path.header.stamp = rospy.Time(0)
        for idx, point in enumerate(segment_points):
            quat = self._path_tangent_quaternion_for_points(segment_points, idx)
            pose = PoseStamped()
            pose.header.frame_id = nav_path.header.frame_id
            pose.header.stamp = rospy.Time(0)
            pose.pose.position.x = float(point.get("x", 0.0))
            pose.pose.position.y = float(point.get("y", 0.0))
            pose.pose.orientation.x = float(quat["x"])
            pose.pose.orientation.y = float(quat["y"])
            pose.pose.orientation.z = float(quat["z"])
            pose.pose.orientation.w = float(quat["w"])
            nav_path.poses.append(pose)
        return nav_path

    def _active_segment_window_s(self):
        if not self._active_segments:
            return None
        index = max(0, min(int(self._active_segment_index), len(self._active_segments) - 1))
        segment = self._active_segments[index]
        end_s = float(segment.get("end_s", 0.0))
        start_s = float(self._active_segment_start_s)
        if start_s >= end_s:
            start_s = float(segment.get("start_s", 0.0))
        start_s = max(0.0, min(start_s, end_s))
        return start_s, end_s, segment

    def _publish_active_segment_plan(self, reason=""):
        try:
            if self.current_path is None or not self.current_path.points:
                return
            if not self._active_segments:
                self._rebuild_active_segments()
            window = self._active_segment_window_s()
            if window is None:
                return
            start_s, end_s, _segment = window
            segment_points = self._slice_path_points_by_arc(start_s, end_s)
            if not segment_points:
                return
            msg = self._build_navigation_path_from_segment_points(segment_points)
            self.active_segment_plan_pub.publish(msg)
            rospy.loginfo_throttle(
                2.0,
                "Published active segment plan: segment=%d/%d points=%d reason=%s",
                int(self._active_segment_index) + 1,
                len(self._active_segments),
                len(msg.poses),
                reason or "<empty>",
            )
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Failed to publish active segment plan: %s", exc)

    def _send_active_segment_goal(self, force=False, reason=""):
        if self.current_path is None or not self.current_path.points:
            return
        if not self._active_segments:
            self._rebuild_active_segments()
        if not self._active_segments:
            return
        segment_index = max(0, min(int(self._active_segment_index), len(self._active_segments) - 1))
        if (not force) and self._active_segment_goal_sent_index == segment_index:
            return
        segment = self._active_segments[segment_index]
        goal_s = float(segment.get("end_s", 0.0))
        endpoint = self._interpolate_path_point_at_s(goal_s)
        goal_index = self._index_at_arc_length(goal_s)
        quat = self._path_tangent_quaternion_at_s(goal_s)
        self.goal_pub.publish(self._build_navigation_goal_msg(endpoint, goal_index, quat))
        self._active_segment_goal_sent_index = segment_index
        self._exec_goal_start_time = time.time()
        rospy.loginfo(
            "Published active segment goal: segment=%d/%d path_index=%d x=%.3f y=%.3f reason=%s",
            segment_index + 1,
            len(self._active_segments),
            int(goal_index),
            float(endpoint.get("x", 0.0)),
            float(endpoint.get("y", 0.0)),
            reason or "<empty>",
        )

    def _project_pose_to_path_progress(self, x, y, min_s=None, max_s=None):
        points = list(self.current_path.points if self.current_path is not None else [])
        path_s = self._ensure_path_arc_lengths()
        if not points:
            return {"progress_s": 0.0, "index": 0, "distance": float("inf")}
        if len(points) == 1 or not path_s:
            return {
                "progress_s": 0.0,
                "index": 0,
                "distance": math.hypot(float(points[0].get("x", 0.0)) - x, float(points[0].get("y", 0.0)) - y),
            }
        min_bound = 0.0 if min_s is None else max(0.0, float(min_s))
        max_bound = float(path_s[-1]) if max_s is None else min(float(path_s[-1]), float(max_s))
        if max_bound < min_bound:
            max_bound = min_bound
        best = {"progress_s": min_bound, "index": 0, "distance": float("inf")}
        for idx in range(len(points) - 1):
            s0 = float(path_s[idx])
            s1 = float(path_s[idx + 1])
            if s1 < min_bound - 1e-6 or s0 > max_bound + 1e-6:
                continue
            x0 = float(points[idx].get("x", 0.0))
            y0 = float(points[idx].get("y", 0.0))
            x1 = float(points[idx + 1].get("x", 0.0))
            y1 = float(points[idx + 1].get("y", 0.0))
            dx = x1 - x0
            dy = y1 - y0
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq <= 1e-12:
                raw_s = s0
            else:
                t = ((float(x) - x0) * dx + (float(y) - y0) * dy) / seg_len_sq
                t = max(0.0, min(1.0, t))
                raw_s = s0 + (s1 - s0) * t
            raw_s = max(min_bound, min(max_bound, raw_s))
            if s1 > s0 + 1e-9:
                ratio = max(0.0, min(1.0, (raw_s - s0) / (s1 - s0)))
            else:
                ratio = 0.0
            px = x0 + dx * ratio
            py = y0 + dy * ratio
            distance = math.hypot(px - float(x), py - float(y))
            if distance < float(best["distance"]):
                best = {
                    "progress_s": float(raw_s),
                    "index": self._index_at_arc_length(raw_s),
                    "distance": float(distance),
                }
        if best["distance"] == float("inf"):
            nearest_idx = self._index_at_arc_length(min_bound)
            interp = points[nearest_idx] if 0 <= nearest_idx < len(points) else points[0]
            best = {
                "progress_s": float(min_bound),
                "index": int(nearest_idx),
                "distance": math.hypot(float(interp.get("x", 0.0)) - float(x), float(interp.get("y", 0.0)) - float(y)),
            }
        return best

    def _project_robot_to_active_segment(self):
        pose = self.aurora_bridge.get_pose()
        window = self._active_segment_window_s()
        if window is None:
            return self._project_pose_to_path_progress(float(pose.get("x", 0.0)), float(pose.get("y", 0.0)))
        start_s, end_s, _segment = window
        return self._project_pose_to_path_progress(
            float(pose.get("x", 0.0)),
            float(pose.get("y", 0.0)),
            min_s=start_s,
            max_s=end_s,
        )

    def _activate_segment(self, segment_index, projection=None, send_goal=True, reason=""):
        if not self._active_segments:
            self._rebuild_active_segments()
        if not self._active_segments:
            return False
        segment_index = max(0, min(int(segment_index), len(self._active_segments) - 1))
        if projection is None:
            pose = self.aurora_bridge.get_pose()
            projection = self._project_pose_to_path_progress(float(pose.get("x", 0.0)), float(pose.get("y", 0.0)))
        progress_s = float(projection.get("progress_s", 0.0))
        self._active_segment_index = segment_index
        self._active_segment_start_s = max(0.0, progress_s - float(self._segment_overlap_behind_m))
        self._active_segment_last_progress_s = progress_s
        self._active_segment_stall_progress_s = progress_s
        self._active_segment_last_progress_update_time = time.time()
        self._active_segment_goal_sent_index = -1
        self.current_path_index = max(self.current_path_index, int(projection.get("index", self.current_path_index)))
        self._publish_active_segment_plan(reason=reason or "segment_activate")
        if send_goal:
            self._send_active_segment_goal(force=True, reason=reason or "segment_activate")
        return True

    def _init_segment_execution_cursor(self):
        if self.current_path is None or not self.current_path.points:
            self.current_path_index = 0
            self._reset_active_segment_state()
            return
        if not self._active_segments:
            self._rebuild_active_segments()
        pose = self.aurora_bridge.get_pose()
        projection = self._project_pose_to_path_progress(float(pose.get("x", 0.0)), float(pose.get("y", 0.0)))
        progress_s = float(projection.get("progress_s", 0.0))
        target_index = len(self._active_segments) - 1 if self._active_segments else 0
        for idx, segment in enumerate(self._active_segments):
            if float(segment.get("end_s", 0.0)) > progress_s + 0.05:
                target_index = idx
                break
        self.current_path_index = int(projection.get("index", 0))
        self._activate_segment(target_index, projection=projection, send_goal=False, reason="cursor_init")

    def _switch_to_next_active_segment(self, projection):
        if not self._active_segments:
            return False
        if self._active_segment_index >= len(self._active_segments) - 1:
            return False
        now = time.time()
        if now - float(self._active_segment_last_switch_time) < 0.2:
            return False
        self._active_segment_last_switch_time = now
        next_index = int(self._active_segment_index) + 1
        return self._activate_segment(
            next_index,
            projection=projection,
            send_goal=True,
            reason="segment_switch",
        )

    def _refresh_active_segment_goal_if_stalled(self, now, reason="stalled_goal_refresh"):
        if float(self._active_segment_goal_refresh_s) <= 0.0:
            return False
        stalled_for = float(now) - float(self._active_segment_last_progress_update_time)
        if stalled_for < float(self._active_segment_goal_refresh_s):
            return False
        rospy.logwarn_throttle(
            2.0,
            "Active segment progress stalled for %.2fs; refreshing segment plan and move_base goal",
            stalled_for,
        )
        self._publish_active_segment_plan(reason=reason)
        self._send_active_segment_goal(force=True, reason=reason)
        self._active_segment_last_progress_update_time = float(now)
        return True

    def _finish_path_execution(self, stop_reason="completed"):
        if self.current_path is None or not self.current_path.points:
            return
        self._mark_current_region_repeat_done()
        if self._try_advance_to_next_region():
            return
        self._exec_active = False
        self._disc_auto_cover_desired = False
        self._reset_disc_motion_guard()
        self._set_cmd_vel_forward_runtime_active(False, publish_zero=True, reason="task_complete")
        self.disc_enable_pub.publish(Bool(data=False))
        self.light_pub.publish(Bool(data=False))
        self._safe_stop_motion()
        self._exec_region_order = []
        self._exec_region_index = -1
        self.state = SchedulerState.COMPLETED
        self._task_stop_reason = str(stop_reason or "completed")
        self._finalize_task_result(stop_reason=self._task_stop_reason)
        rospy.loginfo(
            "Path execution completed: task_id=%s path_version=%s reason=%s",
            self.current_path.task_id,
            self.current_path.path_version,
            self._task_stop_reason,
        )

    def _navigation_alignment_yaw(self):
        if not self._live_map_align_to_initial_yaw:
            return None
        yaw = self._alignment_yaw_for_map_id(self._current_map_id())
        if yaw is None:
            return None
        return float(yaw)

    @staticmethod
    def _yaw_from_quaternion(quat):
        try:
            x = float(quat.get("x", 0.0))
            y = float(quat.get("y", 0.0))
            z = float(quat.get("z", 0.0))
            w = float(quat.get("w", 1.0))
        except Exception:
            return 0.0
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _quaternion_to_navigation_frame(self, quat, alignment_yaw):
        if alignment_yaw is None:
            return quat
        return self._yaw_to_quaternion(self._yaw_from_quaternion(quat) + float(alignment_yaw))

    def _point_to_navigation_frame(self, x, y, alignment_yaw):
        if alignment_yaw is None:
            return float(x), float(y)
        return self._transform_point_by_yaw(x, y, float(alignment_yaw))

    def _pose_for_sl_link_report(self):
        # SL-LinkA previews and APP edit coordinates use the original map
        # frame. Report the robot position and heading in that same frame;
        # alignment/app rotation values are carried as separate metadata.
        return self.aurora_bridge.get_pose() or {}

    def _alignment_yaw_deg_for_sl_link_report(self, map_id=None):
        alignment_yaw = (
            self._alignment_yaw_for_map_id(map_id)
            if str(map_id or "").strip()
            else self._navigation_alignment_yaw()
        )
        if alignment_yaw is None:
            return 0.0
        return math.degrees(float(alignment_yaw))

    def _app_rotation_deg_for_map_id(self, map_id=None):
        target_map_id = str(map_id or "").strip() or self._current_map_id()
        if self._is_live_map_id(target_map_id):
            if self._live_map_app_rotation_deg is not None:
                return float(self._live_map_app_rotation_deg)
        else:
            record = self._find_recorded_map_by_id(target_map_id)
            if isinstance(record, dict):
                value = record.get("app_rotation_deg", None)
                if value is not None and str(value).strip() != "":
                    try:
                        value = float(value)
                        if math.isfinite(value):
                            return value
                    except Exception:
                        pass
        # Backward compatibility for maps saved before app_rotation_deg existed.
        return float(self._alignment_yaw_deg_for_sl_link_report(target_map_id))

    def _rotation_alignment_delta_deg_for_map_id(self, map_id=None):
        target_map_id = str(map_id or "").strip() or self._current_map_id()
        # Recompute from the two source values so legacy records that stored a
        # normalized shortest-angle delta cannot change the requested angle.
        return (
            float(self._app_rotation_deg_for_map_id(target_map_id))
            - float(self._alignment_yaw_deg_for_sl_link_report(target_map_id))
        )

    def _planning_rotation_yaw(self, map_id=None):
        target_map_id = str(map_id or "").strip() or self._current_map_id()
        alignment_yaw = self._alignment_yaw_for_map_id(target_map_id)
        delta_deg = self._rotation_alignment_delta_deg_for_map_id(target_map_id)
        if alignment_yaw is None:
            if abs(float(delta_deg)) <= 1e-9:
                return None
            alignment_yaw = 0.0
        # Keep the signed arithmetic result exactly as configured. Do not fold
        # it into [-180, 180] or an equivalent 180-degree planning axis.
        effective_deg = math.degrees(float(alignment_yaw)) + float(delta_deg)
        # APP angles are clockwise-positive, while the standard XY rotation
        # matrix is counter-clockwise-positive. Convert only at application.
        return math.radians(-effective_deg)

    @staticmethod
    def _normalize_angle_deg(angle_deg):
        value = (float(angle_deg) + 180.0) % 360.0 - 180.0
        return 180.0 if value == -180.0 else value

    def _apply_alignment_yaw_to_response(self, response, map_id=None):
        alignment_deg = float(self._alignment_yaw_deg_for_sl_link_report(map_id))
        app_rotation_deg = float(self._app_rotation_deg_for_map_id(map_id))
        delta_deg = float(self._rotation_alignment_delta_deg_for_map_id(map_id))
        if hasattr(response, "alignment_yaw_deg"):
            response.alignment_yaw_deg = alignment_deg
        if hasattr(response, "app_rotation_deg"):
            response.app_rotation_deg = app_rotation_deg
        if hasattr(response, "rotation_alignment_delta_deg"):
            response.rotation_alignment_delta_deg = delta_deg

    def _build_navigation_path_for_move_base(self):
        nav_path = Path()
        if self.current_path is None:
            return nav_path
        source_frame = (
            self.current_path.nav_path.header.frame_id
            if self.current_path.nav_path is not None and self.current_path.nav_path.header.frame_id
            else self._live_map_source_frame
        )
        nav_path.header.frame_id = source_frame
        nav_path.header.stamp = rospy.Time.now()
        points = list(self.current_path.points or [])
        cached_xy = [(float(p.get("x", 0.0)), float(p.get("y", 0.0))) for p in points]
        for index, point in enumerate(points):
            quat = self._resolve_point_orientation(point, index, cached_xy)
            pose = PoseStamped()
            pose.header = nav_path.header
            pose.pose.position.x = float(point.get("x", 0.0))
            pose.pose.position.y = float(point.get("y", 0.0))
            pose.pose.orientation.x = float(quat["x"])
            pose.pose.orientation.y = float(quat["y"])
            pose.pose.orientation.z = float(quat["z"])
            pose.pose.orientation.w = float(quat["w"])
            nav_path.poses.append(pose)
        return nav_path

    def _build_navigation_goal_msg(self, point, index, quat=None):
        source_frame = (
            self.current_path.nav_path.header.frame_id
            if self.current_path is not None
            and self.current_path.nav_path is not None
            and self.current_path.nav_path.header.frame_id
            else self._live_map_source_frame
        )
        if quat is None:
            cached_xy = [(float(p.get("x", 0.0)), float(p.get("y", 0.0))) for p in self.current_path.points]
            quat = self._resolve_point_orientation(point, index, cached_xy)
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = source_frame
        msg.pose.position.x = float(point.get("x", 0.0))
        msg.pose.position.y = float(point.get("y", 0.0))
        msg.pose.orientation.x = float(quat["x"])
        msg.pose.orientation.y = float(quat["y"])
        msg.pose.orientation.z = float(quat["z"])
        msg.pose.orientation.w = float(quat["w"])
        return msg

    def _publish_path_endpoint_goal(self, reason=""):
        if self.current_path is None or not self.current_path.points:
            return
        endpoint_index = len(self.current_path.points) - 1
        target_index = endpoint_index
        try:
            pose = self.aurora_bridge.get_pose() or {}
            rx = float(pose.get("x", 0.0))
            ry = float(pose.get("y", 0.0))
            nearest_index = 0
            nearest_distance = float("inf")
            for idx, point in enumerate(self.current_path.points):
                dist = math.hypot(float(point.get("x", 0.0)) - rx, float(point.get("y", 0.0)) - ry)
                if dist < nearest_distance:
                    nearest_distance = dist
                    nearest_index = idx
            # Keep cursor in sync for status/progress logic.
            self.current_path_index = nearest_index
            target_index = self._select_precomputed_goal_index(nearest_index)
        except Exception:
            target_index = endpoint_index
        endpoint = self.current_path.points[target_index]
        cached_xy = [(float(p.get("x", 0.0)), float(p.get("y", 0.0))) for p in self.current_path.points]
        quat = self._resolve_point_orientation(endpoint, target_index, cached_xy)
        self.goal_pub.publish(self._build_navigation_goal_msg(endpoint, target_index, quat))

    # Backward compatibility for older internal calls; keeps behavior periodic.
    def _publish_path_endpoint_goal_once(self, reason=""):
        self._publish_path_endpoint_goal(reason=reason)

    def _select_precomputed_goal_index(self, nearest_index):
        if not self._goal_points:
            return self._resolve_straight_segment_end_index(nearest_index)
        for item in self._goal_points:
            idx = int(item.get("path_index", 0))
            if idx >= int(nearest_index):
                return idx
        return int(self._goal_points[-1].get("path_index", 0))

    def _tick(self, _event):
        raw_map = self.aurora_bridge.get_map()
        current_map_id = self._current_map_id()
        if raw_map is not None:
            if self._is_live_map_id(current_map_id):
                self.map_service._grinder_map_source = "live"
                self.map_service._grinder_map_id = self._live_map_id
                self.map_service.set_raw_map(raw_map)
            else:
                rospy.loginfo_throttle(
                    5.0,
                    "Skip live raw map update because saved map is active: active_map_id=%s",
                    current_map_id,
                )
            if current_map_id != self._last_seen_map_id:
                self._last_seen_map_id = current_map_id
                self._sync_task_map_binding(update_binding=False)
                self._sync_task_regions_from_overlay()
                rospy.loginfo(
                    "Map switched, loaded task-map binding: map_id=%s task_id=%s selected_regions=%s",
                    self.task_config.map_id or "<empty>",
                    self.task_config.task_id or "<empty>",
                    ",".join(self.task_config.selected_work_region_ids or []) or "<none>",
                )
            if self._initial_map_preview_enabled:
                self._save_initial_map_preview_once()
            self._update_live_preview(raw_map)
            self._update_live_map_files(raw_map)

        if self.replan_requested and self.state == SchedulerState.RUNNING:
            self._safe_stop_motion()
            if self._plan_current_task():
                self.replan_requested = False
                self._resume_execution()

        self._tick_path_execution()
        self._tick_disc_motion_guard()
        self._update_progress()
        self._record_task_trajectory_sample()
        self._publish_status()
        self._publish_diagnostics()

    def _save_initial_map_preview_once(self):
        if self._initial_map_preview_saved:
            return
        try:
            snapshot = self.map_service.create_preview(
                self.aurora_bridge.get_pose(),
                self._initial_map_preview_max_edge,
                self._initial_map_preview_format,
                True,
                **self._map_preview_alignment_kwargs()
            )
            os.makedirs(self._initial_map_preview_dir, exist_ok=True)
            preview_width, preview_height, shrink_factor = self._preview_meta(
                snapshot.width, snapshot.height, self._initial_map_preview_max_edge
            )
            filename = (
                "aurora_map_preview"
                "_s{}"
                "_res{}"
                "_ox{}"
                "_oy{}"
                "_ow{}"
                "_oh{}"
                "_pw{}"
                "_ph{}"
                "_v{}"
                ".{}"
            ).format(
                self._safe_num(shrink_factor),
                self._safe_num(snapshot.resolution),
                self._safe_num(snapshot.origin_x),
                self._safe_num(snapshot.origin_y),
                snapshot.width,
                snapshot.height,
                preview_width,
                preview_height,
                snapshot.map_version,
                snapshot.preview_format,
            )
            full_path = os.path.join(self._initial_map_preview_dir, filename)
            with open(full_path, "wb") as handle:
                handle.write(snapshot.preview_data)
            self._initial_map_preview_saved = True
            rospy.loginfo("Saved initial map preview to %s", full_path)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Failed to save initial map preview: %s", exc)

    def _preview_meta(self, orig_w, orig_h, max_edge):
        max_edge = max(64.0, float(max_edge))
        max_wh = float(max(orig_w, orig_h))
        scale = min(max_edge / max_wh, 1.0)
        preview_w = max(1, int(round(float(orig_w) * scale)))
        preview_h = max(1, int(round(float(orig_h) * scale)))
        shrink_factor = (max_wh / float(max(preview_w, preview_h))) if scale < 1.0 else 1.0
        return preview_w, preview_h, shrink_factor

    @staticmethod
    def _apply_response_map_info(response, map_info):
        if not isinstance(map_info, dict):
            return
        response.width = int(map_info.get("width", 0))
        response.height = int(map_info.get("height", 0))
        response.resolution = float(map_info.get("resolution", 0.0))
        response.origin.x = float(map_info.get("origin_x", 0.0))
        response.origin.y = float(map_info.get("origin_y", 0.0))
        response.origin.heading_deg = 0.0
        response.frame_id = str(map_info.get("frame_id", ""))

    def _load_robot_config_from_params(self):
        self.task_config.vehicle_width = max(
            0.1, float(rospy.get_param("~vehicle_width", self.task_config.vehicle_width))
        )
        self.task_config.vehicle_length = max(
            0.1, float(rospy.get_param("~vehicle_length", self.task_config.vehicle_length))
        )
        self.task_config.default_path_spacing = max(
            0.2, float(rospy.get_param("~default_path_spacing", self.task_config.default_path_spacing))
        )
        self.task_config.turn_radius = max(
            0.1, float(rospy.get_param("~turn_radius", self.task_config.turn_radius))
        )
        self.task_config.overlap_ratio = max(
            0.0, min(0.95, float(rospy.get_param("~overlap_ratio", self.task_config.overlap_ratio)))
        )
        self.task_config.inflation_radius = max(
            0.0, float(rospy.get_param("~inflation_radius", self.task_config.inflation_radius))
        )

    def _sync_robot_config_to_yaml(self):
        path = self._robot_config_yaml_path
        if not path:
            return
        keys = {
            "vehicle_width": float(self.task_config.vehicle_width),
            "vehicle_length": float(self.task_config.vehicle_length),
            "default_path_spacing": float(self.task_config.default_path_spacing),
            "turn_radius": float(self.task_config.turn_radius),
            "overlap_ratio": float(self.task_config.overlap_ratio),
            "inflation_radius": float(self.task_config.inflation_radius),
        }

        def _fmt(value):
            text = "{:.6f}".format(float(value)).rstrip("0").rstrip(".")
            return text if text else "0"

        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        content = ""
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as handle:
                content = handle.read()
        for key, value in keys.items():
            pattern = r"^(\s*" + re.escape(key) + r"\s*:\s*).*$"
            if re.search(pattern, content, flags=re.MULTILINE):
                # Use callable replacement to avoid backreference ambiguity like "\10.5".
                content = re.sub(
                    pattern,
                    lambda m, v=_fmt(value): "{}{}".format(m.group(1), v),
                    content,
                    flags=re.MULTILINE,
                )
            else:
                if content and not content.endswith("\n"):
                    content += "\n"
                content += "{}: {}\n".format(key, _fmt(value))
        tmp_path = path + ".tmp"
        with open(tmp_path, "w", encoding="utf-8") as handle:
            handle.write(content)
        os.replace(tmp_path, path)

    def _reload_robot_config_from_yaml(self):
        path = self._robot_config_yaml_path
        if not path or (not os.path.exists(path)):
            return False

        values = {}
        with open(path, "r", encoding="utf-8") as handle:
            for raw in handle:
                line = raw.strip()
                if (not line) or line.startswith("#") or (":" not in line):
                    continue
                key, value = line.split(":", 1)
                key = key.strip()
                if key not in (
                    "vehicle_width",
                    "vehicle_length",
                    "default_path_spacing",
                    "turn_radius",
                    "overlap_ratio",
                    "inflation_radius",
                ):
                    continue
                token = value.split("#", 1)[0].strip()
                if not token:
                    continue
                try:
                    values[key] = float(token)
                except Exception:
                    continue

        if not values:
            return False

        if "vehicle_width" in values:
            self.task_config.vehicle_width = max(0.1, float(values["vehicle_width"]))
        if "vehicle_length" in values:
            self.task_config.vehicle_length = max(0.1, float(values["vehicle_length"]))
        if "default_path_spacing" in values:
            self.task_config.default_path_spacing = max(0.2, float(values["default_path_spacing"]))
        if "turn_radius" in values:
            self.task_config.turn_radius = max(0.1, float(values["turn_radius"]))
        if "overlap_ratio" in values:
            self.task_config.overlap_ratio = max(0.0, min(0.95, float(values["overlap_ratio"])))
        if "inflation_radius" in values:
            self.task_config.inflation_radius = max(0.0, float(values["inflation_radius"]))
        return True

    def _safe_num(self, value):
        text = "{:.4f}".format(float(value)).rstrip("0").rstrip(".")
        if text == "-0":
            text = "0"
        return text.replace("-", "m").replace(".", "p")

    def _detect_preview_rga_available(self):
        # RK3588 exposes RGA as /dev/rga. The Python render path still needs a
        # librga binding before we can safely replace OpenCV operations.
        return os.path.exists("/dev/rga")

    def _update_live_preview(self, raw_map):
        if not self._live_preview_enabled:
            return
        if self.state == SchedulerState.PLANNING:
            rospy.loginfo_throttle(2.0, "Skip live preview update while planning")
            return
        now = time.time()
        if now < self._live_preview_next_time:
            return
        self._live_preview_next_time = now + (1.0 / self._live_preview_hz)
        try:
            snapshot = self.map_service.create_preview(
                self.aurora_bridge.get_pose(),
                self._live_preview_max_edge,
                self._live_preview_format,
                True,
                **self._map_preview_alignment_kwargs()
            )
            os.makedirs(os.path.dirname(self._live_preview_file) or ".", exist_ok=True)
            tmp_path = self._live_preview_file + ".tmp"
            with open(tmp_path, "wb") as handle:
                handle.write(snapshot.preview_data)
            os.replace(tmp_path, self._live_preview_file)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Failed to update live map preview: %s", exc)

    def _update_live_map_files(self, raw_map):
        if not self._live_map_enabled:
            return
        if self.state == SchedulerState.PLANNING:
            rospy.loginfo_throttle(2.0, "Skip live map export while planning")
            return
        now = time.time()
        if now < self._live_map_next_time:
            return
        self._live_map_next_time = now + (1.0 / self._live_map_hz)
        try:
            map_info = self.map_service.get_map_info()
            composed_map = self.map_service.compose_map()
            if map_info is not None and composed_map is not None:
                self._export_runtime_map(
                    raw_map,
                    self._live_map_dir,
                    self._live_map_yaml_name,
                    self._live_map_image_name,
                    grid_override=composed_map,
                    map_info_override=map_info,
                )
            else:
                self._export_runtime_map(raw_map, self._live_map_dir, self._live_map_yaml_name, self._live_map_image_name)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Failed to export live map files: %s", exc)

    def _export_runtime_map(
        self,
        raw_map,
        out_dir,
        yaml_name,
        image_name,
        grid_override=None,
        map_info_override=None,
        align_to_initial_yaw=None,
        crop_to_free_space=None,
    ):
        os.makedirs(out_dir, exist_ok=True)
        if grid_override is not None and map_info_override is not None:
            width = int(map_info_override["width"])
            height = int(map_info_override["height"])
            grid = np.array(grid_override, dtype=np.int16).reshape((height, width))
            origin_x = float(map_info_override["origin_x"])
            origin_y = float(map_info_override["origin_y"])
            resolution = float(map_info_override["resolution"])
            map_version = int(map_info_override.get("map_version", -1))
        else:
            info = raw_map.info
            width = int(info.width)
            height = int(info.height)
            grid = np.array(raw_map.data, dtype=np.int16).reshape((height, width))
            origin_x = float(info.origin.position.x)
            origin_y = float(info.origin.position.y)
            resolution = float(info.resolution)
            map_version = -1

        if align_to_initial_yaw is None:
            align_to_initial_yaw = self._live_map_align_to_initial_yaw
        if crop_to_free_space is None:
            crop_to_free_space = self._live_map_crop_to_free_space

        if (
            bool(align_to_initial_yaw)
            and self._tf_buffer is not None
            and euler_from_quaternion is not None
            and width > 0
            and height > 0
        ):
            yaw = self._lookup_live_map_alignment_yaw()
            if yaw is not None and abs(float(yaw)) > 1e-6:
                cache_hit = (
                    self._live_map_last_rotated_grid is not None
                    and self._live_map_last_rotated_origin is not None
                    and map_version >= 0
                    and self._live_map_last_rotated_version == map_version
                    and self._live_map_last_rotated_yaw is not None
                    and abs(float(self._live_map_last_rotated_yaw) - float(yaw)) < 1e-9
                )
                if cache_hit:
                    grid = self._live_map_last_rotated_grid
                    origin_x, origin_y = self._live_map_last_rotated_origin
                    height, width = grid.shape[:2]
                else:
                    grid, origin_x, origin_y = self._rotate_grid_to_aligned_frame(
                        grid, origin_x, origin_y, resolution, float(yaw)
                    )
                    height, width = grid.shape[:2]
                    self._live_map_last_rotated_version = map_version
                    self._live_map_last_rotated_yaw = float(yaw)
                    self._live_map_last_rotated_grid = grid
                    self._live_map_last_rotated_origin = (float(origin_x), float(origin_y))

        if bool(crop_to_free_space) and width > 0 and height > 0:
            free_rows, free_cols = np.where(grid == 0)
            if free_rows.size > 0 and free_cols.size > 0:
                margin_cells = int(round(self._live_map_crop_margin_m / max(resolution, 1e-6)))
                min_row = max(0, int(free_rows.min()) - margin_cells)
                max_row = min(height - 1, int(free_rows.max()) + margin_cells)
                min_col = max(0, int(free_cols.min()) - margin_cells)
                max_col = min(width - 1, int(free_cols.max()) + margin_cells)
                grid = grid[min_row:max_row + 1, min_col:max_col + 1]
                height, width = grid.shape[:2]
                origin_x += float(min_col) * resolution
                origin_y += float(min_row) * resolution
                rospy.loginfo_throttle(
                    5.0,
                    "Live map cropped to free-space bbox: size=%dx%d margin=%.2fm",
                    width,
                    height,
                    self._live_map_crop_margin_m,
                )
        image = np.full((height, width), 205, dtype=np.uint8)
        image[grid == 0] = 254
        image[grid >= 100] = 0
        image = np.flipud(image)

        image_path = os.path.join(out_dir, image_name)
        yaml_path = os.path.join(out_dir, yaml_name)
        if not cv2.imwrite(image_path, image):
            raise RuntimeError("Failed to write map image: {}".format(image_path))
        yaml_text = "\n".join(
            [
                "image: {}".format(image_name),
                "resolution: {:.6f}".format(resolution),
                "origin: [{:.6f}, {:.6f}, 0.000000]".format(origin_x, origin_y),
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "",
            ]
        )
        tmp_yaml = yaml_path + ".tmp"
        with open(tmp_yaml, "w", encoding="utf-8") as handle:
            handle.write(yaml_text)
        os.replace(tmp_yaml, yaml_path)
        return yaml_path, image_path

    def _save_raw_grid_snapshot_for_map(self, map_id, raw_map, alignment_yaw=None):
        target_map_id = str(map_id or "").strip()
        if not target_map_id or raw_map is None:
            return "", ""
        out_dir = os.path.join(self._map_state_dir(target_map_id), "raw_grid")
        yaml_path, image_path = self._export_runtime_map(
            raw_map,
            out_dir,
            "map.yaml",
            "map.pgm",
            align_to_initial_yaw=False,
            crop_to_free_space=False,
        )
        record = self._find_recorded_map_by_id(target_map_id)
        saved_yaw_for_log = None
        if isinstance(record, dict):
            record["raw_grid_yaml_path"] = yaml_path
            record["raw_grid_image_path"] = image_path
            record["raw_grid_saved_at"] = int(time.time())
            saved_yaw = alignment_yaw
            if saved_yaw is None and self._live_map_align_to_initial_yaw:
                saved_yaw = self._lookup_live_map_alignment_yaw()
            if saved_yaw is not None:
                try:
                    saved_yaw = float(saved_yaw)
                    if math.isfinite(saved_yaw):
                        record["alignment_yaw"] = saved_yaw
                        record["alignment_yaw_deg"] = math.degrees(saved_yaw)
                        record["alignment_source_frame_id"] = self._live_map_source_frame
                        record["alignment_frame_id"] = self._live_map_aligned_frame
                        saved_yaw_for_log = saved_yaw
                except Exception:
                    pass
            try:
                record["raw_grid_width"] = int(raw_map.info.width)
                record["raw_grid_height"] = int(raw_map.info.height)
                record["raw_grid_resolution"] = float(raw_map.info.resolution)
                record["raw_grid_origin_x"] = float(raw_map.info.origin.position.x)
                record["raw_grid_origin_y"] = float(raw_map.info.origin.position.y)
                record["raw_grid_frame_id"] = str(raw_map.header.frame_id or "")
            except Exception:
                pass
            self._save_map_registry_state()
        rospy.loginfo(
            "Saved raw grid snapshot for map: map_id=%s yaml=%s image=%s alignment_yaw=%s",
            target_map_id,
            yaml_path,
            image_path,
            "{:.6f}".format(float(saved_yaw_for_log)) if saved_yaw_for_log is not None else "<none>",
        )
        return yaml_path, image_path

    def _is_live_map_id(self, map_id):
        text = str(map_id or "").strip()
        return text in ("", self._live_map_id, DEFAULT_LIVE_MAP_ID)

    def _saved_raw_grid_paths_for_map(self, map_id):
        target_map_id = str(map_id or "").strip()
        if not target_map_id:
            return "", ""
        record = self._find_recorded_map_by_id(target_map_id)
        yaml_path = ""
        image_path = ""
        if isinstance(record, dict):
            yaml_path = str(record.get("raw_grid_yaml_path", "") or "").strip()
            image_path = str(record.get("raw_grid_image_path", "") or "").strip()
        if not yaml_path:
            yaml_path = os.path.join(self._map_state_dir(target_map_id), "raw_grid", "map.yaml")
        if not image_path:
            image_path = os.path.join(os.path.dirname(yaml_path), "map.pgm")
        return yaml_path, image_path

    def _load_raw_grid_yaml(self, yaml_path):
        data = {}
        with open(yaml_path, "r", encoding="utf-8") as handle:
            for raw_line in handle:
                line = raw_line.strip()
                if not line or line.startswith("#") or ":" not in line:
                    continue
                key, value = line.split(":", 1)
                data[key.strip()] = value.strip()
        return data

    def _load_saved_raw_grid_map(self, map_id):
        target_map_id = str(map_id or "").strip()
        yaml_path, image_path = self._saved_raw_grid_paths_for_map(target_map_id)
        if not yaml_path or not os.path.isfile(yaml_path):
            raise RuntimeError("saved raw grid yaml not found for map_id={}: {}".format(target_map_id, yaml_path))
        meta = self._load_raw_grid_yaml(yaml_path)
        image_name = str(meta.get("image", "") or "").strip()
        if image_name:
            candidate = image_name
            if not os.path.isabs(candidate):
                candidate = os.path.join(os.path.dirname(yaml_path), candidate)
            image_path = candidate
        if not image_path or not os.path.isfile(image_path):
            raise RuntimeError("saved raw grid image not found for map_id={}: {}".format(target_map_id, image_path))
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise RuntimeError("failed to read saved raw grid image: {}".format(image_path))
        grid_image = np.flipud(image)
        grid = np.full(grid_image.shape, -1, dtype=np.int16)
        grid[grid_image >= 250] = 0
        grid[grid_image <= 10] = 100

        resolution = float(meta.get("resolution", 0.05) or 0.05)
        origin_text = str(meta.get("origin", "[0, 0, 0]") or "[0, 0, 0]").strip()
        origin_values = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", origin_text)
        origin_x = float(origin_values[0]) if len(origin_values) >= 1 else 0.0
        origin_y = float(origin_values[1]) if len(origin_values) >= 2 else 0.0

        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.info.width = int(grid.shape[1])
        msg.info.height = int(grid.shape[0])
        msg.info.resolution = float(resolution)
        msg.info.origin.position.x = float(origin_x)
        msg.info.origin.position.y = float(origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = [int(value) for value in grid.reshape(-1)]
        return msg

    def _build_offline_map_service_for_map(self, map_id):
        target_map_id = str(map_id or "").strip()
        raw_map = self._load_saved_raw_grid_map(target_map_id)
        service = MapService()
        service._grinder_map_source = "offline_raw_grid"
        service._grinder_map_id = target_map_id
        service.set_draw_region_id_on_preview(self._draw_region_id_on_preview)
        service.set_draw_region_label_on_preview(self._draw_region_label_on_preview)
        service.configure_preview_rga(
            self._preview_rga_enabled,
            self._preview_rga_backend,
            self._preview_rga_available,
        )
        service.set_raw_map(raw_map)
        service.load_local_state(self._map_state_dir(target_map_id))
        return service

    def _ensure_offline_map_service_for_current_map(self, reason=""):
        current_map_id = self._current_map_id()
        if self._is_live_map_id(current_map_id):
            return False
        source = str(getattr(self.map_service, "_grinder_map_source", "") or "")
        source_map_id = str(getattr(self.map_service, "_grinder_map_id", "") or "")
        if source == "offline_raw_grid" and source_map_id == current_map_id:
            return False
        self.map_service = self._build_offline_map_service_for_map(current_map_id)
        self._preview_snapshot_cache_key = None
        self._preview_snapshot_cache = None
        map_info = self.map_service.get_map_info() or {}
        rospy.loginfo(
            "Forced offline map service for planning: reason=%s map_id=%s map_size=%sx%s resolution=%.4f previous_source=%s previous_map_id=%s",
            reason or "<empty>",
            current_map_id,
            int(map_info.get("width", 0) or 0),
            int(map_info.get("height", 0) or 0),
            float(map_info.get("resolution", 0.0) or 0.0),
            source or "<unknown>",
            source_map_id or "<empty>",
        )
        return True

    def _map_service_for_preview(self, map_id):
        target_map_id = str(map_id or "").strip()
        if self._is_live_map_id(target_map_id):
            return self.map_service, False
        service = self._build_offline_map_service_for_map(target_map_id)
        rospy.loginfo("Using offline raw grid for preview: map_id=%s", target_map_id)
        return service, True

    def _lookup_live_map_alignment_yaw(self):
        if self._initial_pose_alignment_yaw is not None:
            return self._initial_pose_alignment_yaw
        if self._tf_buffer is not None:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._live_map_aligned_frame,
                    self._live_map_source_frame,
                    rospy.Time(0),
                    rospy.Duration(self._live_map_align_yaw_timeout),
                )
                q = transform.transform.rotation
                _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                if math.isfinite(float(yaw)):
                    self._initial_pose_alignment_yaw = float(yaw)
                    rospy.loginfo(
                        "Map preview yaw alignment initialized from tf: %.6f rad",
                        self._initial_pose_alignment_yaw,
                    )
                    return self._initial_pose_alignment_yaw
            except Exception as exc:
                rospy.logwarn_throttle(2.0, "Failed to lookup live map alignment yaw: %s", exc)
        return self._initial_pose_alignment_yaw_from_pose(self.aurora_bridge.get_initial_pose())

    def _initial_pose_alignment_yaw_from_pose(self, pose):
        if self._initial_pose_alignment_yaw is not None:
            return self._initial_pose_alignment_yaw
        if not isinstance(pose, dict):
            self._initial_pose_alignment_yaw = 0.0
            rospy.logwarn_throttle(
                2.0,
                "Initial pose unavailable; using zero yaw fallback for %s publishing",
                self._live_map_aligned_frame,
            )
            return self._initial_pose_alignment_yaw
        try:
            heading_deg = float(pose.get("heading_deg", 0.0))
        except Exception:
            heading_deg = 0.0
        if not math.isfinite(heading_deg):
            heading_deg = 0.0
        self._initial_pose_alignment_yaw = -math.radians(heading_deg)
        rospy.loginfo("Map preview yaw alignment initialized from odom: %.6f rad", self._initial_pose_alignment_yaw)
        return self._initial_pose_alignment_yaw

    def _alignment_yaw_for_map_id(self, map_id=None):
        if not self._live_map_align_to_initial_yaw:
            return None
        target_map_id = str(map_id or "").strip() or self._current_map_id()
        if self._is_live_map_id(target_map_id):
            return self._lookup_live_map_alignment_yaw()
        record = self._find_recorded_map_by_id(target_map_id)
        if isinstance(record, dict):
            for key in ("alignment_yaw", "raw_grid_alignment_yaw"):
                value = record.get(key, None)
                if value is None or str(value).strip() == "":
                    continue
                try:
                    yaw = float(value)
                    if math.isfinite(yaw):
                        return yaw
                except Exception:
                    continue
        yaw = self._lookup_live_map_alignment_yaw()
        rospy.logwarn_throttle(
            2.0,
            "Saved map alignment yaw missing, fallback to current live yaw: map_id=%s yaw=%s",
            target_map_id,
            "{:.6f}".format(float(yaw)) if yaw is not None else "<none>",
        )
        return yaw

    def _map_preview_alignment_kwargs(self, map_id=None):
        # All externally returned previews stay in the original map frame.
        return {}

    def _path_planning_preview_alignment_kwargs(self, map_id=None):
        # mst27 plans directly in the original map frame. Keep the planning
        # response preview in that same frame instead of rotating its base map.
        return {}

    def _invalidate_preview_caches(self):
        self._preview_snapshot_cache_key = None
        self._preview_snapshot_cache = None
        self._invalidate_path_preview_payload_cache()
        self._invalidate_path_preview_overlay_base_cache()

    def _set_app_rotation_for_map_id(self, map_id, rotation_deg):
        target_map_id = str(map_id or "").strip() or self._current_map_id()
        if not math.isfinite(float(rotation_deg)):
            raise RuntimeError("invalid rotation angle")
        rotation_deg = float(rotation_deg)
        rotation_rad = math.radians(rotation_deg)
        alignment_deg = self._alignment_yaw_deg_for_sl_link_report(target_map_id)
        delta_deg = rotation_deg - alignment_deg
        if self._is_live_map_id(target_map_id):
            self._live_map_app_rotation_deg = rotation_deg
            self._live_map_rotation_alignment_delta_deg = delta_deg
        else:
            if not self._write_app_rotation_to_record(target_map_id, rotation_deg):
                raise RuntimeError("map_id not found: {}".format(target_map_id))
        # A rotation change alters both the planning grid and every planning
        # region. Never allow the next PathPlanRequest to reuse the old path.
        self._path_plan_request_cache_key = None
        self._path_plan_request_cache_path_version = 0
        self._invalidate_preview_caches()
        # Saved-map angles are already written to map_registry.json above;
        # this additionally persists LIVE_MAP rotation in scheduler_state.json.
        self._save_local_state()
        rospy.loginfo(
            "Map APP rotation updated without changing alignment yaw; path cache invalidated: map_id=%s rotation_deg=%.3f alignment_yaw_deg=%.3f delta_deg=%.3f",
            target_map_id or self._live_map_id,
            rotation_deg,
            alignment_deg,
            delta_deg,
        )
        return target_map_id or self._live_map_id, rotation_deg, rotation_rad

    def _write_alignment_yaw_to_record(self, map_id, yaw_rad):
        target_map_id = str(map_id or "").strip()
        if not target_map_id or yaw_rad is None:
            return False
        try:
            yaw_rad = float(yaw_rad)
        except Exception:
            return False
        if not math.isfinite(yaw_rad):
            return False
        record = self._find_recorded_map_by_id(target_map_id)
        if not isinstance(record, dict):
            return False
        record["alignment_yaw"] = yaw_rad
        record["alignment_yaw_deg"] = math.degrees(yaw_rad)
        record["raw_grid_alignment_yaw"] = yaw_rad
        record["raw_grid_alignment_yaw_deg"] = math.degrees(yaw_rad)
        record["alignment_source_frame_id"] = self._live_map_source_frame
        record["alignment_frame_id"] = self._live_map_aligned_frame
        self._save_map_registry_state()
        return True

    def _write_app_rotation_to_record(self, map_id, rotation_deg):
        target_map_id = str(map_id or "").strip()
        if not target_map_id:
            return False
        try:
            rotation_deg = float(rotation_deg)
        except Exception:
            return False
        if not math.isfinite(rotation_deg):
            return False
        record = self._find_recorded_map_by_id(target_map_id)
        if not isinstance(record, dict):
            return False
        record["app_rotation_deg"] = rotation_deg
        record["rotation_alignment_delta_deg"] = (
            rotation_deg - self._alignment_yaw_deg_for_sl_link_report(target_map_id)
        )
        self._save_map_registry_state()
        return True

    def _alignment_yaw_from_map_save_request(self, request, fallback_map_id=None):
        # APP rotation is independent metadata. It must never replace the
        # startup alignment yaw captured from the radar's initial pose.
        return self._alignment_yaw_for_map_id(fallback_map_id)

    def _app_rotation_deg_from_map_save_request(self, request, fallback_map_id=None):
        if bool(getattr(request, "has_rotation_deg", False)):
            rotation_deg = float(getattr(request, "rotation_deg", 0.0))
            if not math.isfinite(rotation_deg):
                raise RuntimeError("invalid rotation_deg")
            return rotation_deg
        return self._app_rotation_deg_for_map_id(fallback_map_id)

    @staticmethod
    def _transform_point_by_yaw(point_x, point_y, yaw):
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            cos_yaw * float(point_x) - sin_yaw * float(point_y),
            sin_yaw * float(point_x) + cos_yaw * float(point_y),
        )

    def _point_from_aligned_to_source_map(self, point, alignment_yaw):
        source_x, source_y = self._transform_point_by_yaw(
            point.get("x", 0.0),
            point.get("y", 0.0),
            -float(alignment_yaw),
        )
        return {"x": source_x, "y": source_y}

    def _points_from_aligned_to_source_map(self, points, alignment_yaw):
        if not points:
            return []
        return [self._point_from_aligned_to_source_map(point, alignment_yaw) for point in points]

    def _pose_from_aligned_to_source_map(self, pose, alignment_yaw):
        if not isinstance(pose, dict) or not pose:
            return {}
        source_x, source_y = self._transform_point_by_yaw(
            pose.get("x", 0.0),
            pose.get("y", 0.0),
            -float(alignment_yaw),
        )
        source_pose = dict(pose)
        source_pose["x"] = source_x
        source_pose["y"] = source_y
        try:
            source_pose["heading_deg"] = float(pose.get("heading_deg", 0.0)) - math.degrees(float(alignment_yaw))
        except Exception:
            source_pose["heading_deg"] = 0.0
        return source_pose

    def _point_from_source_to_aligned_map(self, point, alignment_yaw):
        aligned_x, aligned_y = self._transform_point_by_yaw(
            point.get("x", 0.0),
            point.get("y", 0.0),
            float(alignment_yaw),
        )
        aligned_point = dict(point)
        aligned_point["x"] = aligned_x
        aligned_point["y"] = aligned_y
        return aligned_point

    def _points_from_source_to_aligned_map(self, points, alignment_yaw):
        if not points:
            return []
        return [self._point_from_source_to_aligned_map(point, alignment_yaw) for point in points]

    def _pose_from_source_to_aligned_map(self, pose, alignment_yaw):
        if not isinstance(pose, dict) or not pose:
            return {}
        aligned_x, aligned_y = self._transform_point_by_yaw(
            pose.get("x", 0.0),
            pose.get("y", 0.0),
            float(alignment_yaw),
        )
        aligned_pose = dict(pose)
        aligned_pose["x"] = aligned_x
        aligned_pose["y"] = aligned_y
        try:
            aligned_pose["heading_deg"] = float(pose.get("heading_deg", 0.0)) + math.degrees(float(alignment_yaw))
        except Exception:
            aligned_pose["heading_deg"] = 0.0
        orientation = pose.get("orientation") if isinstance(pose, dict) else None
        if isinstance(orientation, dict):
            aligned_pose["orientation"] = self._yaw_to_quaternion(
                self._yaw_from_quaternion(orientation) + float(alignment_yaw)
            )
        covariance = pose.get("localization_covariance")
        if isinstance(covariance, dict):
            aligned_pose["localization_covariance"] = self._rotate_localization_covariance(
                covariance,
                float(alignment_yaw),
            )
        return aligned_pose

    @staticmethod
    def _rotate_localization_covariance(covariance, yaw):
        if not isinstance(covariance, dict) or not bool(covariance.get("valid", False)):
            return {"valid": False}
        try:
            c = math.cos(float(yaw))
            s = math.sin(float(yaw))
            xx = float(covariance.get("x_variance", 0.0))
            yy = float(covariance.get("y_variance", 0.0))
            xy = float(covariance.get("xy_covariance", 0.0))
            x_yaw = float(covariance.get("x_yaw_covariance", 0.0))
            y_yaw = float(covariance.get("y_yaw_covariance", 0.0))
            return {
                "valid": True,
                "x_variance": c * c * xx - 2.0 * c * s * xy + s * s * yy,
                "y_variance": s * s * xx + 2.0 * c * s * xy + c * c * yy,
                "xy_covariance": c * s * (xx - yy) + (c * c - s * s) * xy,
                "x_yaw_covariance": c * x_yaw - s * y_yaw,
                "y_yaw_covariance": s * x_yaw + c * y_yaw,
                "yaw_variance": float(covariance.get("yaw_variance", 0.0)),
            }
        except Exception:
            return {"valid": False}

    def _apply_localization_covariance(self, message, pose=None):
        if not hasattr(message, "localization_covariance"):
            return
        if pose is None:
            pose = self._pose_for_sl_link_report()
        covariance = pose.get("localization_covariance") if isinstance(pose, dict) else None
        if not isinstance(covariance, dict) or not bool(covariance.get("valid", False)):
            message.localization_covariance.valid = False
            return
        message.localization_covariance.valid = True
        message.localization_covariance.x_variance = float(covariance.get("x_variance", 0.0))
        message.localization_covariance.y_variance = float(covariance.get("y_variance", 0.0))
        message.localization_covariance.yaw_variance = float(covariance.get("yaw_variance", 0.0))
        message.localization_covariance.xy_covariance = float(covariance.get("xy_covariance", 0.0))
        message.localization_covariance.x_yaw_covariance = float(covariance.get("x_yaw_covariance", 0.0))
        message.localization_covariance.y_yaw_covariance = float(covariance.get("y_yaw_covariance", 0.0))

    def _region_from_source_to_aligned_map(self, region, alignment_yaw):
        if not isinstance(region, dict):
            return region
        aligned_region = deepcopy(region)
        aligned_region["points"] = self._points_from_source_to_aligned_map(
            region.get("points", []) or [],
            alignment_yaw,
        )
        if isinstance(region.get("start_pose", None), dict) and region.get("start_pose"):
            aligned_region["start_pose"] = self._pose_from_source_to_aligned_map(region.get("start_pose"), alignment_yaw)
        if isinstance(region.get("end_pose", None), dict) and region.get("end_pose"):
            aligned_region["end_pose"] = self._pose_from_source_to_aligned_map(region.get("end_pose"), alignment_yaw)
        return aligned_region

    def _build_aligned_planning_map(self, composed_map, map_info, alignment_yaw):
        resolution = max(1e-9, float((map_info or {}).get("resolution", 0.05)))
        origin_x = float((map_info or {}).get("origin_x", 0.0))
        origin_y = float((map_info or {}).get("origin_y", 0.0))
        aligned_grid, aligned_origin_x, aligned_origin_y = self._rotate_grid_to_aligned_frame(
            composed_map,
            origin_x,
            origin_y,
            resolution,
            float(alignment_yaw),
        )
        aligned_map_info = dict(map_info or {})
        aligned_map_info.update(
            {
                "width": int(aligned_grid.shape[1]),
                "height": int(aligned_grid.shape[0]),
                "origin_x": float(aligned_origin_x),
                "origin_y": float(aligned_origin_y),
                "resolution": resolution,
                "frame_id": self._live_map_aligned_frame,
                "source_frame_id": str((map_info or {}).get("frame_id", self._live_map_source_frame)),
                "alignment_yaw": float(alignment_yaw),
            }
        )
        return aligned_grid, aligned_map_info

    def _task_config_for_aligned_planning(self, task_config, alignment_yaw):
        direction = normalize_planning_direction(
            getattr(task_config, "global_direction", "x")
        )
        return TaskConfigModel(
            task_id=task_config.task_id,
            map_id=task_config.map_id,
            work_regions=[
                self._region_from_source_to_aligned_map(region, alignment_yaw)
                for region in list(task_config.work_regions or [])
            ],
            obstacle_regions=[
                self._region_from_source_to_aligned_map(region, alignment_yaw)
                for region in list(task_config.obstacle_regions or [])
            ],
            erase_regions=[
                self._region_from_source_to_aligned_map(region, alignment_yaw)
                for region in list(task_config.erase_regions or [])
            ],
            crop_region=(
                self._region_from_source_to_aligned_map(task_config.crop_region, alignment_yaw)
                if isinstance(task_config.crop_region, dict) and task_config.crop_region
                else {}
            ),
            active_work_region_id=task_config.active_work_region_id,
            selected_work_region_ids=list(task_config.selected_work_region_ids or []),
            region_repeat_config=dict(task_config.region_repeat_config or {}),
            vehicle_width=task_config.vehicle_width,
            vehicle_length=task_config.vehicle_length,
            default_path_spacing=task_config.default_path_spacing,
            global_direction=direction,
            turn_radius=task_config.turn_radius,
            overlap_ratio=task_config.overlap_ratio,
            inflation_radius=task_config.inflation_radius,
            current_pose=self._pose_from_source_to_aligned_map(task_config.current_pose, alignment_yaw),
            start_pose=self._pose_from_source_to_aligned_map(task_config.start_pose, alignment_yaw),
            end_pose=self._pose_from_source_to_aligned_map(task_config.end_pose, alignment_yaw),
        )

    def _planner_path_from_aligned_to_source(self, planner_path, alignment_yaw, source_map_info):
        if planner_path is None:
            return None
        source_frame = str((source_map_info or {}).get("frame_id", self._live_map_source_frame) or self._live_map_source_frame)
        resolution = max(1e-9, float((source_map_info or {}).get("resolution", 0.05)))
        origin_x = float((source_map_info or {}).get("origin_x", 0.0))
        origin_y = float((source_map_info or {}).get("origin_y", 0.0))
        converted_points = []
        for point_index, point in enumerate(list(planner_path.points or [])):
            if not isinstance(point, dict):
                continue
            source_point = deepcopy(point)
            source_x, source_y = self._transform_point_by_yaw(
                point.get("x", 0.0),
                point.get("y", 0.0),
                -float(alignment_yaw),
            )
            source_point["index"] = int(point_index)
            source_point["x"] = float(source_x)
            source_point["y"] = float(source_y)
            source_point["col"] = float((source_x - origin_x) / resolution)
            source_point["row"] = float((source_y - origin_y) / resolution)
            orientation = point.get("orientation")
            if isinstance(orientation, dict):
                source_point["orientation"] = self._yaw_to_quaternion(
                    self._yaw_from_quaternion(orientation) - float(alignment_yaw)
                )
            converted_points.append(source_point)
        nav_path = self._build_nav_path_from_points(converted_points, source_frame)
        return PlannerPath(
            task_id=planner_path.task_id,
            path_version=int(planner_path.path_version),
            points=converted_points,
            nav_path=nav_path,
            length_m=float(planner_path.length_m),
        )

    def _path_points_for_external_map_frame(self, points):
        # Planned paths are transformed back to the original map frame before
        # storage. Return that representation unchanged to SL-Link clients.
        return deepcopy(list(points or [])), self._live_map_source_frame, 0.0

    def _map_point_to_render_frame(self, point, render_map_info):
        try:
            point_x = float(point.get("x", 0.0))
            point_y = float(point.get("y", 0.0))
        except Exception:
            point_x = 0.0
            point_y = 0.0
        try:
            alignment_yaw = render_map_info.get("alignment_yaw", None)
            if alignment_yaw is not None:
                yaw = float(alignment_yaw)
                if math.isfinite(yaw) and abs(yaw) > 1e-6:
                    point_x, point_y = self._transform_point_by_yaw(point_x, point_y, yaw)
        except Exception:
            pass
        return point_x, point_y

    def _map_point_to_preview_pixel(self, point, render_map_info, scale_x, scale_y, preview_w, preview_h):
        point_x, point_y = self._map_point_to_render_frame(point, render_map_info)
        resolution = max(float(render_map_info["resolution"]), 1e-6)
        col = (point_x - float(render_map_info["origin_x"])) / resolution
        row = (point_y - float(render_map_info["origin_y"])) / resolution
        row = float(render_map_info["height"] - 1) - row
        pixel_x = int(round(col * float(scale_x)))
        pixel_y = int(round(row * float(scale_y)))
        pixel_x = max(0, min(int(preview_w) - 1, pixel_x))
        pixel_y = max(0, min(int(preview_h) - 1, pixel_y))
        return pixel_x, pixel_y

    def _map_points_to_preview_pixels(self, points, render_map_info, scale_x, scale_y, preview_w, preview_h):
        points_list = list(points or [])
        if not points_list:
            return []
        coords = np.zeros((len(points_list), 2), dtype=np.float64)
        for idx, point in enumerate(points_list):
            if not isinstance(point, dict):
                continue
            try:
                coords[idx, 0] = float(point.get("x", 0.0))
                coords[idx, 1] = float(point.get("y", 0.0))
            except Exception:
                coords[idx, 0] = 0.0
                coords[idx, 1] = 0.0
        try:
            alignment_yaw = render_map_info.get("alignment_yaw", None)
            if alignment_yaw is not None:
                yaw = float(alignment_yaw)
                if math.isfinite(yaw) and abs(yaw) > 1e-6:
                    cos_yaw = math.cos(yaw)
                    sin_yaw = math.sin(yaw)
                    x_vals = coords[:, 0].copy()
                    y_vals = coords[:, 1].copy()
                    coords[:, 0] = cos_yaw * x_vals - sin_yaw * y_vals
                    coords[:, 1] = sin_yaw * x_vals + cos_yaw * y_vals
        except Exception:
            pass
        resolution = max(float(render_map_info["resolution"]), 1e-6)
        cols = (coords[:, 0] - float(render_map_info["origin_x"])) / resolution
        rows = (coords[:, 1] - float(render_map_info["origin_y"])) / resolution
        rows = float(render_map_info["height"] - 1) - rows
        pixel_x = np.rint(cols * float(scale_x)).astype(np.int32)
        pixel_y = np.rint(rows * float(scale_y)).astype(np.int32)
        pixel_x = np.clip(pixel_x, 0, max(0, int(preview_w) - 1))
        pixel_y = np.clip(pixel_y, 0, max(0, int(preview_h) - 1))
        return [(int(pixel_x[idx]), int(pixel_y[idx])) for idx in range(len(points_list))]

    @staticmethod
    def _rotate_grid_to_aligned_frame(grid, origin_x, origin_y, resolution, yaw):
        height, width = grid.shape[:2]
        if height <= 0 or width <= 0:
            return grid, origin_x, origin_y
        if abs(float(yaw)) <= 1e-12:
            return grid, origin_x, origin_y

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        rot = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]], dtype=np.float64)

        x0 = float(origin_x)
        y0 = float(origin_y)
        x1 = x0 + float(width) * float(resolution)
        y1 = y0 + float(height) * float(resolution)
        corners = np.array([[x0, y0], [x1, y0], [x0, y1], [x1, y1]], dtype=np.float64)
        rotated = corners @ rot.T

        min_x = float(rotated[:, 0].min())
        max_x = float(rotated[:, 0].max())
        min_y = float(rotated[:, 1].min())
        max_y = float(rotated[:, 1].max())

        out_width = max(1, int(math.ceil((max_x - min_x) / max(float(resolution), 1e-12))))
        out_height = max(1, int(math.ceil((max_y - min_y) / max(float(resolution), 1e-12))))

        cols = np.arange(out_width, dtype=np.float64)
        rows = np.arange(out_height, dtype=np.float64)
        x_aligned = min_x + (cols + 0.5) * float(resolution)
        y_aligned = min_y + (rows + 0.5) * float(resolution)

        xx, yy = np.meshgrid(x_aligned, y_aligned)
        # Inverse rotation: map = R(-yaw) * aligned.
        x_map = cos_yaw * xx + sin_yaw * yy
        y_map = -sin_yaw * xx + cos_yaw * yy

        src_col = np.floor((x_map - x0) / max(float(resolution), 1e-12)).astype(np.int64)
        src_row = np.floor((y_map - y0) / max(float(resolution), 1e-12)).astype(np.int64)

        valid = (
            (src_col >= 0)
            & (src_col < int(width))
            & (src_row >= 0)
            & (src_row < int(height))
        )
        out_grid = np.full((out_height, out_width), -1, dtype=np.int16)
        if int(np.count_nonzero(valid)) > 0:
            out_grid[valid] = grid[src_row[valid], src_col[valid]]

        return out_grid, float(min_x), float(min_y)

    def _reload_navigation_map(self, yaml_path):
        try:
            if self._change_map_proxy is None:
                self._change_map_proxy = rospy.ServiceProxy(self._change_map_service, LoadMap)
            response = self._change_map_proxy(map_url=yaml_path)
            result_code = int(response.result)
            return result_code == 0, result_code
        except Exception as exc:
            rospy.logwarn("Failed to reload navigation map via %s: %s", self._change_map_service, exc)
            return False, None

    def _ensure_sync_proxies(self):
        if SyncGetStcm is None or SyncSetStcm is None:
            raise RuntimeError("slamware_ros_sdk python services are unavailable")
        if self._sync_get_proxy is None:
            self._sync_get_proxy = rospy.ServiceProxy(self._sync_get_stcm_service, SyncGetStcm)
        if self._sync_set_proxy is None:
            self._sync_set_proxy = rospy.ServiceProxy(self._sync_set_stcm_service, SyncSetStcm)

    def _map_state_dirname(self, map_id):
        text = str(map_id or "").strip() or self._live_map_id
        safe = re.sub(r"[^0-9A-Za-z._-]", "_", text)
        safe = safe.strip("._") or "LIVE_MAP"
        return safe[:96]

    def _map_state_dir(self, map_id=None):
        effective = self._current_map_id() if map_id is None else map_id
        return os.path.join(self._persist_state_dir, "map_states", self._map_state_dirname(effective))

    def _remove_map_overlay_state_for_map(self, map_id):
        if not self._persist_state_enabled:
            return False
        target_dir = self._map_state_dir(map_id)
        if not os.path.isdir(target_dir):
            return False
        try:
            shutil.rmtree(target_dir)
            rospy.loginfo("Removed map overlay state: map_id=%s dir=%s", map_id, target_dir)
            return True
        except Exception as exc:
            rospy.logwarn("Failed to remove map overlay state for map_id=%s dir=%s: %s", map_id, target_dir, exc)
            return False

    def _remove_map_overlay_states_for_aliases(self, aliases):
        removed = 0
        seen = set()
        for alias in list(aliases or []):
            text = str(alias or "").strip()
            if not text:
                continue
            dirname = self._map_state_dirname(text)
            if dirname in seen:
                continue
            seen.add(dirname)
            if self._remove_map_overlay_state_for_map(text):
                removed += 1
        return removed

    def _cleanup_orphan_map_overlay_states(self):
        if not self._persist_state_enabled:
            return 0
        root_dir = os.path.join(self._persist_state_dir, "map_states")
        if not os.path.isdir(root_dir):
            return 0
        valid_dirs = {self._map_state_dirname(self._live_map_id)}
        for record in (self._map_registry or {}).values():
            if not isinstance(record, dict):
                continue
            aliases = [
                record.get("map_id", ""),
                record.get("name", ""),
            ]
            path = str(record.get("path", "") or "").strip()
            if path:
                parsed_name, parsed_id = self._split_map_name_and_id_from_path(path)
                aliases.extend([parsed_name, parsed_id])
            for alias in aliases:
                text = str(alias or "").strip()
                if text:
                    valid_dirs.add(self._map_state_dirname(text))

        removed = 0
        for dirname in os.listdir(root_dir):
            full_path = os.path.join(root_dir, dirname)
            if not os.path.isdir(full_path):
                continue
            if dirname in valid_dirs:
                continue
            try:
                shutil.rmtree(full_path)
                removed += 1
                rospy.loginfo("Removed orphan map overlay state dir: %s", full_path)
            except Exception as exc:
                rospy.logwarn("Failed to remove orphan map overlay state dir=%s: %s", full_path, exc)
        return removed

    def _clear_live_map_files(self):
        target_dir = os.path.abspath(str(self._live_map_dir or "").strip())
        if not target_dir:
            raise RuntimeError("live_map_dir is empty")
        if os.path.isdir(target_dir):
            shutil.rmtree(target_dir)
        os.makedirs(target_dir, exist_ok=True)
        rospy.loginfo("Cleared LIVE_MAP files dir: %s", target_dir)

    def _overlay_region_counts(self):
        try:
            regions = self.map_service.get_overlay_regions() or {}
        except Exception:
            regions = {}
        work_count = len(list(regions.get("work_regions", []) or [])) if isinstance(regions, dict) else 0
        obstacle_count = len(list(regions.get("obstacle_regions", []) or [])) if isinstance(regions, dict) else 0
        erase_count = len(list(regions.get("erase_regions", []) or [])) if isinstance(regions, dict) else 0
        crop_count = 1 if isinstance(regions, dict) and isinstance(regions.get("crop_region"), dict) else 0
        return int(work_count), int(obstacle_count), int(erase_count), int(crop_count)

    def _stored_overlay_region_counts_for_map(self, map_id):
        target_dir = self._map_state_dir(map_id)
        meta_path = os.path.join(target_dir, "map_overlay_state.json")
        try:
            if not os.path.exists(meta_path):
                return (0, 0, 0, 0)
            with open(meta_path, "r", encoding="utf-8") as handle:
                data = json.load(handle)
            regions = data.get("regions", {}) if isinstance(data, dict) else {}
            work_count = len(list(regions.get("work_regions", []) or [])) if isinstance(regions, dict) else 0
            obstacle_count = len(list(regions.get("obstacle_regions", []) or [])) if isinstance(regions, dict) else 0
            erase_count = len(list(regions.get("erase_regions", []) or [])) if isinstance(regions, dict) else 0
            crop_count = 1 if isinstance(regions, dict) and isinstance(regions.get("crop_region"), dict) else 0
            return int(work_count), int(obstacle_count), int(erase_count), int(crop_count)
        except Exception as exc:
            rospy.logwarn("Failed to inspect stored overlay state for map_id=%s: %s", map_id, exc)
            return (0, 0, 0, 0)

    def _save_map_overlay_state_for_map(self, map_id, allow_empty_overwrite=False):
        if not self._persist_state_enabled:
            return
        try:
            target_dir = self._map_state_dir(map_id)
            current_counts = self._overlay_region_counts()
            stored_counts = self._stored_overlay_region_counts_for_map(map_id)
            is_live_map = str(map_id or "").strip() in ("", self._live_map_id, DEFAULT_LIVE_MAP_ID)
            if (
                (not bool(allow_empty_overwrite))
                and (not is_live_map)
                and sum(current_counts) <= 0
                and sum(stored_counts) > 0
            ):
                rospy.logwarn(
                    "Skip saving empty overlay over existing saved-map regions: map_id=%s current=%s stored=%s",
                    str(map_id),
                    str(current_counts),
                    str(stored_counts),
                )
                return
            os.makedirs(target_dir, exist_ok=True)
            self.map_service.save_local_state(target_dir)
            rospy.loginfo(
                "Saved map overlay state: map_id=%s dir=%s regions=%s allow_empty_overwrite=%s",
                str(map_id),
                target_dir,
                str(current_counts),
                str(bool(allow_empty_overwrite)).lower(),
            )
        except Exception as exc:
            rospy.logwarn("Failed to save overlay state for map_id=%s: %s", map_id, exc)

    def _copy_map_overlay_state(self, src_map_id, dst_map_id, overwrite=True):
        if not self._persist_state_enabled:
            return False
        src_dir = self._map_state_dir(src_map_id)
        dst_dir = self._map_state_dir(dst_map_id)
        if not os.path.isdir(src_dir):
            return False
        try:
            if overwrite and os.path.isdir(dst_dir):
                shutil.rmtree(dst_dir)
            os.makedirs(os.path.dirname(dst_dir), exist_ok=True)
            shutil.copytree(src_dir, dst_dir, dirs_exist_ok=overwrite)
            rospy.loginfo(
                "Copied map overlay state: %s -> %s",
                str(src_map_id),
                str(dst_map_id),
            )
            return True
        except Exception as exc:
            rospy.logwarn(
                "Failed to copy map overlay state: %s -> %s, err=%s",
                str(src_map_id),
                str(dst_map_id),
                str(exc),
            )
            return False

    def _load_map_overlay_state_for_map(self, map_id):
        if not self._persist_state_enabled:
            return False
        target_dir = self._map_state_dir(map_id)
        loaded = False
        try:
            loaded = bool(self.map_service.load_local_state(target_dir))
        except Exception:
            loaded = False
        if loaded:
            rospy.loginfo("Loaded map overlay state: map_id=%s dir=%s", map_id, target_dir)
            return True
        # one-time compatibility: fallback to legacy single-file overlay state
        legacy_meta = os.path.join(self._persist_state_dir, "map_overlay_state.json")
        if os.path.exists(legacy_meta):
            try:
                legacy_loaded = bool(self.map_service.load_local_state(self._persist_state_dir))
            except Exception:
                legacy_loaded = False
            if legacy_loaded:
                self._save_map_overlay_state_for_map(map_id)
                rospy.loginfo(
                    "Migrated legacy overlay state into map-specific state: map_id=%s dir=%s",
                    map_id,
                    target_dir,
                )
                return True
        self.map_service.reset_local_state()
        rospy.loginfo("No overlay state for map_id=%s, reset to empty overlay", map_id)
        return False

    def _save_local_state(self, allow_empty_saved_map_overlay=False):
        if not self._persist_state_enabled:
            return
        try:
            os.makedirs(self._persist_state_dir, exist_ok=True)
            self._save_map_overlay_state_for_map(
                self._current_map_id(),
                allow_empty_overwrite=bool(allow_empty_saved_map_overlay),
            )
            self._save_map_registry_state()
            self._save_task_registry_state()
            self._save_task_obstacle_regions_state()
            payload = {
                "chassis_settings": dict(self._chassis_settings),
                "active_map_id": self._current_map_id(),
                # LIVE_MAP is not present in map_registry.json. Persist its
                # APP rotation here so a scheduler restart does not silently
                # fall back to alignment_yaw for the next planning request.
                "live_map_app_rotation_deg": self._live_map_app_rotation_deg,
                "live_map_rotation_alignment_delta_deg": self._live_map_rotation_alignment_delta_deg,
                "state": self.state.value,
                "replan_requested": bool(self.replan_requested),
                "saved_at": int(time.time()),
                "persist_runtime_task_state": bool(self._persist_runtime_task_state),
            }
            if self._persist_runtime_task_state:
                payload["task_config"] = asdict(self.task_config)
                payload["task_bindings"] = dict(self._task_bindings)
                payload["current_path"] = None
            if self._persist_runtime_task_state and self.current_path is not None:
                payload["current_path"] = {
                    "task_id": self.current_path.task_id,
                    "path_version": int(self.current_path.path_version),
                    "length_m": float(self.current_path.length_m),
                    "points": self.current_path.points,
                    "frame_id": self.current_path.nav_path.header.frame_id if self.current_path.nav_path is not None else "map",
                }
            tmp_path = os.path.join(self._persist_state_dir, "scheduler_state.json.tmp")
            final_path = os.path.join(self._persist_state_dir, "scheduler_state.json")
            with open(tmp_path, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, ensure_ascii=False, indent=2)
            os.replace(tmp_path, final_path)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Failed to save scheduler local state: %s", exc)

    def _manual_speed_mps_to_rpm(self, speed_mps):
        speed = max(0.0, float(speed_mps))
        if speed <= 0.0:
            return 0
        wheel_rpm = speed * 60.0 / (2.0 * math.pi * self._manual_wheel_radius_m)
        motor_rpm = wheel_rpm * self._manual_gear_ratio * self._manual_speed_scale
        return max(1, min(self._manual_max_abs_wheel_rpm, int(round(motor_rpm))))

    def _update_navigation_speed_limit(self, run_speed, log=False):
        speed = max(0.0, min(self._max_chassis_run_speed, float(run_speed)))
        namespace = self._navigation_speed_reconfigure_namespace.rstrip("/")
        parameter = self._navigation_speed_reconfigure_parameter.strip("/")

        # 即使move_base暂时没有启动，也先保持ROS参数服务器中的值一致。
        rospy.set_param(
            namespace + "/" + parameter,
            speed,
        )

        if DynamicReconfigureClient is None:
            rospy.logwarn_throttle(
                5.0,
                "Navigation speed dynamic update unavailable: "
                "dynamic_reconfigure client is missing",
            )
            return False

        try:
            with self._navigation_speed_lock:
                if self._navigation_speed_client is None:
                    self._navigation_speed_client = DynamicReconfigureClient(
                        namespace,
                        timeout=self._navigation_speed_reconfigure_timeout,
                    )

                result = self._navigation_speed_client.update_configuration(
                    {parameter: speed}
                )

            if log:
                rospy.loginfo(
                    "Navigation speed limit updated: "
                    "namespace=%s parameter=%s value=%.3fm/s",
                    namespace,
                    parameter,
                    float(result.get(parameter, speed)),
                )

            return True

        except Exception as exc:
            # move_base重启后，下次设置速度时重新创建客户端。
            with self._navigation_speed_lock:
                self._navigation_speed_client = None

            rospy.logwarn(
                "Failed to update navigation speed dynamically: "
                "namespace=%s parameter=%s speed=%.3f error=%s",
                namespace,
                parameter,
                speed,
                exc,
            )
            return False

    def _load_local_state(self):
        if not self._persist_state_enabled:
            return
        try:
            os.makedirs(self._persist_state_dir, exist_ok=True)
            state_path = os.path.join(self._persist_state_dir, "scheduler_state.json")
            self._load_map_registry_state()
            self._load_task_obstacle_regions_state()
            if not os.path.exists(state_path):
                self.task_config.map_id = self._current_map_id()
                self._load_map_overlay_state_for_map(self._current_map_id())
                return
            with open(state_path, "r", encoding="utf-8") as handle:
                payload = json.load(handle)
            live_rotation = payload.get("live_map_app_rotation_deg", None)
            try:
                live_rotation = float(live_rotation)
                if math.isfinite(live_rotation):
                    self._live_map_app_rotation_deg = live_rotation
                    self._live_map_rotation_alignment_delta_deg = (
                        live_rotation
                        - self._alignment_yaw_deg_for_sl_link_report(self._live_map_id)
                    )
                else:
                    self._live_map_app_rotation_deg = None
                    self._live_map_rotation_alignment_delta_deg = None
            except (TypeError, ValueError):
                self._live_map_app_rotation_deg = None
                self._live_map_rotation_alignment_delta_deg = None
            if (not self._map_registry) and isinstance(payload.get("map_registry", {}), dict):
                # Backward compatibility: migrate old embedded map_registry.
                self._map_registry = payload.get("map_registry", {})
                self._save_map_registry_state()
            loaded_chassis = payload.get("chassis_settings", {})
            if isinstance(loaded_chassis, dict):
                self._chassis_settings["work_mode"] = int(loaded_chassis.get("work_mode", self._chassis_settings["work_mode"]))
                self._chassis_settings["disc_speed_rpm"] = int(loaded_chassis.get("disc_speed_rpm", self._chassis_settings["disc_speed_rpm"]))
                loaded_run_speed = float(
                    loaded_chassis.get("run_speed", self._chassis_settings["run_speed"])
                )
                self._chassis_settings["run_speed"] = max(
                    0.0,
                    min(self._max_chassis_run_speed, loaded_run_speed),
                )
                loaded_turn_ratio = float(
                    loaded_chassis.get(
                        "max_turn_speed_ratio",
                        self._chassis_settings["max_turn_speed_ratio"],
                    )
                )
                if loaded_turn_ratio > 0.0:
                    self._chassis_settings["max_turn_speed_ratio"] = max(
                        0.01,
                        min(1.0, loaded_turn_ratio),
                    )
            saved_active_map_id = str(payload.get("active_map_id", "")).strip()
            if saved_active_map_id:
                self._active_map_id = saved_active_map_id
            self._task_bindings = {}
            self.current_path = None
            self.current_path_index = 0
            if self._persist_runtime_task_state:
                self._task_bindings = payload.get("task_bindings", {}) if isinstance(payload.get("task_bindings", {}), dict) else {}
                task_cfg = payload.get("task_config", {})
                self.task_config = TaskConfigModel(
                    task_id=task_cfg.get("task_id", ""),
                    map_id=task_cfg.get("map_id", ""),
                    work_regions=task_cfg.get("work_regions", []),
                    obstacle_regions=task_cfg.get("obstacle_regions", []),
                    erase_regions=task_cfg.get("erase_regions", []),
                    crop_region=task_cfg.get("crop_region", {}),
                    active_work_region_id=task_cfg.get("active_work_region_id", ""),
                    selected_work_region_ids=task_cfg.get("selected_work_region_ids", []),
                    region_repeat_config=task_cfg.get("region_repeat_config", {}),
                    vehicle_width=self.task_config.vehicle_width,
                    vehicle_length=self.task_config.vehicle_length,
                    default_path_spacing=self.task_config.default_path_spacing,
                    global_direction=normalize_planning_direction(
                        task_cfg.get("global_direction", "x")
                    ),
                    turn_radius=self.task_config.turn_radius,
                    overlap_ratio=self.task_config.overlap_ratio,
                    inflation_radius=self.task_config.inflation_radius,
                )
                cfg_map_id = str(self.task_config.map_id or "").strip()
                if cfg_map_id:
                    self._active_map_id = cfg_map_id
            self._load_task_registry_state()
            self.task_config.map_id = self._current_map_id()
            self._load_map_overlay_state_for_map(self._current_map_id())
            if self._persist_runtime_task_state:
                self._sync_task_map_binding(update_binding=False)
            self.replan_requested = bool(payload.get("replan_requested", False))

            path_payload = payload.get("current_path")
            if self._persist_runtime_task_state and path_payload and path_payload.get("points"):
                nav_path = self._build_nav_path_from_points(path_payload.get("points", []), path_payload.get("frame_id", "map"))
                self.current_path = PlannerPath(
                    task_id=path_payload.get("task_id", self.task_config.task_id or "task"),
                    path_version=int(path_payload.get("path_version", 0)),
                    points=path_payload.get("points", []),
                    nav_path=nav_path,
                    length_m=float(path_payload.get("length_m", 0.0)),
                )
                self.current_path_index = 0
                self._rebuild_active_segments()
            rospy.loginfo("Loaded scheduler local state from %s", self._persist_state_dir)
        except Exception as exc:
            rospy.logwarn("Failed to load scheduler local state: %s", exc)

    def _save_map_registry_state(self):
        if not self._persist_state_enabled:
            return
        try:
            os.makedirs(self._persist_state_dir, exist_ok=True)
            payload = {
                "map_registry": dict(self._map_registry),
                "saved_at": int(time.time()),
            }
            tmp_path = self._map_registry_state_file + ".tmp"
            with open(tmp_path, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, ensure_ascii=False, indent=2)
            os.replace(tmp_path, self._map_registry_state_file)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Failed to save map registry state: %s", exc)

    def _save_task_registry_state(self):
        if not self._persist_state_enabled:
            return
        try:
            os.makedirs(self._persist_state_dir, exist_ok=True)
            now_ts = int(time.time())
            by_map = {}
            for _key, record in dict(self._task_bindings or {}).items():
                if not isinstance(record, dict):
                    continue
                map_id = str(record.get("map_id", "") or "").strip()
                task_id = str(record.get("task_id", "") or "").strip()
                if not map_id:
                    continue
                item = {
                    "map_id": map_id,
                    "task_id": task_id,
                    "selected_work_region_ids": list(record.get("selected_work_region_ids", []) or []),
                    "region_repeat_config": dict(record.get("region_repeat_config", {}) or {}),
                    "active_work_region_id": str(record.get("active_work_region_id", "") or ""),
                    "updated_at": int(record.get("updated_at", now_ts) or now_ts),
                    "task_result": dict(record.get("task_result", {}) or {}),
                }
                # one map keeps one latest task
                old = by_map.get(map_id)
                if (not isinstance(old, dict)) or int(item["updated_at"]) >= int(old.get("updated_at", 0) or 0):
                    by_map[map_id] = item
            payload = {
                "map_tasks": by_map,
                "task_executions": list(self._task_execution_records or []),
                "saved_at": now_ts,
            }
            tmp_path = self._task_registry_state_file + ".tmp"
            with open(tmp_path, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, ensure_ascii=False, indent=2)
            os.replace(tmp_path, self._task_registry_state_file)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Failed to save task registry state: %s", exc)

    def _load_task_registry_state(self):
        if not self._persist_state_enabled:
            return
        try:
            if not os.path.exists(self._task_registry_state_file):
                return
            with open(self._task_registry_state_file, "r", encoding="utf-8") as handle:
                payload = json.load(handle)
            map_tasks = payload.get("map_tasks", {}) if isinstance(payload, dict) else {}
            if not isinstance(map_tasks, dict):
                return
            raw_executions = payload.get("task_executions", []) if isinstance(payload, dict) else []
            self._task_execution_records = [
                dict(item)
                for item in list(raw_executions or [])
                if isinstance(item, dict) and str(item.get("execution_id", "") or "").strip()
            ]
            restored = 0
            for map_id, item in map_tasks.items():
                if not isinstance(item, dict):
                    continue
                map_id_text = str(map_id or item.get("map_id", "")).strip()
                task_id_text = str(item.get("task_id", "")).strip() or "task"
                if not map_id_text:
                    continue
                key = "{}::{}".format(map_id_text, task_id_text)
                self._task_bindings[key] = {
                    "task_id": task_id_text,
                    "map_id": map_id_text,
                    "selected_work_region_ids": list(item.get("selected_work_region_ids", []) or []),
                    "region_repeat_config": dict(item.get("region_repeat_config", {}) or {}),
                    "active_work_region_id": str(item.get("active_work_region_id", "") or ""),
                    "updated_at": int(item.get("updated_at", int(time.time())) or int(time.time())),
                    "task_result": dict(item.get("task_result", {}) or {}),
                }
                restored += 1
            if restored > 0:
                rospy.loginfo("Loaded task registry state: %d map-bound tasks", restored)
            if self._task_execution_records:
                rospy.loginfo(
                    "Loaded task execution history: records=%d",
                    len(self._task_execution_records),
                )
        except Exception as exc:
            rospy.logwarn("Failed to load task registry state: %s", exc)

    def _save_task_obstacle_regions_state(self):
        if not self._persist_state_enabled:
            return
        try:
            os.makedirs(self._persist_state_dir, exist_ok=True)
            with self._task_obstacle_regions_lock:
                bindings = deepcopy(self._task_obstacle_regions)
            payload = {
                "task_obstacle_regions": bindings,
                "saved_at": int(time.time()),
            }
            tmp_path = self._task_obstacle_regions_state_file + ".tmp"
            with open(tmp_path, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, ensure_ascii=False, indent=2)
            os.replace(tmp_path, self._task_obstacle_regions_state_file)
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0,
                "Failed to save task obstacle regions: %s",
                exc,
            )

    def _load_task_obstacle_regions_state(self):
        if not self._persist_state_enabled:
            return
        try:
            with self._task_obstacle_regions_lock:
                self._task_obstacle_regions = {}
            if not os.path.exists(self._task_obstacle_regions_state_file):
                return
            with open(self._task_obstacle_regions_state_file, "r", encoding="utf-8") as handle:
                payload = json.load(handle)
            bindings = payload.get("task_obstacle_regions", {}) if isinstance(payload, dict) else {}
            if not isinstance(bindings, dict):
                return
            restored = {}
            for key, regions in bindings.items():
                key_text = str(key or "").strip()
                if not key_text or not isinstance(regions, list):
                    continue
                normalized = []
                for region in regions:
                    if not isinstance(region, dict):
                        continue
                    region_id = str(region.get("region_id", "") or "").strip()
                    points = list(region.get("points", []) or [])
                    if not region_id or len(points) < 3:
                        continue
                    normalized.append(deepcopy(region))
                if normalized:
                    restored[key_text] = normalized
            with self._task_obstacle_regions_lock:
                self._task_obstacle_regions = restored
            if restored:
                rospy.loginfo(
                    "Loaded task obstacle regions: task_bindings=%d regions=%d",
                    len(restored),
                    sum(len(items) for items in restored.values()),
                )
        except Exception as exc:
            rospy.logwarn("Failed to load task obstacle regions: %s", exc)

    def _load_map_registry_state(self):
        if not self._persist_state_enabled:
            return
        try:
            self._map_registry = {}
            if not os.path.exists(self._map_registry_state_file):
                return
            with open(self._map_registry_state_file, "r", encoding="utf-8") as handle:
                payload = json.load(handle)
            if isinstance(payload, dict):
                registry = payload.get("map_registry", {})
                if isinstance(registry, dict):
                    self._map_registry = registry
            self._cleanup_orphan_map_overlay_states()
        except Exception as exc:
            rospy.logwarn("Failed to load map registry state: %s", exc)

    def _build_nav_path_from_points(self, points, frame_id):
        nav_path = Path()
        nav_path.header.frame_id = frame_id or "map"
        cached_xy = []
        for item in points:
            cached_xy.append((float(item.get("x", 0.0)), float(item.get("y", 0.0))))
        for idx, item in enumerate(points):
            pose = PoseStamped()
            pose.header.frame_id = nav_path.header.frame_id
            x, y = cached_xy[idx]
            pose.pose.position.x = x
            pose.pose.position.y = y
            quat = self._resolve_point_orientation(item, idx, cached_xy)
            pose.pose.orientation.x = float(quat["x"])
            pose.pose.orientation.y = float(quat["y"])
            pose.pose.orientation.z = float(quat["z"])
            pose.pose.orientation.w = float(quat["w"])
            nav_path.poses.append(pose)
        return nav_path

    def shutdown(self):
        self._save_local_state()
        self.mqtt_reporter.stop()
        self.platform_file_sync.stop()
        self.media_streamer.close()
        self.local_stream_server.stop()
        self.sl_link_server.stop()

    def _chassis_status_callback(self, msg):
        self.last_chassis_status = msg

    def _wheel_speed_state_callback(self, msg):
        with self._wheel_speed_state_lock:
            self.last_wheel_speed_state = msg

    def _wheel_odom_callback(self, msg):
        with self._wheel_odom_lock:
            self._wheel_odom_linear_speed_mps = float(msg.twist.twist.linear.x)
            self._wheel_odom_angular_speed_radps = float(msg.twist.twist.angular.z)
            self._wheel_odom_received_monotonic = time.monotonic()

    def _wheel_odom_snapshot(self):
        with self._wheel_odom_lock:
            linear_speed = self._wheel_odom_linear_speed_mps
            angular_speed = self._wheel_odom_angular_speed_radps
            received_at = self._wheel_odom_received_monotonic
        available = (
            received_at > 0.0
            and (time.monotonic() - received_at) <= self._wheel_odom_timeout_sec
        )
        return {
            "available": bool(available),
            "linear_mps": float(linear_speed) if available else 0.0,
            "angular_radps": float(angular_speed) if available else 0.0,
        }

    def _motor_rpm_to_wheel_mps(self, motor_rpm):
        wheel_rpm = float(motor_rpm) / self._wheel_speed_feedback_gear_ratio
        return wheel_rpm * (2.0 * math.pi * self._wheel_speed_feedback_radius_m) / 60.0

    def _wheel_speed_feedback_snapshot(self):
        with self._wheel_speed_state_lock:
            msg = self.last_wheel_speed_state
        if msg is None or not bool(getattr(msg, "feedback_valid", False)):
            return {
                "available": False,
                "left_mps": 0.0,
                "right_mps": 0.0,
                "vehicle_mps": 0.0,
            }
        left_mps = self._motor_rpm_to_wheel_mps(getattr(msg, "feedback_left_wheel_speed", 0))
        right_mps = self._motor_rpm_to_wheel_mps(getattr(msg, "feedback_right_wheel_speed", 0))
        return {
            "available": True,
            "left_mps": float(left_mps),
            "right_mps": float(right_mps),
            "vehicle_mps": (float(left_mps) + float(right_mps)) * 0.5,
        }

    def _collision_imminent_callback(self, msg):
        active = bool(msg.data)
        with self._collision_imminent_lock:
            changed = active != self._collision_imminent_active
            self._collision_imminent_active = active
            if not active:
                self._collision_pause_latched = False
            should_pause = (
                active
                and not self._collision_pause_latched
                and self.state == SchedulerState.RUNNING
                and self._exec_active
            )
            if should_pause:
                self._collision_pause_latched = True

        if changed:
            rospy.logwarn(
                "Navigation collision imminent changed: topic=%s active=%s",
                self._collision_imminent_topic,
                str(active),
            )
        if not should_pause:
            return

        success, message = self._pause_execution()
        if success:
            rospy.logerr(
                "Task automatically paused by navigation collision imminent: "
                "task_id=%s chassis=stopped disc=off",
                self.task_config.task_id or "<empty>",
            )
        else:
            rospy.logerr(
                "Failed to pause task after navigation collision imminent: "
                "task_id=%s message=%s",
                self.task_config.task_id or "<empty>",
                message,
            )

    def _collision_imminent_snapshot(self):
        with self._collision_imminent_lock:
            return bool(self._collision_imminent_active)

    def _radar_system_status_callback(self, msg):
        status = str(getattr(msg, "status", "") or "")
        timestamp_ns = int(getattr(msg, "timestamp_ns", 0) or 0)
        with self._radar_system_status_lock:
            changed = (
                not self._radar_system_status_available
                or status != self._radar_system_status
            )
            self._radar_system_status_available = True
            self._radar_system_status = status
            self._radar_system_status_timestamp_ns = timestamp_ns
        if changed:
            rospy.loginfo(
                "Radar system status updated: status=%s timestamp_ns=%d",
                status or "<empty>",
                timestamp_ns,
            )

    def _radar_system_status_snapshot(self):
        with self._radar_system_status_lock:
            return {
                "available": bool(self._radar_system_status_available),
                "status": str(self._radar_system_status or ""),
                "timestamp_ns": int(self._radar_system_status_timestamp_ns or 0),
            }

    def _radar_relocalization_status_callback(self, msg):
        raw_status = str(getattr(msg, "status", "") or "").strip()
        timestamp_ns = int(getattr(msg, "timestamp_ns", 0) or 0)
        normalized = raw_status.lower()
        aggregate_status = ""
        if normalized == "relocalizationrunning":
            aggregate_status = "running"
        elif normalized == "relocalizationsucceed":
            aggregate_status = "succeeded"
        elif normalized == "relocalizationfailed":
            aggregate_status = "failed"
        elif normalized == "relocalizationcanceled":
            aggregate_status = "canceled"

        with self._radar_relocalization_status_lock:
            previous = self._radar_relocalization_aggregate_status
            # RelocalizationNone follows one-shot terminal events; preserve the
            # last meaningful raw and aggregate result until another request starts.
            if aggregate_status:
                self._radar_relocalization_raw_available = True
                self._radar_relocalization_raw_status = raw_status
                self._radar_relocalization_raw_timestamp_ns = timestamp_ns
                self._radar_relocalization_aggregate_status = aggregate_status
                self._radar_relocalization_aggregate_timestamp_ns = (
                    timestamp_ns or int(time.time() * 1.0e9)
                )
            elif not self._radar_relocalization_raw_available:
                self._radar_relocalization_raw_available = True
                self._radar_relocalization_raw_status = raw_status
                self._radar_relocalization_raw_timestamp_ns = timestamp_ns
            current = self._radar_relocalization_aggregate_status
        if aggregate_status and current != previous:
            rospy.loginfo(
                "Radar relocalization status updated: raw=%s aggregate=%s timestamp_ns=%d",
                raw_status,
                current,
                timestamp_ns,
            )

    def _radar_relocalization_snapshot(self):
        with self._radar_relocalization_status_lock:
            raw_available = bool(self._radar_relocalization_raw_available)
            raw_status = str(self._radar_relocalization_raw_status or "")
            raw_timestamp_ns = int(self._radar_relocalization_raw_timestamp_ns or 0)
            aggregate_status = str(self._radar_relocalization_aggregate_status or "idle")
            aggregate_timestamp_ns = int(self._radar_relocalization_aggregate_timestamp_ns or 0)

        return {
            "available": raw_available,
            "status": aggregate_status,
            "raw_status": raw_status,
            "timestamp_ns": max(aggregate_timestamp_ns, raw_timestamp_ns),
        }

    def _cmd_vel_callback(self, msg):
        self._last_cmd_vel_linear = math.hypot(float(msg.linear.x), float(msg.linear.y))
        self._last_cmd_vel_angular = abs(float(msg.angular.z))
        self._last_cmd_vel_time = time.time()

    def _reset_disc_motion_guard(self):
        self._disc_stationary_since = 0.0
        self._disc_motion_guard_stopped = False

    def _tick_disc_motion_guard(self):
        if not bool(self._disc_motion_guard_enabled):
            return
        if self.state != SchedulerState.RUNNING or not self._exec_active or not self._disc_auto_cover_desired:
            self._disc_stationary_since = 0.0
            self._disc_motion_guard_stopped = False
            return

        now = time.time()
        stale = self._last_cmd_vel_time <= 0.0 or (now - self._last_cmd_vel_time) > self._disc_cmd_vel_stale_sec
        linear = 0.0 if stale else float(self._last_cmd_vel_linear)
        angular = 0.0 if stale else float(self._last_cmd_vel_angular)
        moving_for_stop = linear > self._disc_stop_linear_threshold or angular > self._disc_stop_angular_threshold
        moving_for_resume = linear > self._disc_resume_linear_threshold or angular > self._disc_resume_angular_threshold
        can_switch = (now - self._disc_last_switch_time) >= self._disc_switch_min_interval

        if moving_for_resume:
            self._disc_stationary_since = 0.0
            if self._disc_motion_guard_stopped and can_switch:
                self.disc_enable_pub.publish(Bool(data=True))
                self._disc_motion_guard_stopped = False
                self._disc_last_switch_time = now
                rospy.loginfo("Disc resumed by cmd_vel motion: linear=%.3f angular=%.3f", linear, angular)
            return

        if moving_for_stop:
            self._disc_stationary_since = 0.0
            return

        if self._disc_stationary_since <= 0.0:
            self._disc_stationary_since = now
            return

        if (
            not self._disc_motion_guard_stopped
            and (now - self._disc_stationary_since) >= self._disc_stop_hold_sec
            and can_switch
        ):
            self.disc_enable_pub.publish(Bool(data=False))
            self._disc_motion_guard_stopped = True
            self._disc_last_switch_time = now
            rospy.loginfo("Disc stopped by cmd_vel idle: stale=%s linear=%.3f angular=%.3f", stale, linear, angular)

    def _sync_task_regions_from_overlay(self, update_task_binding=True):
        overlay_regions = self.map_service.get_overlay_regions() or {}
        crop_region = overlay_regions.get("crop_region")
        if isinstance(crop_region, dict) and bool(crop_region.get("enabled", True)):
            points = crop_region.get("points", []) or []
            if len(points) >= 3:
                self.task_config.crop_region = {
                    "region_id": crop_region.get("region_id", "crop_region_1"),
                    "name": crop_region.get("name", "crop_region"),
                    "points": points,
                    "region_type": 4,
                }
            else:
                self.task_config.crop_region = {}
        else:
            self.task_config.crop_region = {}
        self.task_config.work_regions = []
        for item in overlay_regions.get("work_regions", []):
            points = item.get("points", []) or []
            if len(points) < 3:
                continue
            if not bool(item.get("enabled", True)):
                continue
            region_type = int(item.get("region_type", 1))
            if region_type != 1:
                continue
            self.task_config.work_regions.append(
                {
                    "region_id": item.get("region_id", ""),
                    "name": item.get("name", ""),
                    "points": points,
                    "global_direction": normalize_planning_direction(
                        item.get("global_direction", "x")
                    ),
                    "start_pose": dict(item.get("start_pose", {}) or {}),
                    "end_pose": dict(item.get("end_pose", {}) or {}),
                    "order_index": int(item.get("order_index", 0)),
                    "region_type": region_type,
                }
            )
        self.task_config.obstacle_regions = []
        self.task_config.erase_regions = []
        for item in overlay_regions.get("obstacle_regions", []):
            points = item.get("points", []) or []
            if len(points) < 3:
                continue
            if not bool(item.get("enabled", True)):
                continue
            normalized = {
                "region_id": item.get("region_id", ""),
                "name": item.get("name", ""),
                "points": points,
                "order_index": int(item.get("order_index", 0)),
                "region_type": 2,
            }
            self.task_config.obstacle_regions.append(normalized)
        for item in overlay_regions.get("erase_regions", []):
            points = item.get("points", []) or []
            if len(points) < 3:
                continue
            if not bool(item.get("enabled", True)):
                continue
            self.task_config.erase_regions.append(
                {
                    "region_id": item.get("region_id", ""),
                    "name": item.get("name", ""),
                    "points": points,
                    "order_index": int(item.get("order_index", 0)),
                    "region_type": 3,
                }
            )
        valid_region_ids = set()
        for region in self.task_config.work_regions:
            rid = str(region.get("region_id", "")).strip()
            if rid:
                valid_region_ids.add(rid)
        active_id = (self.task_config.active_work_region_id or "").strip()
        if active_id:
            if active_id not in valid_region_ids:
                self.task_config.active_work_region_id = ""
        selected = []
        for rid in list(self.task_config.selected_work_region_ids or []):
            rid_text = str(rid).strip()
            if rid_text and rid_text in valid_region_ids and rid_text not in selected:
                selected.append(rid_text)
        self.task_config.selected_work_region_ids = selected
        repeat_cfg = {}
        for rid, count in dict(self.task_config.region_repeat_config or {}).items():
            rid_text = str(rid).strip()
            if rid_text not in valid_region_ids:
                continue
            try:
                repeat_cfg[rid_text] = max(1, int(count))
            except Exception:
                repeat_cfg[rid_text] = 1
        self.task_config.region_repeat_config = repeat_cfg
        # Enforce single-region planning policy permanently.
        if self._plan_use_all_work_regions:
            self._plan_use_all_work_regions = False
            rospy.logwarn("Forced single-region planning mode: plan_use_all_work_regions=false")
        if update_task_binding:
            self._sync_task_map_binding(update_binding=True)

    @staticmethod
    def _task_obstacle_binding_key(map_id, task_id):
        return "{}::{}".format(
            str(map_id or "").strip(),
            str(task_id or "").strip(),
        )

    def _task_obstacle_regions_for(self, map_id, task_id):
        key = self._task_obstacle_binding_key(map_id, task_id)
        if not str(map_id or "").strip() or not str(task_id or "").strip():
            return []
        with self._task_obstacle_regions_lock:
            return deepcopy(self._task_obstacle_regions.get(key, []) or [])

    def _merge_task_obstacle_regions_for_planning(self, map_id=None, task_id=None):
        effective_map_id = str(map_id or self._current_map_id()).strip()
        effective_task_id = str(task_id or self.task_config.task_id or "").strip()
        task_regions = self._task_obstacle_regions_for(effective_map_id, effective_task_id)
        if not task_regions:
            return 0
        self.task_config.obstacle_regions.extend(task_regions)
        rospy.loginfo(
            "Task obstacle regions merged for planning: map_id=%s task_id=%s map_regions=%d task_regions=%d total=%d",
            effective_map_id,
            effective_task_id,
            len(self.task_config.obstacle_regions) - len(task_regions),
            len(task_regions),
            len(self.task_config.obstacle_regions),
        )
        return len(task_regions)

    def _current_map_id(self):
        return str(self._active_map_id or self._live_map_id).strip() or self._live_map_id

    def _set_active_map_id(self, map_id, reason="", migrate_bindings=False, save_prev_overlay=True):
        next_map_id = str(map_id or "").strip() or self._live_map_id
        prev_map_id = str(self._active_map_id or "").strip() or self._live_map_id
        if next_map_id == prev_map_id:
            self.task_config.map_id = next_map_id
            return

        if save_prev_overlay:
            self._save_map_overlay_state_for_map(prev_map_id)

        if migrate_bindings:
            moved = 0
            prefix = "{}::".format(prev_map_id)
            for key in list(self._task_bindings.keys()):
                key_text = str(key)
                if not key_text.startswith(prefix):
                    continue
                task_id = key_text[len(prefix) :]
                new_key = "{}::{}".format(next_map_id, task_id)
                record = self._task_bindings.pop(key, {})
                if isinstance(record, dict):
                    record["map_id"] = next_map_id
                    self._task_bindings[new_key] = record
                    moved += 1
            if moved > 0:
                rospy.loginfo(
                    "Migrated task-map bindings: count=%d %s -> %s",
                    moved,
                    prev_map_id,
                    next_map_id,
                )

        self._active_map_id = next_map_id
        self.task_config.map_id = next_map_id
        self._load_map_overlay_state_for_map(next_map_id)
        # Force one-shot map switch sync in next tick.
        self._last_seen_map_id = ""
        rospy.loginfo(
            "Active map switched: %s -> %s%s",
            prev_map_id,
            next_map_id,
            (" reason={}".format(reason) if reason else ""),
        )

    def _sync_task_map_binding(self, update_binding=True):
        map_id = self._current_map_id()
        self.task_config.map_id = map_id
        task_id = (self.task_config.task_id or "task").strip() or "task"
        key = "{}::{}".format(map_id, task_id)
        if update_binding:
            old_record = self._task_bindings.get(key, {}) if isinstance(self._task_bindings.get(key, {}), dict) else {}
            # Keep only one task record per map: new task replaces old task on same map.
            for old_key in list(self._task_bindings.keys()):
                if not isinstance(old_key, str):
                    continue
                if old_key.startswith("{}::".format(map_id)) and old_key != key:
                    self._task_bindings.pop(old_key, None)
            self._task_bindings[key] = {
                "task_id": task_id,
                "map_id": map_id,
                "selected_work_region_ids": list(self.task_config.selected_work_region_ids or []),
                "region_repeat_config": dict(self.task_config.region_repeat_config or {}),
                "active_work_region_id": self.task_config.active_work_region_id or "",
                "updated_at": int(time.time()),
                "task_result": dict(old_record.get("task_result", {}) or {}),
            }
            return
        binding = self._task_bindings.get(key)
        if not isinstance(binding, dict):
            return
        selected = []
        for rid in list(binding.get("selected_work_region_ids", []) or []):
            rid_text = str(rid).strip()
            if rid_text and rid_text not in selected:
                selected.append(rid_text)
        repeat_cfg = {}
        for rid, count in dict(binding.get("region_repeat_config", {}) or {}).items():
            rid_text = str(rid).strip()
            if not rid_text:
                continue
            try:
                repeat_cfg[rid_text] = max(1, int(count))
            except Exception:
                repeat_cfg[rid_text] = 1
        self.task_config.selected_work_region_ids = selected
        self.task_config.region_repeat_config = repeat_cfg
        active = str(binding.get("active_work_region_id", "")).strip()
        if active:
            self.task_config.active_work_region_id = active

    def _map_id_for_task_id(self, task_id):
        target_task_id = str(task_id or "").strip()
        if not target_task_id:
            return ""
        best = None
        best_updated_at = -1
        for _key, record in dict(self._task_bindings or {}).items():
            if not isinstance(record, dict):
                continue
            record_task_id = str(record.get("task_id", "") or "").strip()
            if record_task_id != target_task_id:
                continue
            map_id = str(record.get("map_id", "") or "").strip()
            if not map_id:
                continue
            try:
                updated_at = int(record.get("updated_at", 0) or 0)
            except Exception:
                updated_at = 0
            if best is None or updated_at >= best_updated_at:
                best = map_id
                best_updated_at = updated_at
        if best:
            return best
        if target_task_id == str(self.task_config.task_id or "").strip():
            return str(self.task_config.map_id or self._current_map_id()).strip()
        return ""

    def _effective_request_map_id_for_task(self, request):
        requested_map_id = str(getattr(request, "map_id", "") or "").strip()
        if requested_map_id:
            return requested_map_id
        task_id = str(getattr(request, "task_id", "") or "").strip()
        inferred_map_id = self._map_id_for_task_id(task_id)
        if inferred_map_id:
            rospy.loginfo(
                "PathPlanRequest inferred map_id from task binding: task_id=%s map_id=%s",
                task_id or "<empty>",
                inferred_map_id,
            )
            return inferred_map_id
        return ""

    def _validate_requested_map_id(self, requested_map_id, op_name, keep_current_on_empty=False):
        req = str(requested_map_id or "").strip()
        current = self._current_map_id()
        if not req:
            if keep_current_on_empty:
                rospy.loginfo(
                    "%s uses current active map because request map_id is empty: current_map_id=%s",
                    op_name,
                    current,
                )
                return True, current
            # Empty map_id defaults to realtime live map for legacy behavior.
            if current != self._live_map_id:
                self._set_active_map_id(
                    self._live_map_id,
                    reason="{}_default_live_map".format(op_name),
                    migrate_bindings=False,
                    save_prev_overlay=False,
                )
            return True, self._live_map_id
        if req == self._live_map_id:
            # Empty map_id or explicit LIVE_MAP always means realtime live map.
            if current != self._live_map_id:
                self._set_active_map_id(
                    self._live_map_id,
                    reason="{}_default_live_map".format(op_name),
                    migrate_bindings=False,
                    save_prev_overlay=False,
                )
            return True, self._live_map_id
        if req != current:
            # If caller provides a recorded map_id, switch active map on-demand
            # instead of hard rejecting due to map mismatch.
            record = self._find_recorded_map_by_id(req)
            if record is None:
                rospy.logwarn(
                    "%s rejected: requested_map_id=%s current_map_id=%s (map_id not found)",
                    op_name,
                    req,
                    current,
                )
                return False, current
            self._set_active_map_id(
                req,
                reason="{}_requested_map_switch".format(op_name),
                migrate_bindings=False,
            )
            rospy.loginfo(
                "%s auto-switched active map: %s -> %s",
                op_name,
                current,
                req,
            )
            return True, req
        return True, current

    def _effective_selected_work_region_ids(self, ordered_regions):
        selected = []
        configured = list(self.task_config.selected_work_region_ids or [])
        valid_ids = []
        for region in ordered_regions:
            rid = str(region.get("region_id", "")).strip()
            if rid:
                valid_ids.append(rid)
        valid_set = set(valid_ids)
        for rid in configured:
            rid_text = str(rid).strip()
            if rid_text and rid_text in valid_set and rid_text not in selected:
                selected.append(rid_text)
        if selected:
            return selected
        active_id = (self.task_config.active_work_region_id or "").strip()
        if active_id and active_id in valid_set:
            return [active_id]
        return valid_ids

    def _resolve_plan_work_regions(self, force_use_all_regions=False):
        regions = list(self.task_config.work_regions or [])
        if not regions:
            return []
        # Base order: order_index (stable sort keeps insertion order for ties).
        regions.sort(key=lambda region: int(region.get("order_index", 0)))
        region_by_id = {}
        for region in regions:
            rid = str(region.get("region_id", "")).strip()
            if rid and rid not in region_by_id:
                region_by_id[rid] = region
        if force_use_all_regions:
            selected_ids = [str(region.get("region_id", "")).strip() for region in regions if str(region.get("region_id", "")).strip()]
        else:
            selected_ids = self._effective_selected_work_region_ids(regions)
        selected_set = set(selected_ids)
        filtered = [region for region in regions if str(region.get("region_id", "")).strip() in selected_set]
        # If task explicitly provides selected_work_region_ids, respect that exact order.
        if (not force_use_all_regions) and selected_ids:
            ordered = []
            seen = set()
            for rid in selected_ids:
                rid_text = str(rid).strip()
                if (not rid_text) or (rid_text in seen):
                    continue
                region = region_by_id.get(rid_text)
                if region is not None:
                    ordered.append(region)
                    seen.add(rid_text)
            if ordered:
                filtered = ordered
        if not filtered:
            return []
        # Keep original region_id here.
        # Repeat expansion is handled centrally in planner_adapter by region_repeat_config.
        # If we rewrite IDs to "__lap_n" here, selected_work_region_ids matching may fail.
        return filtered

    def _sorted_work_region_ids(self):
        regions = list(self.task_config.work_regions or [])
        regions.sort(key=lambda region: int(region.get("order_index", 0)))
        return self._effective_selected_work_region_ids(regions)

    def _advance_active_work_region_for_request(self):
        """Round-robin select active work region for each explicit PathPlanRequest."""
        regions = list(self.task_config.work_regions or [])
        if not regions:
            self.task_config.active_work_region_id = ""
            return
        regions.sort(key=lambda region: int(region.get("order_index", 0)))
        region_ids = [str(item.get("region_id", "")).strip() for item in regions]
        region_ids = [item for item in region_ids if item]
        if not region_ids:
            self.task_config.active_work_region_id = ""
            return
        current = (self.task_config.active_work_region_id or "").strip()
        if current not in region_ids:
            self.task_config.active_work_region_id = region_ids[0]
            return
        next_idx = (region_ids.index(current) + 1) % len(region_ids)
        self.task_config.active_work_region_id = region_ids[next_idx]

    def _effective_crop_region(self, allow_overlay_fallback=True):
        crop = self.task_config.crop_region if isinstance(self.task_config.crop_region, dict) else {}
        points = crop.get("points", []) if isinstance(crop, dict) else []
        if len(points) >= 3 and bool(crop.get("enabled", True)):
            return crop
        if not allow_overlay_fallback:
            return {}
        try:
            overlay = self.map_service.get_overlay_regions() or {}
            overlay_crop = overlay.get("crop_region")
            if isinstance(overlay_crop, dict):
                overlay_points = overlay_crop.get("points", []) or []
                if len(overlay_points) >= 3 and bool(overlay_crop.get("enabled", True)):
                    return overlay_crop
        except Exception:
            pass
        return {}

    def _crop_preview_image_by_region(self, image, map_info):
        if image is None or map_info is None:
            return image
        # Path planning requests call _sync_task_regions_from_overlay() before rendering,
        # so task_config.crop_region is the cheap authoritative source here. Avoid
        # get_overlay_regions() on the hot render path because it deep-copies all regions.
        crop_region = self._effective_crop_region(allow_overlay_fallback=False)
        crop_points = crop_region.get("points", []) if isinstance(crop_region, dict) else []
        if len(crop_points) < 3:
            return image
        preview_h, preview_w = image.shape[:2]
        crop_cache_key = (
            self._region_cache_fingerprint([crop_region]),
            int(map_info.get("width", 0) or 0),
            int(map_info.get("height", 0) or 0),
            self._round_float(map_info.get("resolution", 0.0), 5),
            self._round_float(map_info.get("origin_x", 0.0), 4),
            self._round_float(map_info.get("origin_y", 0.0), 4),
            self._round_float(map_info.get("alignment_yaw", 0.0) or 0.0, 6),
            int(preview_w),
            int(preview_h),
        )
        if crop_cache_key == self._path_preview_crop_cache_key and self._path_preview_crop_cache_bbox is not None:
            x0, y0, x1, y1 = self._path_preview_crop_cache_bbox
        else:
            scale_x = float(preview_w) / float(max(1, map_info["width"]))
            scale_y = float(preview_h) / float(max(1, map_info["height"]))
            polygon = self._map_points_to_preview_pixels(
                crop_points,
                map_info,
                scale_x,
                scale_y,
                preview_w,
                preview_h,
            )
            if len(polygon) < 3:
                return image
            arr = np.array(polygon, dtype=np.int32)
            min_x = int(max(0, np.min(arr[:, 0])))
            max_x = int(min(preview_w - 1, np.max(arr[:, 0])))
            min_y = int(max(0, np.min(arr[:, 1])))
            max_y = int(min(preview_h - 1, np.max(arr[:, 1])))
            if not (max_x > min_x and max_y > min_y):
                return image
            pad = 4
            x0 = max(0, min_x - pad)
            y0 = max(0, min_y - pad)
            x1 = min(preview_w, max_x + 1 + pad)
            y1 = min(preview_h, max_y + 1 + pad)
            self._path_preview_crop_cache_key = crop_cache_key
            self._path_preview_crop_cache_bbox = (x0, y0, x1, y1)
        cropped = image[y0:y1, x0:x1]
        rospy.loginfo_throttle(
            2.0,
            "Path preview crop applied: src=%dx%d dst=%dx%d crop_bbox=(%d,%d)-(%d,%d)",
            preview_w,
            preview_h,
            int(cropped.shape[1]),
            int(cropped.shape[0]),
            x0,
            y0,
            x1,
            y1,
        )
        return cropped

    def _handle_plan_now(self, _req):
        success = self._plan_current_task()
        if success and self.current_path is not None:
            return TriggerResponse(
                success=True,
                message="planning_ok path_version={} points={}".format(
                    self.current_path.path_version,
                    len(self.current_path.points),
                ),
            )
        return TriggerResponse(success=False, message=self.last_error or "planning_failed")

    def _plan_current_task(self, force_use_all_regions=False, request_start_pose=None, request_end_pose=None, request_global_direction=None):
        t_plan_total = time.perf_counter()
        self._ensure_offline_map_service_for_current_map("plan_current_task_entry")
        # Always plan against the latest map edit overlay (work/obstacle regions).
        self._sync_task_regions_from_overlay()
        self._merge_task_obstacle_regions_for_planning(
            self._current_map_id(),
            self.task_config.task_id,
        )
        self._sync_task_map_binding(update_binding=True)
        t_plan_sync = time.perf_counter()
        if self._reload_navigation_map_on_plan:
            try:
                current_map_id = self._current_map_id()
                raw_map_for_nav = None
                if self._is_live_map_id(current_map_id):
                    raw_map_for_nav = self.aurora_bridge.get_map()
                    if raw_map_for_nav is not None:
                        # Keep live map_service raw cache in sync before export.
                        self.map_service.set_raw_map(raw_map_for_nav)
                    rospy.loginfo(
                        "Navigation map refresh source before planning: source=live map_id=%s",
                        current_map_id,
                    )
                else:
                    rospy.loginfo(
                        "Navigation map refresh source before planning: source=offline_raw_grid map_id=%s",
                        current_map_id,
                    )
                map_info_for_nav = self.map_service.get_map_info()
                composed_for_nav = self.map_service.compose_map()
                if map_info_for_nav is None or composed_for_nav is None:
                    raise RuntimeError("composed map unavailable")
                nav_dir = os.path.dirname(self._nav_map_yaml_path)
                nav_yaml_name = os.path.basename(self._nav_map_yaml_path)
                nav_image_name = "map1.pgm"
                nav_yaml_path, _ = self._export_runtime_map(
                    raw_map_for_nav,
                    nav_dir,
                    nav_yaml_name,
                    nav_image_name,
                    grid_override=composed_for_nav,
                    map_info_override=map_info_for_nav,
                )
                reloaded, result_code = self._reload_navigation_map(nav_yaml_path)
                if reloaded:
                    rospy.loginfo("Navigation map refreshed before planning from composed_map: %s", nav_yaml_path)
                else:
                    rospy.logwarn(
                        "Navigation map refresh before planning failed (code=%s), continue planning with current map",
                        str(result_code),
                    )
            except Exception as exc:
                rospy.logwarn("Navigation map refresh before planning failed: %s", exc)
        t_plan_nav_refresh = time.perf_counter()
        selected_work_regions = self._resolve_plan_work_regions(force_use_all_regions=force_use_all_regions)
        has_task_info = bool(list(self.task_config.selected_work_region_ids or [])) or bool(
            dict(self.task_config.region_repeat_config or {})
        )
        rospy.loginfo(
            "Planning input regions: work_total=%d work_selected=%d obstacle=%d active_work_region_id=%s force_use_all=%s has_task_info=%s",
            len(self.task_config.work_regions),
            len(selected_work_regions),
            len(self.task_config.obstacle_regions),
            self.task_config.active_work_region_id or "<auto>",
            str(bool(force_use_all_regions)).lower(),
            str(bool(has_task_info)).lower(),
        )
        if not selected_work_regions:
            self._set_error("Planning failed: no valid work region")
            rospy.logwarn("Planning aborted: no valid work region in overlay")
            return False
        requested_direction = str(request_global_direction or "").strip().lower()
        has_requested_direction = is_planning_direction(requested_direction)
        effective_global_direction = requested_direction if has_requested_direction else str(
            self.task_config.global_direction or "x"
        ).strip().lower()
        effective_global_direction = normalize_planning_direction(effective_global_direction)
        # Keep each work region's own planning direction. PathPlanRequest.global_direction is
        # only a fallback for legacy/empty region direction, not a global override.
        direction_summary = []
        for region in selected_work_regions:
            if not isinstance(region, dict):
                continue
            region_id = str(region.get("region_id", "") or "").strip() or "<empty>"
            region_direction = normalize_planning_direction(
                region.get("global_direction", ""),
                effective_global_direction,
            )
            direction_summary.append("{}:{}".format(region_id, region_direction))
        rospy.loginfo(
            "Planning region directions: fallback=%s regions=%s",
            effective_global_direction,
            ",".join(direction_summary) if direction_summary else "<none>",
        )
        map_info = self.map_service.get_map_info()
        composed_map = self.map_service.compose_map()
        t_plan_map_ready = time.perf_counter()
        if map_info is None or composed_map is None:
            rospy.logwarn("Planning skipped: map_info or composed_map is empty")
            self._set_error("Cannot plan without a map")
            return False
        service_source = str(getattr(self.map_service, "_grinder_map_source", "") or "")
        service_map_id = str(getattr(self.map_service, "_grinder_map_id", "") or "")
        rospy.loginfo(
            "Planning started: task_id=%s map_id=%s map_source=%s service_source=%s service_map_id=%s map_version=%s map_size=%sx%s resolution=%.4f",
            self.task_config.task_id or "task",
            self._current_map_id(),
            "live" if self._is_live_map_id(self._current_map_id()) else "offline_raw_grid",
            service_source or "<unknown>",
            service_map_id or "<empty>",
            map_info.get("map_version", 0),
            map_info.get("width", 0),
            map_info.get("height", 0),
            float(map_info.get("resolution", 0.0)),
        )
        self.state = SchedulerState.PLANNING
        try:
            current_pose = self.aurora_bridge.get_pose()
            t_plan_pose = time.perf_counter()
            rospy.loginfo(
                "Plan request poses: current_pose=(%.3f, %.3f, %.1fdeg) request_start=(%s) request_end=(%s)",
                float(current_pose.get("x", 0.0)),
                float(current_pose.get("y", 0.0)),
                float(current_pose.get("heading_deg", 0.0)),
                (
                    "{:.3f}, {:.3f}, {:.1f}deg".format(
                        float((request_start_pose or {}).get("x", 0.0)),
                        float((request_start_pose or {}).get("y", 0.0)),
                        float((request_start_pose or {}).get("heading_deg", 0.0)),
                    )
                    if request_start_pose
                    else "<empty>"
                ),
                (
                    "{:.3f}, {:.3f}, {:.1f}deg".format(
                        float((request_end_pose or {}).get("x", 0.0)),
                        float((request_end_pose or {}).get("y", 0.0)),
                        float((request_end_pose or {}).get("heading_deg", 0.0)),
                    )
                    if request_end_pose
                    else "<empty>"
                ),
            )
            plan_task_config = TaskConfigModel(
                task_id=self.task_config.task_id,
                map_id=self.task_config.map_id,
                work_regions=selected_work_regions,
                obstacle_regions=self.task_config.obstacle_regions,
                erase_regions=self.task_config.erase_regions,
                crop_region=self.task_config.crop_region,
                active_work_region_id=self.task_config.active_work_region_id,
                selected_work_region_ids=self.task_config.selected_work_region_ids,
                region_repeat_config=self.task_config.region_repeat_config,
                vehicle_width=self.task_config.vehicle_width,
                vehicle_length=self.task_config.vehicle_length,
                default_path_spacing=self.task_config.default_path_spacing,
                global_direction=effective_global_direction,
                planning_angle_deg=(
                    self._alignment_yaw_deg_for_sl_link_report(self._current_map_id())
                    + self._rotation_alignment_delta_deg_for_map_id(self._current_map_id())
                ),
                turn_radius=self.task_config.turn_radius,
                overlap_ratio=self.task_config.overlap_ratio,
                inflation_radius=self.task_config.inflation_radius,
                current_pose=current_pose,
                start_pose=request_start_pose or {},
                end_pose=request_end_pose or {},
            )
            planning_map = composed_map
            planning_map_info = map_info
            planning_task_config = plan_task_config
            planning_map_id = self._current_map_id()
            rospy.loginfo(
                "Planning on source map with mst27 angle: frame_id=%s map_id=%s alignment_yaw_deg=%.3f delta_deg=%.3f direction_angle_deg=%.3f map_size=%sx%s",
                planning_map_info.get("frame_id", self._live_map_source_frame),
                planning_map_id,
                self._alignment_yaw_deg_for_sl_link_report(planning_map_id),
                self._rotation_alignment_delta_deg_for_map_id(planning_map_id),
                float(planning_task_config.planning_angle_deg),
                int(planning_map_info.get("width", 0)),
                int(planning_map_info.get("height", 0)),
            )
            t_plan_align = time.perf_counter()
            planned_path = self.planner.plan(
                self.task_config.task_id or "task",
                planning_map,
                planning_map_info,
                planning_task_config,
            )
            t_plan_algorithm = time.perf_counter()
            t_plan_path_transform = time.perf_counter()
            self.current_path = planned_path
            self._invalidate_path_preview_payload_cache()
            # Disable synthetic prepended start point:
            # if planner injected a first point marked as start_pose/current_pose,
            # drop it so navigation follows only planned region path points.
            try:
                if self.current_path is not None and isinstance(self.current_path.points, list) and len(self.current_path.points) > 1:
                    first = self.current_path.points[0] if isinstance(self.current_path.points[0], dict) else {}
                    first_point_type = str(first.get("point_type", "") or "").strip().lower()
                    first_source = str(first.get("source", "") or "").strip().lower()
                    if first_point_type in ("start_pose", "current_pose", "injected_start") or first_source in (
                        "start_pose",
                        "current_pose",
                        "injected_start",
                    ):
                        self.current_path.points = self.current_path.points[1:]
                        if self.current_path.nav_path is not None and hasattr(self.current_path.nav_path, "poses"):
                            if len(self.current_path.nav_path.poses) > 1:
                                self.current_path.nav_path.poses = self.current_path.nav_path.poses[1:]
                        rospy.loginfo("Removed prepended start point from planned path.")
            except Exception as _exc:
                rospy.logwarn("Failed to remove prepended start point safely: %s", _exc)
            self._current_plan_scope = "all" if len(selected_work_regions) > 1 else "single"
            t_plan_assign = time.perf_counter()
        except Exception as exc:
            rospy.logerr("Planning failed with exception: %s", exc)
            self._set_error("Planning failed: {}".format(exc))
            return False

        if self.current_path is None or not self.current_path.points:
            rospy.logwarn("Planning finished but returned empty path")
            self._set_error("Planning failed: empty path")
            return False

        # Only prepend current pose when task context exists.
        # For pure area preview planning (no task info), keep regions disconnected
        # from robot pose to avoid a fake line from current position.
        if has_task_info and not request_start_pose:
            self._prepend_current_pose_as_start_point(map_info)
        elif request_start_pose:
            rospy.loginfo("Skip prepending current pose point because request_start_pose is provided")
        else:
            rospy.loginfo("Skip prepending current pose point because has_task_info=false")
        t_plan_prepend = time.perf_counter()
        self._rebuild_active_segments()
        t_plan_segments = time.perf_counter()
        self.current_path.nav_path.header.stamp = rospy.Time.now()
        for pose in self.current_path.nav_path.poses:
            pose.header.stamp = self.current_path.nav_path.header.stamp
        self._publish_path_to_navigation(publish_goal=False, reason="plan_success")
        t_plan_publish = time.perf_counter()
        self.current_path_index = 0
        self.last_error = ""
        self._publish_preview_metadata(map_info)
        t_plan_metadata = time.perf_counter()
        if self._planned_path_preview_save_on_plan:
            self._save_planned_path_preview(map_info)
        else:
            rospy.loginfo("Skip planned path preview save inside planning; PathPlanResponse will build/save it.")
        t_plan_preview_save = time.perf_counter()
        self._save_planned_path_debug_json(map_info)
        t_plan_debug = time.perf_counter()
        if self._save_state_on_plan_success:
            self._save_local_state()
        else:
            rospy.loginfo("Skip scheduler state save on plan success; persistent task/map state is unchanged.")
        t_plan_state = time.perf_counter()
        self.state = SchedulerState.READY
        rospy.loginfo(
            "Plan current task perf: total=%.1fms sync=%.1fms nav_refresh=%.1fms map_ready=%.1fms pose=%.1fms align=%.1fms algorithm=%.1fms path_transform=%.1fms assign=%.1fms prepend=%.1fms segments=%.1fms publish=%.1fms metadata=%.1fms preview_save=%.1fms debug=%.1fms state_save=%.1fms path_points=%d",
            (t_plan_state - t_plan_total) * 1000.0,
            (t_plan_sync - t_plan_total) * 1000.0,
            (t_plan_nav_refresh - t_plan_sync) * 1000.0,
            (t_plan_map_ready - t_plan_nav_refresh) * 1000.0,
            (t_plan_pose - t_plan_map_ready) * 1000.0,
            (t_plan_align - t_plan_pose) * 1000.0,
            (t_plan_algorithm - t_plan_align) * 1000.0,
            (t_plan_path_transform - t_plan_algorithm) * 1000.0,
            (t_plan_assign - t_plan_path_transform) * 1000.0,
            (t_plan_prepend - t_plan_assign) * 1000.0,
            (t_plan_segments - t_plan_prepend) * 1000.0,
            (t_plan_publish - t_plan_segments) * 1000.0,
            (t_plan_metadata - t_plan_publish) * 1000.0,
            (t_plan_preview_save - t_plan_metadata) * 1000.0,
            (t_plan_debug - t_plan_preview_save) * 1000.0,
            (t_plan_state - t_plan_debug) * 1000.0,
            len(self.current_path.points),
        )
        rospy.loginfo(
            "Planning success: path_version=%s points=%s length_m=%.3f",
            self.current_path.path_version,
            len(self.current_path.points),
            float(self.current_path.length_m),
        )
        return True

    def _save_planned_path_preview(self, map_info):
        if self.current_path is None or not self.current_path.points:
            return
        try:
            payload = self._build_path_preview_payload(map_info)
            if payload is None:
                raise RuntimeError("Failed to build path preview payload")
            self._save_path_preview_bytes(payload[0], payload[1], map_info)
        except Exception as exc:
            rospy.logwarn("Failed to save planned path preview: %s", exc)

    def _save_path_preview_bytes(self, image_data, image_format, map_info):
        if self.current_path is None:
            return
        os.makedirs(self._planned_path_preview_dir, exist_ok=True)
        ext = str(image_format or "jpg")
        latest_files = [
            os.path.join(self._planned_path_preview_dir, "aurora_path_preview_latest.{}".format(ext)),
        ]
        for full_path in latest_files:
            with open(full_path, "wb") as handle:
                handle.write(image_data)
        rospy.loginfo(
            "Saved planned path preview latest file: %s",
            latest_files[0],
        )

    @staticmethod
    def _round_float(value, digits=4):
        try:
            return round(float(value), digits)
        except Exception:
            return 0.0

    def _region_cache_fingerprint(self, regions):
        fingerprint = []
        for region in list(regions or []):
            if not isinstance(region, dict):
                continue
            points = []
            for point in list(region.get("points", []) or []):
                if isinstance(point, dict):
                    points.append((
                        self._round_float(point.get("x", 0.0), 4),
                        self._round_float(point.get("y", 0.0), 4),
                    ))
            start_pose = region.get("start_pose", {}) if isinstance(region.get("start_pose", {}), dict) else {}
            end_pose = region.get("end_pose", {}) if isinstance(region.get("end_pose", {}), dict) else {}
            fingerprint.append((
                str(region.get("region_id", "") or ""),
                str(region.get("name", "") or ""),
                int(region.get("order_index", 0) or 0),
                str(region.get("global_direction", "") or "").lower(),
                tuple(points),
                (
                    self._round_float(start_pose.get("x", 0.0), 4),
                    self._round_float(start_pose.get("y", 0.0), 4),
                    self._round_float(start_pose.get("heading_deg", 0.0), 2),
                ),
                (
                    self._round_float(end_pose.get("x", 0.0), 4),
                    self._round_float(end_pose.get("y", 0.0), 4),
                    self._round_float(end_pose.get("heading_deg", 0.0), 2),
                ),
            ))
        return tuple(fingerprint)

    def _current_pose_cache_fingerprint(self):
        pose = self.aurora_bridge.get_pose() or {}
        heading_deg = self._round_float(pose.get("heading_deg", 0.0), 1)
        return (
            self._round_float(pose.get("x", 0.0), 1),
            self._round_float(pose.get("y", 0.0), 1),
            int(round(float(heading_deg) / 5.0)) * 5,
        )

    def _path_plan_request_cache_fingerprint(
        self,
        map_info,
        selected_ids,
        request_start_pose,
        request_end_pose,
        request_global_direction,
        use_all,
    ):
        map_info = map_info if isinstance(map_info, dict) else {}
        return (
            str(self.task_config.map_id or self._current_map_id() or ""),
            str(self.task_config.task_id or ""),
            int(map_info.get("map_version", 0) or 0),
            int(map_info.get("width", 0) or 0),
            int(map_info.get("height", 0) or 0),
            self._round_float(map_info.get("resolution", 0.0), 5),
            self._round_float(map_info.get("origin_x", 0.0), 4),
            self._round_float(map_info.get("origin_y", 0.0), 4),
            tuple(str(item or "") for item in list(selected_ids or [])),
            tuple(sorted((str(k), int(v)) for k, v in dict(self.task_config.region_repeat_config or {}).items())),
            self._region_cache_fingerprint(self.task_config.work_regions),
            self._region_cache_fingerprint(self.task_config.obstacle_regions),
            self._region_cache_fingerprint(self.task_config.erase_regions),
            self._region_cache_fingerprint([self.task_config.crop_region] if self.task_config.crop_region else []),
            tuple(sorted((str(k), self._round_float(v, 3)) for k, v in dict(request_start_pose or {}).items())),
            tuple(sorted((str(k), self._round_float(v, 3)) for k, v in dict(request_end_pose or {}).items())),
            str(request_global_direction or "x"),
            bool(use_all),
            self._current_pose_cache_fingerprint(),
            self._round_float(self._navigation_alignment_yaw() or 0.0, 6),
            self._round_float(
                self._rotation_alignment_delta_deg_for_map_id(self.task_config.map_id),
                6,
            ),
        )

    def _path_preview_payload_cache_fingerprint(self, map_info):
        if self.current_path is None or not self.current_path.points:
            return None
        map_info = map_info if isinstance(map_info, dict) else {}
        return (
            str(self.task_config.map_id or self._current_map_id() or ""),
            str(self.current_path.task_id or ""),
            int(self.current_path.path_version),
            int(len(self.current_path.points)),
            self._round_float(self.current_path.length_m, 3),
            int(map_info.get("map_version", 0) or 0),
            int(map_info.get("width", 0) or 0),
            int(map_info.get("height", 0) or 0),
            self._round_float(map_info.get("resolution", 0.0), 5),
            self._round_float(map_info.get("origin_x", 0.0), 4),
            self._round_float(map_info.get("origin_y", 0.0), 4),
            str(self._current_plan_scope or ""),
            int(self._planned_path_preview_max_edge),
            str(self._planned_path_preview_format or "").lower(),
            bool(self._planned_path_preview_include_overlay),
            bool(self._planned_path_preview_show_direction),
            self._region_cache_fingerprint(self.task_config.work_regions),
            self._region_cache_fingerprint(self.task_config.obstacle_regions),
            self._region_cache_fingerprint(self.task_config.erase_regions),
            self._region_cache_fingerprint([self.task_config.crop_region] if self.task_config.crop_region else []),
            self._round_float(self._navigation_alignment_yaw() or 0.0, 6),
        )

    def _invalidate_path_preview_payload_cache(self):
        self._path_preview_payload_cache_key = None
        self._path_preview_payload_cache = None

    def _path_preview_overlay_base_cache_fingerprint(self, render_map_info, preview_w, preview_h):
        map_info = render_map_info if isinstance(render_map_info, dict) else {}
        return (
            str(self.task_config.map_id or self._current_map_id() or ""),
            int(map_info.get("map_version", 0) or 0),
            int(map_info.get("width", 0) or 0),
            int(map_info.get("height", 0) or 0),
            int(preview_w),
            int(preview_h),
            self._round_float(map_info.get("resolution", 0.0), 5),
            self._round_float(map_info.get("origin_x", 0.0), 4),
            self._round_float(map_info.get("origin_y", 0.0), 4),
            self._round_float(map_info.get("alignment_yaw", 0.0) or 0.0, 6),
            self._region_cache_fingerprint(self.task_config.obstacle_regions),
            self._region_cache_fingerprint(self.task_config.erase_regions),
            self._region_cache_fingerprint([self.task_config.crop_region] if self.task_config.crop_region else []),
        )

    def _invalidate_path_preview_overlay_base_cache(self):
        self._path_preview_overlay_base_cache_key = None
        self._path_preview_overlay_base_cache = None

    def _preview_snapshot_cache_token(self, map_info, max_edge, image_format, include_overlay, alignment_kwargs):
        if not isinstance(map_info, dict):
            return None
        alignment_yaw = alignment_kwargs.get("alignment_yaw", None)
        aligned_frame_id = alignment_kwargs.get("aligned_frame_id", "")
        return (
            str(self._current_map_id() or ""),
            int(map_info.get("seq", 0) or 0),
            int(map_info.get("stamp_ns", 0) or 0),
            int(map_info.get("map_version", 0) or 0),
            int(map_info.get("width", 0) or 0),
            int(map_info.get("height", 0) or 0),
            float(map_info.get("resolution", 0.0) or 0.0),
            float(map_info.get("origin_x", 0.0) or 0.0),
            float(map_info.get("origin_y", 0.0) or 0.0),
            int(max_edge),
            str(image_format or "").lower(),
            bool(include_overlay),
            None if alignment_yaw is None else round(float(alignment_yaw), 6),
            str(aligned_frame_id or ""),
        )

    def _get_cached_preview_snapshot(self, max_edge, image_format, include_overlay, alignment_kwargs):
        map_info = self.map_service.get_map_info()
        if map_info is None:
            return None
        cache_key = self._preview_snapshot_cache_token(
            map_info,
            max_edge,
            image_format,
            include_overlay,
            alignment_kwargs,
        )
        if cache_key is not None and cache_key == self._preview_snapshot_cache_key and self._preview_snapshot_cache is not None:
            return self._preview_snapshot_cache
        snapshot = self.map_service.create_preview(
            None,
            max_edge,
            image_format,
            include_overlay,
            **alignment_kwargs
        )
        self._preview_snapshot_cache_key = cache_key
        self._preview_snapshot_cache = snapshot
        return snapshot

    def _save_planned_path_debug_json(self, map_info):
        if not self._planned_path_debug_enabled:
            return
        if self.current_path is None or not self.current_path.points:
            return
        try:
            os.makedirs(self._planned_path_debug_dir, exist_ok=True)
            map_id = str(self.task_config.map_id or self._current_map_id() or "").strip()
            filename = "planned_path_map{}_task{}_latest.json".format(
                self._sanitize_name(map_id or "map"),
                self._sanitize_name(self.current_path.task_id or "task"),
            )
            full_path = os.path.join(self._planned_path_debug_dir, filename)
            payload = {
                "saved_at": int(time.time()),
                "task_id": str(self.current_path.task_id or ""),
                "map_id": map_id,
                "path_version": int(self.current_path.path_version),
                "path_length_m": float(self.current_path.length_m),
                "path_point_count": int(len(self.current_path.points)),
                "estimated_time_s": float(self._estimate_plan_time_s(self.current_path.length_m)),
                "frame_id": str((map_info or {}).get("frame_id", "")),
                "map_info": {
                    "map_version": int((map_info or {}).get("map_version", 0)),
                    "width": int((map_info or {}).get("width", 0)),
                    "height": int((map_info or {}).get("height", 0)),
                    "resolution": float((map_info or {}).get("resolution", 0.0)),
                    "origin_x": float((map_info or {}).get("origin_x", 0.0)),
                    "origin_y": float((map_info or {}).get("origin_y", 0.0)),
                },
                "points": list(self.current_path.points),
            }
            with open(full_path, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, ensure_ascii=False, indent=2)
            rospy.loginfo("Saved planned path debug json to %s", full_path)
        except Exception as exc:
            rospy.logwarn("Failed to save planned path debug json: %s", exc)

    def _remove_planned_path_debug_for_map(self, map_id):
        target_map_id = str(map_id or "").strip()
        if not target_map_id or not self._planned_path_debug_dir:
            return 0
        if not os.path.isdir(self._planned_path_debug_dir):
            return 0
        target_token = "planned_path_map{}_".format(self._sanitize_name(target_map_id))
        removed = 0
        for filename in os.listdir(self._planned_path_debug_dir):
            if not filename.endswith(".json"):
                continue
            full_path = os.path.join(self._planned_path_debug_dir, filename)
            if not os.path.isfile(full_path):
                continue
            should_remove = filename.startswith(target_token)
            if not should_remove:
                try:
                    with open(full_path, "r", encoding="utf-8") as handle:
                        payload = json.load(handle)
                    should_remove = str(payload.get("map_id", "") or "").strip() == target_map_id
                except Exception:
                    should_remove = False
            if not should_remove:
                continue
            try:
                os.remove(full_path)
                removed += 1
            except Exception as exc:
                rospy.logwarn("Failed to remove path debug file for map_id=%s path=%s: %s", target_map_id, full_path, exc)
        if removed > 0:
            rospy.loginfo("Removed planned path debug files: map_id=%s count=%d", target_map_id, removed)
        return removed

    def _build_path_preview_payload(self, map_info):
        if self.current_path is None or not self.current_path.points:
            return None
        cache_key = self._path_preview_payload_cache_fingerprint(map_info)
        if cache_key is not None and cache_key == self._path_preview_payload_cache_key and self._path_preview_payload_cache is not None:
            rospy.loginfo(
                "Path preview cache hit: path_version=%s bytes=%d",
                int(self.current_path.path_version),
                len(self._path_preview_payload_cache[0] or b""),
            )
            return self._path_preview_payload_cache
        t0_all = time.perf_counter()
        if self._current_plan_scope == "all":
            selected_work_regions = self._resolve_plan_work_regions(force_use_all_regions=True)
        else:
            selected_work_regions = self._resolve_plan_work_regions(force_use_all_regions=False)
        t1_regions = time.perf_counter()
        alignment_kwargs = self._path_planning_preview_alignment_kwargs(self._current_map_id())
        alignment_yaw = alignment_kwargs.get("alignment_yaw", None)
        snapshot = self._get_cached_preview_snapshot(
            self._planned_path_preview_max_edge,
            self._planned_path_preview_format,
            False,
            alignment_kwargs,
        )
        if snapshot is None:
            return None
        t2_snapshot = time.perf_counter()
        image = snapshot.preview_image.copy() if snapshot.preview_image is not None else None
        if image is None:
            image = cv2.imdecode(np.frombuffer(snapshot.preview_data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return None
        t3_image = time.perf_counter()
        render_map_info = {
            "width": int(snapshot.width),
            "height": int(snapshot.height),
            "origin_x": float(snapshot.origin_x),
            "origin_y": float(snapshot.origin_y),
            "resolution": float(snapshot.resolution),
            "map_version": int(snapshot.map_version),
            "frame_id": snapshot.frame_id,
            "alignment_yaw": alignment_yaw,
        }
        preview_h, preview_w = image.shape[:2]
        scale_x = float(preview_w) / float(max(1, render_map_info["width"]))
        scale_y = float(preview_h) / float(max(1, render_map_info["height"]))
        overlay_base_cache_key = self._path_preview_overlay_base_cache_fingerprint(
            render_map_info,
            preview_w,
            preview_h,
        )

        def _region_to_preview_polygon(region_points):
            return [list(item) for item in self._map_points_to_preview_pixels(
                region_points,
                render_map_info,
                scale_x,
                scale_y,
                preview_w,
                preview_h,
            )]

        if (
            overlay_base_cache_key is not None
            and overlay_base_cache_key == self._path_preview_overlay_base_cache_key
            and self._path_preview_overlay_base_cache is not None
        ):
            image = self._path_preview_overlay_base_cache.copy()
            rospy.loginfo("Path preview overlay base cache hit: preview=%dx%d", preview_w, preview_h)
        else:
            self._paint_region_overrides_for_path_preview(image, render_map_info, scale_x, scale_y, _region_to_preview_polygon)
            self._path_preview_overlay_base_cache_key = overlay_base_cache_key
            self._path_preview_overlay_base_cache = image.copy()
        t4_overlay = time.perf_counter()

        path_pixels = self._map_points_to_preview_pixels(
            self.current_path.points,
            render_map_info,
            scale_x,
            scale_y,
            preview_w,
            preview_h,
        )
        polyline = []
        for point, pixel in zip(self.current_path.points, path_pixels):
            polyline.append({
                "x": int(pixel[0]),
                "y": int(pixel[1]),
                "path_type": str(point.get("path_type", "") or ""),
            })
        t5_path_pixels = time.perf_counter()
        # Display only selected work regions on path preview.
        for region in selected_work_regions:
            region_points = region.get("points", []) if isinstance(region, dict) else []
            if len(region_points) < 3:
                continue
            polygon = _region_to_preview_polygon(region_points)
            if len(polygon) >= 3:
                polygon_np = np.array(polygon, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(image, [polygon_np], isClosed=True, color=(0, 200, 0), thickness=1)
        t6_region_outline = time.perf_counter()
        if len(polyline) >= 2:
            region_segments = []
            def _draw_segments(type_filter, color):
                segment = []
                for item in polyline:
                    if type_filter(item["path_type"]):
                        segment.append([item["x"], item["y"]])
                    else:
                        if len(segment) >= 2:
                            seg_np = np.array(segment, dtype=np.int32).reshape((-1, 1, 2))
                            cv2.polylines(image, [seg_np], isClosed=False, color=color, thickness=1)
                        segment = []
                if len(segment) >= 2:
                    seg_np = np.array(segment, dtype=np.int32).reshape((-1, 1, 2))
                    cv2.polylines(image, [seg_np], isClosed=False, color=color, thickness=1)

            def _draw_non_connection(color):
                segment = []
                last_type = ""
                last_start = None
                for item in polyline:
                    t = str(item.get("path_type", "") or "")
                    if t == "connection":
                        if len(segment) >= 2:
                            seg_np = np.array(segment, dtype=np.int32).reshape((-1, 1, 2))
                            cv2.polylines(image, [seg_np], isClosed=False, color=color, thickness=1)
                            if last_start is not None:
                                region_segments.append((last_type, tuple(last_start), tuple(segment[-1])))
                        segment = []
                        last_type = ""
                        last_start = None
                        continue
                    # Break line when switching between different region path types
                    # to avoid fake straight links across disconnected regions.
                    if last_type and t != last_type:
                        if len(segment) >= 2:
                            seg_np = np.array(segment, dtype=np.int32).reshape((-1, 1, 2))
                            cv2.polylines(image, [seg_np], isClosed=False, color=color, thickness=1)
                            if last_start is not None:
                                region_segments.append((last_type, tuple(last_start), tuple(segment[-1])))
                        segment = []
                        last_start = None
                    if not segment:
                        last_start = [item["x"], item["y"]]
                    segment.append([item["x"], item["y"]])
                    last_type = t
                if len(segment) >= 2:
                    seg_np = np.array(segment, dtype=np.int32).reshape((-1, 1, 2))
                    cv2.polylines(image, [seg_np], isClosed=False, color=color, thickness=1)
                    if last_start is not None:
                        region_segments.append((last_type, tuple(last_start), tuple(segment[-1])))

            # 覆盖段：红色（按各区域分段）；区域间连接段：青色（更容易区分）
            _draw_non_connection((0, 0, 255))
            _draw_segments(lambda t: t == "connection", (255, 255, 0))
            # Bridge tiny visual gaps at connection/coverage boundaries. The
            # path itself is continuous, but drawing connection and coverage in
            # different passes can leave a one-segment gap in the preview.
            connection_boundary_lines = []
            for idx in range(len(polyline) - 1):
                t0 = str(polyline[idx].get("path_type", "") or "")
                t1 = str(polyline[idx + 1].get("path_type", "") or "")
                if (t0 == "connection") == (t1 == "connection"):
                    continue
                p0 = (int(polyline[idx]["x"]), int(polyline[idx]["y"]))
                p1 = (int(polyline[idx + 1]["x"]), int(polyline[idx + 1]["y"]))
                connection_boundary_lines.append((p0, p1))
                cv2.line(image, p0, p1, (255, 255, 0), thickness=1, lineType=cv2.LINE_AA)
            if self._planned_path_preview_show_direction:
                arrow_step = self._planned_path_preview_arrow_step
                arrow_len_px = float(self._planned_path_preview_arrow_len_px)
                for idx in range(0, len(polyline) - 1, arrow_step):
                    p0 = polyline[idx]
                    p1 = polyline[min(idx + 1, len(polyline) - 1)]
                    dx = float(p1["x"] - p0["x"])
                    dy = float(p1["y"] - p0["y"])
                    norm = math.hypot(dx, dy)
                    if norm < 1.0:
                        continue
                    ux = dx / norm
                    uy = dy / norm
                    ex = int(round(float(p0["x"]) + ux * arrow_len_px))
                    ey = int(round(float(p0["y"]) + uy * arrow_len_px))
                    start_pt = (int(p0["x"]), int(p0["y"]))
                    end_pt = (max(0, min(preview_w - 1, ex)), max(0, min(preview_h - 1, ey)))
                    color = (255, 255, 0) if str(p0.get("path_type", "")) == "connection" else (0, 120, 255)
                    cv2.arrowedLine(
                        image,
                        start_pt,
                        end_pt,
                        color,
                        thickness=1,
                        line_type=cv2.LINE_AA,
                        tipLength=0.45,
                    )
            # Keep only start/end markers; index labels and direction arrows stay disabled by policy.
            cv2.circle(image, (polyline[0]["x"], polyline[0]["y"]), 5, (0, 220, 0), thickness=-1)
            cv2.circle(image, (polyline[-1]["x"], polyline[-1]["y"]), 5, (0, 140, 255), thickness=-1)
            # Mark start/end for each region segment to make region handoff explicit.
            for seg_type, seg_start, seg_end in region_segments:
                if not seg_type or seg_type == "connection":
                    continue
                cv2.circle(image, (int(seg_start[0]), int(seg_start[1])), 4, (0, 220, 0), thickness=-1)
                cv2.circle(image, (int(seg_end[0]), int(seg_end[1])), 4, (0, 140, 255), thickness=-1)
            # Redraw boundary lines after markers, otherwise solid start/end
            # markers can visually hide the connection and look like a gap.
            for p0, p1 in connection_boundary_lines:
                cv2.line(image, p0, p1, (255, 255, 0), thickness=1, lineType=cv2.LINE_AA)
        t7_path_draw = time.perf_counter()
        image = self._crop_preview_image_by_region(image, render_map_info)
        t8_crop = time.perf_counter()
        ext = ".png" if snapshot.preview_format.lower() == "png" else ".jpg"
        ok, buffer = cv2.imencode(ext, image)
        if not ok:
            return None
        t9_encode = time.perf_counter()
        rospy.loginfo(
            "Path preview perf: total=%.1fms resolve_regions=%.1fms snapshot=%.1fms image_ready=%.1fms overlay=%.1fms path_pixels=%.1fms region_outline=%.1fms path_draw=%.1fms crop=%.1fms encode=%.1fms path_points=%d selected_regions=%d preview=%dx%d",
            (t9_encode - t0_all) * 1000.0,
            (t1_regions - t0_all) * 1000.0,
            (t2_snapshot - t1_regions) * 1000.0,
            (t3_image - t2_snapshot) * 1000.0,
            (t4_overlay - t3_image) * 1000.0,
            (t5_path_pixels - t4_overlay) * 1000.0,
            (t6_region_outline - t5_path_pixels) * 1000.0,
            (t7_path_draw - t6_region_outline) * 1000.0,
            (t8_crop - t7_path_draw) * 1000.0,
            (t9_encode - t8_crop) * 1000.0,
            len(self.current_path.points),
            len(selected_work_regions),
            int(image.shape[1]),
            int(image.shape[0]),
        )
        payload = (buffer.tobytes(), snapshot.preview_format, render_map_info, int(image.shape[1]), int(image.shape[0]))
        self._path_preview_payload_cache_key = cache_key
        self._path_preview_payload_cache = payload
        return payload

    def _build_failed_plan_preview_payload(self, error_message):
        try:
            t0_all = time.perf_counter()
            alignment_kwargs = self._path_planning_preview_alignment_kwargs(self._current_map_id())
            alignment_yaw = alignment_kwargs.get("alignment_yaw", None)
            snapshot = self._get_cached_preview_snapshot(
                self._planned_path_preview_max_edge,
                self._planned_path_preview_format,
                self._planned_path_preview_include_overlay,
                alignment_kwargs,
            )
            if snapshot is None:
                return None
            t1_snapshot = time.perf_counter()
            image = snapshot.preview_image.copy() if snapshot.preview_image is not None else None
            if image is None:
                image = cv2.imdecode(np.frombuffer(snapshot.preview_data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                return snapshot.preview_data, snapshot.preview_format, None
            t2_image = time.perf_counter()
            render_map_info = {
                "width": int(snapshot.width),
                "height": int(snapshot.height),
                "origin_x": float(snapshot.origin_x),
                "origin_y": float(snapshot.origin_y),
                "resolution": float(snapshot.resolution),
                "map_version": int(snapshot.map_version),
                "frame_id": snapshot.frame_id,
                "alignment_yaw": alignment_yaw,
            }
            if render_map_info is not None:
                preview_h, preview_w = image.shape[:2]
                scale_x = float(preview_w) / float(max(1, render_map_info["width"]))
                scale_y = float(preview_h) / float(max(1, render_map_info["height"]))

                def _region_to_preview_polygon(region_points):
                    polygon = []
                    for point in region_points:
                        pixel_x, pixel_y = self._map_point_to_preview_pixel(
                            point,
                            render_map_info,
                            scale_x,
                            scale_y,
                            preview_w,
                            preview_h,
                        )
                        polygon.append([pixel_x, pixel_y])
                    return polygon

                self._paint_region_overrides_for_path_preview(image, render_map_info, scale_x, scale_y, _region_to_preview_polygon)
                image = self._crop_preview_image_by_region(image, render_map_info)
            t3_overlay_crop = time.perf_counter()
            text = "Planning failed: {}".format(error_message or "unknown")
            text = text[:120]
            cv2.putText(
                image,
                text,
                (16, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            t4_text = time.perf_counter()
            ext = ".png" if snapshot.preview_format.lower() == "png" else ".jpg"
            ok, buffer = cv2.imencode(ext, image)
            if not ok:
                return snapshot.preview_data, snapshot.preview_format, render_map_info
            t5_encode = time.perf_counter()
            rospy.loginfo(
                "Failed preview perf: total=%.1fms snapshot=%.1fms image_ready=%.1fms overlay_crop=%.1fms text=%.1fms encode=%.1fms preview=%dx%d msg_len=%d",
                (t5_encode - t0_all) * 1000.0,
                (t1_snapshot - t0_all) * 1000.0,
                (t2_image - t1_snapshot) * 1000.0,
                (t3_overlay_crop - t2_image) * 1000.0,
                (t4_text - t3_overlay_crop) * 1000.0,
                (t5_encode - t4_text) * 1000.0,
                int(image.shape[1]),
                int(image.shape[0]),
                len(str(error_message or "")),
            )
            return buffer.tobytes(), snapshot.preview_format, render_map_info
        except Exception:
            return None

    def _paint_region_overrides_for_path_preview(self, image, map_info, scale_x, scale_y, polygon_builder):
        try:
            overlay = self.map_service.get_overlay_regions() or {}
            public_obstacle_regions = overlay.get("obstacle_regions", []) or []
            erase_regions = overlay.get("erase_regions", []) or []
        except Exception:
            public_obstacle_regions = []
            erase_regions = []
        # Path planning synchronizes the public map regions first and then
        # appends the active task's private obstacle regions to task_config.
        # Render that effective list so the preview matches planner input,
        # without writing task-private regions back to the map overlay.
        obstacle_regions = list(self.task_config.obstacle_regions or [])
        if not obstacle_regions:
            obstacle_regions = list(public_obstacle_regions)
        obstacle_count = 0
        erase_count = 0
        obstacle_mask = np.zeros(image.shape[:2], dtype=np.uint8)
        erase_mask = np.zeros(image.shape[:2], dtype=np.uint8)
        for region in obstacle_regions:
            if not isinstance(region, dict):
                continue
            if not bool(region.get("enabled", True)):
                continue
            points = region.get("points", []) or []
            if len(points) < 3:
                continue
            poly = polygon_builder(points)
            if len(poly) < 3:
                continue
            # Obstacle-region visualization follows UNKNOWN tone on preview.
            cv2.fillPoly(obstacle_mask, [np.array(poly, dtype=np.int32)], color=255)
            obstacle_count += 1
        for region in erase_regions:
            if not isinstance(region, dict):
                continue
            if not bool(region.get("enabled", True)):
                continue
            points = region.get("points", []) or []
            if len(points) < 3:
                continue
            poly = polygon_builder(points)
            if len(poly) < 3:
                continue
            cv2.fillPoly(erase_mask, [np.array(poly, dtype=np.int32)], color=255)
            erase_count += 1
        # Cover polygon edge quantization gaps after world->preview projection.
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        obstacle_mask = cv2.dilate(obstacle_mask, kernel, iterations=1)
        erase_mask = cv2.dilate(erase_mask, kernel, iterations=1)
        if int(np.count_nonzero(erase_mask)) > 0:
            image[erase_mask > 0] = (245, 245, 245)
        if int(np.count_nonzero(obstacle_mask)) > 0:
            image[obstacle_mask > 0] = (180, 180, 180)
        rospy.loginfo_throttle(
            2.0,
            "Path preview paint override: obstacle_regions=%d erase_regions=%d",
            obstacle_count,
            erase_count,
        )

    def _sanitize_name(self, text):
        safe = "".join(ch if (ch.isalnum() or ch in ("_", "-")) else "_" for ch in str(text))
        return safe[:64] if safe else "task"

    def _sanitize_map_filename_stem(self, text):
        # Keep Chinese/Unicode letters and digits; drop path-unfriendly symbols.
        # This allows Android-provided Chinese map names to be preserved on disk.
        raw = str(text or "").strip()
        if not raw:
            return "地图"
        # Remove extension if user already passed ".stcm".
        if raw.lower().endswith(".stcm"):
            raw = raw[:-5]
        # Keep most printable filename chars, replace reserved separators.
        safe_chars = []
        for ch in raw:
            if ch in ('\\', '/', ':', '*', '?', '"', '<', '>', '|'):
                safe_chars.append("_")
                continue
            # avoid control characters
            if ord(ch) < 32:
                continue
            safe_chars.append(ch)
        safe = "".join(safe_chars).strip().strip(".")
        if not safe:
            safe = "地图"
        return safe[:96]

    def _build_stcm_download_path(self, requested_path, forced_map_id=""):
        req = str(requested_path or "").strip()
        if req:
            req = os.path.expanduser(os.path.expandvars(req))
        if req:
            req_dir = os.path.dirname(req)
            if req_dir:
                save_dir = req_dir if os.path.isabs(req_dir) else os.path.join(self._stcm_local_dir, req_dir)
            else:
                save_dir = self._stcm_local_dir
            base_name = os.path.basename(req)
            stem = self._sanitize_map_filename_stem(base_name)
        else:
            save_dir = self._stcm_local_dir
            stem = "地图"
        map_id = str(forced_map_id or "").strip() or datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = "{}_{}.stcm".format(stem, map_id)
        return os.path.normpath(os.path.join(save_dir, filename))

    def _split_map_name_and_id_from_path(self, stcm_path):
        base = os.path.splitext(os.path.basename(str(stcm_path or "")))[0]
        # Expected generated filename: <map_name>_<YYYYMMDD_HHMMSS>.stcm
        matched = re.match(r"^(.*)_(\d{8}_\d{6})$", base)
        if matched:
            raw_name = matched.group(1).strip("_").strip()
            map_id = matched.group(2)
            return (raw_name or "地图"), map_id
        fallback_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        return (base or "地图"), fallback_id

    def _register_saved_map(
        self,
        stcm_path,
        display_name="",
        explicit_map_id="",
        total_work_area_m2=0.0,
        estimated_time_s=-1.0,
        region_metrics=None,
        thumb_format="",
        thumb_b64="",
        thumb_width=0,
        thumb_height=0,
    ):
        try:
            abs_path = os.path.abspath(str(stcm_path or "").strip())
            if not abs_path:
                return "", ""
            parsed_name, parsed_id = self._split_map_name_and_id_from_path(abs_path)
            name = str(display_name or "").strip() or parsed_name
            map_id = str(explicit_map_id or "").strip() or str(parsed_id)
            size_bytes = 0
            if os.path.isfile(abs_path):
                try:
                    size_bytes = int(os.path.getsize(abs_path))
                except Exception:
                    size_bytes = 0
            old_record = self._map_registry.get(abs_path, {})
            old_created_at = ""
            if isinstance(old_record, dict):
                old_created_at = str(old_record.get("created_at", "") or "").strip()
                if not old_created_at:
                    old_created_at = _format_ts_s(old_record.get("saved_at", 0))
            created_at = old_created_at or datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            new_record = {
                "map_id": map_id,
                "name": name,
                "path": abs_path,
                "size_bytes": size_bytes,
                "created_at": str(created_at),
                "saved_at": int(time.time()),
                "total_work_area_m2": float(max(0.0, float(total_work_area_m2 or 0.0))),
                "estimated_time_s": float(estimated_time_s if estimated_time_s is not None else -1.0),
                "region_metrics": list(region_metrics or []),
                "thumb_format": str(thumb_format or "").strip(),
                "thumb_b64": str(thumb_b64 or ""),
                "thumb_width": int(max(0, int(thumb_width or 0))),
                "thumb_height": int(max(0, int(thumb_height or 0))),
            }
            if isinstance(old_record, dict):
                for key in (
                    "alignment_yaw",
                    "alignment_yaw_deg",
                    "raw_grid_alignment_yaw",
                    "raw_grid_alignment_yaw_deg",
                    "app_rotation_deg",
                    "rotation_alignment_delta_deg",
                    "alignment_source_frame_id",
                    "alignment_frame_id",
                ):
                    if key in old_record:
                        new_record[key] = old_record[key]
            self._map_registry[abs_path] = new_record
            return name, map_id
        except Exception as exc:
            rospy.logwarn("Failed to register saved map metadata: %s", exc)
            return "", ""

    def _queue_saved_map_upload(self, map_id, map_name, stcm_path):
        target_map_id = str(map_id or "").strip()
        if not target_map_id:
            return False
        map_dir = self._map_state_dir(target_map_id)
        try:
            os.makedirs(map_dir, exist_ok=True)
            record = self._find_recorded_map_by_id(target_map_id)
            related_bindings = {}
            for task_id, binding in (self._task_bindings or {}).items():
                if not isinstance(binding, dict):
                    continue
                if str(binding.get("map_id", binding.get("mapId", "")) or "") == target_map_id:
                    related_bindings[str(task_id)] = deepcopy(binding)
            manifest = {
                "schemaVersion": 1,
                "mapId": target_map_id,
                "mapName": str(map_name or ""),
                "stcmFile": os.path.basename(str(stcm_path or "")),
                "map": deepcopy(record) if isinstance(record, dict) else {},
                "taskBindings": related_bindings,
                "savedAt": int(time.time()),
            }
            # Thumbnail bytes already exist as a separate preview source and can
            # make this metadata file unnecessarily large.
            if isinstance(manifest["map"], dict) and "thumb_b64" in manifest["map"]:
                manifest["map"]["thumb_b64"] = ""
            tmp_path = os.path.join(map_dir, "map_info.json.tmp")
            final_path = os.path.join(map_dir, "map_info.json")
            with open(tmp_path, "w", encoding="utf-8") as handle:
                json.dump(manifest, handle, ensure_ascii=False, indent=2)
            os.replace(tmp_path, final_path)
        except Exception as exc:
            rospy.logwarn("Failed to prepare map upload manifest: map_id=%s error=%s", target_map_id, exc)
            return False
        return self.platform_file_sync.enqueue_map(
            target_map_id,
            map_name,
            map_dir,
            stcm_path,
        )

    def _unregister_saved_map(self, target_path):
        abs_path = os.path.abspath(str(target_path or "").strip())
        if abs_path:
            self._map_registry.pop(abs_path, None)

    def _remove_task_bindings_for_map_id(self, map_id, reason=""):
        target_map_id = str(map_id or "").strip()
        if not target_map_id:
            return 0
        prefix = "{}::".format(target_map_id)
        removed = 0
        for key in list(self._task_bindings.keys()):
            if str(key).startswith(prefix):
                self._task_bindings.pop(key, None)
                removed += 1
        with self._task_obstacle_regions_lock:
            for key in list(self._task_obstacle_regions.keys()):
                if str(key).startswith(prefix):
                    self._task_obstacle_regions.pop(key, None)
        if removed > 0:
            rospy.loginfo(
                "Removed task bindings: map_id=%s count=%d%s",
                target_map_id,
                removed,
                (" reason={}".format(reason) if reason else ""),
            )
        if isinstance(self._last_task_result, dict):
            last_map_id = str(self._last_task_result.get("map_id", "") or "").strip()
            if last_map_id == target_map_id:
                self._last_task_result = {}
                rospy.loginfo(
                    "Cleared last task result: map_id=%s%s",
                    target_map_id,
                    (" reason={}".format(reason) if reason else ""),
                )
        return removed

    def _remove_task_bindings_for_map_aliases(self, aliases, reason=""):
        alias_set = {str(alias or "").strip() for alias in list(aliases or []) if str(alias or "").strip()}
        if not alias_set:
            return 0
        removed = 0
        for key in list(self._task_bindings.keys()):
            key_text = str(key)
            key_map_id = key_text.split("::", 1)[0] if "::" in key_text else ""
            record = self._task_bindings.get(key)
            record_map_id = ""
            if isinstance(record, dict):
                record_map_id = str(record.get("map_id", "") or "").strip()
                record_requested_map_id = str(record.get("requested_map_id", "") or "").strip()
            else:
                record_requested_map_id = ""
            if key_map_id in alias_set or record_map_id in alias_set or record_requested_map_id in alias_set:
                self._task_bindings.pop(key, None)
                removed += 1
        with self._task_obstacle_regions_lock:
            for key in list(self._task_obstacle_regions.keys()):
                key_text = str(key)
                key_map_id = key_text.split("::", 1)[0] if "::" in key_text else ""
                if key_map_id in alias_set:
                    self._task_obstacle_regions.pop(key, None)
        if isinstance(self._last_task_result, dict):
            last_map_id = str(self._last_task_result.get("map_id", "") or "").strip()
            if last_map_id in alias_set:
                self._last_task_result = {}
                rospy.loginfo(
                    "Cleared last task result: map_aliases=%s%s",
                    ",".join(sorted(alias_set)),
                    (" reason={}".format(reason) if reason else ""),
                )
        if removed > 0:
            rospy.loginfo(
                "Removed task bindings: map_aliases=%s count=%d%s",
                ",".join(sorted(alias_set)),
                removed,
                (" reason={}".format(reason) if reason else ""),
            )
        return removed

    def _remove_saved_map_data_by_map_id(self, map_id, keep_path="", reason=""):
        target_map_id = str(map_id or "").strip()
        if not target_map_id:
            return 0
        keep_abs = os.path.abspath(str(keep_path or "").strip()) if keep_path else ""
        removed_records = 0
        removed_files = 0
        overlay_aliases = {target_map_id}
        for path, record in list((self._map_registry or {}).items()):
            if not isinstance(record, dict):
                continue
            record_map_id = str(record.get("map_id", "") or "").strip()
            if record_map_id != target_map_id:
                continue
            abs_path = os.path.abspath(str(record.get("path", path) or path).strip())
            if keep_abs and abs_path == keep_abs:
                continue
            parsed_name, parsed_id = self._split_map_name_and_id_from_path(abs_path)
            overlay_aliases.update([
                record_map_id,
                str(record.get("name", "") or "").strip(),
                parsed_name,
                parsed_id,
            ])
            self._map_registry.pop(path, None)
            removed_records += 1
            try:
                if abs_path and os.path.isfile(abs_path):
                    os.remove(abs_path)
                    removed_files += 1
            except Exception as exc:
                rospy.logwarn("Failed to remove old map file for map_id=%s path=%s: %s", target_map_id, abs_path, exc)
        removed_overlay_states = self._remove_map_overlay_states_for_aliases(overlay_aliases)
        removed_path_debug = self._remove_planned_path_debug_for_map(target_map_id)
        removed_tasks = self._remove_task_bindings_for_map_aliases(overlay_aliases, reason=reason or "map_replace")
        rospy.loginfo(
            "Removed old map data by map_id: map_id=%s records=%d files=%d overlay_states=%d path_debug=%d task_bindings=%d keep_path=%s%s",
            target_map_id,
            removed_records,
            removed_files,
            removed_overlay_states,
            removed_path_debug,
            removed_tasks,
            keep_abs or "<none>",
            (" reason={}".format(reason) if reason else ""),
        )
        return removed_records

    def _find_recorded_map_by_id(self, map_id):
        target = str(map_id or "").strip()
        if not target:
            return None
        for record in (self._map_registry or {}).values():
            if not isinstance(record, dict):
                continue
            if str(record.get("map_id", "")).strip() == target:
                return record
        return None

    def _iter_recorded_maps(self, target_dir=""):
        target_dir = os.path.abspath(target_dir) if target_dir else ""
        exts = {".stcm"}
        entries = []
        for record in (self._map_registry or {}).values():
            if not isinstance(record, dict):
                continue
            path = os.path.abspath(str(record.get("path", "")).strip())
            if not path:
                continue
            ext = os.path.splitext(path)[1].lower()
            if ext not in exts:
                continue
            if target_dir:
                try:
                    if os.path.commonpath([path, target_dir]) != target_dir:
                        continue
                except Exception:
                    continue
            parsed_name, parsed_id = self._split_map_name_and_id_from_path(path)
            name = str(record.get("name", "")).strip() or parsed_name
            map_id = str(record.get("map_id", "")).strip() or parsed_id
            size_bytes = int(record.get("size_bytes", 0) or 0)
            if os.path.isfile(path):
                try:
                    size_bytes = int(os.path.getsize(path))
                except Exception:
                    pass
            total_work_area_m2 = float(record.get("total_work_area_m2", 0.0) or 0.0)
            estimated_time_s = float(record.get("estimated_time_s", -1.0) or -1.0)
            entries.append((name, map_id, path, size_bytes, total_work_area_m2, estimated_time_s))
        entries.sort(key=lambda item: item[0].lower())
        return entries

    def _publish_preview_metadata(self, map_info):
        message = MapPreviewMetadata()
        message.header.stamp = rospy.Time.now()
        message.map_version = map_info["map_version"]
        message.width = map_info["width"]
        message.height = map_info["height"]
        message.resolution = map_info["resolution"]
        message.origin.position.x = map_info["origin_x"]
        message.origin.position.y = map_info["origin_y"]
        message.origin.orientation.w = 1.0
        message.frame_id = map_info["frame_id"]
        self.preview_meta_pub.publish(message)

    def _start_execution(self):
        if float(self._chassis_settings.get("run_speed", 0.0)) <= 0.0:
            self._safe_stop_motion()
            return False, "Cannot start task while run_speed is zero"
        with self._collision_imminent_lock:
            collision_imminent = self._collision_imminent_active
        if collision_imminent:
            return False, "Cannot start while navigation collision is imminent"

        t0_start = time.perf_counter()
        t_prev = t0_start

        def mark(step):
            nonlocal t_prev
            now = time.perf_counter()
            rospy.loginfo(
                "TaskStart perf: step=%s step_ms=%.1f elapsed_ms=%.1f",
                step,
                (now - t_prev) * 1000.0,
                (now - t0_start) * 1000.0,
            )
            t_prev = now

        try:
            self._switch_to_localization_mode_after_map_save(publish_count=6)
            mark("switch_to_localization")
            rospy.loginfo("Task start pre-check: radar switched to localization mode")
        except Exception as exc:
            mark("switch_to_localization_failed")
            rospy.logwarn("Task start blocked: failed to switch radar to localization mode: %s", exc)
            return False, "Failed to switch radar to localization mode: {}".format(exc)
        if self._send_radar_map_sync():
            rospy.loginfo("Task start radar map sync sent once")
        else:
            rospy.logwarn("Task start radar map sync was not sent; continue task startup")
        mark("sync_radar_map")
        if self.current_path is None and not self._plan_current_task():
            mark("plan_current_task_failed")
            return False, "Failed to plan task"
        mark("ensure_current_path")
        if self.current_path is None or not self.current_path.points:
            mark("validate_path_failed")
            return False, "No planned path to execute"
        self._sync_task_regions_from_overlay()
        self._exec_region_order = self._sorted_work_region_ids()
        active_id = (self.task_config.active_work_region_id or "").strip()
        if active_id and active_id in self._exec_region_order:
            self._exec_region_index = self._exec_region_order.index(active_id)
        elif self._exec_region_order:
            self._exec_region_index = 0
            self.task_config.active_work_region_id = self._exec_region_order[0]
        else:
            self._exec_region_index = -1
        mark("prepare_region_order")
        self._set_chassis_enabled(True)
        self._manual_travel_disc_active = False
        self.work_mode_pub.publish(UInt16(data=2))
        self.disc_speed_pub.publish(Int16(data=self._configured_disc_speed_rpm()))
        self.disc_enable_pub.publish(Bool(data=True))
        self._disc_auto_cover_desired = True
        self._reset_disc_motion_guard()
        self.light_pub.publish(Bool(data=True))
        mark("publish_chassis_disc_commands")
        self._init_segment_execution_cursor()
        mark("init_segment_cursor")
        self._exec_active = True
        self._exec_publish_start_pose_once = False
        self._exec_region_repeat_done = {}
        self._task_stop_reason = ""
        self._set_cmd_vel_forward_runtime_active(True, publish_zero=False, reason="task_start_nav_cmd_vel_forward")
        mark("enable_task_cmd_vel")
        self._exec_last_send_time = 0.0
        self._exec_goal_start_time = time.time()
        self._publish_path_to_navigation(publish_goal=False, reason="task_start")
        mark("publish_global_path")
        self._publish_active_segment_plan(reason="task_start")
        mark("publish_active_segment")
        self._send_active_segment_goal(force=True, reason="task_start")
        mark("send_active_goal")
        self.state = SchedulerState.RUNNING
        self._begin_task_execution_record()
        mark("set_running")
        rospy.loginfo(
            "Task execution started: regions=%s active=%s mode=%s",
            ",".join(self._exec_region_order) if self._exec_region_order else "<none>",
            self.task_config.active_work_region_id or "<none>",
            self._exec_mode,
        )
        return True, "Task started (mode={})".format(self._exec_mode)

    @staticmethod
    def _mqtt_path_point(point):
        return {
            "index": int(point.get("index", 0) or 0),
            "x": round(float(point.get("x", 0.0) or 0.0), 6),
            "y": round(float(point.get("y", 0.0) or 0.0), 6),
        }

    def _publish_task_path_to_mqtt(self):
        if self.current_path is None or not self.current_path.points:
            rospy.logwarn("MQTT task path report skipped: planned path is empty")
            return False

        enriched_points, segments = self._classify_task_path_points(self.current_path.points)
        within_region_paths = []
        between_region_paths = []
        for segment in segments:
            start_index = int(segment.get("start_point_index", 0) or 0)
            end_index = int(segment.get("end_point_index", start_index) or start_index)
            segment_points = [
                self._mqtt_path_point(point)
                for point in enriched_points[start_index : end_index + 1]
            ]
            if segment.get("path_scope") == "between_regions":
                between_region_paths.append(
                    {
                        "segmentIndex": int(segment.get("segment_index", 0) or 0),
                        "fromRegionId": str(segment.get("from_region_id", "") or ""),
                        "toRegionId": str(segment.get("to_region_id", "") or ""),
                        "points": segment_points,
                    }
                )
            else:
                within_region_paths.append(
                    {
                        "segmentIndex": int(segment.get("segment_index", 0) or 0),
                        "regionId": str(segment.get("region_id", "") or ""),
                        "lapIndex": int(segment.get("lap_index", 0) or 0),
                        "points": segment_points,
                    }
                )

        map_info = self.map_service.get_map_info() or {}
        report = {
            "taskId": str(self.task_config.task_id or ""),
            "mapId": str(self._current_map_id() or ""),
            "frameId": str(map_info.get("frame_id", "map") or "map"),
            "pathVersion": int(self.current_path.path_version),
            "pathPointCount": len(enriched_points),
            "pathLengthM": round(float(self.current_path.length_m), 6),
            "withinRegionPaths": within_region_paths,
            "betweenRegionPaths": between_region_paths,
        }
        success, message = self.mqtt_reporter.publish_report("task/path", report)
        log = rospy.loginfo if success else rospy.logwarn
        log(
            "MQTT task path report: success=%s task_id=%s map_id=%s points=%d within_segments=%d between_segments=%d message=%s",
            str(bool(success)).lower(),
            report["taskId"] or "<empty>",
            report["mapId"] or "<empty>",
            report["pathPointCount"],
            len(within_region_paths),
            len(between_region_paths),
            message,
        )
        return success

    def _pause_execution(self):
        self._exec_active = False
        self._disc_auto_cover_desired = False
        self._reset_disc_motion_guard()
        self._set_cmd_vel_forward_runtime_active(False, publish_zero=True, reason="task_pause")
        self._safe_stop_motion()
        self.disc_enable_pub.publish(Bool(data=False))
        self.state = SchedulerState.PAUSED
        return True, "Task paused"

    def _resume_execution(self):
        if float(self._chassis_settings.get("run_speed", 0.0)) <= 0.0:
            self._safe_stop_motion()
            return False, "Cannot resume task while run_speed is zero"
        with self._collision_imminent_lock:
            collision_imminent = self._collision_imminent_active
        if collision_imminent:
            return False, "Cannot resume while navigation collision is imminent"
        if self.current_path is None or not self.current_path.points:
            return False, "No path to resume"
        self._set_chassis_enabled(True)
        self._disc_last_mode = ""
        self._manual_travel_disc_active = False
        self.disc_speed_pub.publish(Int16(data=self._configured_disc_speed_rpm()))
        self.disc_enable_pub.publish(Bool(data=True))
        self._disc_auto_cover_desired = True
        self._reset_disc_motion_guard()
        if not self._active_segments:
            self._rebuild_active_segments()
        self._init_segment_execution_cursor()
        self._exec_active = True
        self._exec_publish_start_pose_once = False
        self._set_cmd_vel_forward_runtime_active(True, publish_zero=False, reason="task_resume_nav_cmd_vel_forward")
        self._exec_goal_start_time = time.time()
        self._publish_path_to_navigation(publish_goal=False, reason="task_resume")
        self._publish_active_segment_plan(reason="task_resume")
        self._send_active_segment_goal(force=True, reason="task_resume")
        self.state = SchedulerState.RUNNING
        return True, "Task resumed"

    def _stop_execution(self):
        self._exec_active = False
        self._disc_auto_cover_desired = False
        self._reset_disc_motion_guard()
        self._mark_current_region_repeat_done()
        self._set_cmd_vel_forward_runtime_active(False, publish_zero=True, reason="task_stop")
        self._safe_stop_motion()
        self.disc_enable_pub.publish(Bool(data=False))
        self.light_pub.publish(Bool(data=False))
        self._set_chassis_enabled(False)
        self._exec_region_order = []
        self._exec_region_index = -1
        self.state = SchedulerState.STOPPED
        self._task_stop_reason = "stopped_by_command"
        self._finalize_task_result(stop_reason=self._task_stop_reason)
        return True, "Task stopped"

    def _safe_stop_motion(self):
        wheel = WheelSpeedCommand()
        wheel.left_wheel_speed = 0
        wheel.right_wheel_speed = 0
        self.wheel_cmd_pub.publish(wheel)
        self._set_chassis_enabled(False)

    def _normalize_angle(self, rad):
        return math.atan2(math.sin(rad), math.cos(rad))

    def _yaw_to_quaternion(self, yaw):
        half = 0.5 * float(yaw)
        return {
            "x": 0.0,
            "y": 0.0,
            "z": math.sin(half),
            "w": math.cos(half),
        }

    def _prepend_current_pose_as_start_point(self, map_info=None):
        if self.current_path is None or not self.current_path.points:
            return
        pose = self.aurora_bridge.get_pose() or {}
        start_x = float(pose.get("x", 0.0))
        start_y = float(pose.get("y", 0.0))
        heading_deg = float(pose.get("heading_deg", 0.0))
        orientation = pose.get("orientation") if isinstance(pose, dict) else None
        if isinstance(orientation, dict):
            try:
                quat = {
                    "x": float(orientation.get("x", 0.0)),
                    "y": float(orientation.get("y", 0.0)),
                    "z": float(orientation.get("z", 0.0)),
                    "w": float(orientation.get("w", 1.0)),
                }
            except Exception:
                quat = self._yaw_to_quaternion(math.radians(heading_deg))
        else:
            quat = self._yaw_to_quaternion(math.radians(heading_deg))

        if map_info is not None:
            resolution = max(1e-9, float(map_info.get("resolution", 0.05)))
            origin_x = float(map_info.get("origin_x", 0.0))
            origin_y = float(map_info.get("origin_y", 0.0))
            start_col = (start_x - origin_x) / resolution
            start_row = (start_y - origin_y) / resolution
        else:
            start_col = 0.0
            start_row = 0.0

        first = self.current_path.points[0]
        prepend_point = {
            "index": 0,
            "row": float(start_row),
            "col": float(start_col),
            "x": float(start_x),
            "y": float(start_y),
            "path_type": str(first.get("path_type", "start_pose") or "start_pose"),
            "point_type": "start",
            "timestamp": float(time.time()),
            "orientation": quat,
        }
        self.current_path.points.insert(0, prepend_point)
        for idx, item in enumerate(self.current_path.points):
            item["index"] = int(idx)

        first_x = float(first.get("x", start_x))
        first_y = float(first.get("y", start_y))
        self.current_path.length_m = float(self.current_path.length_m) + math.hypot(first_x - start_x, first_y - start_y)

        if self.current_path.nav_path is not None:
            nav_pose = PoseStamped()
            nav_pose.header.frame_id = self.current_path.nav_path.header.frame_id or (map_info or {}).get("frame_id", "map")
            nav_pose.pose.position.x = float(start_x)
            nav_pose.pose.position.y = float(start_y)
            nav_pose.pose.orientation.x = float(quat["x"])
            nav_pose.pose.orientation.y = float(quat["y"])
            nav_pose.pose.orientation.z = float(quat["z"])
            nav_pose.pose.orientation.w = float(quat["w"])
            self.current_path.nav_path.poses.insert(0, nav_pose)

        rospy.loginfo(
            "Prepended start pose point: x=%.3f y=%.3f heading=%.1fdeg path_points=%d",
            start_x,
            start_y,
            heading_deg,
            len(self.current_path.points),
        )

    def _resolve_point_orientation(self, point, index, cached_xy):
        ori = point.get("orientation") if isinstance(point, dict) else None
        if isinstance(ori, dict):
            try:
                return {
                    "x": float(ori.get("x", 0.0)),
                    "y": float(ori.get("y", 0.0)),
                    "z": float(ori.get("z", 0.0)),
                    "w": float(ori.get("w", 1.0)),
                }
            except Exception:
                pass

        # For straight-segment endpoints, keep heading aligned with the current segment
        # (previous point -> current point), not the next segment.
        try:
            if self._goal_points:
                for gp in self._goal_points:
                    if int(gp.get("path_index", -1)) == int(index):
                        if index > 0:
                            cx, cy = cached_xy[index]
                            px, py = cached_xy[index - 1]
                            yaw = math.atan2(cy - py, cx - px)
                            return self._yaw_to_quaternion(yaw)
                        break
        except Exception:
            pass

        if index + 1 < len(cached_xy):
            nx, ny = cached_xy[index + 1]
            cx, cy = cached_xy[index]
            yaw = math.atan2(ny - cy, nx - cx)
        elif index > 0:
            cx, cy = cached_xy[index]
            px, py = cached_xy[index - 1]
            yaw = math.atan2(cy - py, cx - px)
        else:
            yaw = 0.0
        return self._yaw_to_quaternion(yaw)

    def _resolve_straight_segment_end_index(self, start_index):
        if self.current_path is None or not self.current_path.points:
            return 0
        points = self.current_path.points
        n = len(points)
        if start_index >= n - 1:
            return n - 1

        threshold_rad = math.radians(float(self._goal_segment_yaw_threshold_deg))
        seg_type = str(points[start_index].get("path_type", "") or "")

        def _dir_angle(i0, i1):
            x0 = float(points[i0].get("x", 0.0))
            y0 = float(points[i0].get("y", 0.0))
            x1 = float(points[i1].get("x", 0.0))
            y1 = float(points[i1].get("y", 0.0))
            return math.atan2(y1 - y0, x1 - x0), math.hypot(x1 - x0, y1 - y0)

        base_angle = None
        end_index = start_index
        for idx in range(start_index, n - 1):
            cur_type = str(points[idx].get("path_type", "") or "")
            nxt_type = str(points[idx + 1].get("path_type", "") or "")
            if cur_type != seg_type or nxt_type != seg_type:
                break
            angle, seg_len = _dir_angle(idx, idx + 1)
            if seg_len < 1e-6:
                end_index = idx + 1
                continue
            if base_angle is None:
                base_angle = angle
                end_index = idx + 1
                continue
            diff = math.atan2(math.sin(angle - base_angle), math.cos(angle - base_angle))
            if abs(diff) > threshold_rad:
                break
            end_index = idx + 1
        return max(start_index, min(n - 1, end_index))

    def _segment_length(self, cached_xy, start_idx, end_idx):
        length = 0.0
        for idx in range(max(0, int(start_idx)), max(0, int(end_idx))):
            x0, y0 = cached_xy[idx]
            x1, y1 = cached_xy[idx + 1]
            length += math.hypot(x1 - x0, y1 - y0)
        return length

    def _segment_yaw(self, cached_xy, start_idx, end_idx):
        start_idx = max(0, int(start_idx))
        end_idx = max(start_idx, int(end_idx))
        for idx in range(start_idx, end_idx):
            x0, y0 = cached_xy[idx]
            x1, y1 = cached_xy[idx + 1]
            if math.hypot(x1 - x0, y1 - y0) > 1e-6:
                return math.atan2(y1 - y0, x1 - x0)
        if end_idx > start_idx:
            x0, y0 = cached_xy[start_idx]
            x1, y1 = cached_xy[end_idx]
            return math.atan2(y1 - y0, x1 - x0)
        return None

    def _segment_orientation(self, cached_xy, start_idx, end_idx):
        yaw = self._segment_yaw(cached_xy, start_idx, end_idx)
        if yaw is None:
            yaw = 0.0
        return self._yaw_to_quaternion(yaw)

    def _angle_abs_diff(self, a, b):
        return abs(math.atan2(math.sin(float(a) - float(b)), math.cos(float(a) - float(b))))

    def _segments_parallel_or_opposite(self, a, b):
        diff = self._angle_abs_diff(a, b)
        threshold = math.radians(float(self._corner_mid_parallel_threshold_deg))
        return diff <= threshold or abs(math.pi - diff) <= threshold

    def _segment_is_turn_between_rows(self, cur_yaw, ref_yaw):
        diff = self._angle_abs_diff(cur_yaw, ref_yaw)
        away_from_parallel = min(diff, abs(math.pi - diff))
        return away_from_parallel >= math.radians(float(self._corner_mid_turn_min_angle_deg))

    def _should_insert_corner_mid(self, segments, seg_pos):
        if not bool(self._corner_mid_enabled):
            return False
        if seg_pos <= 0 or seg_pos >= len(segments) - 1:
            return False
        prev_seg = segments[seg_pos - 1]
        cur_seg = segments[seg_pos]
        next_seg = segments[seg_pos + 1]
        cur_type = str(cur_seg.get("path_type", "") or "").strip().lower()
        if cur_type == "connection":
            return False
        if str(prev_seg.get("path_type", "") or "").strip().lower() == "connection":
            return False
        if str(next_seg.get("path_type", "") or "").strip().lower() == "connection":
            return False
        if cur_seg["end"] <= cur_seg["start"]:
            return False
        cur_len = float(cur_seg.get("length", 0.0))
        prev_len = float(prev_seg.get("length", 0.0))
        next_len = float(next_seg.get("length", 0.0))
        if cur_len < self._corner_mid_min_length:
            return False
        if cur_len >= min(prev_len, next_len) * self._corner_mid_short_ratio:
            return False
        if cur_seg.get("yaw") is None or prev_seg.get("yaw") is None or next_seg.get("yaw") is None:
            return False
        if not self._segments_parallel_or_opposite(prev_seg["yaw"], next_seg["yaw"]):
            return False
        return (
            self._segment_is_turn_between_rows(cur_seg["yaw"], prev_seg["yaw"])
            and self._segment_is_turn_between_rows(cur_seg["yaw"], next_seg["yaw"])
        )

    def _append_goal_point(self, points, cached_xy, path_index, goal_kind="segment", orientation=None):
        path_index = max(0, min(len(points) - 1, int(path_index)))
        if self._goal_points and int(self._goal_points[-1].get("path_index", -1)) == path_index:
            return False
        point = points[path_index]
        quat = orientation if isinstance(orientation, dict) else self._resolve_point_orientation(point, path_index, cached_xy)
        self._goal_points.append(
            {
                "path_index": int(path_index),
                "x": float(point.get("x", 0.0)),
                "y": float(point.get("y", 0.0)),
                "path_type": str(point.get("path_type", "") or ""),
                "goal_kind": str(goal_kind or "segment"),
                "orientation": quat,
            }
        )
        return True

    def _rebuild_goal_points(self):
        self._goal_points = []
        if self.current_path is None or not self.current_path.points:
            return
        points = self.current_path.points
        n = len(points)
        cursor = 0
        cached_xy = [(float(p.get("x", 0.0)), float(p.get("y", 0.0))) for p in points]
        segments = []
        while cursor < n:
            end_idx = self._resolve_straight_segment_end_index(cursor)
            end_idx = max(cursor, min(n - 1, int(end_idx)))
            # Publish the penultimate point of each straight segment by default.
            # If the segment has only one point, fall back to the segment end.
            goal_idx = end_idx - 1 if end_idx > cursor else end_idx
            goal_idx = max(cursor, min(n - 1, int(goal_idx)))
            segments.append(
                {
                    "start": int(cursor),
                    "end": int(end_idx),
                    "goal": int(goal_idx),
                    "path_type": str(points[goal_idx].get("path_type", "") or ""),
                    "length": self._segment_length(cached_xy, cursor, end_idx),
                    "yaw": self._segment_yaw(cached_xy, cursor, end_idx),
                }
            )
            if end_idx >= n - 1:
                break
            cursor = end_idx + 1

        for pos, segment in enumerate(segments):
            if self._should_insert_corner_mid(segments, pos):
                mid_idx = int((int(segment["start"]) + int(segment["end"])) // 2)
                self._append_goal_point(
                    points,
                    cached_xy,
                    mid_idx,
                    goal_kind="corner_mid",
                    orientation=self._segment_orientation(cached_xy, segment["start"], segment["end"]),
                )
            self._append_goal_point(points, cached_xy, segment["goal"], goal_kind="segment")
        # Do not force append the final point here: goal points are intentionally
        # chosen as segment-penultimate points per current execution strategy.
        corner_mid_count = sum(1 for item in self._goal_points if item.get("goal_kind") == "corner_mid")
        rospy.loginfo("Precomputed goal points: count=%d corner_mid=%d", len(self._goal_points), corner_mid_count)

    def _init_path_execution_cursor(self):
        if self.current_path is None or not self.current_path.points:
            self._exec_goal_index = 0
            return
        if not self._goal_points:
            self._rebuild_goal_points()
        pose = self.aurora_bridge.get_pose()
        nearest_index = 0
        nearest_distance = float("inf")
        for index, point in enumerate(self.current_path.points):
            distance = math.hypot(float(point["x"]) - pose["x"], float(point["y"]) - pose["y"])
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_index = index
        self.current_path_index = nearest_index
        if not self._goal_points:
            self._exec_goal_index = min(nearest_index + 1, len(self.current_path.points) - 1)
            return

        # Do not publish the initial point as execution goal:
        # choose the first precomputed goal strictly ahead of current nearest index.
        target_goal_index = -1
        for i, item in enumerate(self._goal_points):
            if int(item.get("path_index", -1)) > int(nearest_index):
                target_goal_index = i
                break
        if target_goal_index < 0:
            # Fallback: if already near the final point, keep the last goal.
            target_goal_index = len(self._goal_points) - 1
        self._exec_goal_index = max(0, min(target_goal_index, len(self._goal_points) - 1))

    def _dispatch_current_goal(self, force=False):
        if not self._exec_active or self.current_path is None or not self.current_path.points:
            return
        if not self._goal_points:
            self._rebuild_goal_points()
        if not self._goal_points:
            return

        if force and bool(self._exec_publish_start_pose_once):
            self._publish_current_position_goal_once()
            self._exec_publish_start_pose_once = False

        # Before publishing, skip goals that are already reached.
        # This prevents publishing a near/initial point first.
        try:
            pose = self.aurora_bridge.get_pose()
            while 0 <= self._exec_goal_index < (len(self._goal_points) - 1):
                goal_probe = self._goal_points[self._exec_goal_index]
                dist_probe = math.hypot(
                    float(goal_probe.get("x", 0.0)) - float(pose.get("x", 0.0)),
                    float(goal_probe.get("y", 0.0)) - float(pose.get("y", 0.0)),
                )
                if dist_probe > float(self._exec_goal_reach_dist):
                    break
                self._exec_goal_index += 1
                self._exec_goal_start_time = time.time()
        except Exception:
            pass

        now = time.time()
        if (not force) and (now - self._exec_last_send_time < self._exec_goal_interval):
            return
        self._exec_last_send_time = now
        if force:
            self._publish_execution_goal_by_index(self._exec_goal_index, reason="exec_goal_step")
        goal = self._goal_points[self._exec_goal_index]
        rospy.loginfo_throttle(
            2.0,
            "Execution cursor update: idx=%d x=%.3f y=%.3f force=%s",
            int(goal.get("path_index", self._exec_goal_index)),
            float(goal.get("x", 0.0)),
            float(goal.get("y", 0.0)),
            str(bool(force)),
        )

    def _publish_execution_goal_by_index(self, target_index, reason=""):
        if self.current_path is None or not self.current_path.points:
            return
        if not self._goal_points:
            self._rebuild_goal_points()
        if not self._goal_points:
            return
        if target_index < 0 or target_index >= len(self._goal_points):
            return
        goal = self._goal_points[int(target_index)]
        path_index = int(goal.get("path_index", 0))
        if path_index < 0 or path_index >= len(self.current_path.points):
            return
        endpoint = self.current_path.points[path_index]
        self._apply_disc_state_for_path_point(endpoint)
        quat = goal.get("orientation") if isinstance(goal, dict) else None
        if not isinstance(quat, dict):
            cached_xy = [(float(p.get("x", 0.0)), float(p.get("y", 0.0))) for p in self.current_path.points]
            quat = self._resolve_point_orientation(endpoint, path_index, cached_xy)
        self.goal_pub.publish(self._build_navigation_goal_msg(endpoint, path_index, quat))

    def _apply_disc_state_for_path_point(self, path_point):
        if not bool(self._disc_follow_path_type):
            return
        if not isinstance(path_point, dict):
            return
        path_type = str(path_point.get("path_type", "") or "").strip().lower()
        # connection: transition path between regions -> optional low-speed disc rotation
        # others: in-region coverage path -> disc on, with cmd_vel idle guard.
        target_mode = "transition" if path_type == "connection" else "cover"
        if target_mode == self._disc_last_mode:
            return
        if target_mode == "transition":
            self._disc_auto_cover_desired = False
            self._reset_disc_motion_guard()
            self._publish_disc_travel_state(True, reason="connection_path")
        else:
            self._disc_auto_cover_desired = True
            self._reset_disc_motion_guard()
            self.disc_speed_pub.publish(Int16(data=self._configured_disc_speed_rpm()))
            if not self._disc_motion_guard_stopped:
                self.disc_enable_pub.publish(Bool(data=True))
        self._disc_last_mode = target_mode
        rospy.loginfo(
            "Disc mode switched by path_type: mode=%s path_type=%s",
            target_mode,
            path_type or "<empty>",
        )

    def _publish_current_position_goal_once(self):
        if self.current_path is None or not self.current_path.points:
            return
        pose = self.aurora_bridge.get_pose()
        nearest_index = 0
        nearest_distance = float("inf")
        for index, point in enumerate(self.current_path.points):
            distance = math.hypot(float(point.get("x", 0.0)) - pose["x"], float(point.get("y", 0.0)) - pose["y"])
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_index = index
        self.current_path_index = nearest_index
        endpoint = self.current_path.points[nearest_index]
        cached_xy = [(float(p.get("x", 0.0)), float(p.get("y", 0.0))) for p in self.current_path.points]
        quat = self._resolve_point_orientation(endpoint, int(nearest_index), cached_xy)
        self.goal_pub.publish(self._build_navigation_goal_msg(endpoint, int(nearest_index), quat))

    def _advance_goal_or_finish(self):
        if self.current_path is None or not self.current_path.points:
            return
        if not self._goal_points:
            self._rebuild_goal_points()
        if not self._goal_points:
            return
        if self._exec_goal_index >= len(self._goal_points) - 1:
            self._mark_current_region_repeat_done()
            if self._try_advance_to_next_region():
                return
            self._exec_active = False
            self._disc_auto_cover_desired = False
            self._reset_disc_motion_guard()
            self._set_cmd_vel_forward_runtime_active(False, publish_zero=True, reason="task_complete")
            self.disc_enable_pub.publish(Bool(data=False))
            self.light_pub.publish(Bool(data=False))
            self._safe_stop_motion()
            self._exec_region_order = []
            self._exec_region_index = -1
            self.state = SchedulerState.COMPLETED
            self._task_stop_reason = "completed"
            self._finalize_task_result(stop_reason="completed")
            rospy.loginfo("Path execution completed: task_id=%s path_version=%s", self.current_path.task_id, self.current_path.path_version)
            return
        self._exec_goal_index += 1
        self._exec_goal_start_time = time.time()
        prev_goal = self._goal_points[max(0, self._exec_goal_index - 1)]
        self.current_path_index = max(
            self.current_path_index,
            int(prev_goal.get("path_index", self.current_path_index)),
        )
        self._dispatch_current_goal(force=True)

    def _try_advance_to_next_region(self):
        if self._current_plan_scope == "all":
            return False
        self._sync_task_regions_from_overlay()
        region_order = self._sorted_work_region_ids()
        if not region_order or len(region_order) <= 1:
            return False
        current_id = (self.task_config.active_work_region_id or "").strip()
        if current_id in region_order:
            current_idx = region_order.index(current_id)
        else:
            current_idx = -1
        next_idx = current_idx + 1
        if next_idx >= len(region_order):
            return False
        next_region_id = region_order[next_idx]

        # Inter-region transfer uses the optional low-speed travel rotation.
        self._disc_auto_cover_desired = False
        self._reset_disc_motion_guard()
        self._publish_disc_travel_state(True, reason="inter_region_transfer")
        self.task_config.active_work_region_id = next_region_id
        rospy.loginfo(
            "Switching to next work region: from=%s to=%s (%d/%d)",
            current_id or "<none>",
            next_region_id,
            next_idx + 1,
            len(region_order),
        )
        if not self._plan_current_task():
            rospy.logerr("Failed to plan next work region: %s", next_region_id)
            self._set_error("Planning failed for next work region: {}".format(next_region_id))
            return False
        if self.current_path is None or not self.current_path.points:
            rospy.logerr("Planned path is empty for next work region: %s", next_region_id)
            self._set_error("Empty path for next work region: {}".format(next_region_id))
            return False

        # Enter next region: enable working tool again.
        self._set_chassis_enabled(True)
        self.disc_speed_pub.publish(Int16(data=self._configured_disc_speed_rpm()))
        self.disc_enable_pub.publish(Bool(data=True))
        self._disc_auto_cover_desired = True
        self._reset_disc_motion_guard()
        self.light_pub.publish(Bool(data=True))
        self._exec_region_order = region_order
        self._exec_region_index = next_idx
        self._init_segment_execution_cursor()
        self._exec_active = True
        self._exec_last_send_time = 0.0
        self._exec_goal_start_time = time.time()
        self._publish_active_segment_plan(reason="next_region")
        self._send_active_segment_goal(force=True, reason="next_region")
        self.state = SchedulerState.RUNNING
        return True

    def _tick_path_execution(self):
        if self.state != SchedulerState.RUNNING:
            return
        if not self._exec_active:
            return
        if self.current_path is None or not self.current_path.points:
            self._exec_active = False
            return
        if not self._active_segments:
            self._rebuild_active_segments()
        if not self._active_segments:
            self._exec_active = False
            return
        self._active_segment_index = max(
            0, min(int(self._active_segment_index), len(self._active_segments) - 1)
        )
        now = time.time()
        projection = self._project_robot_to_active_segment()
        observed_progress_s = float(projection.get("progress_s", 0.0))
        progress_s = max(float(self._active_segment_last_progress_s), observed_progress_s)
        if progress_s > float(self._active_segment_stall_progress_s) + float(self._active_segment_progress_epsilon_m):
            self._active_segment_stall_progress_s = progress_s
            self._active_segment_last_progress_update_time = now
        self._active_segment_last_progress_s = progress_s
        self.current_path_index = max(self.current_path_index, int(projection.get("index", self.current_path_index)))
        if 0 <= self.current_path_index < len(self.current_path.points):
            self._apply_disc_state_for_path_point(self.current_path.points[self.current_path_index])

        path_s = self._ensure_path_arc_lengths()
        final_point = self.current_path.points[-1]
        pose = self.aurora_bridge.get_pose()
        final_dist = math.hypot(
            float(final_point.get("x", 0.0)) - float(pose.get("x", 0.0)),
            float(final_point.get("y", 0.0)) - float(pose.get("y", 0.0)),
        )
        is_last_segment = self._active_segment_index >= len(self._active_segments) - 1
        if is_last_segment:
            remaining_final_s = float(path_s[-1]) - progress_s if path_s else final_dist
            if final_dist <= self._exec_goal_reach_dist or remaining_final_s <= self._exec_goal_reach_dist:
                self.current_path_index = len(self.current_path.points) - 1
                self._finish_path_execution(stop_reason="completed")
                return
            if self._active_segment_goal_sent_index != self._active_segment_index:
                self._send_active_segment_goal(force=True, reason="last_segment_goal_missing")
            else:
                self._refresh_active_segment_goal_if_stalled(now, reason="last_segment_stalled_refresh")
            return

        segment = self._active_segments[self._active_segment_index]
        segment_start_s = float(segment.get("start_s", 0.0))
        segment_end_s = float(segment.get("end_s", segment_start_s))
        segment_len = max(0.0, segment_end_s - segment_start_s)
        next_segment = self._active_segments[self._active_segment_index + 1]
        next_segment_start_s = float(next_segment.get("start_s", segment_end_s))
        should_switch = False
        if segment_len <= float(self._segment_switch_distance_m):
            ratio = 1.0 if segment_len <= 1e-6 else (progress_s - segment_start_s) / segment_len
            should_switch = ratio >= float(self._segment_short_progress_ratio)
        else:
            should_switch = (segment_end_s - progress_s) <= float(self._segment_switch_distance_m)
        if progress_s >= segment_end_s - 1e-3:
            should_switch = True
        if progress_s < next_segment_start_s - 1e-3:
            # Keep TEB on the current lane until the next segment can start from
            # the robot's vicinity; the effective early-switch margin is lead-in.
            should_switch = False
        if should_switch:
            self._switch_to_next_active_segment(projection)
            return
        if self._active_segment_goal_sent_index != self._active_segment_index:
            self._send_active_segment_goal(force=True, reason="segment_goal_missing")
        else:
            self._refresh_active_segment_goal_if_stalled(now)

    def _set_chassis_enabled(self, enabled):
        # Requirement update:
        # chassis enable/disable is no longer controlled by task flow.
        # Keep this function as a no-op for backward compatibility.
        # If re-enabled in future, call with field name `enable` (not `enabled`):
        #   self.enable_service(enable=bool(enabled))
        rospy.loginfo_throttle(
            2.0,
            "Skip chassis enable control by scheduler (requested enabled=%s).",
            str(bool(enabled)),
        )

    def _configured_disc_speed_rpm(self):
        return max(
            0,
            min(32767, int(self._chassis_settings.get("disc_speed_rpm", 1200))),
        )

    def _publish_disc_travel_state(self, moving, reason=""):
        enabled = bool(moving) and bool(self._disc_travel_spin_enabled)
        if enabled:
            self.disc_speed_pub.publish(Int16(data=int(self._disc_travel_speed_rpm)))
        self.disc_enable_pub.publish(Bool(data=enabled))
        rospy.loginfo(
            "Disc travel state: enabled=%s speed_rpm=%d reason=%s",
            enabled,
            int(self._disc_travel_speed_rpm) if enabled else 0,
            reason or "unspecified",
        )

    def _set_manual_travel_disc_active(self, active, reason=""):
        active = bool(active) and bool(self._disc_travel_spin_enabled)
        if active == self._manual_travel_disc_active:
            return
        self._manual_travel_disc_active = active
        self._publish_disc_travel_state(active, reason=reason)

    def _update_progress(self):
        if self.current_path is None or not self.current_path.points:
            return
        if self.state == SchedulerState.READY:
            self.current_path_index = 0
            return
        pose = self.aurora_bridge.get_pose()
        if self.state == SchedulerState.RUNNING and self._active_segments:
            projection = self._project_robot_to_active_segment()
            self.current_path_index = max(self.current_path_index, int(projection.get("index", self.current_path_index)))
            final_point = self.current_path.points[-1]
            final_dist = math.hypot(
                float(final_point.get("x", 0.0)) - float(pose.get("x", 0.0)),
                float(final_point.get("y", 0.0)) - float(pose.get("y", 0.0)),
            )
            if (
                self._active_segment_index >= len(self._active_segments) - 1
                and final_dist <= self._exec_goal_reach_dist
            ):
                self.current_path_index = len(self.current_path.points) - 1
                self._finish_path_execution(stop_reason="completed_progress")
            return
        nearest_index = 0
        nearest_distance = float("inf")
        for index, point in enumerate(self.current_path.points):
            distance = math.hypot(point["x"] - pose["x"], point["y"] - pose["y"])
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_index = index
        self.current_path_index = nearest_index
        if self.state == SchedulerState.RUNNING and nearest_index >= len(self.current_path.points) - 1:
            self._finish_path_execution(stop_reason="completed_progress")

    def _publish_status(self):
        pose = self.aurora_bridge.get_pose()
        message = SchedulerStatus()
        message.header.stamp = rospy.Time.now()
        message.task_id = self.task_config.task_id
        message.state = self.state.value
        message.progress = self._task_progress()
        map_info = self.map_service.get_map_info()
        message.map_version = map_info["map_version"] if map_info else 0
        message.map_available = map_info is not None
        message.stream_online = self.media_streamer.get_state().online
        message.replan_requested = self.replan_requested
        message.last_error = self.last_error
        message.pose.x = pose["x"]
        message.pose.y = pose["y"]
        message.pose.theta = math.radians(pose["heading_deg"])
        message.path_point_count = len(self.current_path.points) if self.current_path else 0
        self.status_pub.publish(message)

    def _mqtt_status_snapshot(self):
        pose = self.aurora_bridge.get_pose()
        map_info = self.map_service.get_map_info() or {}
        progress = max(0.0, min(1.0, float(self._task_progress())))
        total_area = float(self._total_work_area_m2())
        chassis = self.last_chassis_status
        current_map_id = self._current_map_id()
        map_record = self._find_recorded_map_by_id(current_map_id)
        map_name = str(map_record.get("name", "") or "") if isinstance(map_record, dict) else ""
        wheel_odom = self._wheel_odom_snapshot()
        linear_speed = float(wheel_odom["linear_mps"])
        angular_speed = float(wheel_odom["angular_radps"])
        collision_imminent = self._collision_imminent_snapshot()
        radar_status = self._radar_system_status_snapshot()
        return {
            "projectId": self.platform_file_sync.get_project_id(),
            "taskId": str(self.task_config.task_id or ""),
            "taskState": str(self.state.value),
            "taskProgress": round(progress * 100.0, 2),
            "taskMessage": str(self.last_error or self.state.value),
            "mapId": str(current_map_id or ""),
            "mapName": map_name,
            "mapVersion": int(map_info.get("map_version", 0) or 0),
            "poseX": float(pose.get("x", 0.0) or 0.0),
            "poseY": float(pose.get("y", 0.0) or 0.0),
            "poseHeading": float(pose.get("heading_deg", 0.0) or 0.0),
            "poseAvailable": bool(pose.get("odom_available", False)),
            "localizationQualityAvailable": bool(
                pose.get("localization_quality_available", False)
            ),
            "localizationQuality": int(pose.get("localization_quality", 0) or 0),
            "collisionImminent": collision_imminent,
            "radarSystemStatusAvailable": bool(radar_status["available"]),
            "radarSystemStatus": str(radar_status["status"]),
            "wheelOdomAvailable": bool(wheel_odom["available"]),
            "linearSpeed": linear_speed,
            "angularSpeed": angular_speed,
            "vehicleState": "moving"
            if abs(linear_speed) > 0.01 or abs(angular_speed) > 0.01
            else "stopped",
            "chassisConnected": bool(chassis.connected) if chassis is not None else False,
            "chassisEnabled": bool(chassis.enabled) if chassis is not None else False,
            "chassisWorkMode": int(chassis.work_mode) if chassis is not None else 0,
            "discEnabled": bool(chassis.disc_enabled) if chassis is not None else False,
            "discSpeed": int(chassis.disc_speed_feedback) if chassis is not None else 0,
            "discLiftState": int(chassis.disc_lift_state) if chassis is not None else 0,
            "lightEnabled": bool(chassis.light_enabled) if chassis is not None else False,
            "totalWorkArea": total_area,
            "finishedWorkArea": total_area * progress,
            "remainingWorkArea": max(0.0, total_area * (1.0 - progress)),
            "remainingTime": float(self._estimate_remaining_time_s(progress)),
            "pathPointCount": len(self.current_path.points) if self.current_path else 0,
            "currentPathPointIndex": int(self.current_path_index),
            "lastError": str(self.last_error or ""),
        }

    def _publish_diagnostics(self):
        video_state = self.media_streamer.get_state()
        local_stream_state = self.local_stream_server.get_state()
        local_stream_urls = self.local_stream_server.get_stream_urls()
        status = DiagnosticStatus()
        status.name = "grinder_scheduler"
        status.hardware_id = "scheduler"
        rtsp_available = self.local_stream_server.ffmpeg_available()
        mediamtx_available = self.local_stream_server.mediamtx_available()
        if self.last_error == "" and rtsp_available and mediamtx_available:
            status.level = DiagnosticStatus.OK
            status.message = "ok"
        elif self.last_error == "" and (not rtsp_available or not mediamtx_available):
            status.level = DiagnosticStatus.WARN
            if not rtsp_available:
                status.message = "ffmpeg_not_installed"
            else:
                status.message = "mediamtx_not_installed"
        else:
            status.level = DiagnosticStatus.WARN
            status.message = self.last_error
        status.values = [
            KeyValue(key="state", value=self.state.value),
            KeyValue(key="task_id", value=self.task_config.task_id),
            KeyValue(key="map_available", value=str(self.map_service.has_map())),
            KeyValue(key="stream_online", value=str(video_state.online)),
            KeyValue(key="stream_url", value=video_state.stream_url),
            KeyValue(key="local_stream_online", value=str(local_stream_state.online)),
            KeyValue(key="local_stream_url", value=local_stream_state.stream_url),
            KeyValue(key="local_rtsp_left", value=local_stream_urls["left"]),
            KeyValue(key="local_rtsp_right", value=local_stream_urls["right"]),
            KeyValue(key="local_rtsp_ffmpeg_available", value=str(rtsp_available)),
            KeyValue(key="local_rtsp_mediamtx_available", value=str(mediamtx_available)),
            KeyValue(key="local_rtsp_server_running", value=str(self.local_stream_server.server_running())),
        ]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status = [status]
        self.diagnostics_pub.publish(array)

    def _task_progress(self):
        if self.current_path is None or not self.current_path.points:
            return 0.0
        return float(self.current_path_index) / float(max(1, len(self.current_path.points) - 1))

    def _polygon_area_m2(self, points):
        if not points or len(points) < 3:
            return 0.0
        area2 = 0.0
        count = len(points)
        for i in range(count):
            p1 = points[i]
            p2 = points[(i + 1) % count]
            x1 = float(p1.get("x", 0.0))
            y1 = float(p1.get("y", 0.0))
            x2 = float(p2.get("x", 0.0))
            y2 = float(p2.get("y", 0.0))
            area2 += (x1 * y2 - x2 * y1)
        return abs(area2) * 0.5

    def _total_work_area_m2(self):
        total = 0.0
        for region in list(self.task_config.work_regions or []):
            if not isinstance(region, dict):
                continue
            points = region.get("points", []) or []
            if len(points) < 3:
                continue
            total += self._polygon_area_m2(points)
        return max(0.0, total)

    def _record_progress_sample(self, progress):
        now = time.time()
        p = max(0.0, min(1.0, float(progress)))
        self._progress_history.append((now, p))
        cutoff = now - 120.0
        while len(self._progress_history) >= 2 and self._progress_history[0][0] < cutoff:
            self._progress_history.popleft()

    def _estimate_remaining_time_s(self, progress):
        p = max(0.0, min(1.0, float(progress)))
        if p >= 0.999:
            return 0.0
        if len(self._progress_history) < 2:
            return -1.0
        t0, p0 = self._progress_history[0]
        t1, p1 = self._progress_history[-1]
        dt = max(0.0, float(t1 - t0))
        dp = float(p1 - p0)
        if dt < 3.0 or dp <= 1e-4:
            return -1.0
        rate = dp / dt
        if rate <= 1e-6:
            return -1.0
        remaining = max(0.0, 1.0 - p)
        return remaining / rate

    def _estimate_plan_time_s(self, path_length_m):
        try:
            length = max(0.0, float(path_length_m))
            configured_speed_limit = max(
                0.0,
                float(self._chassis_settings.get("run_speed", 0.0)),
            )
            if configured_speed_limit <= 0.0:
                return -1.0
            # Use ~70% of the persisted SettingsWrite run-speed limit to
            # account for turns and slowdown during coverage execution.
            effective_speed = configured_speed_limit * 0.7
            return length / effective_speed
        except Exception:
            return -1.0

    def _compute_saved_map_metrics(self):
        """Compute map-level task metadata persisted with saved map records."""
        total_area_m2 = float(self._total_work_area_m2())
        estimated_time_s = -1.0
        if self.current_path is not None:
            try:
                estimated_time_s = float(self._estimate_plan_time_s(self.current_path.length_m))
            except Exception:
                estimated_time_s = -1.0
        return total_area_m2, estimated_time_s

    def _compute_saved_region_metrics(self, total_estimated_time_s):
        metrics = []
        regions = list(self.task_config.work_regions or [])
        if not regions:
            return metrics
        per_region_area = []
        total_area_m2 = 0.0
        for region in regions:
            if not isinstance(region, dict):
                continue
            rid = str(region.get("region_id", "") or "").strip()
            name = str(region.get("name", "") or rid)
            points = list(region.get("points", []) or [])
            area = float(self._polygon_area_m2(points))
            repeat = 1
            if rid:
                try:
                    repeat = max(1, int((self.task_config.region_repeat_config or {}).get(rid, 1)))
                except Exception:
                    repeat = 1
            effective_area = max(0.0, area) * float(repeat)
            if effective_area <= 0.0:
                continue
            per_region_area.append((rid, name, repeat, effective_area))
            total_area_m2 += effective_area
        if total_area_m2 <= 0.0:
            return metrics

        for rid, name, repeat, area_m2 in per_region_area:
            if total_estimated_time_s is not None and float(total_estimated_time_s) >= 0.0:
                est_h = (float(total_estimated_time_s) * (area_m2 / total_area_m2)) / 3600.0
            else:
                est_h = -1.0
            metrics.append(
                {
                    "region_id": rid,
                    "region_name": name,
                    "repeat": int(repeat),
                    "area_m2": float(area_m2),
                    "estimated_time_h": float(est_h),
                }
            )
        return metrics

    def _build_saved_map_thumbnail(self):
        """Return (image_format, image_b64, width, height) for saved-map metadata."""
        try:
            snapshot = self.map_service.create_preview(
                self.aurora_bridge.get_pose(),
                int(self._saved_map_thumb_max_edge),
                "jpg",
                True,
                **self._map_preview_alignment_kwargs(self._current_map_id())
            )
            raw = bytes(snapshot.preview_data or b"")
            if not raw:
                return ("", "", 0, 0)
            arr = np.frombuffer(raw, dtype=np.uint8)
            image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if image is None:
                return ("", "", 0, 0)
            ok, enc = cv2.imencode(
                ".jpg",
                image,
                [int(cv2.IMWRITE_JPEG_QUALITY), int(self._saved_map_thumb_jpeg_quality)],
            )
            if not ok:
                return ("", "", 0, 0)
            encoded = bytes(enc.tobytes())
            return (
                "jpg",
                base64.b64encode(encoded).decode("ascii"),
                int(image.shape[1]),
                int(image.shape[0]),
            )
        except Exception as exc:
            rospy.logwarn("Failed to build saved-map thumbnail: %s", exc)
            return ("", "", 0, 0)

    def _set_error(self, message):
        self.last_error = message
        self._mark_current_region_repeat_done()
        self._set_cmd_vel_forward_runtime_active(False, publish_zero=True, reason="scheduler_error")
        self.state = SchedulerState.ERROR
        self._task_stop_reason = str(message or "error")
        self._finalize_task_result(stop_reason=self._task_stop_reason)
        rospy.logerr("Scheduler entered ERROR: %s", message)

    def _task_state_to_pb(self):
        pb = self.sl_link_server.pb
        mapping = {
            SchedulerState.IDLE: pb.TASK_STATE_IDLE,
            SchedulerState.READY: pb.TASK_STATE_READY,
            SchedulerState.PLANNING: pb.TASK_STATE_PLANNING,
            SchedulerState.RUNNING: pb.TASK_STATE_RUNNING,
            SchedulerState.PAUSED: pb.TASK_STATE_PAUSED,
            SchedulerState.COMPLETED: pb.TASK_STATE_COMPLETED,
            SchedulerState.STOPPED: pb.TASK_STATE_STOPPED,
            SchedulerState.ERROR: pb.TASK_STATE_ERROR,
        }
        return mapping[self.state]

    def build_device_status_report(self):
        pb = self.sl_link_server.pb
        report = pb.DeviceStatusReport()
        report.utc_time = int(time.time())
        report.system_status = pb.SYS_STATUS_ERROR if self.last_error else pb.SYS_STATUS_NORMAL
        report.wifi_status = pb.WIFI_SUCCESS
        report.work_mode = pb.WORK_MODE_AUTO if self.state in (SchedulerState.RUNNING, SchedulerState.READY) else pb.WORK_MODE_MANUAL
        disc_speed_setting = max(0, int(self._configured_disc_speed_rpm()))
        if self.last_chassis_status is not None:
            report.disc_speed_rpm = disc_speed_setting
            report.disc_enabled = self.last_chassis_status.disc_enabled
            report.light_enabled = self.last_chassis_status.light_enabled
            report.chassis_enabled = self.last_chassis_status.enabled
        else:
            report.disc_speed_rpm = disc_speed_setting
        wheel_feedback = self._wheel_speed_feedback_snapshot()
        report.left_wheel_speed = float(wheel_feedback["left_mps"])
        report.right_wheel_speed = float(wheel_feedback["right_mps"])
        pose = self._pose_for_sl_link_report()
        report.position.x = pose["x"]
        report.position.y = pose["y"]
        report.position.heading_deg = pose["heading_deg"]
        if hasattr(report, "alignment_yaw_deg"):
            report.alignment_yaw_deg = float(self._alignment_yaw_deg_for_sl_link_report())
        self._apply_localization_covariance(report, pose)
        report.localization_quality_available = bool(
            pose.get("localization_quality_available", False)
        )
        report.localization_quality = int(pose.get("localization_quality", 0) or 0)
        if hasattr(report, "collision_imminent"):
            report.collision_imminent = self._collision_imminent_snapshot()
        if hasattr(report, "radar_system_status_available") or hasattr(report, "radar_system_status"):
            radar_status = self._radar_system_status_snapshot()
            if hasattr(report, "radar_system_status_available"):
                report.radar_system_status_available = bool(radar_status["available"])
            if hasattr(report, "radar_system_status"):
                report.radar_system_status = str(radar_status["status"])
        if hasattr(report, "vehicle_speed"):
            report.vehicle_speed = float(self._wheel_odom_snapshot()["linear_mps"])
        return report.SerializeToString(), pb.MSG_ID_DEVICE_STATUS_REPORT, pb.COMP_SYSTEM

    def build_task_status_report(self):
        pb = self.sl_link_server.pb
        report = pb.TaskStatusReport()
        report.task_id = self.task_config.task_id
        report.state = self._task_state_to_pb()
        progress = self._task_progress()
        self._record_progress_sample(progress)
        report.progress = progress
        map_info = self.map_service.get_map_info()
        report.map_version = map_info["map_version"] if map_info else 0
        report.message = self.last_error or self.state.value
        pose = self._pose_for_sl_link_report()
        report.position.x = pose["x"]
        report.position.y = pose["y"]
        report.position.heading_deg = pose["heading_deg"]
        if hasattr(report, "alignment_yaw_deg"):
            report.alignment_yaw_deg = float(self._alignment_yaw_deg_for_sl_link_report())
        self._apply_localization_covariance(report, pose)
        report.replan_requested = self.replan_requested
        report.path_point_count = len(self.current_path.points) if self.current_path else 0
        report.path_version = self.current_path.path_version if self.current_path else 0
        total_area = self._total_work_area_m2()
        remaining_area = max(0.0, total_area * (1.0 - progress))
        report.total_work_area_m2 = float(total_area)
        report.remaining_work_area_m2 = float(remaining_area)
        report.remaining_time_s = float(self._estimate_remaining_time_s(progress))
        region_id, repeat_index, repeat_total = self._current_region_repeat_progress()
        report.current_region_id = region_id
        report.current_region_repeat_index = int(repeat_index)
        report.current_region_repeat_total = int(repeat_total)
        return report.SerializeToString(), pb.MSG_ID_TASK_STATUS_REPORT, pb.COMP_SCHEDULER

    def _current_region_repeat_progress(self):
        active_id = str(self.task_config.active_work_region_id or "").strip()
        if not active_id:
            return "", 0, 0
        base_id = active_id
        repeat_index = 1
        if "__lap_" in active_id:
            prefix, _, suffix = active_id.rpartition("__lap_")
            if prefix:
                base_id = prefix
            try:
                repeat_index = max(1, int(suffix))
            except Exception:
                repeat_index = 1
        repeat_total = 1
        try:
            repeat_total = max(1, int((self.task_config.region_repeat_config or {}).get(base_id, 1)))
        except Exception:
            repeat_total = 1
        if repeat_index > repeat_total:
            repeat_index = repeat_total
        return base_id, repeat_index, repeat_total

    def _split_region_repeat(self, region_id_text):
        raw = str(region_id_text or "").strip()
        if not raw:
            return "", 1
        if "__lap_" not in raw:
            return raw, 1
        prefix, _, suffix = raw.rpartition("__lap_")
        base = prefix.strip() or raw
        try:
            lap = max(1, int(suffix))
        except Exception:
            lap = 1
        return base, lap

    def _mark_current_region_repeat_done(self):
        base_id, repeat_index, _ = self._current_region_repeat_progress()
        if not base_id:
            return
        old = int(self._exec_region_repeat_done.get(base_id, 0) or 0)
        if repeat_index > old:
            self._exec_region_repeat_done[base_id] = int(repeat_index)

    def _build_task_result_image(self):
        if self.current_path is None or not self.current_path.points:
            return "", b"", 0, 0
        try:
            alignment_kwargs = self._map_preview_alignment_kwargs()
            alignment_yaw = alignment_kwargs.get("alignment_yaw", None)
            snapshot = self._get_cached_preview_snapshot(
                self._planned_path_preview_max_edge,
                self._planned_path_preview_format,
                False,
                alignment_kwargs,
            )
            if snapshot is None:
                return "", b"", 0, 0
            image = snapshot.preview_image.copy() if snapshot.preview_image is not None else None
            if image is None:
                image = cv2.imdecode(np.frombuffer(snapshot.preview_data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                return "", b"", 0, 0
            render_map_info = {
                "width": int(snapshot.width),
                "height": int(snapshot.height),
                "origin_x": float(snapshot.origin_x),
                "origin_y": float(snapshot.origin_y),
                "resolution": float(snapshot.resolution),
                "alignment_yaw": alignment_yaw,
            }
            preview_h, preview_w = image.shape[:2]
            scale_x = float(preview_w) / float(max(1, render_map_info["width"]))
            scale_y = float(preview_h) / float(max(1, render_map_info["height"]))

            # Draw all work-region boundaries in task-result image for clearer context.
            for region in list(self.task_config.work_regions or []):
                if not isinstance(region, dict):
                    continue
                pts = list(region.get("points", []) or [])
                if len(pts) < 3:
                    continue
                poly = np.array(
                    self._map_points_to_preview_pixels(
                        pts,
                        render_map_info,
                        scale_x,
                        scale_y,
                        preview_w,
                        preview_h,
                    ),
                    dtype=np.int32,
                ).reshape((-1, 1, 2))
                cv2.polylines(image, [poly], True, (0, 200, 0), thickness=2, lineType=cv2.LINE_AA)

            path_points = list(self.current_path.points or [])
            if len(path_points) < 2:
                return "", b"", 0, 0
            path_pixels = self._map_points_to_preview_pixels(
                path_points,
                render_map_info,
                scale_x,
                scale_y,
                preview_w,
                preview_h,
            )
            visited_end = int(max(1, min(len(path_points) - 1, int(self.current_path_index or 0))))
            for i in range(1, visited_end + 1):
                p1 = path_points[i]
                seg_type = str(p1.get("path_type", "") or "")
                if seg_type == "connection":
                    # 区域间两点连接路径：固定青色
                    color = (255, 255, 0)
                else:
                    # 区域覆盖路径：按遍数配色
                    _, lap = self._split_region_repeat(seg_type)
                    color = self._task_result_palette_bgr[(max(1, lap) - 1) % len(self._task_result_palette_bgr)]
                cv2.line(image, path_pixels[i - 1], path_pixels[i], color, thickness=2, lineType=cv2.LINE_AA)

            ext = ".png" if snapshot.preview_format.lower() == "png" else ".jpg"
            ok, buffer = cv2.imencode(ext, image)
            if not ok:
                return "", b"", 0, 0
            return str(snapshot.preview_format or "jpg"), bytes(buffer.tobytes()), int(preview_w), int(preview_h)
        except Exception as exc:
            rospy.logwarn("Failed to build task result image: %s", exc)
            return "", b"", 0, 0

    def _build_raw_map_snapshot(self, map_id, max_edge=None, image_format="jpg"):
        target_map_id = str(map_id or "").strip()
        if not target_map_id:
            raise RuntimeError("task map_id is empty")
        image_format = str(image_format or "jpg").strip().lower()
        if image_format == "jpeg":
            image_format = "jpg"
        if image_format not in ("jpg", "png"):
            image_format = "jpg"
        max_edge = self._sanitize_preview_edge(
            int(max_edge or self._preview_max_edge_cap),
            cap_edge=self._preview_max_edge_cap,
        )
        live_map = self._is_live_map_id(target_map_id)
        raw_map = (
            self.aurora_bridge.get_map()
            if live_map
            else self._load_saved_raw_grid_map(target_map_id)
        )
        if raw_map is None:
            raise RuntimeError("raw occupancy grid is unavailable")
        source_width = int(raw_map.info.width)
        source_height = int(raw_map.info.height)
        if source_width <= 0 or source_height <= 0:
            raise RuntimeError("raw occupancy grid dimensions are invalid")
        raw_grid = np.asarray(raw_map.data, dtype=np.int16).reshape(
            (source_height, source_width)
        )
        image = np.full((source_height, source_width, 3), 180, dtype=np.uint8)
        image[raw_grid == 0] = (245, 245, 245)
        image[raw_grid >= 100] = (45, 45, 45)
        image = cv2.flip(image, 0)
        image_width, image_height, _ = self._preview_meta(
            source_width,
            source_height,
            max_edge,
        )
        if image_width != source_width or image_height != source_height:
            image = cv2.resize(
                image,
                (image_width, image_height),
                interpolation=cv2.INTER_AREA,
            )
        encode_ext = ".png" if image_format == "png" else ".jpg"
        encoded_ok, encoded_map = cv2.imencode(encode_ext, image)
        if not encoded_ok:
            raise RuntimeError("raw map image encoding failed")
        if live_map:
            map_version = int(self.map_service.get_map_version())
        else:
            saved_map_service = MapService()
            saved_map_service.load_local_state(self._map_state_dir(target_map_id))
            map_version = int(saved_map_service.get_map_version())
        return {
            "available": True,
            "message": "ok_live_raw_map" if live_map else "ok_saved_raw_map",
            "map_id": target_map_id,
            "version": map_version,
            "source_width": source_width,
            "source_height": source_height,
            "resolution": float(raw_map.info.resolution),
            "origin_x": float(raw_map.info.origin.position.x),
            "origin_y": float(raw_map.info.origin.position.y),
            "frame_id": str(raw_map.header.frame_id or ""),
            "image_format": image_format,
            "image_width": int(image_width),
            "image_height": int(image_height),
            "preview_scale_x": float(image_width) / float(max(1, source_width)),
            "preview_scale_y": float(image_height) / float(max(1, source_height)),
            "alignment_yaw_deg": float(self._alignment_yaw_deg_for_sl_link_report(target_map_id)),
            "app_rotation_deg": float(self._app_rotation_deg_for_map_id(target_map_id)),
            "rotation_alignment_delta_deg": float(
                self._rotation_alignment_delta_deg_for_map_id(target_map_id)
            ),
            "captured_at_ms": int(time.time() * 1000.0),
            "image_data": encoded_map.tobytes(),
        }

    def _save_task_execution_raw_map_snapshot(self, execution_id, map_id):
        safe_execution_id = re.sub(
            r"[^A-Za-z0-9_.-]+",
            "_",
            str(execution_id or "").strip(),
        )
        if not safe_execution_id:
            return {"available": False, "message": "execution_id is empty"}
        try:
            snapshot = self._build_raw_map_snapshot(map_id)
            extension = str(snapshot.get("image_format", "jpg") or "jpg")
            relative_dir = os.path.join("task_executions", safe_execution_id)
            image_relative_path = os.path.join(relative_dir, "raw_map.{}".format(extension))
            metadata_relative_path = os.path.join(relative_dir, "raw_map_metadata.json")
            image_path = os.path.join(self._persist_state_dir, image_relative_path)
            metadata_path = os.path.join(self._persist_state_dir, metadata_relative_path)
            os.makedirs(os.path.dirname(image_path), exist_ok=True)
            image_data = bytes(snapshot.pop("image_data", b"") or b"")
            image_tmp_path = image_path + ".tmp"
            with open(image_tmp_path, "wb") as handle:
                handle.write(image_data)
            os.replace(image_tmp_path, image_path)
            snapshot["image_path"] = image_relative_path
            snapshot["metadata_path"] = metadata_relative_path
            metadata_tmp_path = metadata_path + ".tmp"
            with open(metadata_tmp_path, "w", encoding="utf-8") as handle:
                json.dump(snapshot, handle, ensure_ascii=False, indent=2)
            os.replace(metadata_tmp_path, metadata_path)
            rospy.loginfo(
                "Task execution raw map saved: execution_id=%s map_id=%s image=%s metadata=%s size=%dx%d bytes=%d",
                execution_id,
                map_id,
                image_path,
                metadata_path,
                int(snapshot.get("image_width", 0) or 0),
                int(snapshot.get("image_height", 0) or 0),
                len(image_data),
            )
            return snapshot
        except Exception as exc:
            rospy.logwarn(
                "Failed to save task execution raw map: execution_id=%s map_id=%s err=%s",
                execution_id,
                map_id or "<empty>",
                exc,
            )
            return {
                "available": False,
                "message": "task raw map save failed: {}".format(exc),
                "map_id": str(map_id or ""),
            }

    def _load_task_execution_raw_map_snapshot(self, record):
        metadata = dict(record.get("raw_map_snapshot", {}) or {}) if isinstance(record, dict) else {}
        if not metadata:
            return {
                "available": False,
                "message": "task raw map snapshot was not recorded",
                "image_data": b"",
            }
        image_relative_path = str(metadata.get("image_path", "") or "").strip()
        if not image_relative_path:
            metadata["available"] = False
            metadata["message"] = "task raw map snapshot image path is empty"
            metadata["image_data"] = b""
            return metadata
        image_path = image_relative_path
        if not os.path.isabs(image_path):
            image_path = os.path.join(self._persist_state_dir, image_path)
        try:
            with open(image_path, "rb") as handle:
                metadata["image_data"] = handle.read()
            metadata["available"] = bool(metadata["image_data"])
            return metadata
        except Exception as exc:
            metadata["available"] = False
            metadata["message"] = "task raw map snapshot read failed: {}".format(exc)
            metadata["image_data"] = b""
            return metadata

    def _save_task_execution_preview(self, execution_id, image_format, image_data):
        execution_id = str(execution_id or "").strip()
        if not execution_id or not image_data:
            return ""
        safe_execution_id = re.sub(r"[^A-Za-z0-9_.-]+", "_", execution_id)
        extension = "png" if str(image_format or "").strip().lower() == "png" else "jpg"
        relative_path = os.path.join(
            "task_executions",
            safe_execution_id,
            "preview.{}".format(extension),
        )
        output_path = os.path.join(self._persist_state_dir, relative_path)
        try:
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            tmp_path = output_path + ".tmp"
            with open(tmp_path, "wb") as handle:
                handle.write(image_data)
            os.replace(tmp_path, output_path)
            return relative_path
        except Exception as exc:
            rospy.logwarn(
                "Failed to save task execution preview: execution_id=%s err=%s",
                execution_id,
                exc,
            )
            return ""

    def _task_execution_preview_base64(self, record):
        if not isinstance(record, dict):
            return ""
        image_path = str(record.get("image_path", "") or "").strip()
        if not image_path:
            return ""
        resolved_path = image_path
        if not os.path.isabs(resolved_path):
            resolved_path = os.path.join(self._persist_state_dir, resolved_path)
        try:
            with open(resolved_path, "rb") as handle:
                return base64.b64encode(handle.read()).decode("ascii")
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0,
                "Failed to load task execution preview: execution_id=%s path=%s err=%s",
                str(record.get("execution_id", "") or ""),
                resolved_path,
                exc,
            )
            return ""

    def _finalize_task_result(self, stop_reason=""):
        try:
            map_id = str(self._current_map_id() or self.task_config.map_id or "").strip()
            task_id = str(self.task_config.task_id or "task").strip() or "task"
            if not map_id:
                return
            self._mark_current_region_repeat_done()
            region_results = []
            all_completed = True
            selected = list(self._effective_selected_work_region_ids(list(self.task_config.work_regions or [])) or [])
            name_by_id = {}
            for region in list(self.task_config.work_regions or []):
                if not isinstance(region, dict):
                    continue
                rid = str(region.get("region_id", "")).strip()
                if rid:
                    name_by_id[rid] = str(region.get("name", "") or rid)
            for rid in selected:
                target_repeat = max(1, int((self.task_config.region_repeat_config or {}).get(rid, 1)))
                executed_repeat = max(0, min(target_repeat, int(self._exec_region_repeat_done.get(rid, 0) or 0)))
                completed = bool(executed_repeat >= target_repeat)
                if not completed:
                    all_completed = False
                region_results.append(
                    {
                        "region_id": rid,
                        "region_name": name_by_id.get(rid, rid),
                        "target_repeat": int(target_repeat),
                        "executed_repeat": int(executed_repeat),
                        "completed": bool(completed),
                        "unfinished_reason": "" if completed else str(stop_reason or self.last_error or "not_finished"),
                    }
                )
            image_format, image_data, image_width, image_height = self._build_task_result_image()
            finished_at = int(time.time())
            execution_progress = max(0.0, min(1.0, float(self._task_progress())))
            planned_area_m2 = float(self._selected_task_work_area_m2())
            if self.state == SchedulerState.COMPLETED and all_completed:
                execution_progress = 1.0
            executed_area_m2 = planned_area_m2 * execution_progress
            execution_record = self._finalize_active_task_execution_record(
                final_state=str(self.state.value),
                stop_reason=str(stop_reason or self.last_error or ""),
                finished_at=finished_at,
                planned_area_m2=planned_area_m2,
                executed_area_m2=executed_area_m2,
                progress=execution_progress,
                path_version=int(self.current_path.path_version if self.current_path is not None else 0),
                all_completed=bool(all_completed),
            )
            if execution_record:
                execution_record.update(
                    {
                        "image_format": str(image_format or ""),
                        "image_width": int(image_width),
                        "image_height": int(image_height),
                        "image_path": self._save_task_execution_preview(
                            execution_record.get("execution_id", ""),
                            image_format,
                            image_data,
                        ),
                    }
                )
            key = "{}::{}".format(map_id, task_id)
            record = self._task_bindings.get(key, {}) if isinstance(self._task_bindings.get(key, {}), dict) else {}
            record.update(
                {
                    "task_id": task_id,
                    "map_id": map_id,
                    "selected_work_region_ids": list(self.task_config.selected_work_region_ids or selected),
                    "region_repeat_config": dict(self.task_config.region_repeat_config or {}),
                    "active_work_region_id": self.task_config.active_work_region_id or "",
                    "updated_at": int(time.time()),
                    "task_result": {
                        "map_id": map_id,
                        "task_id": task_id,
                        "final_state": str(self.state.value),
                        "all_completed": bool(all_completed),
                        "stop_reason": str(stop_reason or self.last_error or ""),
                        "path_version": int(self.current_path.path_version if self.current_path is not None else 0),
                        "execution_id": str(execution_record.get("execution_id", "") or ""),
                        "started_at": int(execution_record.get("started_at", 0) or 0),
                        "finished_at": finished_at,
                        "planned_area_m2": planned_area_m2,
                        "executed_area_m2": executed_area_m2,
                        "execution_progress": execution_progress,
                        "selected_work_region_ids": list(self.task_config.selected_work_region_ids or selected),
                        "region_results": region_results,
                        "image_format": str(image_format or ""),
                        "image_b64": base64.b64encode(image_data).decode("ascii") if image_data else "",
                        "image_width": int(image_width),
                        "image_height": int(image_height),
                    },
                }
            )
            self._task_bindings[key] = record
            self._last_task_result = dict(record.get("task_result", {}) or {})
            self._save_local_state()
        except Exception as exc:
            rospy.logwarn("Failed to finalize task result: %s", exc)

    def _selected_task_work_area_m2(self):
        regions = [item for item in list(self.task_config.work_regions or []) if isinstance(item, dict)]
        selected_ids = set(self._effective_selected_work_region_ids(regions))
        total = 0.0
        for region in regions:
            region_id = str(region.get("region_id", "") or "").strip()
            if selected_ids and region_id not in selected_ids:
                continue
            total += float(self._polygon_area_m2(region.get("points", []) or []))
        return max(0.0, total)

    @staticmethod
    def _task_trajectory_relative_path(execution_id):
        safe_execution_id = re.sub(
            r"[^A-Za-z0-9_.-]+",
            "_",
            str(execution_id or "").strip(),
        )
        return os.path.join("task_executions", safe_execution_id, "trajectory.pbstream")

    def _record_task_trajectory_sample(self):
        execution_id = str(self._active_task_execution_id or "").strip()
        if not execution_id or self.state not in (SchedulerState.RUNNING, SchedulerState.PAUSED):
            return
        now_monotonic = time.monotonic()
        if now_monotonic < self._task_trajectory_next_sample_monotonic:
            return
        self._task_trajectory_next_sample_monotonic = (
            now_monotonic + self._task_trajectory_sample_interval_sec
        )
        try:
            record = next(
                (
                    item
                    for item in reversed(self._task_execution_records)
                    if str(item.get("execution_id", "") or "").strip() == execution_id
                ),
                None,
            )
            if not isinstance(record, dict):
                return
            relative_path = str(record.get("trajectory_path", "") or "").strip()
            if not relative_path:
                relative_path = self._task_trajectory_relative_path(execution_id)
                record["trajectory_path"] = relative_path
            output_path = os.path.join(self._persist_state_dir, relative_path)
            pose = self._pose_for_sl_link_report()
            wheel_odom = self._wheel_odom_snapshot()
            disc_enabled = bool(
                self.last_chassis_status is not None
                and bool(getattr(self.last_chassis_status, "disc_enabled", False))
            )
            point_index = int(record.get("trajectory_point_count", 0) or 0)
            if point_index >= self._task_trajectory_max_samples:
                return
            now_ms = int(time.time() * 1000.0)
            started_at_ms = int(record.get("started_at_ms", 0) or 0)
            if started_at_ms <= 0:
                started_at_ms = int(record.get("started_at", 0) or 0) * 1000
            point = self.sl_link_server.pb.TaskTrajectoryPoint()
            point.index = point_index
            point.offset_ms = max(0, min(0xFFFFFFFF, now_ms - started_at_ms))
            point.x_mm = int(round(float(pose.get("x", 0.0)) * 1000.0))
            point.y_mm = int(round(float(pose.get("y", 0.0)) * 1000.0))
            point.heading_mdeg = int(round(float(pose.get("heading_deg", 0.0)) * 1000.0))
            point.linear_speed_mmps = int(
                round(float(wheel_odom.get("linear_mps", 0.0)) * 1000.0)
            )
            point.angular_speed_mradps = int(
                round(float(wheel_odom.get("angular_radps", 0.0)) * 1000.0)
            )
            point.disc_speed_rpm = max(0, int(self._configured_disc_speed_rpm()))
            point.speed_available = bool(wheel_odom.get("available", False))
            point.disc_enabled = disc_enabled
            point.task_state = self._task_state_to_pb()
            encoded = point.SerializeToString()
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            with self._task_trajectory_lock:
                with open(output_path, "ab") as handle:
                    handle.write(struct.pack("<H", len(encoded)))
                    handle.write(encoded)
            record["trajectory_point_count"] = point_index + 1
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0,
                "Failed to record task trajectory sample: execution_id=%s err=%s",
                execution_id,
                exc,
            )

    def _begin_task_execution_record(self):
        task_id = str(self.task_config.task_id or "task").strip() or "task"
        map_id = str(self._current_map_id() or self.task_config.map_id or "").strip()
        started_at_ms = int(time.time() * 1000.0)
        started_at = started_at_ms // 1000
        execution_id = "{}_{}".format(task_id, started_at_ms)
        trajectory_path = self._task_trajectory_relative_path(execution_id)
        record = {
            "execution_id": execution_id,
            "project_id": "",
            "map_id": map_id,
            "task_id": task_id,
            "final_state": "RUNNING",
            "stop_reason": "",
            "started_at": started_at,
            "started_at_ms": started_at_ms,
            "finished_at": 0,
            "planned_area_m2": float(self._selected_task_work_area_m2()),
            "executed_area_m2": 0.0,
            "progress": 0.0,
            "path_version": int(self.current_path.path_version if self.current_path is not None else 0),
            "all_completed": False,
            "trajectory_path": trajectory_path,
            "trajectory_point_count": 0,
        }
        record["raw_map_snapshot"] = self._save_task_execution_raw_map_snapshot(
            execution_id,
            map_id,
        )
        try:
            absolute_trajectory_path = os.path.join(self._persist_state_dir, trajectory_path)
            os.makedirs(os.path.dirname(absolute_trajectory_path), exist_ok=True)
            with open(absolute_trajectory_path, "wb"):
                pass
        except Exception as exc:
            rospy.logwarn(
                "Failed to initialize task trajectory file: execution_id=%s err=%s",
                execution_id,
                exc,
            )
        self._task_execution_records.append(record)
        self._active_task_execution_id = execution_id
        self._task_trajectory_next_sample_monotonic = 0.0
        self._save_task_registry_state()
        rospy.loginfo(
            "Task execution record started: execution_id=%s task_id=%s map_id=%s started_at=%d planned_area_m2=%.3f",
            execution_id,
            task_id,
            map_id or "<empty>",
            started_at,
            float(record["planned_area_m2"]),
        )
        return record

    def _finalize_active_task_execution_record(
        self,
        final_state,
        stop_reason,
        finished_at,
        planned_area_m2,
        executed_area_m2,
        progress,
        path_version,
        all_completed,
    ):
        execution_id = str(self._active_task_execution_id or "").strip()
        record = {}
        if execution_id:
            for item in reversed(self._task_execution_records):
                if str(item.get("execution_id", "") or "").strip() == execution_id:
                    record = item
                    break
        if not record:
            return record
        record.update(
            {
                "final_state": str(final_state or "ERROR"),
                "stop_reason": str(stop_reason or ""),
                "finished_at": int(finished_at),
                "planned_area_m2": float(max(0.0, planned_area_m2)),
                "executed_area_m2": float(max(0.0, executed_area_m2)),
                "progress": float(max(0.0, min(1.0, progress))),
                "path_version": int(path_version),
                "all_completed": bool(all_completed),
            }
        )
        self._active_task_execution_id = ""
        rospy.loginfo(
            "Task execution record finalized: execution_id=%s task_id=%s state=%s started_at=%d finished_at=%d executed_area_m2=%.3f",
            execution_id,
            str(record.get("task_id", "") or ""),
            str(record.get("final_state", "") or ""),
            int(record.get("started_at", 0) or 0),
            int(record.get("finished_at", 0) or 0),
            float(record.get("executed_area_m2", 0.0) or 0.0),
        )
        return record

    def handle_settings_read_request(self, payload):
        pb = self.sl_link_server.pb
        request = pb.SettingsReadRequest()
        request.ParseFromString(payload)
        response = pb.SettingsReadResponse()
        response.result = pb.RESULT_SUCCESS
        response.message = "Settings read success"
        if request.read_chassis:
            status = self.last_chassis_status
            response.chassis.run_speed = float(self._chassis_settings.get("run_speed", 0.4))
            response.chassis.disc_speed_rpm = max(0, int(self._chassis_settings.get("disc_speed_rpm", 1200)))
            saved_mode = int(self._chassis_settings.get("work_mode", 1))
            response.chassis.work_mode = pb.WORK_MODE_MANUAL if saved_mode == 2 else pb.WORK_MODE_AUTO
            response.chassis.disc_enabled = bool(status.disc_enabled) if status is not None else False
            response.chassis.max_turn_speed_ratio = float(
                self._chassis_settings.get("max_turn_speed_ratio", 1.0)
            )
        if request.read_map:
            try:
                self._reload_robot_config_from_yaml()
            except Exception as exc:
                rospy.logwarn("Failed to realtime-load robot config from yaml %s: %s", self._robot_config_yaml_path, exc)
            response.map.vehicle_width = float(self.task_config.vehicle_width)
            response.map.vehicle_length = float(self.task_config.vehicle_length)
            response.map.default_path_spacing = self.task_config.default_path_spacing
            response.map.turn_radius = self.task_config.turn_radius
            response.map.overlap_ratio = self.task_config.overlap_ratio
            response.map.inflation_radius = self.task_config.inflation_radius
            for region in self.map_service.get_overlay_regions()["obstacle_regions"]:
                pb_region = response.map.obstacle_regions.add()
                pb_region.name = region["name"]
                pb_region.region_id = region.get("region_id", "")
                pb_region.priority = int(region.get("order_index", region.get("priority", 0)))
                pb_region.enabled = bool(region.get("enabled", True))
                pb_region.color_argb = int(region.get("color_argb", 0))
                pb_region.closed = bool(region.get("closed", True))
                pb_region.region_type = int(region.get("region_type", pb.REGION_TYPE_OBSTACLE))
                for point in region["points"]:
                    pb_point = pb_region.points.add()
                    pb_point.x = point["x"]
                    pb_point.y = point["y"]
            for region in self.map_service.get_overlay_regions()["work_regions"]:
                pb_region = response.map.work_regions.add()
                pb_region.name = region["name"]
                pb_region.region_id = region.get("region_id", "")
                pb_region.priority = int(region.get("order_index", region.get("priority", 0)))
                pb_region.enabled = bool(region.get("enabled", True))
                pb_region.color_argb = int(region.get("color_argb", 0))
                pb_region.closed = bool(region.get("closed", True))
                pb_region.region_type = int(region.get("region_type", pb.REGION_TYPE_WORK))
                for point in region["points"]:
                    pb_point = pb_region.points.add()
                    pb_point.x = point["x"]
                    pb_point.y = point["y"]
        return response.SerializeToString(), pb.MSG_ID_SETTINGS_READ_RESPONSE, pb.COMP_SETTINGS

    def handle_settings_write_request(self, payload):
        pb = self.sl_link_server.pb
        request = pb.SettingsWriteRequest()
        request.ParseFromString(payload)
        if request.HasField("chassis"):
            work_mode = int(request.chassis.work_mode)
            if work_mode == int(pb.WORK_MODE_AUTO):
                self.work_mode_pub.publish(UInt16(data=1))
                self._chassis_settings["work_mode"] = 1
            elif work_mode == int(pb.WORK_MODE_MANUAL):
                self.work_mode_pub.publish(UInt16(data=2))
                self._chassis_settings["work_mode"] = 2
            disc_speed = int(request.chassis.disc_speed_rpm)
            disc_speed = max(-32768, min(32767, disc_speed))
            self.disc_speed_pub.publish(Int16(data=disc_speed))
            self._chassis_settings["disc_speed_rpm"] = int(max(0, disc_speed))
            disc_enabled = bool(request.chassis.disc_enabled)
            self.disc_enable_pub.publish(Bool(data=disc_enabled))
            rospy.loginfo(
                "Disc settings applied: speed=%drpm enabled=%s topics=(/chassis/disc_speed_cmd,/chassis/disc_enable_cmd)",
                disc_speed,
                str(disc_enabled),
            )
            requested_run_speed = float(request.chassis.run_speed)
            applied_run_speed = max(
                0.0,
                min(self._max_chassis_run_speed, requested_run_speed),
            )
            self._chassis_settings["run_speed"] = applied_run_speed
            self._update_navigation_speed_limit(applied_run_speed, log=True)
            if applied_run_speed <= 0.0:
                if self.state == SchedulerState.RUNNING:
                    self._pause_execution()
                rospy.logwarn("Run speed set to zero; task/navigation motion is disabled")
            if abs(applied_run_speed - requested_run_speed) > 1e-6:
                rospy.logwarn(
                    "Requested run speed clamped: requested=%.3fm/s applied=%.3fm/s max=%.3fm/s",
                    requested_run_speed,
                    applied_run_speed,
                    self._max_chassis_run_speed,
                )
            if float(request.chassis.max_turn_speed_ratio) > 0.0:
                requested_turn_ratio = float(request.chassis.max_turn_speed_ratio)
                applied_turn_ratio = max(0.01, min(1.0, requested_turn_ratio))
                self._chassis_settings["max_turn_speed_ratio"] = applied_turn_ratio
                rospy.loginfo(
                    "Manual turn speed ratio updated: requested=%.3f applied=%.3f",
                    requested_turn_ratio,
                    applied_turn_ratio,
                )
                if abs(applied_turn_ratio - requested_turn_ratio) > 1e-6:
                    rospy.logwarn(
                        "Requested turn speed ratio clamped: requested=%.3f applied=%.3f",
                        requested_turn_ratio,
                        applied_turn_ratio,
                    )
            self._save_local_state()
        if request.HasField("map"):
            if request.map.vehicle_width > 0.0:
                self.task_config.vehicle_width = float(request.map.vehicle_width)
            if request.map.vehicle_length > 0.0:
                self.task_config.vehicle_length = float(request.map.vehicle_length)
            self.task_config.default_path_spacing = request.map.default_path_spacing or self.task_config.default_path_spacing
            self.task_config.turn_radius = request.map.turn_radius or self.task_config.turn_radius
            self.task_config.overlap_ratio = request.map.overlap_ratio
            self.task_config.inflation_radius = request.map.inflation_radius or self.task_config.inflation_radius
            for region in request.map.work_regions:
                self.map_service.apply_edit(
                    {
                        "operation": "UPSERT_WORK_REGION",
                        "region": {
                            "name": region.name,
                            "points": [{"x": p.x, "y": p.y} for p in region.points],
                            "region_id": region.region_id,
                            "priority": int(region.priority),
                            "enabled": bool(region.enabled),
                            "color_argb": int(region.color_argb),
                            "closed": bool(region.closed),
                            "region_type": int(region.region_type) if int(region.region_type) != 0 else int(pb.REGION_TYPE_WORK),
                        },
                    }
                )
            for region in request.map.obstacle_regions:
                self.map_service.apply_edit(
                    {
                        "operation": "UPSERT_OBSTACLE_REGION",
                        "region": {
                            "name": region.name,
                            "points": [{"x": p.x, "y": p.y} for p in region.points],
                            "region_id": region.region_id,
                            "priority": int(region.priority),
                            "enabled": bool(region.enabled),
                            "color_argb": int(region.color_argb),
                            "closed": bool(region.closed),
                            "region_type": int(region.region_type) if int(region.region_type) != 0 else int(pb.REGION_TYPE_OBSTACLE),
                        },
                    }
                )
            try:
                self._sync_robot_config_to_yaml()
            except Exception as exc:
                rospy.logwarn("Failed to sync robot config to yaml %s: %s", self._robot_config_yaml_path, exc)
            self._save_local_state()
        response = pb.SettingsWriteResponse()
        response.result = pb.RESULT_SUCCESS
        response.message = "Settings applied"
        if request.HasField("chassis"):
            response.chassis.CopyFrom(request.chassis)
        return response.SerializeToString(), pb.MSG_ID_SETTINGS_WRITE_RESPONSE, pb.COMP_SETTINGS

    def handle_control_command(self, payload):
        pb = self.sl_link_server.pb
        request = pb.ControlCommand()
        request.ParseFromString(payload)
        control_fields = set(getattr(request, "DESCRIPTOR").fields_by_name.keys())
        has_disc_control = "disc_control" in control_fields
        has_emergency_stop = "emergency_stop" in control_fields
        handled = False
        error_message = ""
        if request.HasField("disc_lift"):
            if not self._disc_lift_supported:
                error_message = "Disc lift is not supported by this machine"
            else:
                self.disc_lift_pub.publish(
                    UInt16(data=2 if request.disc_lift.command == pb.DISC_LIFT_CMD_UP else 1)
                )
                handled = True
        elif has_disc_control and request.HasField("disc_control"):
            if bool(request.disc_control.enabled):
                # Disc outputs are ignored by chassis driver when disabled.
                # Enable chassis first to make sure disc command reaches RS485 writes.
                self._set_chassis_enabled(True)
            disc_speed = int(request.disc_control.speed_rpm)
            disc_speed = max(-32768, min(32767, disc_speed))
            self.disc_speed_pub.publish(Int16(data=disc_speed))
            self.disc_enable_pub.publish(Bool(data=bool(request.disc_control.enabled)))
            handled = True
        elif request.HasField("lighting"):
            self.light_pub.publish(Bool(data=request.lighting.enabled))
            handled = True
        elif request.HasField("manual_drive"):
            self._handle_manual_drive(
                request.manual_drive.motion,
                request.manual_drive.speed_ratio,
                request.manual_drive.remote_x,
                request.manual_drive.remote_y,
                request.manual_drive.max_speed_mps,
                request.manual_drive.max_turn_speed_ratio,
            )
            handled = True
        elif has_emergency_stop and request.HasField("emergency_stop"):
            self._apply_emergency_stop(bool(request.emergency_stop.enabled))
            handled = True
        elif request.HasField("chassis_power"):
            self._set_chassis_enabled(request.chassis_power.enabled)
            handled = True
        else:
            error_message = "Unsupported or empty ControlCommand"
        response = pb.ControlCommandResponse()
        response.result = pb.RESULT_SUCCESS if handled else pb.RESULT_UNSUPPORTED
        response.message = "Control applied" if handled else error_message
        response.applied_command.CopyFrom(request)
        return response.SerializeToString(), pb.MSG_ID_CONTROL_COMMAND_RESPONSE, pb.COMP_CONTROL

    def _apply_emergency_stop(self, enabled):
        if not bool(enabled):
            rospy.loginfo("EmergencyStopControl received with enabled=false, ignored.")
            return
        # Emergency stop sequence:
        # 1) disable task cmd_vel forwarding before any zero-speed command
        # 2) trigger chassis-driver-level safe stop, bypassing wheel_speed/cmd_vel arbitration
        # 3) publish zero wheel speeds and disable disc as a compatibility fallback
        self._exec_active = False
        self._disc_auto_cover_desired = False
        self._set_cmd_vel_forward_runtime_active(False, publish_zero=True, reason="emergency_stop")
        self.chassis_emergency_stop_pub.publish(Bool(data=True))
        wheel = WheelSpeedCommand()
        wheel.left_wheel_speed = 0
        wheel.right_wheel_speed = 0
        self.wheel_cmd_pub.publish(wheel)
        self._reset_disc_motion_guard()
        self.disc_enable_pub.publish(Bool(data=False))
        self.light_pub.publish(Bool(data=False))
        self.state = SchedulerState.STOPPED
        self._task_stop_reason = "emergency_stop"
        rospy.logwarn("Emergency stop applied: task_enable=false, chassis_safe_stop=true, wheels=0, disc=off.")

    def _handle_manual_drive(
        self,
        motion,
        speed_ratio,
        remote_x=0.0,
        remote_y=0.0,
        max_speed_mps=0.0,
        max_turn_speed_ratio=0.0,
    ):
        with self._manual_drive_lock:
            self._handle_manual_drive_locked(
                motion,
                speed_ratio,
                remote_x,
                remote_y,
                max_speed_mps,
                max_turn_speed_ratio,
            )

    def _handle_manual_drive_locked(
        self,
        motion,
        speed_ratio,
        remote_x=0.0,
        remote_y=0.0,
        max_speed_mps=0.0,
        max_turn_speed_ratio=0.0,
    ):
        saved_run_speed = max(0.0, float(self._chassis_settings.get("run_speed", 0.0)))
        requested_max_speed = float(max_speed_mps)
        manual_speed_limit = (
            requested_max_speed
            if requested_max_speed > 0.0
            else saved_run_speed
        )
        applied_speed_ratio = max(0.0, min(1.0, float(speed_ratio)))
        base_max = self._manual_speed_mps_to_rpm(manual_speed_limit)
        base = int(applied_speed_ratio * base_max)
        rospy.loginfo_throttle(
            1.0,
            "Manual drive speed: source=%s requested_max=%.3fm/s saved_run_speed=%.3fm/s "
            "applied_limit=%.3fm/s speed_ratio=%.3f target_speed=%.3fm/s",
            "command" if requested_max_speed > 0.0 else "saved_run_speed",
            requested_max_speed,
            saved_run_speed,
            manual_speed_limit,
            applied_speed_ratio,
            manual_speed_limit * applied_speed_ratio,
        )
        configured_turn_ratio = float(self._chassis_settings.get("max_turn_speed_ratio", 1.0))
        requested_turn_ratio = float(max_turn_speed_ratio) if float(max_turn_speed_ratio) > 0.0 else configured_turn_ratio
        turn_ratio = max(0.01, min(1.0, requested_turn_ratio))
        turn_base = int(round(base * turn_ratio))
        wheel = WheelSpeedCommand()
        use_remote = abs(float(remote_x)) > 1e-3 or abs(float(remote_y)) > 1e-3
        if use_remote:
            x = max(-1.0, min(1.0, float(remote_x)))
            y = max(-1.0, min(1.0, float(remote_y)))
            # Use joystick Cartesian quadrant direction directly:
            # positive y means forward, negative y means backward.
            linear = y * float(base)
            angular = x * float(turn_base)
            left = int(round(linear - angular))
            right = int(round(linear + angular))
            wheel.left_wheel_speed = int(max(-base_max, min(base_max, left)))
            wheel.right_wheel_speed = int(max(-base_max, min(base_max, right)))
        elif motion == self.sl_link_server.pb.MANUAL_MOTION_FORWARD:
            wheel.left_wheel_speed = base
            wheel.right_wheel_speed = base
        elif motion == self.sl_link_server.pb.MANUAL_MOTION_BACKWARD:
            wheel.left_wheel_speed = -base
            wheel.right_wheel_speed = -base
        elif motion == self.sl_link_server.pb.MANUAL_MOTION_FORWARD_LEFT:
            delta = int(round(base * 0.5 * turn_ratio))
            wheel.left_wheel_speed = max(0, base - delta)
            wheel.right_wheel_speed = base
        elif motion == self.sl_link_server.pb.MANUAL_MOTION_FORWARD_RIGHT:
            wheel.left_wheel_speed = base
            delta = int(round(base * 0.5 * turn_ratio))
            wheel.right_wheel_speed = max(0, base - delta)
        elif motion == self.sl_link_server.pb.MANUAL_MOTION_BACKWARD_LEFT:
            delta = int(round(base * 0.5 * turn_ratio))
            wheel.left_wheel_speed = min(0, -base + delta)
            wheel.right_wheel_speed = -base
        elif motion == self.sl_link_server.pb.MANUAL_MOTION_BACKWARD_RIGHT:
            wheel.left_wheel_speed = -base
            delta = int(round(base * 0.5 * turn_ratio))
            wheel.right_wheel_speed = min(0, -base + delta)
        else:
            wheel.left_wheel_speed = 0
            wheel.right_wheel_speed = 0
        now = time.time()
        next_left = int(wheel.left_wheel_speed)
        next_right = int(wheel.right_wheel_speed)
        reverse_switch = (
            (
                abs(self._manual_last_left_cmd) > 0
                and abs(next_left) > 0
                and ((self._manual_last_left_cmd > 0 and next_left < 0) or (self._manual_last_left_cmd < 0 and next_left > 0))
            )
            or (
                abs(self._manual_last_right_cmd) > 0
                and abs(next_right) > 0
                and (
                    (self._manual_last_right_cmd > 0 and next_right < 0)
                    or (self._manual_last_right_cmd < 0 and next_right > 0)
                )
            )
        )
        # Protect the motor driver from immediate direction reversal:
        # on sign flip, force one short zero-speed dwell before allowing reverse.
        if self._manual_reverse_guard_sec > 0.0:
            if reverse_switch:
                self._manual_reverse_until = now + self._manual_reverse_guard_sec
                wheel.left_wheel_speed = 0
                wheel.right_wheel_speed = 0
                rospy.logwarn_throttle(
                    1.0,
                    "Manual reverse switch detected, applying %.0fms stop guard."
                    % (self._manual_reverse_guard_sec * 1000.0),
                )
            elif now < self._manual_reverse_until and (abs(next_left) > 0 or abs(next_right) > 0):
                wheel.left_wheel_speed = 0
                wheel.right_wheel_speed = 0
        # If manual drive sends non-zero motion while chassis is disabled,
        # try to re-enable chassis automatically to avoid "no RS485 output" confusion.
        moving = abs(int(wheel.left_wheel_speed)) > 0 or abs(int(wheel.right_wheel_speed)) > 0
        chassis_enabled = bool(self.last_chassis_status.enabled) if self.last_chassis_status is not None else False
        if moving and not chassis_enabled:
            if now - self._last_manual_enable_try > 1.0:
                self._last_manual_enable_try = now
                self._set_chassis_enabled(True)
        non_task_manual = self.state != SchedulerState.RUNNING or not self._exec_active
        if non_task_manual and moving:
            # Start the disc before wheel motion on lift-less machines.
            self._set_manual_travel_disc_active(True, reason="manual_drive_start")
        elif not non_task_manual:
            # A stale manual-session flag must not disable a running task's disc.
            self._manual_travel_disc_active = False
        self.wheel_cmd_pub.publish(wheel)
        if non_task_manual and not moving:
            self._set_manual_travel_disc_active(False, reason="manual_drive_stop")
        self._manual_last_left_cmd = int(wheel.left_wheel_speed)
        self._manual_last_right_cmd = int(wheel.right_wheel_speed)
        self._manual_last_command_monotonic = time.monotonic()
        self._manual_command_active = bool(moving)

    def _manual_drive_watchdog_tick(self, _event):
        with self._manual_drive_lock:
            if not self._manual_command_active:
                return
            now = time.monotonic()
            age_sec = now - float(self._manual_last_command_monotonic)
            if age_sec <= self._manual_command_timeout_sec:
                return
            wheel = WheelSpeedCommand()
            wheel.left_wheel_speed = 0
            wheel.right_wheel_speed = 0
            self.wheel_cmd_pub.publish(wheel)
            self._manual_last_left_cmd = 0
            self._manual_last_right_cmd = 0
            self._manual_reverse_until = 0.0
            self._manual_command_active = False
            self._set_manual_travel_disc_active(False, reason="manual_drive_watchdog")
        rospy.logwarn(
            "Manual drive watchdog stopped stale motion: age_ms=%.1f timeout_ms=%.1f",
            age_sec * 1000.0,
            self._manual_command_timeout_sec * 1000.0,
        )

    def handle_task_config(self, payload):
        pb = self.sl_link_server.pb
        request = pb.TaskConfig()
        request.ParseFromString(payload)
        task_id = str(request.task_id or "").strip()
        if not task_id:
            response = pb.TaskConfigResponse()
            response.result = pb.RESULT_INVALID_PARAM
            response.message = "task_id is required"
            return response.SerializeToString(), pb.MSG_ID_TASK_CONFIG_RESPONSE, pb.COMP_SCHEDULER
        map_id_ok, current_map_id = self._validate_requested_map_id(getattr(request, "map_id", ""), "TaskConfig")
        if not map_id_ok:
            response = pb.TaskConfigResponse()
            response.result = pb.RESULT_INVALID_PARAM
            response.message = "map_id mismatch with current map"
            response.task_id = task_id
            return response.SerializeToString(), pb.MSG_ID_TASK_CONFIG_RESPONSE, pb.COMP_SCHEDULER
        task_obstacle_regions = []
        seen_task_obstacle_ids = set()
        for region in request.obstacle_regions:
            region_id = str(region.region_id or "").strip()
            points = [{"x": float(point.x), "y": float(point.y)} for point in region.points]
            if not region_id:
                response = pb.TaskConfigResponse()
                response.result = pb.RESULT_INVALID_PARAM
                response.message = "task obstacle region_id is required"
                response.task_id = task_id
                return response.SerializeToString(), pb.MSG_ID_TASK_CONFIG_RESPONSE, pb.COMP_SCHEDULER
            if region_id in seen_task_obstacle_ids:
                response = pb.TaskConfigResponse()
                response.result = pb.RESULT_INVALID_PARAM
                response.message = "duplicate task obstacle region_id: {}".format(region_id)
                response.task_id = task_id
                return response.SerializeToString(), pb.MSG_ID_TASK_CONFIG_RESPONSE, pb.COMP_SCHEDULER
            if len(points) < 3:
                response = pb.TaskConfigResponse()
                response.result = pb.RESULT_INVALID_PARAM
                response.message = "task obstacle region {} requires at least 3 points".format(region_id)
                response.task_id = task_id
                return response.SerializeToString(), pb.MSG_ID_TASK_CONFIG_RESPONSE, pb.COMP_SCHEDULER
            if any(
                not math.isfinite(point["x"]) or not math.isfinite(point["y"])
                for point in points
            ):
                response = pb.TaskConfigResponse()
                response.result = pb.RESULT_INVALID_PARAM
                response.message = "task obstacle region {} contains non-finite points".format(region_id)
                response.task_id = task_id
                return response.SerializeToString(), pb.MSG_ID_TASK_CONFIG_RESPONSE, pb.COMP_SCHEDULER
            seen_task_obstacle_ids.add(region_id)
            task_obstacle_regions.append(
                {
                    "region_id": region_id,
                    "name": str(region.name or ""),
                    "points": points,
                    "order_index": int(region.priority),
                    "priority": int(region.priority),
                    "enabled": True,
                    "color_argb": int(region.color_argb),
                    "closed": True,
                    "region_type": 2,
                }
            )
        self.task_config = TaskConfigModel(
            task_id=task_id,
            map_id=current_map_id,
            work_regions=[
                {
                    "region_id": region.region_id,
                    "name": region.name,
                    "points": [{"x": p.x, "y": p.y} for p in region.points],
                }
                for region in request.work_regions
            ],
            obstacle_regions=[],
            erase_regions=[],
            active_work_region_id=(request.work_regions[0].region_id if len(request.work_regions) > 0 else ""),
            selected_work_region_ids=[
                str(rid).strip()
                for rid in list(getattr(request, "selected_work_region_ids", []) or [])
                if str(rid).strip()
            ]
            or [region.region_id for region in request.work_regions if str(region.region_id).strip()],
            region_repeat_config={
                str(item.region_id).strip(): max(1, int(item.repeat))
                for item in list(getattr(request, "region_repeats", []) or [])
                if str(getattr(item, "region_id", "")).strip()
            },
            vehicle_width=self.task_config.vehicle_width,
            vehicle_length=self.task_config.vehicle_length,
            default_path_spacing=request.default_path_spacing or self.task_config.default_path_spacing,
            global_direction=self.task_config.global_direction or "x",
            turn_radius=request.turn_radius or self.task_config.turn_radius,
            overlap_ratio=request.overlap_ratio or self.task_config.overlap_ratio,
            inflation_radius=request.inflation_radius or self.task_config.inflation_radius,
        )
        task_obstacle_key = self._task_obstacle_binding_key(current_map_id, task_id)
        with self._task_obstacle_regions_lock:
            if task_obstacle_regions:
                self._task_obstacle_regions[task_obstacle_key] = task_obstacle_regions
            else:
                self._task_obstacle_regions.pop(task_obstacle_key, None)
        for region in self.task_config.work_regions:
            self.map_service.apply_edit({"operation": "UPSERT_WORK_REGION", "region": region})
        self._sync_task_regions_from_overlay()
        self._merge_task_obstacle_regions_for_planning(current_map_id, task_id)
        self._sync_task_map_binding(update_binding=True)
        self._path_plan_request_cache_key = None
        self._path_plan_request_cache_path_version = 0
        self._invalidate_path_preview_payload_cache()
        self._invalidate_path_preview_overlay_base_cache()
        self._save_local_state()
        rospy.loginfo(
            "TaskConfig task obstacle regions replaced: map_id=%s task_id=%s count=%d",
            current_map_id,
            task_id,
            len(task_obstacle_regions),
        )
        if self._task_config_auto_plan_enabled:
            rospy.loginfo("TaskConfig auto planning enabled, start planning immediately.")
            self._plan_current_task()
        else:
            rospy.loginfo(
                "TaskConfig accepted without auto planning; waiting for PathPlanRequest to avoid duplicate planning."
            )
        response = pb.TaskConfigResponse()
        response.result = pb.RESULT_SUCCESS
        response.message = "Task config accepted"
        response.task_id = self.task_config.task_id
        return response.SerializeToString(), pb.MSG_ID_TASK_CONFIG_RESPONSE, pb.COMP_SCHEDULER

    def handle_task_command(self, payload):
        pb = self.sl_link_server.pb
        request = pb.TaskCommand()
        request.ParseFromString(payload)
        response = pb.TaskCommandResponse()
        response.task_id = request.task_id or self.task_config.task_id
        if request.command == pb.TASK_CMD_START:
            success, message = self._start_execution()
            if success:
                try:
                    self._publish_task_path_to_mqtt()
                except Exception as exc:
                    rospy.logwarn(
                        "MQTT task path report failed without blocking task start response: %s",
                        exc,
                    )
        elif request.command == pb.TASK_CMD_PAUSE:
            success, message = self._pause_execution()
        elif request.command == pb.TASK_CMD_RESUME:
            success, message = self._resume_execution()
        else:
            success, message = self._stop_execution()
        response.result = pb.RESULT_SUCCESS if success else pb.RESULT_FAILED
        response.message = message
        return response.SerializeToString(), pb.MSG_ID_TASK_COMMAND_RESPONSE, pb.COMP_SCHEDULER

    def _serialize_path_point_plan_chunks(
        self,
        task_id,
        request_id,
        map_id,
        max_chunk_size,
        planned,
        result,
        message,
    ):
        pb = self.sl_link_server.pb
        response_task_id = str(task_id or self.task_config.task_id or "").strip()
        response_request_id = str(request_id or "").strip()
        response_map_id = str(map_id or self.task_config.map_id or self._current_map_id() or "").strip()

        response_points, response_frame_id, response_alignment_yaw = self._path_points_for_external_map_frame(
            self.current_path.points if planned and self.current_path else []
        )
        response_points, response_segments = self._classify_task_path_points(
            response_points,
            compact_points=True,
        )
        total_work_area_m2 = float(self._total_work_area_m2())
        estimated_time_s = (
            float(self._estimate_plan_time_s(self.current_path.length_m))
            if planned and self.current_path
            else -1.0
        )
        path_json = json.dumps(
            {
                "result": result,
                "message": message,
                "planned": bool(planned),
                "request_id": response_request_id,
                "map_id": response_map_id,
                "task_id": response_task_id,
                "path_version": self.current_path.path_version if planned and self.current_path else 0,
                "frame_id": response_frame_id,
                "alignment_yaw": response_alignment_yaw,
                "path_point_count": len(response_points),
                "path_length_m": float(self.current_path.length_m) if planned and self.current_path else 0.0,
                "total_work_area_m2": total_work_area_m2,
                "estimated_time_s": estimated_time_s,
                "segments": response_segments,
                "points": response_points,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        chunk_size = max(256, min(4096, int(max_chunk_size or 2048)))
        total = max(1, int(math.ceil(len(path_json) / float(chunk_size))))
        chunks = []
        map_info = self.map_service.get_map_info() or {}
        map_version = max(0, int(map_info.get("map_version", 0) or 0))
        for index in range(total):
            chunk = pb.PathPointPlanResponse()
            chunk.task_id = response_task_id
            chunk.chunk_index = index
            chunk.total_chunks = total
            chunk.path_version = self.current_path.path_version if planned and self.current_path else 0
            chunk.data = path_json[index * chunk_size : (index + 1) * chunk_size]
            chunk.request_id = response_request_id
            chunk.map_id = response_map_id
            chunk.result = pb.RESULT_SUCCESS if planned else pb.RESULT_FAILED
            chunk.message = str(message or "")
            chunk.planned = bool(planned)
            chunk.map_version = map_version
            chunk.path_point_count = len(response_points)
            chunk.path_length_m = (
                float(self.current_path.length_m)
                if planned and self.current_path
                else 0.0
            )
            chunk.total_work_area_m2 = float(total_work_area_m2)
            chunk.estimated_time_s = float(estimated_time_s)
            chunk.frame_id = str(response_frame_id or "")
            chunks.append(
                (
                    chunk.SerializeToString(),
                    pb.MSG_ID_PATH_POINT_PLAN_RESPONSE,
                    pb.COMP_SCHEDULER,
                )
            )
        rospy.loginfo(
            "PathPointPlanRequest completed: task_id=%s planned=%s path_version=%d points=%d segments=%d chunks=%d bytes=%d point_format=index_xy preview_generated=false",
            response_task_id or "<empty>",
            str(bool(planned)).lower(),
            int(self.current_path.path_version) if planned and self.current_path else 0,
            len(response_points),
            len(response_segments),
            len(chunks),
            len(path_json),
        )
        return chunks

    def build_path_point_plan_chunks(self, payload):
        pb = self.sl_link_server.pb
        request = pb.PathPointPlanRequest()
        request.ParseFromString(payload)

        plan_request = pb.PathPlanRequest()
        plan_request.request_id = str(request.request_id or "")
        plan_request.task_id = str(request.task_id or "")
        plan_request.force_replan = bool(request.force_replan)
        plan_request.return_path_chunks = True
        plan_request.max_chunk_size = int(request.max_chunk_size or 2048)
        plan_request.global_direction = str(request.global_direction or "")
        plan_request.map_id = str(request.map_id or "")
        if request.HasField("start_pose"):
            plan_request.start_pose.CopyFrom(request.start_pose)
        if request.HasField("end_pose"):
            plan_request.end_pose.CopyFrom(request.end_pose)

        responses = self.handle_path_plan_request(
            plan_request.SerializeToString(),
            include_preview=False,
        )
        if isinstance(responses, tuple):
            responses = [responses]
        chunks = [
            item
            for item in responses
            if int(item[1]) == int(pb.MSG_ID_PATH_POINT_PLAN_RESPONSE)
        ]
        if chunks:
            return chunks

        plan_response = pb.PathPlanResponse()
        for response_payload, response_msg_id, _ in responses:
            if int(response_msg_id) == int(pb.MSG_ID_PATH_PLAN_RESPONSE):
                plan_response.ParseFromString(response_payload)
                break
        return self._serialize_path_point_plan_chunks(
            task_id=str(request.task_id or ""),
            request_id=str(request.request_id or ""),
            map_id=str(request.map_id or ""),
            max_chunk_size=int(request.max_chunk_size or 2048),
            planned=False,
            result="failed",
            message=str(plan_response.message or "path point planning failed"),
        )

    @staticmethod
    def _task_path_point_classification(point):
        raw_path_type = str((point or {}).get("path_type", "") or "").strip()
        point_type = str((point or {}).get("point_type", "") or "").strip().lower()
        if raw_path_type == "connection" or point_type in ("start", "start_pose", "current_pose"):
            return {
                "path_scope": "between_regions",
                "path_category": "connection",
                "region_id": "",
                "lap_index": 0,
            }

        region_id = raw_path_type
        lap_index = 0
        match = re.match(r"^(.*)__lap_([0-9]+)$", raw_path_type)
        if match is not None:
            region_id = str(match.group(1) or "").strip()
            lap_index = int(match.group(2))
        return {
            "path_scope": "within_region",
            "path_category": "coverage",
            "region_id": region_id,
            "lap_index": lap_index,
        }

    def _classify_task_path_points(self, points, compact_points=False):
        enriched = []
        segments = []
        for index, source_point in enumerate(list(points or [])):
            source_point = source_point if isinstance(source_point, dict) else {}
            classification = self._task_path_point_classification(source_point)
            if compact_points:
                point = {
                    "index": int(index),
                    "x": round(float(source_point.get("x", 0.0) or 0.0), 6),
                    "y": round(float(source_point.get("y", 0.0) or 0.0), 6),
                }
            else:
                point = deepcopy(source_point)
                point.update(classification)
                point["index"] = int(index)
            enriched.append(point)

            segment_key = (
                classification["path_scope"],
                classification["region_id"],
                int(classification["lap_index"]),
            )
            previous_key = None
            if segments:
                previous = segments[-1]
                previous_key = (
                    previous["path_scope"],
                    previous["region_id"],
                    int(previous["lap_index"]),
                )
            if segment_key != previous_key:
                segments.append(
                    {
                        "segment_index": len(segments),
                        "path_scope": classification["path_scope"],
                        "path_category": classification["path_category"],
                        "region_id": classification["region_id"],
                        "lap_index": int(classification["lap_index"]),
                        "from_region_id": "",
                        "to_region_id": "",
                        "start_point_index": int(index),
                        "end_point_index": int(index),
                        "point_count": 1,
                    }
                )
            else:
                segments[-1]["end_point_index"] = int(index)
                segments[-1]["point_count"] = int(segments[-1]["point_count"]) + 1

        for segment_index, segment in enumerate(segments):
            if segment["path_scope"] != "between_regions":
                continue
            previous_region = ""
            next_region = ""
            for candidate in reversed(segments[:segment_index]):
                if candidate["path_scope"] == "within_region" and candidate["region_id"]:
                    previous_region = candidate["region_id"]
                    break
            for candidate in segments[segment_index + 1 :]:
                if candidate["path_scope"] == "within_region" and candidate["region_id"]:
                    next_region = candidate["region_id"]
                    break
            segment["from_region_id"] = previous_region
            segment["to_region_id"] = next_region
        return enriched, segments

    def build_camera_frame_chunks(self, payload):
        pb = self.sl_link_server.pb
        request = pb.CameraFrameRequest()
        request.ParseFromString(payload)
        chunk_size = max(256, min(4096, int(request.max_chunk_size or 1024)))
        left, right, _, _ = self.aurora_bridge.get_latest_frames()
        frames = [("left", left), ("right", right)]
        outputs = []
        now_utc = int(time.time())
        for side, frame in frames:
            if frame is None:
                continue
            ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if not ok:
                continue
            data = encoded.tobytes()
            total = max(1, int(math.ceil(len(data) / float(chunk_size))))
            side_frame_id = 1 if side == "left" else 2
            for index in range(total):
                chunk = pb.CameraFrameChunk()
                chunk.frame_id = side_frame_id
                chunk.utc_time = now_utc
                chunk.width = int(frame.shape[1])
                chunk.height = int(frame.shape[0])
                chunk.codec = pb.CAMERA_CODEC_JPEG
                chunk.chunk_index = index
                chunk.total_chunks = total
                chunk.data = data[index * chunk_size : (index + 1) * chunk_size]
                outputs.append((chunk.SerializeToString(), pb.MSG_ID_CAMERA_FRAME_CHUNK, pb.COMP_MEDIA))
        return outputs

    def build_map_chunks(self, payload):
        pb = self.sl_link_server.pb
        request = pb.MapRequest()
        request.ParseFromString(payload)
        requested_map_id = str(getattr(request, "map_id", "") or "").strip()
        if self._is_live_map_id(requested_map_id):
            raw_map = self.aurora_bridge.get_map()
            raw_map_source = "live"
        else:
            try:
                raw_map = self._load_saved_raw_grid_map(requested_map_id)
                raw_map_source = "saved"
                rospy.loginfo("MapRequest using saved raw grid: map_id=%s", requested_map_id)
            except Exception as exc:
                rospy.logwarn("MapRequest failed to load saved raw grid: map_id=%s error=%s", requested_map_id, exc)
                return []
        if raw_map is None:
            rospy.logwarn_throttle(
                2.0,
                "MapRequest received but raw map is unavailable: map_id=%s source=%s",
                requested_map_id or "<empty>",
                raw_map_source,
            )
            return []

        # Keep chunk size compatible with legacy protocol max payload.
        chunk_size = int(request.max_chunk_size) if request.max_chunk_size else 512
        chunk_size = max(64, min(512, chunk_size))

        info = raw_map.info
        width = int(info.width)
        height = int(info.height)
        resolution = float(info.resolution)
        origin_x = float(info.origin.position.x)
        origin_y = float(info.origin.position.y)
        frame_id = str(raw_map.header.frame_id)
        grid = np.array(raw_map.data, dtype=np.int16).reshape((height, width))
        preview_scale_x = 1.0
        preview_scale_y = 1.0
        output_width = width
        output_height = height
        rospy.loginfo(
            "MapRequest raw map output without rotation: source=%s map_id=%s frame_id=%s map_size=%sx%s",
            raw_map_source,
            requested_map_id or self._live_map_id,
            frame_id or "<empty>",
            int(width),
            int(height),
        )

        # Default to image payload for Android-friendly rendering.
        # Fallback: set ~map_request_encoding:=grid to output raw OccupancyGrid bytes.
        if self._map_request_encoding in ("grid", "occupancy", "occupancy_grid"):
            encoding = pb.MAP_ENCODING_OCCUPANCY_GRID
            data = grid.astype(np.int8).tobytes()
        else:
            encoding = pb.MAP_ENCODING_PNG
            image = np.zeros((grid.shape[0], grid.shape[1], 3), dtype=np.uint8)
            image[:, :] = (180, 180, 180)
            image[grid == 0] = (245, 245, 245)
            image[grid >= 100] = (45, 45, 45)
            image = cv2.flip(image, 0)
            max_edge = max(64, int(self._preview_max_edge_cap))
            output_width, output_height, _ = self._preview_meta(width, height, max_edge)
            if output_width != width or output_height != height:
                image = cv2.resize(
                    image,
                    (output_width, output_height),
                    interpolation=cv2.INTER_AREA,
                )
                preview_scale_x = float(output_width) / float(max(1, width))
                preview_scale_y = float(output_height) / float(max(1, height))
            ok, buffer = cv2.imencode(".png", image)
            if not ok:
                rospy.logwarn("MapRequest PNG encode failed, fallback to OccupancyGrid bytes")
                encoding = pb.MAP_ENCODING_OCCUPANCY_GRID
                data = grid.astype(np.int8).tobytes()
                output_width = width
                output_height = height
                preview_scale_x = 1.0
                preview_scale_y = 1.0
            else:
                data = buffer.tobytes()
                rospy.loginfo(
                    "MapRequest PNG prepared: source_size=%sx%s output_size=%sx%s "
                    "scale_x=%.6f scale_y=%.6f max_edge=%d bytes=%d",
                    width,
                    height,
                    output_width,
                    output_height,
                    preview_scale_x,
                    preview_scale_y,
                    max_edge,
                    len(data),
                )
        total = max(1, int(math.ceil(len(data) / float(chunk_size))))
        if raw_map_source == "live":
            map_version = self.map_service.get_map_version()
        else:
            saved_map_service = MapService()
            saved_map_service.load_local_state(self._map_state_dir(requested_map_id))
            map_version = saved_map_service.get_map_version()
        if map_version > 0:
            base_map_id = map_version
        else:
            # map_version is often 0 when only raw map is used.
            # Fallback to a stable non-zero id generated from stamp or map data.
            stamp = raw_map.header.stamp
            stamp_ms = int(getattr(stamp, "secs", 0)) * 1000 + int(getattr(stamp, "nsecs", 0)) // 1000000
            if stamp_ms > 0:
                base_map_id = stamp_ms & 0xFFFFFFFF
            else:
                base_map_id = zlib.crc32(data) & 0xFFFFFFFF
        identity = "{}|{}|{}|{:.9f}|{:.9f}|{:.9f}".format(
            frame_id,
            int(width),
            int(height),
            float(resolution),
            float(origin_x),
            float(origin_y),
        )
        map_id = zlib.crc32("{}|{}".format(int(base_map_id), identity).encode("utf-8")) & 0xFFFFFFFF
        if map_id == 0:
            map_id = 1
        now_utc = int(time.time())

        outputs = []
        for index in range(total):
            chunk = pb.MapChunk()
            chunk.map_id = map_id
            chunk.utc_time = now_utc
            chunk.encoding = encoding
            chunk.width = int(width)
            chunk.height = int(height)
            chunk.resolution = float(resolution)
            chunk.origin.x = float(origin_x)
            chunk.origin.y = float(origin_y)
            chunk.origin.heading_deg = 0.0
            chunk.frame_id = frame_id
            chunk.preview_scale_x = float(preview_scale_x)
            chunk.preview_scale_y = float(preview_scale_y)
            chunk.map_version = max(0, int(map_version))
            self._apply_localization_covariance(chunk)
            self._apply_alignment_yaw_to_response(
                chunk,
                requested_map_id or self._live_map_id,
            )
            chunk.chunk_index = index
            chunk.total_chunks = total
            chunk.data = data[index * chunk_size : (index + 1) * chunk_size]
            outputs.append((chunk.SerializeToString(), pb.MSG_ID_MAP_CHUNK, pb.COMP_MEDIA))
        return outputs

    def handle_map_sync_request(self, payload):
        pb = self.sl_link_server.pb
        request = pb.MapSyncRequest()
        request.ParseFromString(payload)
        response = pb.MapSyncResponse()
        op_download = int(getattr(pb, "MAP_SYNC_OP_DOWNLOAD_FROM_AURORA", 1))
        op_upload = int(getattr(pb, "MAP_SYNC_OP_UPLOAD_TO_AURORA", 2))
        req_op = int(request.operation)
        response.operation = req_op
        response.navigation_map_reloaded = False
        os.makedirs(self._stcm_local_dir, exist_ok=True)
        requested_map_id = str(getattr(request, "map_id", "")).strip()
        stcm_path = ""
        saved_name = ""
        saved_map_id = ""

        try:
            if req_op == op_download:
                # map_id-only mode: always create a fresh local file; request.map_id is optional hint.
                stcm_path = self._build_stcm_download_path("")
                self._ensure_sync_proxies()
                result = self._sync_get_proxy(mapfile=stcm_path)
                if not result.success:
                    raise RuntimeError(result.message or "sync_get_stcm failed")
                if (not os.path.exists(stcm_path)) or os.path.getsize(stcm_path) <= 0:
                    raise RuntimeError("sync_get_stcm finished but file is empty: {}".format(stcm_path))
                _saved_name, saved_map_id = self._register_saved_map(stcm_path, "")
                saved_name = _saved_name or ""
                if saved_map_id:
                    self._set_active_map_id(saved_map_id, reason="map_sync_download", migrate_bindings=True)
                response.message = "stcm_downloaded"
            elif req_op == op_upload:
                import_result = self._import_saved_map_to_radar(requested_map_id, reason="map_sync_upload")
                stcm_path = import_result["stcm_path"]
                saved_name = import_result["map_name"]
                saved_map_id = import_result["map_id"]
                response.message = (
                    "map_already_active_import_skipped"
                    if bool(import_result.get("import_skipped", False))
                    else "stcm_uploaded"
                )
            else:
                raise RuntimeError("unsupported map sync operation")

            self._attach_runtime_map_paths(response, False)
            self._save_local_state()
            response.result = pb.RESULT_SUCCESS
            if hasattr(response, "map_id"):
                response.map_id = str(saved_map_id or requested_map_id or "")
            if hasattr(response, "map_name"):
                response.map_name = str(saved_name or "")
            if not response.message:
                response.message = "ok"
        except Exception as exc:
            response.result = pb.RESULT_FAILED
            response.message = str(exc)
            if hasattr(response, "map_id"):
                response.map_id = str(requested_map_id or "")

        return response.SerializeToString(), pb.MSG_ID_MAP_SYNC_RESPONSE, pb.COMP_SCHEDULER

    def _import_saved_map_to_radar(self, map_id, reason="map_import_to_radar"):
        requested_map_id = str(map_id or "").strip()
        if not requested_map_id:
            raise RuntimeError("map_id is required")
        record = self._find_recorded_map_by_id(requested_map_id)
        if record is None:
            raise RuntimeError("map_id not found: {}".format(requested_map_id))
        map_name = str(record.get("name", "")).strip()
        stcm_path_value = str(record.get("path", "")).strip()
        stcm_path = os.path.abspath(stcm_path_value) if stcm_path_value else ""
        current_map_id = str(self._current_map_id() or "").strip()
        if current_map_id == requested_map_id:
            rospy.loginfo(
                "Map import skipped because requested map is already active: map_id=%s map_name=%s",
                requested_map_id,
                map_name or "<empty>",
            )
            return {
                "map_id": requested_map_id,
                "map_name": map_name,
                "stcm_path": stcm_path,
                "relocalization_accepted": False,
                "import_skipped": True,
            }

        if not stcm_path:
            raise RuntimeError("map_id found but path is empty: {}".format(requested_map_id))
        if not os.path.exists(stcm_path):
            raise RuntimeError("stcm file not found: {}".format(stcm_path))
        self._ensure_sync_proxies()
        self._send_radar_map_cache_clear(wait_after_sec=self._radar_clear_before_import_delay_sec)
        result = self._sync_set_proxy(mapfile=stcm_path)
        if not result.success:
            raise RuntimeError(result.message or "sync_set_stcm failed")
        self._set_active_map_id(requested_map_id, reason=reason, migrate_bindings=False)
        self._switch_to_localization_mode_after_map_save()
        if self._radar_relocalization_after_import_delay_sec > 0.0:
            rospy.sleep(self._radar_relocalization_after_import_delay_sec)
        relocalization_accepted = self._request_radar_relocalization(
            reason="map_import:{}".format(requested_map_id),
            raise_on_error=False,
        )
        rospy.loginfo(
            "Map imported to radar: map_id=%s map_name=%s stcm=%s "
            "localization_on=true relocalization_accepted=%s",
            requested_map_id,
            map_name,
            stcm_path,
            str(relocalization_accepted),
        )
        return {
            "map_id": requested_map_id,
            "map_name": map_name,
            "stcm_path": stcm_path,
            "relocalization_accepted": bool(relocalization_accepted),
            "import_skipped": False,
        }

    def handle_map_import_to_radar_request(self, payload):
        pb = self.sl_link_server.pb
        request = pb.MapImportToRadarRequest()
        request.ParseFromString(payload)
        response = pb.MapImportToRadarResponse()
        requested_map_id = str(getattr(request, "map_id", "") or "").strip()
        response.map_id = requested_map_id
        response.imported = False
        try:
            import_result = self._import_saved_map_to_radar(requested_map_id, reason="map_import_to_radar")
            response.result = pb.RESULT_SUCCESS
            import_skipped = bool(import_result.get("import_skipped", False))
            response.message = (
                "map_already_active_import_skipped"
                if import_skipped
                else "map_imported_to_radar_and_localization_on"
            )
            response.map_id = str(import_result.get("map_id", requested_map_id))
            response.map_name = str(import_result.get("map_name", ""))
            response.imported = not import_skipped
            self._save_local_state()
        except Exception as exc:
            response.result = pb.RESULT_FAILED
            response.message = str(exc)
        return response.SerializeToString(), pb.MSG_ID_MAP_IMPORT_TO_RADAR_RESPONSE, pb.COMP_SCHEDULER

    def _attach_runtime_map_paths(self, response, update_navigation_map):
        raw_map = self.aurora_bridge.get_map()
        if raw_map is None:
            return
        map_info = self.map_service.get_map_info()
        composed_map = self.map_service.compose_map()
        out_dir = self._live_map_dir
        yaml_name = self._live_map_yaml_name
        image_name = self._live_map_image_name
        if map_info is not None and composed_map is not None:
            yaml_path, image_path = self._export_runtime_map(
                raw_map,
                out_dir,
                yaml_name,
                image_name,
                grid_override=composed_map,
                map_info_override=map_info,
            )
        else:
            yaml_path, image_path = self._export_runtime_map(raw_map, out_dir, yaml_name, image_name)
        response.map_yaml_path = yaml_path
        response.map_image_path = image_path
        response.navigation_map_reloaded = False
        if not update_navigation_map:
            return
        nav_dir = os.path.dirname(self._nav_map_yaml_path)
        nav_yaml_name = os.path.basename(self._nav_map_yaml_path)
        nav_image_name = "map1.pgm"
        if map_info is not None and composed_map is not None:
            nav_yaml_path, nav_image_path = self._export_runtime_map(
                raw_map,
                nav_dir,
                nav_yaml_name,
                nav_image_name,
                grid_override=composed_map,
                map_info_override=map_info,
            )
        else:
            nav_yaml_path, nav_image_path = self._export_runtime_map(raw_map, nav_dir, nav_yaml_name, nav_image_name)
        reloaded, _ = self._reload_navigation_map(nav_yaml_path)
        response.map_yaml_path = nav_yaml_path
        response.map_image_path = nav_image_path
        response.navigation_map_reloaded = bool(reloaded)

    def handle_map_save_request(self, payload):
        pb = self.sl_link_server.pb
        request = pb.MapSaveRequest()
        request.ParseFromString(payload)
        response = pb.MapSaveResponse()
        response.navigation_map_reloaded = False
        os.makedirs(self._stcm_local_dir, exist_ok=True)

        requested_name = (request.map_name or "").strip()
        requested_map_id = str(getattr(request, "map_id", "") or "").strip()
        if requested_name:
            stcm_path = self._build_stcm_download_path(requested_name, forced_map_id=requested_map_id)
        else:
            stcm_path = self._build_stcm_download_path("", forced_map_id=requested_map_id)
        if hasattr(response, "stcm_path"):
            response.stcm_path = stcm_path

        try:
            existing_record = self._find_recorded_map_by_id(requested_map_id) if requested_map_id else None
            if requested_map_id and isinstance(existing_record, dict):
                map_id_ok, current_map_id = self._validate_requested_map_id(
                    requested_map_id,
                    "MapSaveRequest",
                    keep_current_on_empty=False,
                )
                if not map_id_ok:
                    raise RuntimeError("map_id mismatch with current map")
                self._ensure_offline_map_service_for_current_map("map_save_existing")
                self._sync_task_regions_from_overlay()
                self._sync_task_map_binding(update_binding=True)
                total_work_area_m2, estimated_time_s = self._compute_saved_map_metrics()
                region_metrics = self._compute_saved_region_metrics(estimated_time_s)
                thumb_format, thumb_b64, thumb_width, thumb_height = self._build_saved_map_thumbnail()
                self._save_map_overlay_state_for_map(current_map_id, allow_empty_overwrite=True)
                saved_alignment_yaw = self._alignment_yaw_from_map_save_request(request, current_map_id)
                saved_app_rotation_deg = self._app_rotation_deg_from_map_save_request(
                    request,
                    current_map_id,
                )

                existing_path = os.path.abspath(str(existing_record.get("path", "") or "").strip())
                if hasattr(response, "stcm_path"):
                    response.stcm_path = existing_path
                if requested_name:
                    existing_record["name"] = requested_name
                existing_record["saved_at"] = int(time.time())
                existing_record["total_work_area_m2"] = float(max(0.0, total_work_area_m2))
                existing_record["estimated_time_s"] = float(estimated_time_s if estimated_time_s is not None else -1.0)
                existing_record["region_metrics"] = list(region_metrics or [])
                existing_record["thumb_format"] = str(thumb_format or "").strip()
                existing_record["thumb_b64"] = str(thumb_b64 or "")
                existing_record["thumb_width"] = int(max(0, int(thumb_width or 0)))
                existing_record["thumb_height"] = int(max(0, int(thumb_height or 0)))
                if saved_alignment_yaw is not None:
                    self._write_alignment_yaw_to_record(
                        current_map_id,
                        saved_alignment_yaw,
                    )
                self._write_app_rotation_to_record(current_map_id, saved_app_rotation_deg)
                self._save_map_registry_state()
                self._save_local_state()

                response.result = pb.RESULT_SUCCESS
                response.message = "map_saved_offline"
                if hasattr(response, "map_id"):
                    response.map_id = requested_map_id
                if hasattr(response, "map_name"):
                    response.map_name = str(existing_record.get("name", "") or requested_name or "")
                if hasattr(response, "total_work_area_m2"):
                    response.total_work_area_m2 = float(max(0.0, total_work_area_m2))
                if hasattr(response, "estimated_time_s"):
                    response.estimated_time_s = float(estimated_time_s)
                if hasattr(response, "created_at"):
                    created_at = str(existing_record.get("created_at", "") or "").strip()
                    if not created_at:
                        created_at = _format_ts_s(existing_record.get("saved_at", 0))
                    response.created_at = str(created_at or "")
                rospy.loginfo(
                    "Saved existing map metadata only: map_id=%s map_name=%s stcm=%s regions=%d raw_grid_unchanged=true",
                    requested_map_id,
                    str(existing_record.get("name", "") or ""),
                    existing_path or "<empty>",
                    len(self.task_config.work_regions or []),
                )
                self._queue_saved_map_upload(
                    requested_map_id,
                    str(existing_record.get("name", "") or requested_name or ""),
                    existing_path,
                )
                return response.SerializeToString(), pb.MSG_ID_MAP_SAVE_RESPONSE, pb.COMP_SCHEDULER

            if requested_map_id:
                rospy.loginfo(
                    "MapSaveRequest creates new saved map because map_id is not recorded yet: map_id=%s",
                    requested_map_id,
                )
            if not self._is_live_map_id(self._current_map_id()):
                self._set_active_map_id(
                    self._live_map_id,
                    reason="map_save_new_map_source_live",
                    migrate_bindings=False,
                    save_prev_overlay=True,
                )
            live_raw_map = self.aurora_bridge.get_map()
            if live_raw_map is not None:
                self.map_service._grinder_map_source = "live"
                self.map_service._grinder_map_id = self._live_map_id
                self.map_service.set_raw_map(live_raw_map)
            else:
                rospy.logwarn("MapSaveRequest new map source has no live raw map before sync_get_stcm")
            self._sync_task_regions_from_overlay()
            self._sync_task_map_binding(update_binding=True)
            self._ensure_sync_proxies()
            result = self._sync_get_proxy(mapfile=stcm_path)
            if not result.success:
                raise RuntimeError(result.message or "sync_get_stcm failed")
            if (not os.path.exists(stcm_path)) or os.path.getsize(stcm_path) <= 0:
                raise RuntimeError("save map finished but file is empty: {}".format(stcm_path))
            total_work_area_m2, estimated_time_s = self._compute_saved_map_metrics()
            region_metrics = self._compute_saved_region_metrics(estimated_time_s)
            thumb_format, thumb_b64, thumb_width, thumb_height = self._build_saved_map_thumbnail()
            parsed_name, parsed_map_id = self._split_map_name_and_id_from_path(stcm_path)
            target_map_id = str(requested_map_id or parsed_map_id or "").strip()
            if target_map_id:
                self._remove_saved_map_data_by_map_id(
                    target_map_id,
                    keep_path=stcm_path,
                    reason="map_save_replace",
                )
            _saved_name, saved_map_id = self._register_saved_map(
                stcm_path,
                requested_name,
                explicit_map_id=requested_map_id,
                total_work_area_m2=total_work_area_m2,
                estimated_time_s=estimated_time_s,
                region_metrics=region_metrics,
                thumb_format=thumb_format,
                thumb_b64=thumb_b64,
                thumb_width=thumb_width,
                thumb_height=thumb_height,
            )
            if saved_map_id:
                prev_map_id = self._current_map_id()
                saved_alignment_yaw = self._alignment_yaw_from_map_save_request(request, prev_map_id)
                saved_app_rotation_deg = self._app_rotation_deg_from_map_save_request(
                    request,
                    prev_map_id,
                )
                if bool(getattr(request, "has_rotation_deg", False)) and self._is_live_map_id(prev_map_id):
                    self._live_map_app_rotation_deg = float(saved_app_rotation_deg)
                    self._live_map_rotation_alignment_delta_deg = (
                        float(saved_app_rotation_deg)
                        - self._alignment_yaw_deg_for_sl_link_report(prev_map_id)
                    )
                # Carry current overlay regions into newly saved map state.
                self._save_map_overlay_state_for_map(prev_map_id)
                self._copy_map_overlay_state(prev_map_id, saved_map_id, overwrite=True)
                self._set_active_map_id(saved_map_id, reason="map_save", migrate_bindings=True)
                if saved_alignment_yaw is not None:
                    self._write_alignment_yaw_to_record(
                        saved_map_id,
                        saved_alignment_yaw,
                    )
                self._write_app_rotation_to_record(saved_map_id, saved_app_rotation_deg)
                # After saving from LIVE_MAP, clear LIVE_MAP overlay regions.
                # This ensures next time LIVE_MAP is entered, it starts with empty regions.
                if prev_map_id == self._live_map_id:
                    self._remove_map_overlay_state_for_map(prev_map_id)
                # Save raw OccupancyGrid after overlay-state copy because copy may
                # replace the map state directory when overwriting the same map_id.
                raw_map_snapshot = self.aurora_bridge.get_map()
                if raw_map_snapshot is not None:
                    self._save_raw_grid_snapshot_for_map(saved_map_id, raw_map_snapshot, saved_alignment_yaw)
                else:
                    rospy.logwarn(
                        "Map saved without raw grid snapshot because current OccupancyGrid is unavailable: map_id=%s",
                        saved_map_id,
                    )
            localization_switched = False
            localization_err = ""
            try:
                self._switch_to_localization_mode_after_map_save()
                localization_switched = True
            except Exception as loc_exc:
                localization_err = str(loc_exc)
                rospy.logwarn("Map saved but failed to switch localization mode: %s", localization_err)
            self._attach_runtime_map_paths(response, False)
            self._save_local_state()
            self._queue_saved_map_upload(
                saved_map_id or requested_map_id,
                _saved_name or requested_name,
                stcm_path,
            )
            response.result = pb.RESULT_SUCCESS
            response.message = "map_saved_and_localization_on" if localization_switched else "map_saved"
            if localization_err:
                response.message = "{} ({})".format(response.message, localization_err)
            if hasattr(response, "map_id"):
                response.map_id = str(saved_map_id or requested_map_id or "")
            if hasattr(response, "map_name"):
                response.map_name = str(_saved_name or requested_name or "")
            if hasattr(response, "total_work_area_m2"):
                response.total_work_area_m2 = float(max(0.0, total_work_area_m2))
            if hasattr(response, "estimated_time_s"):
                response.estimated_time_s = float(estimated_time_s)
            if hasattr(response, "created_at"):
                record = self._find_recorded_map_by_id(saved_map_id or requested_map_id or "")
                created_at = ""
                if isinstance(record, dict):
                    created_at = str(record.get("created_at", "") or "").strip()
                    if not created_at:
                        created_at = _format_ts_s(record.get("saved_at", 0))
                response.created_at = str(created_at or "")
        except Exception as exc:
            response.result = pb.RESULT_FAILED
            response.message = str(exc)

        return response.SerializeToString(), pb.MSG_ID_MAP_SAVE_RESPONSE, pb.COMP_SCHEDULER

    def handle_map_metrics_request(self, payload):
        pb = self.sl_link_server.pb
        request = pb.MapMetricsRequest()
        request.ParseFromString(payload)
        response = pb.MapMetricsResponse()
        requested_map_id = str(getattr(request, "map_id", "") or "").strip()
        response.map_id = requested_map_id
        try:
            if not requested_map_id:
                raise RuntimeError("map_id is empty")
            map_id_ok, current_map_id = self._validate_requested_map_id(requested_map_id, "MapMetricsRequest")
            if not map_id_ok:
                raise RuntimeError("map_id mismatch with current map")
            record = self._find_recorded_map_by_id(requested_map_id)
            if record is None:
                raise RuntimeError("map_id not found: {}".format(requested_map_id))
            map_name = str(record.get("name", "") or "").strip()
            rospy.loginfo(
                "MapMetricsRequest loaded map overlay: requested_map_id=%s active_map_id=%s",
                requested_map_id,
                current_map_id,
            )

            response.result = pb.RESULT_SUCCESS
            response.message = "ok"
            response.map_id = requested_map_id
            response.map_name = map_name
            for item in list(record.get("region_metrics", []) or []):
                if not isinstance(item, dict):
                    continue
                region_item = response.region_metrics.add()
                region_item.region_id = str(item.get("region_id", "") or "")
                region_item.region_name = str(item.get("region_name", "") or "")
                region_item.repeat = int(max(1, int(item.get("repeat", 1) or 1)))
                region_item.area_m2 = float(max(0.0, float(item.get("area_m2", 0.0) or 0.0)))
                region_item.estimated_time_h = float(item.get("estimated_time_h", -1.0) or -1.0)
        except Exception as exc:
            response.result = pb.RESULT_FAILED
            response.message = str(exc)
            response.map_name = ""

        return response.SerializeToString(), pb.MSG_ID_MAP_METRICS_RESPONSE, pb.COMP_SCHEDULER

    def handle_task_result_request(self, payload):
        pb = self.sl_link_server.pb
        request = pb.TaskResultRequest()
        request.ParseFromString(payload)
        response = pb.TaskResultResponse()
        self._apply_localization_covariance(response)
        requested_map_id = str(getattr(request, "map_id", "") or "").strip()
        requested_task_id = str(getattr(request, "task_id", "") or "").strip()
        self._apply_alignment_yaw_to_response(
            response,
            requested_map_id or self._current_map_id(),
        )

        def _fill_live_preview_fallback(map_id, message):
            target_map_id = str(map_id or "").strip() or self._current_map_id()
            current_map_id = str(self._current_map_id() or "").strip()
            is_live_request = target_map_id in ("", self._live_map_id, DEFAULT_LIVE_MAP_ID)
            if (not is_live_request) and target_map_id != current_map_id:
                rospy.logwarn(
                    "TaskResult fallback live preview skipped: requested_map_id=%s current_map_id=%s",
                    target_map_id,
                    current_map_id,
                )
                return False
            try:
                snapshot = self.map_service.create_preview(
                    self.aurora_bridge.get_pose(),
                    int(self._saved_map_thumb_max_edge),
                    "jpg",
                    True,
                    **self._map_preview_alignment_kwargs()
                )
                image_data = bytes(snapshot.preview_data or b"")
                if not image_data:
                    rospy.logwarn("TaskResult fallback live preview failed: preview data is empty")
                    return False
                response.result = pb.RESULT_SUCCESS
                response.message = str(message or "fallback_to_live_map_preview")
                response.map_id = target_map_id or current_map_id or self._live_map_id
                self._apply_alignment_yaw_to_response(response, response.map_id)
                response.task_id = requested_task_id
                response.final_state = pb.TASK_STATE_UNKNOWN if hasattr(pb, "TASK_STATE_UNKNOWN") else pb.TASK_STATE_IDLE
                response.all_completed = False
                response.stop_reason = "no_task_result"
                response.path_version = 0
                response.finished_at = 0
                response.image_format = "jpg"
                response.image_width = int(getattr(snapshot, "width", 0) or 0)
                response.image_height = int(getattr(snapshot, "height", 0) or 0)
                response.image_data = image_data
                rospy.loginfo(
                    "TaskResult fallback live preview: map_id=%s bytes=%d size=%dx%d",
                    response.map_id,
                    len(response.image_data),
                    response.image_width,
                    response.image_height,
                )
                return True
            except Exception as exc:
                rospy.logwarn("TaskResult fallback live preview failed: %s", exc)
                return False

        def _fill_map_thumbnail_fallback(map_id, message):
            target_map_id = str(map_id or "").strip()
            if not target_map_id:
                return False
            record = self._find_recorded_map_by_id(target_map_id)
            if not isinstance(record, dict):
                rospy.logwarn("TaskResult fallback thumbnail failed: map_id not found: %s", target_map_id)
                return _fill_live_preview_fallback(target_map_id, "fallback_to_live_map_preview: map record not found")
            thumb_b64 = str(record.get("thumb_b64", "") or "")
            if not thumb_b64:
                rospy.logwarn("TaskResult fallback thumbnail failed: thumbnail is empty for map_id=%s", target_map_id)
                if _fill_live_preview_fallback(target_map_id, "fallback_to_live_map_preview: map thumbnail is empty"):
                    return True
                return False
            response.result = pb.RESULT_SUCCESS
            response.message = str(message or "fallback_to_map_thumbnail")
            response.map_id = target_map_id
            self._apply_alignment_yaw_to_response(response, target_map_id)
            response.task_id = requested_task_id
            response.final_state = pb.TASK_STATE_UNKNOWN if hasattr(pb, "TASK_STATE_UNKNOWN") else pb.TASK_STATE_IDLE
            response.all_completed = False
            response.stop_reason = "no_task_result"
            response.path_version = 0
            response.finished_at = 0
            response.image_format = str(record.get("thumb_format", "") or "jpg")
            response.image_width = int(record.get("thumb_width", 0) or 0)
            response.image_height = int(record.get("thumb_height", 0) or 0)
            try:
                response.image_data = base64.b64decode(thumb_b64.encode("ascii"), validate=False)
            except Exception:
                response.image_data = b""
            if not response.image_data:
                rospy.logwarn("TaskResult fallback thumbnail failed: decode produced empty image for map_id=%s", target_map_id)
                return _fill_live_preview_fallback(target_map_id, "fallback_to_live_map_preview: thumbnail decode failed")
            rospy.loginfo(
                "TaskResult fallback thumbnail: map_id=%s bytes=%d size=%dx%d",
                response.map_id,
                len(response.image_data),
                response.image_width,
                response.image_height,
            )
            return bool(response.image_data)

        try:
            record = None
            if requested_map_id and requested_task_id:
                key = "{}::{}".format(requested_map_id, requested_task_id)
                found = self._task_bindings.get(key)
                if isinstance(found, dict):
                    record = found
            elif requested_map_id:
                for _key, item in dict(self._task_bindings or {}).items():
                    if not isinstance(item, dict):
                        continue
                    if str(item.get("map_id", "")).strip() == requested_map_id:
                        record = item
                        break
            elif requested_task_id:
                latest_ts = -1
                for _key, item in dict(self._task_bindings or {}).items():
                    if not isinstance(item, dict):
                        continue
                    if str(item.get("task_id", "")).strip() != requested_task_id:
                        continue
                    task_result = item.get("task_result", {}) if isinstance(item.get("task_result", {}), dict) else {}
                    ts = int(task_result.get("finished_at", item.get("updated_at", 0)) or 0)
                    if ts >= latest_ts:
                        latest_ts = ts
                        record = item
            else:
                latest_ts = -1
                for _key, item in dict(self._task_bindings or {}).items():
                    if not isinstance(item, dict):
                        continue
                    task_result = item.get("task_result", {}) if isinstance(item.get("task_result", {}), dict) else {}
                    ts = int(task_result.get("finished_at", item.get("updated_at", 0)) or 0)
                    if ts >= latest_ts:
                        latest_ts = ts
                        record = item
            if not isinstance(record, dict):
                if _fill_map_thumbnail_fallback(requested_map_id, "fallback_to_map_thumbnail: task result not found"):
                    return response.SerializeToString(), pb.MSG_ID_TASK_RESULT_RESPONSE, pb.COMP_SCHEDULER
                if _fill_live_preview_fallback(requested_map_id, "fallback_to_live_map_preview: task result not found"):
                    return response.SerializeToString(), pb.MSG_ID_TASK_RESULT_RESPONSE, pb.COMP_SCHEDULER
                raise RuntimeError("task result not found")
            task_result = record.get("task_result", {}) if isinstance(record.get("task_result", {}), dict) else {}
            if not task_result:
                fallback_map_id = requested_map_id or str(record.get("map_id", "") or "").strip()
                if _fill_map_thumbnail_fallback(fallback_map_id, "fallback_to_map_thumbnail: task result is empty"):
                    return response.SerializeToString(), pb.MSG_ID_TASK_RESULT_RESPONSE, pb.COMP_SCHEDULER
                raise RuntimeError("task result is empty")

            response.result = pb.RESULT_SUCCESS
            response.message = "ok"
            response.map_id = str(task_result.get("map_id", record.get("map_id", "")) or "")
            self._apply_alignment_yaw_to_response(response, response.map_id)
            response.task_id = str(task_result.get("task_id", record.get("task_id", "")) or "")
            final_state_text = str(task_result.get("final_state", "") or "").strip().upper()
            state_map = {
                "IDLE": pb.TASK_STATE_IDLE,
                "READY": pb.TASK_STATE_READY,
                "PLANNING": pb.TASK_STATE_PLANNING,
                "RUNNING": pb.TASK_STATE_RUNNING,
                "PAUSED": pb.TASK_STATE_PAUSED,
                "COMPLETED": pb.TASK_STATE_COMPLETED,
                "STOPPED": pb.TASK_STATE_STOPPED,
                "ERROR": pb.TASK_STATE_ERROR,
            }
            response.final_state = state_map.get(final_state_text, pb.TASK_STATE_UNKNOWN if hasattr(pb, "TASK_STATE_UNKNOWN") else pb.TASK_STATE_IDLE)
            response.all_completed = bool(task_result.get("all_completed", False))
            response.stop_reason = str(task_result.get("stop_reason", "") or "")
            response.path_version = int(task_result.get("path_version", 0) or 0)
            response.finished_at = int(task_result.get("finished_at", 0) or 0)
            if hasattr(response, "execution_id"):
                response.execution_id = str(task_result.get("execution_id", "") or "")
            if hasattr(response, "started_at"):
                response.started_at = int(task_result.get("started_at", 0) or 0)
            if hasattr(response, "planned_area_m2"):
                response.planned_area_m2 = float(task_result.get("planned_area_m2", 0.0) or 0.0)
            if hasattr(response, "executed_area_m2"):
                response.executed_area_m2 = float(task_result.get("executed_area_m2", 0.0) or 0.0)
            if hasattr(response, "execution_progress"):
                response.execution_progress = float(task_result.get("execution_progress", 0.0) or 0.0)
            response.selected_work_region_ids.extend(list(task_result.get("selected_work_region_ids", []) or []))
            response.image_format = str(task_result.get("image_format", "") or "")
            response.image_width = int(task_result.get("image_width", 0) or 0)
            response.image_height = int(task_result.get("image_height", 0) or 0)
            image_b64 = str(task_result.get("image_b64", "") or "")
            if image_b64:
                try:
                    response.image_data = base64.b64decode(image_b64.encode("ascii"), validate=False)
                except Exception:
                    response.image_data = b""
            for item in list(task_result.get("region_results", []) or []):
                if not isinstance(item, dict):
                    continue
                row = response.region_results.add()
                row.region_id = str(item.get("region_id", "") or "")
                row.region_name = str(item.get("region_name", "") or "")
                row.target_repeat = int(max(1, int(item.get("target_repeat", 1) or 1)))
                row.executed_repeat = int(max(0, int(item.get("executed_repeat", 0) or 0)))
                row.completed = bool(item.get("completed", False))
                row.unfinished_reason = str(item.get("unfinished_reason", "") or "")
            if hasattr(response, "execution_records"):
                max_records = int(getattr(request, "max_execution_records", 0) or 100)
                max_records = max(1, min(500, max_records))
                history = []
                for item in list(self._task_execution_records or []):
                    if not isinstance(item, dict):
                        continue
                    if requested_map_id and str(item.get("map_id", "") or "").strip() != requested_map_id:
                        continue
                    if requested_task_id and str(item.get("task_id", "") or "").strip() != requested_task_id:
                        continue
                    history.append(item)
                history.sort(
                    key=lambda item: (
                        int(item.get("started_at", 0) or 0),
                        str(item.get("execution_id", "") or ""),
                    ),
                    reverse=True,
                )
                for item in history[:max_records]:
                    execution = response.execution_records.add()
                    execution.execution_id = str(item.get("execution_id", "") or "")
                    execution.map_id = str(item.get("map_id", "") or "")
                    execution.task_id = str(item.get("task_id", "") or "")
                    execution.final_state = state_map.get(
                        str(item.get("final_state", "") or "").strip().upper(),
                        pb.TASK_STATE_UNKNOWN if hasattr(pb, "TASK_STATE_UNKNOWN") else pb.TASK_STATE_IDLE,
                    )
                    execution.stop_reason = str(item.get("stop_reason", "") or "")
                    execution.started_at = int(item.get("started_at", 0) or 0)
                    execution.finished_at = int(item.get("finished_at", 0) or 0)
                    execution.planned_area_m2 = float(item.get("planned_area_m2", 0.0) or 0.0)
                    execution.executed_area_m2 = float(item.get("executed_area_m2", 0.0) or 0.0)
                    execution.progress = float(item.get("progress", 0.0) or 0.0)
                    execution.path_version = int(item.get("path_version", 0) or 0)
                    execution.all_completed = bool(item.get("all_completed", False))
        except Exception as exc:
            response.result = pb.RESULT_FAILED
            response.message = str(exc)
            response.map_id = requested_map_id
            response.task_id = requested_task_id

        max_payload_safe = 65000
        response_payload = response.SerializeToString()
        removed_execution_records = 0
        if hasattr(response, "execution_records"):
            while len(response_payload) > max_payload_safe and response.execution_records:
                del response.execution_records[-1]
                removed_execution_records += 1
                response_payload = response.SerializeToString()
        if removed_execution_records > 0:
            suffix = "execution_records_truncated_oversize"
            response.message = "{};{}".format(response.message, suffix) if response.message else suffix
            response_payload = response.SerializeToString()
            rospy.logwarn(
                "TaskResultResponse execution history truncated for SL-Link frame: removed=%d returned=%d payload_bytes=%d",
                removed_execution_records,
                len(response.execution_records),
                len(response_payload),
            )
        if len(response_payload) > max_payload_safe and response.image_data:
            response.image_data = b""
            response.image_format = ""
            response.image_width = 0
            response.image_height = 0
            suffix = "result_image_omitted_oversize"
            response.message = "{};{}".format(response.message, suffix) if response.message else suffix
            response_payload = response.SerializeToString()
            rospy.logwarn(
                "TaskResultResponse image omitted for SL-Link frame safety: payload_bytes=%d",
                len(response_payload),
            )
        if len(response_payload) > max_payload_safe:
            rospy.logerr(
                "TaskResultResponse still exceeds SL-Link safe payload: payload_bytes=%d limit=%d",
                len(response_payload),
                max_payload_safe,
            )
        return response_payload, pb.MSG_ID_TASK_RESULT_RESPONSE, pb.COMP_SCHEDULER

    def build_task_execution_history_chunks(self, payload):
        pb = self.sl_link_server.pb
        request = pb.TaskExecutionHistoryRequest()
        request.ParseFromString(payload)
        requested_map_id = str(request.map_id or "").strip()
        requested_task_id = str(request.task_id or "").strip()
        start_time = int(request.start_time or 0)
        end_time = int(request.end_time or 0)
        chunk_size = max(256, min(4096, int(request.max_chunk_size or 2048)))

        valid_range = not (start_time > 0 and end_time > 0 and start_time > end_time)
        records = []
        if valid_range:
            for source in list(self._task_execution_records or []):
                if not isinstance(source, dict):
                    continue
                map_id = str(source.get("map_id", "") or "").strip()
                task_id = str(source.get("task_id", "") or "").strip()
                started_at = int(source.get("started_at", 0) or 0)
                if requested_map_id and map_id != requested_map_id:
                    continue
                if requested_task_id and task_id != requested_task_id:
                    continue
                if start_time > 0 and started_at < start_time:
                    continue
                if end_time > 0 and started_at > end_time:
                    continue
                records.append(
                    {
                        "execution_id": str(source.get("execution_id", "") or ""),
                        "map_id": map_id,
                        "task_id": task_id,
                        "final_state": str(source.get("final_state", "") or ""),
                        "stop_reason": str(source.get("stop_reason", "") or ""),
                        "started_at": started_at,
                        "finished_at": int(source.get("finished_at", 0) or 0),
                        "planned_area_m2": float(source.get("planned_area_m2", 0.0) or 0.0),
                        "executed_area_m2": float(source.get("executed_area_m2", 0.0) or 0.0),
                        "progress": float(source.get("progress", 0.0) or 0.0),
                        "path_version": int(source.get("path_version", 0) or 0),
                        "all_completed": bool(source.get("all_completed", False)),
                        "image_format": str(source.get("image_format", "") or ""),
                        "image_width": int(source.get("image_width", 0) or 0),
                        "image_height": int(source.get("image_height", 0) or 0),
                        "image_base64": self._task_execution_preview_base64(source),
                        "trajectory_point_count": int(
                            source.get("trajectory_point_count", 0) or 0
                        ),
                    }
                )
        records.sort(
            key=lambda item: (int(item["started_at"]), str(item["execution_id"])),
            reverse=True,
        )
        result_text = "success" if valid_range else "failed"
        message = (
            "task_execution_history_ready"
            if valid_range
            else "start_time must be less than or equal to end_time"
        )
        history_json = json.dumps(
            {
                "result": result_text,
                "message": message,
                "map_id": requested_map_id,
                "task_id": requested_task_id,
                "start_time": start_time,
                "end_time": end_time,
                "total_record_count": len(records),
                "records": records,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        ).encode("utf-8")
        total_chunks = max(1, int(math.ceil(len(history_json) / float(chunk_size))))
        outputs = []
        for index in range(total_chunks):
            chunk = pb.TaskExecutionHistoryChunk()
            chunk.result = pb.RESULT_SUCCESS if valid_range else pb.RESULT_INVALID_PARAM
            chunk.message = message
            chunk.chunk_index = int(index)
            chunk.total_chunks = int(total_chunks)
            chunk.total_record_count = int(len(records))
            chunk.data = history_json[index * chunk_size : (index + 1) * chunk_size]
            chunk.start_time = int(start_time)
            chunk.end_time = int(end_time)
            outputs.append(
                (
                    chunk.SerializeToString(),
                    pb.MSG_ID_TASK_EXECUTION_HISTORY_CHUNK,
                    pb.COMP_SCHEDULER,
                )
            )
        rospy.loginfo(
            "Task execution history query: map_id=%s task_id=%s start_time=%d end_time=%d records=%d chunks=%d bytes=%d",
            requested_map_id or "<all>",
            requested_task_id or "<all>",
            start_time,
            end_time,
            len(records),
            total_chunks,
            len(history_json),
        )
        return outputs

    def build_task_trajectory_chunks(self, payload):
        pb = self.sl_link_server.pb
        request = pb.TaskTrajectoryRequest()
        request.ParseFromString(payload)
        requested_execution_id = str(request.execution_id or "").strip()
        requested_task_id = str(request.task_id or "").strip()
        start_time = int(request.start_time or 0)
        end_time = int(request.end_time or 0)
        start_index = max(0, int(request.start_index or 0))
        max_points = int(request.max_points or self._task_trajectory_default_max_points)
        max_points = max(1, min(self._task_trajectory_max_points, max_points))
        sample_step = max(1, int(request.sample_step or 1))
        chunk_size = max(256, min(4096, int(request.max_chunk_size or 2048)))

        valid_range = not (start_time > 0 and end_time > 0 and start_time > end_time)
        candidates = [
            item
            for item in list(self._task_execution_records or [])
            if isinstance(item, dict)
        ]
        candidates.sort(
            key=lambda item: (
                int(item.get("started_at", 0) or 0),
                str(item.get("execution_id", "") or ""),
            ),
            reverse=True,
        )
        execution_record = None
        if valid_range:
            for item in candidates:
                execution_id = str(item.get("execution_id", "") or "").strip()
                task_id = str(item.get("task_id", "") or "").strip()
                if requested_execution_id and execution_id != requested_execution_id:
                    continue
                if requested_task_id and task_id != requested_task_id:
                    continue
                execution_record = item
                break

        result_code = pb.RESULT_SUCCESS
        message = "task_trajectory_ready"
        if not valid_range:
            result_code = pb.RESULT_INVALID_PARAM
            message = "start_time must be less than or equal to end_time"
        elif execution_record is None:
            result_code = pb.RESULT_FAILED
            message = "task execution record not found"

        points = []
        total_point_count = 0
        selected_execution_id = ""
        selected_task_id = ""
        selected_map_id = ""
        selected_started_at_ms = 0
        if execution_record is not None:
            selected_execution_id = str(execution_record.get("execution_id", "") or "")
            selected_task_id = str(execution_record.get("task_id", "") or "")
            selected_map_id = str(execution_record.get("map_id", "") or "")
            selected_started_at_ms = int(execution_record.get("started_at_ms", 0) or 0)
            if selected_started_at_ms <= 0:
                selected_started_at_ms = int(execution_record.get("started_at", 0) or 0) * 1000
            relative_path = str(execution_record.get("trajectory_path", "") or "").strip()
            trajectory_path = relative_path
            if trajectory_path and not os.path.isabs(trajectory_path):
                trajectory_path = os.path.join(self._persist_state_dir, trajectory_path)
            if not trajectory_path or not os.path.exists(trajectory_path):
                message = "task_trajectory_not_recorded"
            else:
                try:
                    matched_index = 0
                    with self._task_trajectory_lock:
                        with open(trajectory_path, "rb") as handle:
                            while True:
                                header = handle.read(2)
                                if not header:
                                    break
                                if len(header) != 2:
                                    raise RuntimeError("truncated trajectory length prefix")
                                point_size = struct.unpack("<H", header)[0]
                                point_payload = handle.read(point_size)
                                if len(point_payload) != point_size:
                                    raise RuntimeError("truncated trajectory point payload")
                                point = pb.TaskTrajectoryPoint()
                                point.ParseFromString(point_payload)
                                timestamp_ms = selected_started_at_ms + int(point.offset_ms)
                                timestamp_sec = timestamp_ms // 1000
                                if start_time > 0 and timestamp_sec < start_time:
                                    continue
                                if end_time > 0 and timestamp_sec > end_time:
                                    continue
                                current_match = matched_index
                                matched_index += 1
                                if current_match % sample_step != 0:
                                    continue
                                sampled_index = total_point_count
                                total_point_count += 1
                                if sampled_index < start_index or len(points) >= max_points:
                                    continue
                                points.append(point)
                except Exception as exc:
                    result_code = pb.RESULT_FAILED
                    message = "task trajectory read failed: {}".format(exc)
                    points = []
                    total_point_count = 0

        next_index = min(total_point_count, start_index + len(points))
        has_more = next_index < total_point_count

        map_details = {
            "available": False,
            "message": "task map unavailable",
            "version": 0,
            "source_width": 0,
            "source_height": 0,
            "resolution": 0.0,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "frame_id": "",
            "image_format": "",
            "image_width": 0,
            "image_height": 0,
            "preview_scale_x": 0.0,
            "preview_scale_y": 0.0,
            "alignment_yaw_deg": 0.0,
            "app_rotation_deg": 0.0,
            "rotation_alignment_delta_deg": 0.0,
            "image_data": b"",
        }
        if execution_record is not None:
            map_details.update(self._load_task_execution_raw_map_snapshot(execution_record))

        point_groups = []
        point_budget = max(128, chunk_size - 256)
        current_group = []
        current_size = 0
        for point in points:
            estimated_size = int(point.ByteSize()) + 5
            if current_group and current_size + estimated_size > point_budget:
                point_groups.append(current_group)
                current_group = []
                current_size = 0
            current_group.append(point)
            current_size += estimated_size
        if current_group or not point_groups:
            point_groups.append(current_group)
        map_image_data = bytes(map_details["image_data"] or b"")
        map_image_budget = max(128, chunk_size - 512)
        map_image_groups = [
            map_image_data[offset : offset + map_image_budget]
            for offset in range(0, len(map_image_data), map_image_budget)
        ]
        total_chunks = len(point_groups) + len(map_image_groups)
        outputs = []
        total_payload_bytes = 0
        for index in range(total_chunks):
            point_group = point_groups[index] if index < len(point_groups) else []
            image_group_index = index - len(point_groups)
            image_group = (
                map_image_groups[image_group_index]
                if 0 <= image_group_index < len(map_image_groups)
                else b""
            )
            chunk = pb.TaskTrajectoryChunk()
            chunk.result = result_code
            chunk.message = message
            chunk.execution_id = selected_execution_id or requested_execution_id
            chunk.chunk_index = index
            chunk.total_chunks = total_chunks
            chunk.total_point_count = total_point_count
            chunk.returned_point_count = len(points)
            chunk.start_index = start_index
            chunk.next_index = next_index
            chunk.has_more = has_more
            chunk.started_at_ms = selected_started_at_ms
            chunk.task_id = selected_task_id or requested_task_id
            chunk.map_id = selected_map_id
            chunk.start_time = start_time
            chunk.end_time = end_time
            chunk.sample_step = sample_step
            chunk.map_available = bool(map_details["available"])
            chunk.map_message = str(map_details["message"] or "")
            chunk.map_version = max(0, int(map_details["version"]))
            chunk.map_source_width = max(0, int(map_details["source_width"]))
            chunk.map_source_height = max(0, int(map_details["source_height"]))
            chunk.map_resolution = float(map_details["resolution"])
            chunk.map_origin.x = float(map_details["origin_x"])
            chunk.map_origin.y = float(map_details["origin_y"])
            chunk.map_origin.heading_deg = 0.0
            chunk.map_frame_id = str(map_details["frame_id"] or "")
            chunk.map_image_format = str(map_details["image_format"] or "")
            chunk.map_image_width = max(0, int(map_details["image_width"]))
            chunk.map_image_height = max(0, int(map_details["image_height"]))
            chunk.map_preview_scale_x = float(map_details["preview_scale_x"])
            chunk.map_preview_scale_y = float(map_details["preview_scale_y"])
            chunk.map_image_data = image_group
            chunk.map_image_chunk_index = max(0, image_group_index)
            chunk.map_image_total_chunks = len(map_image_groups)
            chunk.alignment_yaw_deg = float(map_details["alignment_yaw_deg"])
            chunk.app_rotation_deg = float(map_details["app_rotation_deg"])
            chunk.rotation_alignment_delta_deg = float(
                map_details["rotation_alignment_delta_deg"]
            )
            for point in point_group:
                chunk.points.add().CopyFrom(point)
            serialized_chunk = chunk.SerializeToString()
            total_payload_bytes += len(serialized_chunk)
            outputs.append(
                (
                    serialized_chunk,
                    pb.MSG_ID_TASK_TRAJECTORY_CHUNK,
                    pb.COMP_SCHEDULER,
                )
            )
        rospy.loginfo(
            "Task trajectory query: execution_id=%s task_id=%s start_time=%d end_time=%d total_points=%d returned_points=%d start_index=%d next_index=%d sample_step=%d chunks=%d bytes=%d map_available=%s map_id=%s map_image_bytes=%d map_image_chunks=%d",
            selected_execution_id or requested_execution_id or "<latest>",
            selected_task_id or requested_task_id or "<any>",
            start_time,
            end_time,
            total_point_count,
            len(points),
            start_index,
            next_index,
            sample_step,
            total_chunks,
            total_payload_bytes,
            str(bool(map_details["available"])),
            selected_map_id or "<empty>",
            len(map_image_data),
            len(map_image_groups),
        )
        return outputs

    def _switch_to_localization_mode_after_map_save(self, publish_count=6):
        if self._set_map_localization_pub is None or SetMapLocalizationRequest is None:
            raise RuntimeError("set_map_localization publisher is unavailable")
        conn = int(self._set_map_localization_pub.get_num_connections())
        if conn <= 0:
            raise RuntimeError("set_map_localization has no subscribers; check slamware_ros_sdk node")
        msg = SetMapLocalizationRequest()
        msg.enabled = True
        publish_count = max(1, int(publish_count))
        for _ in range(publish_count):
            self._set_map_localization_pub.publish(msg)
            rospy.sleep(0.15)
        self._radar_mapping_mode_active = False
        self._radar_mapping_sync_remaining = 0
        rospy.loginfo(
            "Radar mapping sync stopped: localization mode active publish_count=%d interval_sec=0.15",
            publish_count,
        )

    def _switch_to_mapping_mode(self, map_kind=0, set_live_map_active=True):
        if self._set_map_update_pub is None or SetMapUpdateRequest is None:
            raise RuntimeError("set_map_update publisher is unavailable")
        conn = int(self._set_map_update_pub.get_num_connections())
        if conn <= 0:
            raise RuntimeError("set_map_update has no subscribers; check slamware_ros_sdk node")
        msg = SetMapUpdateRequest()
        msg.enabled = True
        # Default to EXPLORERMAP when map_kind is unset.
        if MapKind is not None:
            kind_value = int(map_kind) if int(map_kind or 0) != 0 else int(MapKind.EXPLORERMAP)
            kind_value = max(int(MapKind.UNKNOWN), min(int(MapKind.LOCALSLAMMAP), kind_value))
        else:
            kind_value = int(map_kind) if int(map_kind or 0) != 0 else 1
        msg.kind.kind = kind_value
        for _ in range(12):
            self._set_map_update_pub.publish(msg)
            rospy.sleep(0.15)
        self._radar_mapping_mode_active = True
        self._radar_mapping_sync_remaining = self._radar_mapping_sync_burst_count
        rospy.loginfo(
            "Radar mapping sync burst enabled: count=%d period=%.3fs topic=%s",
            self._radar_mapping_sync_remaining,
            self._radar_mapping_sync_period_sec,
            self._sync_map_topic,
        )
        if set_live_map_active:
            self._set_active_map_id(
                self._live_map_id,
                reason="map_mode_mapping_on",
                migrate_bindings=False,
                save_prev_overlay=False,
            )
        return conn, kind_value

    def handle_map_mode_request(self, payload):
        pb = self.sl_link_server.pb
        request = pb.MapModeRequest()
        request.ParseFromString(payload)
        response = pb.MapModeResponse()
        response.mode = request.mode
        response.enabled = bool(request.enabled)
        response.map_kind = int(request.map_kind)

        try:
            if request.mode == pb.MAP_MODE_MAPPING:
                if bool(request.enabled):
                    conn, kind_value = self._switch_to_mapping_mode(
                        map_kind=int(request.map_kind),
                        set_live_map_active=True,
                    )
                    response.map_kind = kind_value
                    response.result = pb.RESULT_SUCCESS
                    response.message = "mapping_mode_command_published enabled=1 subscribers={}".format(conn)
                else:
                    # aurora_ros set_map_update callback ignores msg.enabled and always enters mapping mode.
                    # To support "mapping off" without modifying aurora_ros, switch to localization mode.
                    if self._set_map_localization_pub is None or SetMapLocalizationRequest is None:
                        raise RuntimeError(
                            "mapping_off fallback requires set_map_localization publisher, but it is unavailable"
                        )
                    conn = int(self._set_map_localization_pub.get_num_connections())
                    if conn <= 0:
                        raise RuntimeError(
                            "mapping_off fallback failed: set_map_localization has no subscribers; check slamware_ros_sdk node"
                        )
                    msg = SetMapLocalizationRequest()
                    msg.enabled = True
                    for _ in range(12):
                        self._set_map_localization_pub.publish(msg)
                        rospy.sleep(0.15)
                    self._radar_mapping_mode_active = False
                    rospy.loginfo("Radar mapping sync stopped: mapping disabled via localization fallback")
                    response.result = pb.RESULT_SUCCESS
                    response.message = (
                        "mapping_off_fallback_to_localization enabled=1 subscribers={}".format(conn)
                    )
            elif request.mode == pb.MAP_MODE_LOCALIZATION:
                if self._set_map_localization_pub is None or SetMapLocalizationRequest is None:
                    raise RuntimeError("set_map_localization publisher is unavailable")
                conn = int(self._set_map_localization_pub.get_num_connections())
                if conn <= 0:
                    raise RuntimeError("set_map_localization has no subscribers; check slamware_ros_sdk node")
                msg = SetMapLocalizationRequest()
                msg.enabled = bool(request.enabled)
                for _ in range(12):
                    self._set_map_localization_pub.publish(msg)
                    rospy.sleep(0.15)
                if bool(request.enabled):
                    self._radar_mapping_mode_active = False
                    rospy.loginfo("Radar mapping sync stopped: localization mode command enabled")
                response.result = pb.RESULT_SUCCESS
                response.message = "localization_mode_command_published enabled={} subscribers={}".format(
                    int(bool(request.enabled)), conn
                )
            else:
                response.result = pb.RESULT_INVALID_PARAM
                response.message = "unsupported map mode"
        except Exception as exc:
            response.result = pb.RESULT_FAILED
            response.message = str(exc)

        return response.SerializeToString(), pb.MSG_ID_MAP_MODE_RESPONSE, pb.COMP_SCHEDULER

    def handle_map_alignment_request(self, payload):
        pb = self.sl_link_server.pb
        request = pb.MapAlignmentRequest()
        request.ParseFromString(payload)
        response = pb.MapAlignmentResponse()
        requested_map_id = str(getattr(request, "map_id", "") or "").strip()
        target_map_id = requested_map_id or self._current_map_id()
        try:
            rotation_deg = float(getattr(request, "rotation_deg", 0.0))
            if not math.isfinite(rotation_deg):
                raise RuntimeError("invalid rotation_deg")
            # APP rotation is persisted independently and must not modify the
            # startup alignment yaw captured from the radar pose.
            saved_map_id, saved_deg, saved_rad = self._set_app_rotation_for_map_id(
                target_map_id,
                rotation_deg,
            )
            response.result = pb.RESULT_SUCCESS
            response.message = "ok"
            response.map_id = str(saved_map_id)
            response.rotation_deg = float(saved_deg)
            response.rotation_rad = float(saved_rad)
            response.alignment_yaw_deg = float(
                self._alignment_yaw_deg_for_sl_link_report(saved_map_id)
            )
            response.rotation_alignment_delta_deg = float(
                self._rotation_alignment_delta_deg_for_map_id(saved_map_id)
            )
        except Exception as exc:
            response.result = pb.RESULT_FAILED
            response.message = str(exc)
            response.map_id = str(target_map_id or "")
            response.rotation_deg = float(getattr(request, "rotation_deg", 0.0) or 0.0)
            response.rotation_rad = math.radians(float(response.rotation_deg))
            response.alignment_yaw_deg = float(
                self._alignment_yaw_deg_for_sl_link_report(target_map_id)
            )
            response.rotation_alignment_delta_deg = float(
                self._rotation_alignment_delta_deg_for_map_id(target_map_id)
            )

        return response.SerializeToString(), pb.MSG_ID_MAP_ALIGNMENT_RESPONSE, pb.COMP_SCHEDULER

    def _is_sub_path(self, child_path, parent_path):
        try:
            child = os.path.realpath(child_path)
            parent = os.path.realpath(parent_path)
            return os.path.commonpath([child, parent]) == parent
        except Exception:
            return False

    def handle_map_catalog_request(self, payload):
        pb = self.sl_link_server.pb
        request = pb.MapCatalogRequest()
        request.ParseFromString(payload)
        response = pb.MapCatalogResponse()
        target_dir = os.path.abspath(self._stcm_local_dir)
        try:
            self._cleanup_orphan_map_overlay_states()
            # Query should return recorded map metadata, not raw folder filenames.
            entries = self._iter_recorded_maps(target_dir=target_dir)
            response.total_count = int(len(entries))
            bounded_count, thumbs_attached, thumbs_dropped = fill_map_catalog_response(
                response=response,
                entries=entries,
                find_record_by_id=self._find_recorded_map_by_id,
                include_thumbnails=self._map_catalog_include_thumbnails,
                max_items=self._map_catalog_max_items,
                max_thumb_b64_total=self._map_catalog_max_thumbnail_b64_total,
            )
            response.result = pb.RESULT_SUCCESS
            response.message = "ok"
            if bounded_count < len(entries):
                rospy.logwarn(
                    "MapCatalog trimmed: total=%d returned=%d (max_items=%d)",
                    len(entries),
                    bounded_count,
                    self._map_catalog_max_items,
                )
            if thumbs_dropped > 0:
                rospy.loginfo(
                    "MapCatalog thumbnail budget applied: attached=%d dropped=%d include=%s budget_b64=%d",
                    thumbs_attached,
                    thumbs_dropped,
                    str(self._map_catalog_include_thumbnails).lower(),
                    self._map_catalog_max_thumbnail_b64_total,
                )
            else:
                rospy.loginfo(
                    "MapCatalog thumbnails attached: count=%d include=%s budget_b64=%d",
                    thumbs_attached,
                    str(self._map_catalog_include_thumbnails).lower(),
                    self._map_catalog_max_thumbnail_b64_total,
                )
        except Exception as exc:
            response.result = pb.RESULT_FAILED
            response.message = str(exc)
            response.total_count = 0
        return response.SerializeToString(), pb.MSG_ID_MAP_CATALOG_RESPONSE, pb.COMP_SCHEDULER

    def handle_map_delete_request(self, payload):
        pb = self.sl_link_server.pb
        request = pb.MapDeleteRequest()
        request.ParseFromString(payload)
        response = pb.MapDeleteResponse()
        requested_map_id = str(getattr(request, "map_id", "")).strip()
        requested = requested_map_id
        if hasattr(response, "map_id"):
            response.map_id = requested_map_id
        response.deleted = False
        try:
            if not requested:
                raise RuntimeError("map_id is empty")
            record = self._find_recorded_map_by_id(requested_map_id)
            if record is None:
                raise RuntimeError("map_id not found: {}".format(requested_map_id))
            target_path = os.path.abspath(str(record.get("path", "")).strip())
            if not target_path:
                raise RuntimeError("map_id found but path is empty: {}".format(requested_map_id))
            if not os.path.exists(target_path):
                raise RuntimeError("map file not found: {}".format(target_path))
            if not os.path.isfile(target_path):
                raise RuntimeError("target is not a file: {}".format(target_path))
            record_name, record_id = self._split_map_name_and_id_from_path(target_path)
            for rec in (self._map_registry or {}).values():
                if isinstance(rec, dict) and os.path.abspath(str(rec.get("path", "")).strip()) == target_path:
                    if str(rec.get("map_id", "")).strip():
                        record_id = str(rec.get("map_id", "")).strip()
                    if str(rec.get("name", "")).strip():
                        record_name = str(rec.get("name", "")).strip()
                    break
            os.remove(target_path)
            self._unregister_saved_map(target_path)
            target_map_id = str(record_id or requested_map_id or "").strip()
            parsed_name, parsed_id = self._split_map_name_and_id_from_path(target_path)
            deleted_aliases = {
                str(requested_map_id or "").strip(),
                str(record_id or "").strip(),
                str(record_name or "").strip(),
                str(parsed_name or "").strip(),
                str(parsed_id or "").strip(),
            }
            if self._current_map_id() in deleted_aliases:
                self._set_active_map_id(
                    self._live_map_id,
                    reason="map_delete_active_fallback",
                    migrate_bindings=False,
                    save_prev_overlay=False,
                )
            overlay_aliases = {
                requested_map_id,
                record_id,
                record_name,
            }
            overlay_aliases.update([parsed_name, parsed_id])
            self._remove_map_overlay_states_for_aliases(overlay_aliases)
            # Also remove task bindings associated with this map_id.
            self._remove_planned_path_debug_for_map(target_map_id)
            self._remove_task_bindings_for_map_aliases(deleted_aliases, reason="map_delete")
            self._cleanup_orphan_map_overlay_states()
            self._save_local_state()
            if hasattr(response, "map_id"):
                response.map_id = record_id
            if hasattr(response, "map_name"):
                response.map_name = record_name
            response.deleted = True
            response.result = pb.RESULT_SUCCESS
            response.message = "deleted"
        except Exception as exc:
            response.result = pb.RESULT_FAILED
            response.message = str(exc)
        return response.SerializeToString(), pb.MSG_ID_MAP_DELETE_RESPONSE, pb.COMP_SCHEDULER

    def handle_live_map_cache_clear_request(self, payload):
        pb = self.sl_link_server.pb
        request = pb.LiveMapCacheClearRequest()
        request.ParseFromString(payload)
        response = pb.LiveMapCacheClearResponse()
        response.result = pb.RESULT_SUCCESS
        response.message = "cleared"
        try:
            # Request has no parameters by design.
            # Cleanup scope is decided by scheduler local policy.
            if self._live_map_cache_clear_mode == "memory_only":
                self.map_service.reset_overlay_regions()
                self._sync_task_regions_from_overlay()
                self._save_local_state()
            else:
                self._remove_map_overlay_state_for_map(self._live_map_id)
                # Also clear in-memory overlay immediately.
                self.map_service.reset_overlay_regions()
                self._sync_task_regions_from_overlay()
                self._save_local_state()
                self._clear_live_map_files()

            self._set_active_map_id(self._live_map_id, reason="live_map_cache_clear", migrate_bindings=False)
            response.message = "cleared(mode={})".format(self._live_map_cache_clear_mode)
        except Exception as exc:
            response.result = pb.RESULT_FAILED
            response.message = str(exc)
        return response.SerializeToString(), pb.MSG_ID_LIVE_MAP_CACHE_CLEAR_RESPONSE, pb.COMP_SCHEDULER

    def _send_radar_map_cache_clear(self, wait_after_sec=0.0):
        if self._clear_map_pub is None or ClearMapRequest is None:
            raise RuntimeError("clear_map publisher is unavailable")
        conn = int(self._clear_map_pub.get_num_connections())
        if conn <= 0:
            raise RuntimeError("clear_map has no subscribers; check slamware_ros_sdk node")

        msg = ClearMapRequest()
        if MapKind is not None:
            msg.kind.kind = int(MapKind.EXPLORERMAP)
        self._clear_map_pub.publish(msg)
        rospy.loginfo("Radar map cache clear sent: topic=%s kind=EXPLORERMAP", self._clear_map_topic)
        self._send_radar_map_sync()
        if wait_after_sec > 0.0:
            rospy.sleep(float(wait_after_sec))

    def _send_radar_map_sync(self):
        if self._sync_map_pub is None or SyncMapRequest is None:
            rospy.logwarn("sync_map publisher is unavailable; skip radar map sync request")
            return False
        conn = int(self._sync_map_pub.get_num_connections())
        if conn <= 0:
            rospy.logwarn("sync_map has no subscribers; skip radar map sync request")
            return False

        self._sync_map_pub.publish(SyncMapRequest())
        rospy.loginfo("Radar map sync sent: topic=%s", self._sync_map_topic)
        return True

    def handle_radar_map_cache_clear_request(self, payload):
        pb = self.sl_link_server.pb
        request = pb.RadarMapCacheClearRequest()
        request.ParseFromString(payload)
        response = pb.RadarMapCacheClearResponse()
        response.result = pb.RESULT_SUCCESS
        response.message = "radar_map_clear_sent_and_mapping_on"
        try:
            self._send_radar_map_cache_clear()
            conn, kind_value = self._switch_to_mapping_mode(set_live_map_active=True)
            rospy.loginfo(
                "Radar map cache clear followed by sync_map and mapping mode: subscribers=%d map_kind=%d",
                conn,
                kind_value,
            )
        except Exception as exc:
            response.result = pb.RESULT_FAILED
            response.message = str(exc)
        return response.SerializeToString(), pb.MSG_ID_RADAR_MAP_CACHE_CLEAR_RESPONSE, pb.COMP_SCHEDULER

    def handle_radar_system_status_request(self, payload):
        pb = self.sl_link_server.pb
        request = pb.RadarSystemStatusRequest()
        request.ParseFromString(payload)
        response = pb.RadarSystemStatusResponse()
        with self._radar_system_status_lock:
            response.available = bool(self._radar_system_status_available)
            response.status = str(self._radar_system_status)
            response.timestamp_ns = int(self._radar_system_status_timestamp_ns)
        if response.available:
            response.result = pb.RESULT_SUCCESS
            response.message = "ok"
        else:
            response.result = pb.RESULT_FAILED
            response.message = "radar_system_status_unavailable"
        rospy.loginfo(
            "SL-LinkA radar system status response: available=%s status=%s timestamp_ns=%d",
            str(response.available),
            response.status or "<empty>",
            int(response.timestamp_ns),
        )
        return (
            response.SerializeToString(),
            pb.MSG_ID_RADAR_SYSTEM_STATUS_RESPONSE,
            pb.COMP_SYSTEM,
        )

    def handle_radar_map_sync_request(self, payload):
        pb = self.sl_link_server.pb
        request = pb.RadarMapSyncRequest()
        request.ParseFromString(payload)
        response = pb.RadarMapSyncResponse()
        try:
            response.sent = bool(self._send_radar_map_sync())
            if response.sent:
                response.result = pb.RESULT_SUCCESS
                response.message = "radar_map_sync_sent"
            else:
                response.result = pb.RESULT_FAILED
                response.message = "radar_map_sync_unavailable"
        except Exception as exc:
            response.result = pb.RESULT_FAILED
            response.message = str(exc)
            response.sent = False
        rospy.loginfo(
            "SL-LinkA radar map sync response: sent=%s result=%s message=%s",
            str(response.sent),
            str(response.result),
            response.message,
        )
        return (
            response.SerializeToString(),
            pb.MSG_ID_RADAR_MAP_SYNC_RESPONSE,
            pb.COMP_SYSTEM,
        )

    def _request_radar_relocalization(self, reason="manual", raise_on_error=True):
        try:
            if RadarRelocalizationService is None:
                raise RuntimeError("slamware_ros_sdk/RelocalizationRequest service is unavailable")
            rospy.wait_for_service(
                self._radar_relocalization_service,
                timeout=self._radar_relocalization_service_wait_sec,
            )
            if self._radar_relocalization_proxy is None:
                self._radar_relocalization_proxy = rospy.ServiceProxy(
                    self._radar_relocalization_service,
                    RadarRelocalizationService,
                )
            service_response = self._radar_relocalization_proxy()
            accepted = bool(service_response.success)
            if accepted:
                request_timestamp_ns = int(time.time() * 1.0e9)
                with self._radar_relocalization_status_lock:
                    self._radar_relocalization_raw_available = True
                    self._radar_relocalization_raw_status = "RelocalizationRunning"
                    self._radar_relocalization_raw_timestamp_ns = request_timestamp_ns
                    self._radar_relocalization_aggregate_status = "running"
                    self._radar_relocalization_aggregate_timestamp_ns = request_timestamp_ns
            rospy.loginfo(
                "Radar relocalization requested: service=%s accepted=%s reason=%s",
                self._radar_relocalization_service,
                str(accepted),
                str(reason),
            )
            return accepted
        except Exception as exc:
            self._radar_relocalization_proxy = None
            if raise_on_error:
                raise
            rospy.logwarn(
                "Radar relocalization request failed: service=%s reason=%s error=%s",
                self._radar_relocalization_service,
                str(reason),
                exc,
            )
            return False

    def handle_radar_relocalization_request(self, payload):
        pb = self.sl_link_server.pb
        request = pb.RadarRelocalizationRequest()
        request.ParseFromString(payload)
        response = pb.RadarRelocalizationResponse()
        response.accepted = False
        try:
            response.accepted = self._request_radar_relocalization(
                reason="sl_link_request",
                raise_on_error=True,
            )
            response.result = pb.RESULT_SUCCESS if response.accepted else pb.RESULT_FAILED
            response.message = (
                "radar_relocalization_accepted"
                if response.accepted
                else "radar_relocalization_rejected"
            )
        except Exception as exc:
            response.result = pb.RESULT_FAILED
            response.message = str(exc)
            response.accepted = False
        response.status = self._radar_relocalization_snapshot()["status"]
        rospy.loginfo(
            "SL-LinkA radar relocalization response: service=%s accepted=%s status=%s "
            "result=%s message=%s",
            self._radar_relocalization_service,
            str(response.accepted),
            response.status,
            str(response.result),
            response.message,
        )
        return (
            response.SerializeToString(),
            pb.MSG_ID_RADAR_RELOCALIZATION_RESPONSE,
            pb.COMP_SYSTEM,
        )

    def handle_radar_relocalization_status_request(self, payload):
        pb = self.sl_link_server.pb
        request = pb.RadarRelocalizationStatusRequest()
        request.ParseFromString(payload)
        response = pb.RadarRelocalizationStatusResponse()
        snapshot = self._radar_relocalization_snapshot()
        response.raw_status = str(snapshot["raw_status"])
        response.timestamp_ns = int(snapshot["timestamp_ns"])
        rospy.loginfo(
            "SL-LinkA radar relocalization status response: raw_status=%s timestamp_ns=%d",
            response.raw_status or "<empty>",
            response.timestamp_ns,
        )
        return (
            response.SerializeToString(),
            pb.MSG_ID_RADAR_RELOCALIZATION_STATUS_RESPONSE,
            pb.COMP_SYSTEM,
        )

    def handle_map_preview_request(self, payload):
        pb = self.sl_link_server.pb
        request = pb.MapPreviewRequest()
        request.ParseFromString(payload)
        response = pb.MapPreviewResponse()
        try:
            requested_map_id = str(getattr(request, "map_id", "") or "").strip()
            has_saved_maps = any(isinstance(item, dict) for item in (self._map_registry or {}).values())
            # If there is no saved map at all, always fallback to LIVE_MAP preview.
            # This avoids UI preview failure when APP still carries an old/non-live map_id.
            if has_saved_maps:
                map_id_ok, _ = self._validate_requested_map_id(requested_map_id, "MapPreviewRequest")
                if not map_id_ok:
                    raise RuntimeError("map_id mismatch with current map")
            else:
                if requested_map_id and requested_map_id not in (self._live_map_id, DEFAULT_LIVE_MAP_ID):
                    rospy.loginfo(
                        "MapPreviewRequest fallback to LIVE_MAP: no saved map available, requested_map_id=%s",
                        requested_map_id,
                    )
            requested_edge = request.max_edge if request.max_edge > 0 else PREVIEW_MAX_EDGE_CAP
            max_edge = self._sanitize_preview_edge(requested_edge, cap_edge=self._preview_max_edge_cap)
            preview_service, offline_preview = self._map_service_for_preview(requested_map_id)
            snapshot = preview_service.create_preview(
                self.aurora_bridge.get_pose(),
                max_edge,
                request.image_format or "jpg",
                request.include_overlay,
            )
            rospy.loginfo(
                "MapPreviewRequest raw map preview without rotation: source=%s map_id=%s frame_id=%s preview=%sx%s",
                "saved" if offline_preview else "live",
                requested_map_id or self._live_map_id,
                snapshot.frame_id or self._live_map_source_frame,
                int(snapshot.width),
                int(snapshot.height),
            )
            response.result = pb.RESULT_SUCCESS
            response.message = "ok_offline_map" if offline_preview else "ok"
            response.map_version = snapshot.map_version
            response.width = snapshot.width
            response.height = snapshot.height
            response.resolution = snapshot.resolution
            response.origin.x = snapshot.origin_x
            response.origin.y = snapshot.origin_y
            response.frame_id = snapshot.frame_id
            response.image_data = snapshot.preview_data
            response.overlay_json = snapshot.overlay_json
            preview_w, preview_h, _ = self._preview_meta(snapshot.width, snapshot.height, max_edge)
            response.preview_scale_x = float(preview_w) / float(max(1, snapshot.width))
            response.preview_scale_y = float(preview_h) / float(max(1, snapshot.height))
            self._apply_localization_covariance(response)
            response_map_id = (
                requested_map_id
                if offline_preview
                else self._current_map_id()
            )
            self._apply_alignment_yaw_to_response(response, response_map_id)
        except Exception as exc:
            response.result = pb.RESULT_FAILED
            response.message = str(exc)
            rospy.logwarn(
                "MapPreviewRequest failed: map_id=%s error=%s",
                requested_map_id or self._live_map_id,
                exc,
            )
            self._apply_localization_covariance(response)
            self._apply_alignment_yaw_to_response(
                response,
                requested_map_id or self._current_map_id(),
            )
        return response.SerializeToString(), pb.MSG_ID_MAP_PREVIEW_RESPONSE, pb.COMP_SCHEDULER

    @staticmethod
    def _fill_polygon_region_message(pb_region, region, default_region_type):
        pb_region.name = str(region.get("name", "") or "")
        pb_region.region_id = str(region.get("region_id", "") or "")
        pb_region.priority = max(
            0,
            int(region.get("order_index", region.get("priority", 0)) or 0),
        )
        pb_region.enabled = bool(region.get("enabled", True))
        pb_region.color_argb = max(0, int(region.get("color_argb", 0) or 0))
        pb_region.closed = bool(region.get("closed", True))
        pb_region.region_type = int(region.get("region_type", default_region_type))
        pb_region.global_direction = str(region.get("global_direction", "") or "")
        for point in list(region.get("points", []) or []):
            if not isinstance(point, dict):
                continue
            pb_point = pb_region.points.add()
            pb_point.x = float(point.get("x", 0.0) or 0.0)
            pb_point.y = float(point.get("y", 0.0) or 0.0)

    @staticmethod
    def _fill_pose2d_message(pb_pose, pose):
        pb_pose.x = float(pose.get("x", 0.0) or 0.0)
        pb_pose.y = float(pose.get("y", 0.0) or 0.0)
        pb_pose.heading_deg = float(pose.get("heading_deg", 0.0) or 0.0)

    def handle_map_region_point_request(self, payload):
        pb = self.sl_link_server.pb
        request = pb.MapRegionPointRequest()
        request.ParseFromString(payload)
        response = pb.MapRegionPointResponse()
        requested_map_id = str(request.map_id or "").strip()
        effective_map_id = requested_map_id or self._current_map_id() or self._live_map_id

        try:
            if self._is_live_map_id(effective_map_id) or effective_map_id == self._current_map_id():
                region_service = self.map_service
            else:
                if self._find_recorded_map_by_id(effective_map_id) is None:
                    raise RuntimeError("map_id not found: {}".format(effective_map_id))
                region_service = MapService()
                region_service.load_local_state(self._map_state_dir(effective_map_id))

            regions = region_service.get_overlay_regions() or {}
            response.result = pb.RESULT_SUCCESS
            response.message = "region_point_info_ready"
            response.map_id = effective_map_id
            response.map_version = region_service.get_map_version()

            for region in list(regions.get("work_regions", []) or []):
                item = response.work_regions.add()
                self._fill_polygon_region_message(item.region, region, pb.REGION_TYPE_WORK)
                start_pose = region.get("start_pose", {})
                if isinstance(start_pose, dict) and start_pose:
                    item.start_pose_available = True
                    self._fill_pose2d_message(item.start_pose, start_pose)
                end_pose = region.get("end_pose", {})
                if isinstance(end_pose, dict) and end_pose:
                    item.end_pose_available = True
                    self._fill_pose2d_message(item.end_pose, end_pose)

            for region in list(regions.get("obstacle_regions", []) or []):
                self._fill_polygon_region_message(
                    response.obstacle_regions.add(),
                    region,
                    pb.REGION_TYPE_OBSTACLE,
                )
            for region in list(regions.get("erase_regions", []) or []):
                self._fill_polygon_region_message(
                    response.erase_regions.add(),
                    region,
                    pb.REGION_TYPE_ERASE,
                )
            crop_region = regions.get("crop_region")
            if isinstance(crop_region, dict) and crop_region:
                response.crop_region_available = True
                self._fill_polygon_region_message(
                    response.crop_region,
                    crop_region,
                    pb.REGION_TYPE_CROP,
                )
            rospy.loginfo(
                "SL-LinkA map region/point response: map_id=%s map_version=%d "
                "work=%d obstacle=%d erase=%d crop=%s",
                response.map_id,
                response.map_version,
                len(response.work_regions),
                len(response.obstacle_regions),
                len(response.erase_regions),
                str(response.crop_region_available),
            )
        except Exception as exc:
            response.result = pb.RESULT_FAILED
            response.message = str(exc)
            response.map_id = effective_map_id
            rospy.logwarn(
                "SL-LinkA map region/point query failed: map_id=%s error=%s",
                effective_map_id,
                exc,
            )
        return (
            response.SerializeToString(),
            pb.MSG_ID_MAP_REGION_POINT_RESPONSE,
            pb.COMP_SCHEDULER,
        )

    def handle_map_edit_command(self, payload):
        pb = self.sl_link_server.pb
        request = pb.MapEditCommand()
        request.ParseFromString(payload)
        # Legacy APP versions omit map_id from MapEditCommand. In that case
        # the edit belongs to the map currently opened by the APP. Switching
        # back to LIVE_MAP here would separate regions from the saved rotation.
        map_id_ok, edit_map_id = self._validate_requested_map_id(
            getattr(request, "map_id", ""),
            "MapEditCommand",
            keep_current_on_empty=True,
        )
        if not map_id_ok:
            response = pb.MapEditResponse()
            response.result = pb.RESULT_INVALID_PARAM
            response.message = "map_id mismatch with current map"
            map_info = self.map_service.get_map_info()
            response.map_version = int(map_info.get("map_version", 0)) if map_info else 0
            return response.SerializeToString(), pb.MSG_ID_MAP_EDIT_RESPONSE, pb.COMP_SCHEDULER
        region_points = [{"x": p.x, "y": p.y} for p in request.region.points]
        polygon_points = [{"x": p.x, "y": p.y} for p in request.polygon]
        operation_map = {
            pb.MAP_EDIT_OP_UPSERT_WORK_REGION: "UPSERT_WORK_REGION",
            pb.MAP_EDIT_OP_UPSERT_OBSTACLE_REGION: "UPSERT_OBSTACLE_REGION",
            pb.MAP_EDIT_OP_UPSERT_ERASE_REGION: "UPSERT_ERASE_REGION",
            pb.MAP_EDIT_OP_UPSERT_CROP_REGION: "UPSERT_CROP_REGION",
            pb.MAP_EDIT_OP_DELETE_REGION: "DELETE_REGION",
            pb.MAP_EDIT_OP_PAINT_FREE: "PAINT_FREE",
            pb.MAP_EDIT_OP_PAINT_OCCUPIED: "PAINT_OCCUPIED",
            pb.MAP_EDIT_OP_PAINT_UNKNOWN: "PAINT_UNKNOWN",
            pb.MAP_EDIT_OP_CLEAR_OVERLAY_PATCH: "CLEAR_OVERLAY_PATCH",
        }
        operation_name = operation_map.get(request.operation, "UNKNOWN")
        request_start_pose = {}
        request_end_pose = {}
        if request.HasField("start_pose"):
            request_start_pose = {
                "x": float(request.start_pose.x),
                "y": float(request.start_pose.y),
                "heading_deg": float(request.start_pose.heading_deg),
            }
        if request.HasField("end_pose"):
            request_end_pose = {
                "x": float(request.end_pose.x),
                "y": float(request.end_pose.y),
                "heading_deg": float(request.end_pose.heading_deg),
            }
        region_id_text = (request.region.region_id or "").strip()
        region_name_text = (request.region.name or request.region_name or "").strip().lower()
        target_region_id_text = (request.target_region_id or "").strip()
        region_type_value = int(request.region.region_type)
        target_region_type_value = int(request.target_region_type)
        crop_region_type = int(getattr(pb, "REGION_TYPE_CROP", 4))
        # Robust crop detection across proto/version mismatches:
        # if id/name clearly indicates crop, force type=4.
        if region_id_text == "crop_region_1" or region_name_text.startswith("crop"):
            region_type_value = crop_region_type
        if target_region_id_text == "crop_region_1":
            target_region_type_value = crop_region_type
        edit_alignment_yaw = (
            self._alignment_yaw_for_map_id(self._current_map_id())
            if self._live_map_align_to_initial_yaw and self._sl_link_input_points_frame == self._live_map_aligned_frame
            else None
        )
        if edit_alignment_yaw is not None:
            region_points = self._points_from_aligned_to_source_map(region_points, edit_alignment_yaw)
            polygon_points = self._points_from_aligned_to_source_map(polygon_points, edit_alignment_yaw)
            request_start_pose = self._pose_from_aligned_to_source_map(request_start_pose, edit_alignment_yaw)
            request_end_pose = self._pose_from_aligned_to_source_map(request_end_pose, edit_alignment_yaw)
            rospy.loginfo(
                "MapEdit coordinates converted: input_frame=%s output_frame=%s alignment_yaw=%.6f",
                self._live_map_aligned_frame,
                self._live_map_source_frame,
                float(edit_alignment_yaw),
            )
        else:
            rospy.loginfo(
                "MapEdit coordinates kept: input_frame=%s output_frame=%s",
                self._sl_link_input_points_frame,
                self._live_map_source_frame,
            )

        def _format_points(points, max_points=64):
            if not points:
                return "[]"
            shown = points[:max_points]
            body = ", ".join("({:.3f},{:.3f})".format(float(item["x"]), float(item["y"])) for item in shown)
            if len(points) > max_points:
                body += ", ... ({} more)".format(len(points) - max_points)
            return "[" + body + "]"

        rospy.loginfo(
            "MapEdit request: op=%s edit_id=%s map_id=%s region_id=%s region_name=%s region_type=%d region_points=%d polygon_points=%d target_region_id=%s target_region_type=%d brush_radius=%.3f",
            operation_name,
            (request.edit_id or "").strip(),
            str(getattr(request, "map_id", "")).strip() or "<empty>",
            region_id_text or "<empty>",
            (request.region.name or request.region_name or "").strip() or "<empty>",
            region_type_value,
            len(region_points),
            len(polygon_points),
            target_region_id_text or "<empty>",
            target_region_type_value,
            float(request.brush_radius),
        )
        rospy.loginfo(
            "MapEdit planning rotation binding: map_id=%s app_rotation_deg=%.3f alignment_yaw_deg=%.3f delta_deg=%.3f effective_planning_angle_deg=%.3f",
            edit_map_id or self._live_map_id,
            self._app_rotation_deg_for_map_id(edit_map_id),
            self._alignment_yaw_deg_for_sl_link_report(edit_map_id),
            self._rotation_alignment_delta_deg_for_map_id(edit_map_id),
            (
                self._alignment_yaw_deg_for_sl_link_report(edit_map_id)
                + self._rotation_alignment_delta_deg_for_map_id(edit_map_id)
            ),
        )
        if operation_name in ("UPSERT_WORK_REGION", "UPSERT_OBSTACLE_REGION", "UPSERT_ERASE_REGION", "UPSERT_CROP_REGION"):
            rospy.loginfo(
                "MapEdit region points: op=%s region_id=%s points=%s",
                operation_name,
                region_id_text or "<empty>",
                _format_points(region_points),
            )
        if request.operation == pb.MAP_EDIT_OP_UPSERT_WORK_REGION:
            if request_start_pose:
                rospy.loginfo(
                    "MapEdit work_region start_pose: x=%.3f y=%.3f heading=%.1f",
                    float(request_start_pose.get("x", 0.0)),
                    float(request_start_pose.get("y", 0.0)),
                    float(request_start_pose.get("heading_deg", 0.0)),
                )
            else:
                rospy.loginfo("MapEdit work_region start_pose: <empty>")
            if request_end_pose:
                rospy.loginfo(
                    "MapEdit work_region end_pose: x=%.3f y=%.3f heading=%.1f",
                    float(request_end_pose.get("x", 0.0)),
                    float(request_end_pose.get("y", 0.0)),
                    float(request_end_pose.get("heading_deg", 0.0)),
                )
            else:
                rospy.loginfo("MapEdit work_region end_pose: <empty>")
        if request.operation == pb.MAP_EDIT_OP_UPSERT_WORK_REGION and region_points:
            if (not request_start_pose) or (not request_end_pose):
                xs = [float(item.get("x", 0.0)) for item in region_points]
                ys = [float(item.get("y", 0.0)) for item in region_points]
                min_x, max_x = min(xs), max(xs)
                min_y, max_y = min(ys), max(ys)
                if not request_start_pose:
                    request_start_pose = {"x": min_x, "y": max_y, "heading_deg": 0.0}
                if not request_end_pose:
                    request_end_pose = {"x": max_x, "y": min_y, "heading_deg": 0.0}
                rospy.loginfo(
                    "MapEdit UPSERT_WORK auto-fill start/end: start=(%.3f,%.3f) end=(%.3f,%.3f)",
                    float(request_start_pose.get("x", 0.0)),
                    float(request_start_pose.get("y", 0.0)),
                    float(request_end_pose.get("x", 0.0)),
                    float(request_end_pose.get("y", 0.0)),
                )
        # Region-level planning direction:
        # do not auto-infer; consume direction from request if protocol provides it.
        # If absent, keep existing region direction (MapService upsert fallback), or default x for new region.
        region_global_direction = ""
        try:
            candidate = str(getattr(request.region, "global_direction", "") or "").strip().lower()
            if is_planning_direction(candidate):
                region_global_direction = candidate
        except Exception:
            region_global_direction = ""

        region_payload = {
            "name": request.region.name or request.region_name,
            "points": region_points,
            "region_id": request.region.region_id,
            "priority": int(request.region.priority),
            "enabled": bool(request.region.enabled),
            "color_argb": int(request.region.color_argb),
            "closed": bool(request.region.closed),
            "region_type": region_type_value,
            "start_pose": request_start_pose,
            "end_pose": request_end_pose,
        }
        if region_global_direction:
            region_payload["global_direction"] = region_global_direction

        success, message, map_version = self.map_service.apply_edit(
            {
                "operation": operation_name,
                "region_name": request.region_name,
                "region": region_payload,
                "polygon": polygon_points,
                "brush_radius": request.brush_radius,
                "paint_value": request.paint_value,
                "target_region_id": request.target_region_id,
                "target_region_type": target_region_type_value,
            }
        )
        if success:
            if request.operation == pb.MAP_EDIT_OP_UPSERT_WORK_REGION:
                selected_id = (request.region.region_id or "").strip()
                if selected_id:
                    self.task_config.active_work_region_id = selected_id
            elif request.operation == pb.MAP_EDIT_OP_DELETE_REGION:
                target_type = int(request.target_region_type)
                target_id = (request.target_region_id or "").strip()
                if target_id and target_type in (0, int(pb.REGION_TYPE_WORK)):
                    if self.task_config.active_work_region_id == target_id:
                        self.task_config.active_work_region_id = ""
            # Keep task_config cache consistent with overlay source-of-truth immediately.
            self._sync_task_regions_from_overlay()
        if success and self.state in (SchedulerState.READY, SchedulerState.PLANNING):
            self._plan_current_task()
        elif success and self.state == SchedulerState.RUNNING:
            self.replan_requested = True
        if success:
            self._save_local_state(allow_empty_saved_map_overlay=True)
            rospy.loginfo(
                "MapEdit applied: op=%s success=true map_version=%d replan_requested=%s active_work_region_id=%s",
                operation_name,
                int(map_version),
                str(self.replan_requested).lower(),
                self.task_config.active_work_region_id or "<empty>",
            )
        else:
            rospy.logwarn(
                "MapEdit applied: op=%s success=false map_version=%d reason=%s",
                operation_name,
                int(map_version),
                message,
            )

        response = pb.MapEditResponse()
        response.result = pb.RESULT_SUCCESS if success else pb.RESULT_FAILED
        response.message = message
        response.map_version = map_version

        status = pb.MapEditStatusReport()
        status.map_version = map_version
        status.applied_to_planner = success
        status.message = "planner_refresh_requested" if self.replan_requested else message

        return [
            (response.SerializeToString(), pb.MSG_ID_MAP_EDIT_RESPONSE, pb.COMP_SCHEDULER),
            (status.SerializeToString(), pb.MSG_ID_MAP_EDIT_STATUS_REPORT, pb.COMP_SCHEDULER),
        ]

    def handle_video_stream_request(self, payload):
        pb = self.sl_link_server.pb
        request = pb.VideoStreamInfoRequest()
        request.ParseFromString(payload)
        state = self.media_streamer.get_state()
        local_state = self.local_stream_server.get_state()
        response = pb.VideoStreamInfoResponse()
        response.result = pb.RESULT_SUCCESS
        if state.online:
            response.message = "ok"
        elif not self.local_stream_server.ffmpeg_available():
            response.message = "ffmpeg_not_installed"
        elif not self.local_stream_server.mediamtx_available():
            response.message = "mediamtx_not_installed"
        else:
            response.message = "ok"
        if state.online:
            response.stream_url = state.stream_url
            response.codec = state.codec
            response.width = state.width
            response.height = state.height
            response.online = state.online
            response.utc_time = state.last_update_utc
        else:
            response.stream_url = local_state.stream_url or state.stream_url
            response.codec = local_state.codec
            response.width = local_state.width
            response.height = local_state.height
            response.online = local_state.online
            response.utc_time = local_state.last_update_utc
        return response.SerializeToString(), pb.MSG_ID_VIDEO_STREAM_INFO_RESPONSE, pb.COMP_SCHEDULER

    def handle_path_plan_request(self, payload, include_preview=True):
        pb = self.sl_link_server.pb
        request = pb.PathPlanRequest()
        try:
            request.ParseFromString(payload)
        except Exception:
            return self._handle_path_plan_request_impl(payload, include_preview=include_preview)

        effective_map_id = self._effective_request_map_id_for_task(request)
        map_id_ok, current_map_id = self._validate_requested_map_id(
            effective_map_id,
            "PathPlanRequest",
            keep_current_on_empty=False,
        )
        if (not map_id_ok) or self._is_live_map_id(current_map_id):
            return self._handle_path_plan_request_impl(payload, include_preview=include_preview)

        original_service = self.map_service
        original_cache_key = self._preview_snapshot_cache_key
        original_cache = self._preview_snapshot_cache
        try:
            self.map_service = self._build_offline_map_service_for_map(current_map_id)
            self._preview_snapshot_cache_key = None
            self._preview_snapshot_cache = None
            map_info = self.map_service.get_map_info() or {}
            raw_grid_yaml, raw_grid_image = self._saved_raw_grid_paths_for_map(current_map_id)
            rospy.loginfo(
                "PathPlanRequest map source: source=offline_raw_grid map_id=%s raw_grid_yaml=%s raw_grid_image=%s map_size=%sx%s resolution=%.4f",
                current_map_id,
                raw_grid_yaml or "<empty>",
                raw_grid_image or "<empty>",
                int(map_info.get("width", 0) or 0),
                int(map_info.get("height", 0) or 0),
                float(map_info.get("resolution", 0.0) or 0.0),
            )
            return self._handle_path_plan_request_impl(payload, include_preview=include_preview)
        finally:
            self.map_service = original_service
            self._preview_snapshot_cache_key = original_cache_key
            self._preview_snapshot_cache = original_cache

    def _handle_path_plan_request_impl(self, payload, include_preview=True):
        pb = self.sl_link_server.pb
        request = pb.PathPlanRequest()
        request.ParseFromString(payload)
        if str(request.task_id or "").strip():
            return self._handle_path_plan_request_impl_core(
                payload,
                task_scoped=True,
                include_preview=include_preview,
            )

        # An ad-hoc request must not inherit selections, repeats or temporary
        # obstacles from the last configured task. Keep the generated path,
        # but restore the formal task configuration after building the reply.
        original_task_config = self.task_config
        self.task_config = deepcopy(original_task_config)
        self.task_config.task_id = ""
        self.task_config.selected_work_region_ids = []
        self.task_config.region_repeat_config = {}
        self.task_config.active_work_region_id = ""
        self.task_config.start_pose = {}
        self.task_config.end_pose = {}
        try:
            rospy.loginfo(
                "PathPlanRequest without task_id: plan from current map regions only; "
                "task selection, repeats and temporary obstacles are excluded"
            )
            return self._handle_path_plan_request_impl_core(
                payload,
                task_scoped=False,
                include_preview=include_preview,
            )
        finally:
            self.task_config = original_task_config

    def _handle_path_plan_request_impl_core(
        self,
        payload,
        task_scoped=True,
        include_preview=True,
    ):
        t0_all = time.perf_counter()
        pb = self.sl_link_server.pb
        request = pb.PathPlanRequest()
        request.ParseFromString(payload)
        t1_parse = time.perf_counter()

        effective_map_id = self._effective_request_map_id_for_task(request)
        map_id_ok, current_map_id = self._validate_requested_map_id(
            effective_map_id,
            "PathPlanRequest",
            keep_current_on_empty=False,
        )
        if not map_id_ok:
            response = pb.PathPlanResponse()
            response.request_id = request.request_id
            response.task_id = self.task_config.task_id
            response.result = pb.RESULT_FAILED
            response.message = "map_id mismatch with current map"
            response.planned = False
            return response.SerializeToString(), pb.MSG_ID_PATH_PLAN_RESPONSE, pb.COMP_SCHEDULER

        if request.task_id:
            self.task_config.task_id = request.task_id
        self.task_config.map_id = current_map_id

        # PathPlanRequest planning scope:
        # True  -> plan all configured/selected regions in order
        # False -> plan by current selected/active policy
        self._sync_task_regions_from_overlay(update_task_binding=task_scoped)
        if task_scoped:
            self._merge_task_obstacle_regions_for_planning(
                current_map_id,
                self.task_config.task_id,
            )
            self._sync_task_map_binding(update_binding=False)
        total_regions = len(self.task_config.work_regions or [])
        selected_ids = self._effective_selected_work_region_ids(
            sorted(
                list(self.task_config.work_regions or []),
                key=lambda region: int(region.get("order_index", 0)),
            ),
        )
        # PathPlanRequest has no region-repeat fields. If the APP is doing a
        # task plan and every selected region is repeat=1, older clients may
        # omit TaskConfig/region_repeats entirely. In that case the planner
        # used to treat the request as a plain preview and skipped the first
        # robot->region connector. Preserve the selected order and mark it as a
        # task plan when task_id is present.
        if request.task_id and selected_ids and not list(self.task_config.selected_work_region_ids or []):
            self.task_config.selected_work_region_ids = list(selected_ids)
            repeat_cfg = dict(self.task_config.region_repeat_config or {})
            for rid in selected_ids:
                rid_text = str(rid).strip()
                if rid_text and rid_text not in repeat_cfg:
                    repeat_cfg[rid_text] = 1
            self.task_config.region_repeat_config = repeat_cfg
            rospy.loginfo(
                "PathPlanRequest task selection fallback: task_id=%s selected_regions=%s repeat_default=1",
                request.task_id or "<empty>",
                ",".join(selected_ids),
            )
        use_all = bool(self._path_plan_request_use_all_regions)
        request_start_pose = {}
        request_end_pose = {}
        request_global_direction = str(getattr(request, "global_direction", "") or "").strip().lower()
        request_global_direction = normalize_planning_direction(request_global_direction)
        self.task_config.global_direction = request_global_direction
        if request.HasField("start_pose"):
            request_start_pose = {
                "x": float(request.start_pose.x),
                "y": float(request.start_pose.y),
                "heading_deg": float(request.start_pose.heading_deg),
            }
        if request.HasField("end_pose"):
            request_end_pose = {
                "x": float(request.end_pose.x),
                "y": float(request.end_pose.y),
                "heading_deg": float(request.end_pose.heading_deg),
            }
        plan_alignment_yaw = (
            self._alignment_yaw_for_map_id(current_map_id)
            if self._live_map_align_to_initial_yaw and self._sl_link_input_points_frame == self._live_map_aligned_frame
            else None
        )
        if plan_alignment_yaw is not None:
            request_start_pose = self._pose_from_aligned_to_source_map(request_start_pose, plan_alignment_yaw)
            request_end_pose = self._pose_from_aligned_to_source_map(request_end_pose, plan_alignment_yaw)
            rospy.loginfo(
                "PathPlanRequest poses converted: input_frame=%s output_frame=%s alignment_yaw=%.6f",
                self._live_map_aligned_frame,
                self._live_map_source_frame,
                float(plan_alignment_yaw),
            )
        else:
            rospy.loginfo(
                "PathPlanRequest poses kept: input_frame=%s output_frame=%s",
                self._sl_link_input_points_frame,
                self._live_map_source_frame,
            )
        rospy.loginfo(
            "PathPlanRequest: request_id=%s active_work_region_id=%s force_replan=%s use_all_regions=%s work_region_count=%d has_start=%s has_end=%s global_direction=%s",
            request.request_id or "<empty>",
            self.task_config.active_work_region_id or "<empty>",
            str(bool(request.force_replan)).lower(),
            str(bool(use_all)).lower(),
            int(total_regions),
            str(bool(request_start_pose)).lower(),
            str(bool(request_end_pose)).lower(),
            request_global_direction,
        )
        rospy.loginfo(
            "PathPlanRequest selection: requested_map_id=%s effective_map_id=%s map_id=%s task_id=%s selected_regions=%s region_repeat_config=%s",
            str(getattr(request, "map_id", "")).strip() or "<empty>",
            effective_map_id or "<empty>",
            self.task_config.map_id or "<empty>",
            self.task_config.task_id or "<empty>",
            ",".join(selected_ids) if selected_ids else "<all_valid_regions>",
            json.dumps(self.task_config.region_repeat_config or {}, ensure_ascii=False, sort_keys=True),
        )
        if request_start_pose:
            rospy.loginfo(
                "PathPlanRequest start_pose: x=%.3f y=%.3f heading=%.1f",
                float(request_start_pose.get("x", 0.0)),
                float(request_start_pose.get("y", 0.0)),
                float(request_start_pose.get("heading_deg", 0.0)),
            )
        if request_end_pose:
            rospy.loginfo(
                "PathPlanRequest end_pose: x=%.3f y=%.3f heading=%.1f",
                float(request_end_pose.get("x", 0.0)),
                float(request_end_pose.get("y", 0.0)),
                float(request_end_pose.get("heading_deg", 0.0)),
            )
        pre_plan_map_info = self.map_service.get_map_info()
        plan_cache_key = self._path_plan_request_cache_fingerprint(
            pre_plan_map_info,
            selected_ids,
            request_start_pose,
            request_end_pose,
            request_global_direction,
            use_all,
        )
        planned = False
        if (
            not bool(request.force_replan)
            and self.current_path is not None
            and self.current_path.points
            and plan_cache_key == self._path_plan_request_cache_key
            and int(self.current_path.path_version or 0) == int(self._path_plan_request_cache_path_version or 0)
        ):
            planned = True
            self.state = SchedulerState.READY
            self.last_error = ""
            rospy.loginfo(
                "PathPlanRequest cache hit: skip replanning path_version=%s points=%s",
                int(self.current_path.path_version),
                len(self.current_path.points),
            )
        else:
            planned = self._plan_current_task(
                force_use_all_regions=use_all,
                request_start_pose=request_start_pose,
                request_end_pose=request_end_pose,
                request_global_direction=request_global_direction,
            )
            if planned and self.current_path is not None and self.current_path.points:
                self._path_plan_request_cache_key = plan_cache_key
                self._path_plan_request_cache_path_version = int(self.current_path.path_version or 0)
        t2_plan = time.perf_counter()

        response = pb.PathPlanResponse()
        response.request_id = request.request_id
        response.task_id = self.task_config.task_id
        map_info = self.map_service.get_map_info()
        response.map_version = map_info["map_version"] if map_info else 0
        if map_info:
            self._apply_response_map_info(response, map_info)
        else:
            response.width = 0
            response.height = 0
            response.resolution = 0.0
            response.origin.x = 0.0
            response.origin.y = 0.0
            response.origin.heading_deg = 0.0
            response.frame_id = ""
        response.preview_scale_x = 0.0
        response.preview_scale_y = 0.0
        total_area = self._total_work_area_m2()
        response.total_work_area_m2 = float(total_area)
        response.estimated_time_s = -1.0
        response.planned = planned
        self._apply_localization_covariance(response)
        self._apply_alignment_yaw_to_response(response, current_map_id)
        t3_response_base = time.perf_counter()

        if planned and self.current_path is not None:
            response.result = pb.RESULT_SUCCESS
            response.message = "planning_ok"
            response.path_version = self.current_path.path_version
            response.path_point_count = len(self.current_path.points)
            response.path_length_m = float(self.current_path.length_m)
            response.estimated_time_s = float(self._estimate_plan_time_s(self.current_path.length_m))
            t4_fields = time.perf_counter()
            try:
                preview_payload = (
                    self._build_path_preview_payload(map_info)
                    if include_preview
                    else None
                )
                t5_preview = time.perf_counter()
                if preview_payload is not None:
                    response.preview_image = preview_payload[0]
                    response.preview_format = preview_payload[1]
                    if len(preview_payload) > 2:
                        self._apply_response_map_info(response, preview_payload[2])
                    if response.width > 0 and response.height > 0 and response.preview_image:
                        if len(preview_payload) > 4:
                            response.preview_scale_x = float(preview_payload[3]) / float(max(1, response.width))
                            response.preview_scale_y = float(preview_payload[4]) / float(max(1, response.height))
                        else:
                            try:
                                decoded = cv2.imdecode(np.frombuffer(response.preview_image, dtype=np.uint8), cv2.IMREAD_COLOR)
                                if decoded is not None:
                                    ph, pw = decoded.shape[:2]
                                    response.preview_scale_x = float(pw) / float(max(1, response.width))
                                    response.preview_scale_y = float(ph) / float(max(1, response.height))
                            except Exception:
                                pass
                    t6_scale = time.perf_counter()
                    if self._planned_path_preview_save_response_file:
                        self._save_path_preview_bytes(preview_payload[0], preview_payload[1], map_info)
                    else:
                        rospy.loginfo("Skip PathPlanResponse preview file save; preview is returned in response.")
                    t7_save = time.perf_counter()
                else:
                    t6_scale = time.perf_counter()
                    t7_save = t6_scale
            except Exception as exc:
                rospy.logwarn("Failed to build/save path preview in PathPlanResponse: %s", exc)
                t5_preview = time.perf_counter()
                t6_scale = t5_preview
                t7_save = t5_preview
        else:
            response.result = pb.RESULT_FAILED
            response.message = self.last_error or "planning_failed"
            response.path_version = 0
            response.path_point_count = 0
            response.path_length_m = 0.0
            response.estimated_time_s = -1.0
            response.preview_image = b""
            response.preview_format = ""
            t4_fields = time.perf_counter()
            # Even if planning fails, return current map preview for UI continuity.
            try:
                failed_preview = (
                    self._build_failed_plan_preview_payload(response.message)
                    if include_preview
                    else None
                )
                t5_preview = time.perf_counter()
                if failed_preview is not None:
                    response.preview_image = failed_preview[0]
                    response.preview_format = failed_preview[1]
                    if len(failed_preview) > 2:
                        self._apply_response_map_info(response, failed_preview[2])
                    if response.width > 0 and response.height > 0 and response.preview_image:
                        try:
                            decoded = cv2.imdecode(np.frombuffer(response.preview_image, dtype=np.uint8), cv2.IMREAD_COLOR)
                            if decoded is not None:
                                ph, pw = decoded.shape[:2]
                                response.preview_scale_x = float(pw) / float(max(1, response.width))
                                response.preview_scale_y = float(ph) / float(max(1, response.height))
                        except Exception:
                            pass
                    t6_scale = time.perf_counter()
                else:
                    t6_scale = time.perf_counter()
            except Exception as exc:
                rospy.logwarn("Failed to build fallback preview for failed PathPlanResponse: %s", exc)
                t5_preview = time.perf_counter()
                t6_scale = t5_preview
            t7_save = t6_scale

        response_payload = response.SerializeToString()
        t8_serialize = time.perf_counter()
        # SL-Link frame payload_len is uint16, keep a safety margin for robustness.
        max_payload_safe = 65000
        if len(response_payload) > max_payload_safe:
            rospy.logwarn(
                "PathPlanResponse payload too large (%d bytes), strip preview image for transport safety",
                len(response_payload),
            )
            response.preview_image = b""
            response.preview_format = ""
            response.preview_scale_x = 0.0
            response.preview_scale_y = 0.0
            if response.message:
                response.message = "{};preview_omitted_oversize".format(response.message)
            else:
                response.message = "preview_omitted_oversize"
            response_payload = response.SerializeToString()
        t9_oversize = time.perf_counter()

        outputs = [
            (response_payload, pb.MSG_ID_PATH_PLAN_RESPONSE, pb.COMP_SCHEDULER),
        ]

        if planned and request.return_path_chunks and self.current_path is not None:
            chunks = self._serialize_path_point_plan_chunks(
                task_id=self.task_config.task_id,
                request_id=request.request_id,
                map_id=current_map_id,
                max_chunk_size=request.max_chunk_size,
                planned=True,
                result="success",
                message="path_point_plan_ready",
            )
            outputs.extend(chunks)
            response.path_chunked = True
            response_payload = response.SerializeToString()
            if len(response_payload) > max_payload_safe:
                rospy.logwarn(
                    "PathPlanResponse payload too large after chunk-flag update (%d bytes), strip preview image",
                    len(response_payload),
                )
                response.preview_image = b""
                response.preview_format = ""
                response.preview_scale_x = 0.0
                response.preview_scale_y = 0.0
                if response.message:
                    response.message = "{};preview_omitted_oversize".format(response.message)
                else:
                    response.message = "preview_omitted_oversize"
                response_payload = response.SerializeToString()
            outputs[0] = (response_payload, pb.MSG_ID_PATH_PLAN_RESPONSE, pb.COMP_SCHEDULER)

        t10_chunks = time.perf_counter()
        rospy.loginfo(
            "PathPlanRequest total perf: total=%.1fms parse=%.1fms plan=%.1fms response_base=%.1fms fields=%.1fms preview=%.1fms scale=%.1fms save=%.1fms serialize=%.1fms oversize=%.1fms chunks=%.1fms planned=%s preview_bytes=%d payload_bytes=%d path_points=%d",
            (t10_chunks - t0_all) * 1000.0,
            (t1_parse - t0_all) * 1000.0,
            (t2_plan - t1_parse) * 1000.0,
            (t3_response_base - t2_plan) * 1000.0,
            (t4_fields - t3_response_base) * 1000.0,
            (t5_preview - t4_fields) * 1000.0,
            (t6_scale - t5_preview) * 1000.0,
            (t7_save - t6_scale) * 1000.0,
            (t8_serialize - t7_save) * 1000.0,
            (t9_oversize - t8_serialize) * 1000.0,
            (t10_chunks - t9_oversize) * 1000.0,
            str(bool(planned)).lower(),
            len(response.preview_image or b""),
            len(response_payload or b""),
            len(self.current_path.points) if (self.current_path is not None and self.current_path.points) else 0,
        )
        return outputs


def main():
    rospy.init_node("grinder_scheduler")
    node = SchedulerNode()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
