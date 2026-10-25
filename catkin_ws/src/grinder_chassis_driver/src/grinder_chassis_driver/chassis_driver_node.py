#!/usr/bin/env python3

import os
import threading
import time
import math
import json
from contextlib import contextmanager

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, Int16, UInt16, String
from std_srvs.srv import Trigger, TriggerResponse
from grinder_chassis_driver.motion_safety import MotionInterlock, MotionCancelled, motion_action

from grinder_chassis_driver.msg import ChassisStatus, WheelSpeedCommand, WheelSpeedState
from grinder_chassis_driver.indicator_emergency import EmergencyIndicatorSource
from grinder_chassis_driver.srv import (
    ClearFault,
    ClearFaultResponse,
    EnableChassis,
    EnableChassisResponse,
)
from grinder_chassis_driver.modbus_transport import (
    ModbusExceptionResponse,
    ModbusTransport,
    ModbusTransportError,
)
from grinder_chassis_driver.register_map import (
    DISC_ENABLE_OFF,
    DISC_ENABLE_ON,
    DISC_LIFT_DOWN,
    DISC_LIFT_UP,
    ACTUAL_SPEED_READ_COUNT,
    INT16_MAX,
    INT16_MIN,
    LIGHT_OFF,
    LIGHT_ON,
    READ_BLOCK_START,
    STATUS_AND_ACTUAL_READ_COUNT,
    STATUS_READ_COUNT,
    STATUS_READ_START,
    REGISTER_DISC_ENABLE,
    REGISTER_DISC_LIFT,
    REGISTER_DISC_SPEED,
    REGISTER_LEFT_MOTOR_ACTUAL_SPEED,
    REGISTER_LEFT_WHEEL_SPEED,
    REGISTER_LIGHT,
    REGISTER_RIGHT_MOTOR_ACTUAL_SPEED,
    REGISTER_RIGHT_WHEEL_SPEED,
    REGISTER_WORK_MODE,
    WORK_MODE_MANUAL,
    ChassisCommand,
    RegisterSnapshot,
    clamp,
    motor_rpm_to_base_twist,
    to_int16,
)


class ChassisDriverNode:
    def __init__(self):
        self._lock = threading.RLock()
        self._io_lock = threading.RLock()
        self._motion = MotionInterlock(rospy.get_param(
            "~motion_emergency_file", "~/.ros/grinder_safety/chassis_emergency.json"),
            legacy_path=rospy.get_param("~indicator_emergency_file", "~/.ros/grinder_indicator/chassis_emergency.json"))
        self._outputs_ready = False
        self._safe_write_confirmed = False
        self._reset_error = ""
        self._reset_service_lock = threading.Lock()
        self.command = ChassisCommand()
        self.snapshot = RegisterSnapshot()
        self.actual_left_motor_speed = 0
        self.actual_right_motor_speed = 0
        self.actual_speed_valid = False
        self._last_actual_read_monotonic = 0.0
        self._last_status_read_monotonic = 0.0
        self.enabled = True
        self.connected = False
        self.last_error = ""
        self.consecutive_failures = 0
        self.last_command_time = rospy.Time.now()
        self.last_sent_left_wheel_speed = 0
        self.last_sent_right_wheel_speed = 0
        self._last_written_block = None
        self._last_written_registers = {}
        self._last_safe_stop_on_error_time = 0.0

        self.config_file = rospy.get_param("~config_file", "")
        self.port = rospy.get_param("~port", "/dev/ttyS0")
        self.baudrate = rospy.get_param("~baudrate", 115200)
        self.slave_id = rospy.get_param("~slave_id", 1)
        self.timeout = rospy.get_param("~timeout", 0.1)
        self.poll_rate_hz = rospy.get_param("~poll_rate_hz", 20.0)
        self.status_poll_rate_hz = float(rospy.get_param("~status_poll_rate_hz", 2.0))
        self.diagnostics_rate_hz = float(rospy.get_param("~diagnostics_rate_hz", 1.0))
        self.status_combined_read = bool(rospy.get_param("~status_combined_read", True))
        self._next_status_poll_monotonic = 0.0
        self._next_diagnostics_monotonic = 0.0
        self.command_timeout = rospy.get_param("~command_timeout", 0.5)
        self.write_verify = rospy.get_param("~write_verify", False)
        self.max_retries = rospy.get_param("~max_retries", 3)
        self.stop_on_timeout = rospy.get_param("~stop_on_timeout", True)
        self.startup_zero_output = rospy.get_param("~startup_zero_output", True)
        self.left_speed_min = rospy.get_param("~left_speed_min", INT16_MIN)
        self.left_speed_max = rospy.get_param("~left_speed_max", INT16_MAX)
        self.right_speed_min = rospy.get_param("~right_speed_min", INT16_MIN)
        self.right_speed_max = rospy.get_param("~right_speed_max", INT16_MAX)
        self.disc_speed_min = rospy.get_param("~disc_speed_min", INT16_MIN)
        self.disc_speed_max = rospy.get_param("~disc_speed_max", INT16_MAX)
        self.disc_lift_supported = bool(rospy.get_param("~disc_lift_supported", True))
        self.max_cmd_step_rpm = int(rospy.get_param("~max_cmd_step_rpm", 50))
        self.enable_cmd_ramp_limit = bool(rospy.get_param("~enable_cmd_ramp_limit", True))
        self.safe_stop_on_error_interval = float(rospy.get_param("~safe_stop_on_error_interval", 1.0))
        self.enable_safe_stop_on_comm_error = bool(rospy.get_param("~enable_safe_stop_on_comm_error", True))
        self.rs485_raw_log_enabled = bool(rospy.get_param("~rs485_raw_log_enabled", False))
        self.rs485_raw_log_file = rospy.get_param("~rs485_raw_log_file", "temp/chassis_rs485_raw.log")
        if self.rs485_raw_log_enabled:
            base = str(self.rs485_raw_log_file or "temp/chassis_rs485_raw.log")
            root, ext = os.path.splitext(base)
            if not ext:
                ext = ".log"
            ts = time.strftime("%Y%m%d_%H%M%S", time.localtime())
            self.rs485_raw_log_file = "{}_{}{}".format(root, ts, ext)
            rospy.loginfo("RS485 raw log file: %s", self.rs485_raw_log_file)
        self.cmd_vel_enabled = bool(rospy.get_param("~cmd_vel_enabled", True))
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.task_enable = bool(rospy.get_param("~task_enable_default", False))
        self.cmd_vel_wheel_track_m = float(rospy.get_param("~cmd_vel_wheel_track_m", 0.5))
        self.cmd_vel_wheel_radius_m = float(rospy.get_param("~cmd_vel_wheel_radius_m", 0.1))
        self.cmd_vel_gear_ratio = float(rospy.get_param("~cmd_vel_gear_ratio", 60.0))
        self.wheel_odom_topic = rospy.get_param("~wheel_odom_topic", "/odom_wheel")
        self.wheel_odom_frame_id = rospy.get_param("~wheel_odom_frame_id", "odom")
        self.wheel_odom_child_frame_id = rospy.get_param("~wheel_odom_child_frame_id", "base_link")
        self.wheel_twist_topic = rospy.get_param("~wheel_twist_topic", "/chassis/wheel_twis")
        self.wheel_feedback_delay_sec = float(rospy.get_param("~wheel_feedback_delay_sec", 0.0))
        self.wheel_twist_linear_variance = float(rospy.get_param("~wheel_twist_linear_variance", 0.04))
        self.wheel_twist_angular_variance = float(rospy.get_param("~wheel_twist_angular_variance", 0.04))
        self.wheel_twist_cross_covariance = float(rospy.get_param("~wheel_twist_cross_covariance", 0.0))
        variances = (self.wheel_twist_linear_variance, self.wheel_twist_angular_variance)
        if (not all(math.isfinite(value) and value > 0.0 for value in variances)
                or not math.isfinite(self.wheel_twist_cross_covariance)
                or self.wheel_twist_cross_covariance ** 2 >= variances[0] * variances[1]
                or not math.isfinite(self.wheel_feedback_delay_sec) or self.wheel_feedback_delay_sec < 0.0):
            raise ValueError("Invalid wheel fusion covariance or feedback delay")
        self._actual_feedback_stamp = None
        self.actual_motor_speed_scale = float(rospy.get_param("~actual_motor_speed_scale", 1.0))
        self.actual_speed_left_sign = -1 if int(rospy.get_param("~actual_speed_left_sign", 1)) < 0 else 1
        self.actual_speed_right_sign = -1 if int(rospy.get_param("~actual_speed_right_sign", 1)) < 0 else 1
        self.actual_speed_swap_lr = bool(rospy.get_param("~actual_speed_swap_lr", False))
        self.cmd_vel_scale = float(rospy.get_param("~cmd_vel_scale", 1.0))
        self.cmd_vel_max_linear = float(rospy.get_param("~cmd_vel_max_linear", 0.6))
        self.cmd_vel_max_angular = float(rospy.get_param("~cmd_vel_max_angular", 1.5))
        self.cmd_vel_max_input_v = float(rospy.get_param("~cmd_vel_max_input_v", 0.083))
        self.cmd_vel_max_input_w = float(rospy.get_param("~cmd_vel_max_input_w", 0.15))
        self.cmd_vel_max_abs_wheel_rpm = float(rospy.get_param("~cmd_vel_max_abs_wheel_rpm", 1500.0))
        self.cmd_vel_deadband_linear = float(rospy.get_param("~cmd_vel_deadband_linear", 0.0))
        self.cmd_vel_deadband_angular = float(rospy.get_param("~cmd_vel_deadband_angular", 0.0))
        self.cmd_source_hold_sec = float(rospy.get_param("~cmd_source_hold_sec", 0.2))
        configured_manual_speed = float(rospy.get_param("~manual_drive_max_wheel_speed", 0.2))
        if configured_manual_speed > 10.0:
            # Upgrade old configs where this field was stored as motor RPM.
            configured_manual_speed = self._manual_speed_limit_from_rpm(configured_manual_speed)
            rospy.logwarn(
                "legacy manual_drive_max_wheel_speed converted from RPM to %.3fm/s",
                configured_manual_speed,
            )
        self.manual_drive_max_wheel_speed = max(0.0, configured_manual_speed)
        self.manual_speed_limit_rpm = self._manual_speed_limit_to_rpm(self.manual_drive_max_wheel_speed)
        rospy.loginfo(
            "manual speed limit initialized from config: max=%.3fm/s (%drpm)",
            self.manual_drive_max_wheel_speed,
            self.manual_speed_limit_rpm,
        )
        #=================================================================================================================
        self.cmd_vel_filter_enabled = bool(rospy.get_param("~cmd_vel_filter_enabled", True))
        self.cmd_vel_filter_linear_accel = float(rospy.get_param("~cmd_vel_filter_linear_accel", 0.20))
        self.cmd_vel_filter_angular_accel = float(rospy.get_param("~cmd_vel_filter_angular_accel", 0.20))
        self.cmd_vel_filter_angular_deadband = float(rospy.get_param("~cmd_vel_filter_angular_deadband", 0.02))
        self.cmd_vel_filter_sign_confirm_count = int(rospy.get_param("~cmd_vel_filter_sign_confirm_count", 2))

        self._last_filtered_linear = 0.0
        self._last_filtered_angular = 0.0
        self._last_cmd_filter_time = time.monotonic()
        self._pending_angular_sign = 0
        self._pending_angular_sign_count = 0
        #=================================================================================================================
        # Manual wheel-speed input anti-jitter around zero crossing.
        self.manual_wheel_deadband_rpm = int(rospy.get_param("~manual_wheel_deadband_rpm", 0))
        self.manual_wheel_hyst_enter_rpm = int(rospy.get_param("~manual_wheel_hyst_enter_rpm", 0))
        self.manual_wheel_hyst_exit_rpm = int(rospy.get_param("~manual_wheel_hyst_exit_rpm", 0))
        if self.manual_wheel_hyst_enter_rpm < self.manual_wheel_hyst_exit_rpm:
            self.manual_wheel_hyst_enter_rpm = self.manual_wheel_hyst_exit_rpm
        self._manual_last_left_input = 0
        self._manual_last_right_input = 0
        # Direction correction:
        # default to +1/+1 (no inversion), and leave fine-tuning to YAML params.
        self.drive_left_sign = -1 if int(rospy.get_param("~drive_left_sign", 1)) < 0 else 1
        self.drive_right_sign = -1 if int(rospy.get_param("~drive_right_sign", 1)) < 0 else 1
        self.drive_swap_lr = bool(rospy.get_param("~drive_swap_lr", False))
        self.cmd_vel_drive_left_sign = -1 if int(rospy.get_param("~cmd_vel_drive_left_sign", 1)) < 0 else 1
        self.cmd_vel_drive_right_sign = -1 if int(rospy.get_param("~cmd_vel_drive_right_sign", 1)) < 0 else 1
        self.cmd_vel_drive_swap_lr = bool(rospy.get_param("~cmd_vel_drive_swap_lr", False))
        self._last_cmd_vel_recv_monotonic = 0.0

        self.transport = ModbusTransport(
            port=self.port,
            slave_id=self.slave_id,
            baudrate=self.baudrate,
            timeout=self.timeout,
            write_verify=self.write_verify,
            raw_log_enabled=self.rs485_raw_log_enabled,
            raw_log_file=self.rs485_raw_log_file,
        )

        self.wheel_speed_pub = rospy.Publisher("/chassis/wheel_speed_state", WheelSpeedState, queue_size=10)
        self.wheel_odom_pub = rospy.Publisher(self.wheel_odom_topic, Odometry, queue_size=10)
        self.wheel_twist_pub = rospy.Publisher(self.wheel_twist_topic, TwistWithCovarianceStamped, queue_size=10)
        self.status_pub = rospy.Publisher("/chassis/status", ChassisStatus, queue_size=10)
        self.diagnostics_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=10)
        self._indicator_emergency = EmergencyIndicatorSource(
            "/chassis", "~/.ros/grinder_indicator/chassis_emergency.json"
        )
        self.safety_pub = rospy.Publisher("/chassis/motion_safety", String, queue_size=1, latch=True)
        if self._motion.latched:
            self._indicator_emergency.observe()

        rospy.Subscriber("/chassis/wheel_speed_cmd", WheelSpeedCommand, self._handle_wheel_speed_cmd, queue_size=1, tcp_nodelay=True)
        if self.cmd_vel_enabled:
            rospy.Subscriber(self.cmd_vel_topic, Twist, self._handle_cmd_vel, queue_size=10)
            rospy.Subscriber("/chassis/task_enable", Bool, self._handle_task_enable_cmd, queue_size=10)
        rospy.Subscriber("/chassis/disc_speed_cmd", Int16, self._handle_disc_speed_cmd, queue_size=10)
        rospy.Subscriber("/chassis/disc_enable_cmd", Bool, self._handle_disc_enable_cmd, queue_size=10)
        rospy.Subscriber("/chassis/work_mode_cmd", UInt16, self._handle_work_mode_cmd, queue_size=10)
        rospy.Subscriber("/chassis/disc_lift_cmd", UInt16, self._handle_disc_lift_cmd, queue_size=10)
        rospy.Subscriber("/chassis/light_cmd", Bool, self._handle_light_cmd, queue_size=10)
        rospy.Subscriber("/chassis/manual_speed_limit", Float32, self._handle_manual_speed_limit_cmd, queue_size=1)
        rospy.Subscriber("/chassis/emergency_stop", Bool, self._handle_emergency_stop_cmd, queue_size=1)

        rospy.Service("/chassis/enable", EnableChassis, self._handle_enable_service)
        rospy.Service("/chassis/clear_fault", ClearFault, self._handle_clear_fault_service)
        rospy.Service("/chassis/reset_emergency_stop", Trigger, self._reset_emergency_stop)

        # Mandatory reconnect barrier, independent of the legacy startup flag.
        self._apply_startup_safe_state()

        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0 / max(self.poll_rate_hz, 1.0)), self._poll_once)
        rospy.on_shutdown(self._on_shutdown)

    def _apply_startup_safe_state(self):
        with self._lock:
            self.command.left_wheel_speed = 0
            self.command.right_wheel_speed = 0
            self.command.disc_speed = 0
            self.command.disc_enable = DISC_ENABLE_OFF
            self.command.light = LIGHT_OFF
        try:
            self._issue_safe_stop()
        except ModbusTransportError as exc:
            rospy.logwarn("Failed to send startup safe state: %s", exc)

    def _manual_speed_limit_to_rpm(self, speed_mps):
        speed = abs(float(speed_mps))
        if speed <= 0.0 or self.cmd_vel_wheel_radius_m <= 0.0:
            return 0
        rpm = speed * 60.0 / (2.0 * math.pi * self.cmd_vel_wheel_radius_m)
        rpm *= self.cmd_vel_gear_ratio * self.cmd_vel_scale
        safe_cap = max(1, int(abs(self.cmd_vel_max_abs_wheel_rpm)))
        safe_cap = min(safe_cap, abs(int(self.left_speed_max)), abs(int(self.right_speed_max)))
        return max(1, min(safe_cap, int(round(rpm))))

    def _manual_speed_limit_from_rpm(self, motor_rpm):
        denominator = self.cmd_vel_gear_ratio * self.cmd_vel_scale
        if denominator <= 0.0:
            return 0.0
        wheel_rpm = abs(float(motor_rpm)) / denominator
        return wheel_rpm * (2.0 * math.pi * self.cmd_vel_wheel_radius_m) / 60.0

    def _persist_manual_drive_max_wheel_speed(self, limit_mps):
        path = str(self.config_file or "").strip()
        if not path:
            rospy.logwarn_throttle(5.0, "manual speed limit config persistence skipped: config_file is empty")
            return
        if not os.path.isfile(path):
            rospy.logwarn_throttle(5.0, "manual speed limit config persistence skipped: file not found: %s", path)
            return
        try:
            with open(path, "r", encoding="utf-8") as handle:
                lines = handle.readlines()
            updated = False
            new_lines = []
            for line in lines:
                stripped = line.lstrip()
                if stripped.startswith("manual_drive_max_wheel_speed:"):
                    indent = line[: len(line) - len(stripped)]
                    suffix = "\n" if line.endswith("\n") else ""
                    new_lines.append("{}manual_drive_max_wheel_speed: {:.6g}{}".format(indent, float(limit_mps), suffix))
                    updated = True
                else:
                    new_lines.append(line)
            if not updated:
                if new_lines and not new_lines[-1].endswith("\n"):
                    new_lines[-1] += "\n"
                new_lines.append("manual_drive_max_wheel_speed: {:.6g}\n".format(float(limit_mps)))
            tmp_path = path + ".tmp"
            with open(tmp_path, "w", encoding="utf-8") as handle:
                handle.writelines(new_lines)
            os.replace(tmp_path, path)
            rospy.loginfo(
                "manual speed limit persisted: manual_drive_max_wheel_speed=%.3fm/s file=%s",
                float(limit_mps),
                path,
            )
        except Exception as exc:
            rospy.logwarn("Failed to persist manual speed limit to %s: %s", path, exc)

    def _handle_manual_speed_limit_cmd(self, msg):
        limit_mps = max(0.0, float(msg.data))
        limit_rpm = self._manual_speed_limit_to_rpm(limit_mps)
        with self._lock:
            self.manual_speed_limit_rpm = limit_rpm
            self.manual_drive_max_wheel_speed = limit_mps
        rospy.set_param("~manual_drive_max_wheel_speed", limit_mps)
        self._persist_manual_drive_max_wheel_speed(limit_mps)
        rospy.loginfo(
            "manual speed limit updated: %.3fm/s -> %drpm (max=%drpm)",
            limit_mps,
            limit_rpm,
            limit_rpm,
        )

    def _apply_manual_speed_limit(self, value):
        limit = max(0, abs(int(self.manual_speed_limit_rpm)))
        return max(-limit, min(limit, int(value)))

    @motion_action()
    def _handle_wheel_speed_cmd(self, msg):
        self._motion.local.callback_arrival = time.monotonic()
        if self.cmd_vel_enabled:
            now_mono = time.monotonic()
            recent_cmd_vel = (now_mono - self._last_cmd_vel_recv_monotonic) <= max(0.0, self.cmd_source_hold_sec)
            is_zero_stop = int(msg.left_wheel_speed) == 0 and int(msg.right_wheel_speed) == 0
            if recent_cmd_vel and self.task_enable and not is_zero_stop:
                rospy.logwarn_throttle(
                    2.0,
                    "Ignore /chassis/wheel_speed_cmd because cmd_vel source is active (hold=%.3fs).",
                    self.cmd_source_hold_sec,
                )
                return
        req_left = clamp(msg.left_wheel_speed, self.left_speed_min, self.left_speed_max)
        req_right = clamp(msg.right_wheel_speed, self.right_speed_min, self.right_speed_max)
        req_left = self._filter_manual_wheel_input(req_left, self._manual_last_left_input)
        req_right = self._filter_manual_wheel_input(req_right, self._manual_last_right_input)
        req_left = self._apply_manual_speed_limit(req_left)
        req_right = self._apply_manual_speed_limit(req_right)
        self._manual_last_left_input = int(req_left)
        self._manual_last_right_input = int(req_right)
        mapped_left, mapped_right = self._apply_drive_direction(req_left, req_right)
        with self._command_guard():
            self.command.left_wheel_speed = mapped_left
            self.command.right_wheel_speed = mapped_right
            self.last_command_time = rospy.Time.now()
        if self.enabled:
            self._write_motion_registers()
    #=================================================================================================================
    def _cmd_vel_sign(self, value, deadband):
        if value > deadband:
            return 1
        if value < -deadband:
            return -1
        return 0

    def _rate_limit_value(self, target, last_value, max_rate, dt):
        max_step = max(0.0, float(max_rate)) * max(0.0, float(dt))
        delta = float(target) - float(last_value)
        if delta > max_step:
            return float(last_value) + max_step
        if delta < -max_step:
            return float(last_value) - max_step
        return float(target)

    def _reset_cmd_vel_filter(self):
        self._last_filtered_linear = 0.0
        self._last_filtered_angular = 0.0
        self._last_cmd_filter_time = time.monotonic()
        self._pending_angular_sign = 0
        self._pending_angular_sign_count = 0




    def _apply_cmd_vel_filter(self, linear, angular):
        if not self.cmd_vel_filter_enabled:
            self._last_filtered_linear = float(linear)
            self._last_filtered_angular = float(angular)
            self._last_cmd_filter_time = time.monotonic()
            return float(linear), float(angular)

        now = time.monotonic()
        dt = now - self._last_cmd_filter_time
        if dt <= 0.0:
            dt = 0.02
        dt = max(0.02, min(0.20, dt))

        angular_deadband = max(0.0, float(self.cmd_vel_filter_angular_deadband))
        if abs(angular) < angular_deadband:
            angular = 0.0

        requested_sign = self._cmd_vel_sign(angular, angular_deadband)
        last_sign = self._cmd_vel_sign(self._last_filtered_angular, angular_deadband)
        confirm_count = max(1, int(self.cmd_vel_filter_sign_confirm_count))

        if confirm_count > 1 and requested_sign != 0 and last_sign != 0 and requested_sign != last_sign:
            if requested_sign == self._pending_angular_sign:
                self._pending_angular_sign_count += 1
            else:
                self._pending_angular_sign = requested_sign
                self._pending_angular_sign_count = 1

            if self._pending_angular_sign_count < confirm_count:
                angular = 0.0
            else:
                self._pending_angular_sign = 0
                self._pending_angular_sign_count = 0
        else:
            self._pending_angular_sign = 0
            self._pending_angular_sign_count = 0

        linear = self._rate_limit_value(
            linear,
            self._last_filtered_linear,
            self.cmd_vel_filter_linear_accel,
            dt,
        )
        angular = self._rate_limit_value(
            angular,
            self._last_filtered_angular,
            self.cmd_vel_filter_angular_accel,
            dt,
        )

        if abs(angular) < angular_deadband:
            angular = 0.0

        self._last_filtered_linear = float(linear)
        self._last_filtered_angular = float(angular)
        self._last_cmd_filter_time = now

        return float(linear), float(angular)    

    #=================================================================================================================

    @motion_action()
    def _handle_cmd_vel(self, msg):
        self._motion.local.callback_arrival = time.monotonic()
        # if not self.task_enable:
        #     return
        if not self.task_enable:
            self._reset_cmd_vel_filter()
            return
        self._last_cmd_vel_recv_monotonic = time.monotonic()
        if self.cmd_vel_wheel_radius_m <= 0.0 or self.cmd_vel_wheel_track_m <= 0.0:
            rospy.logwarn_throttle(
                2.0,
                "cmd_vel conversion disabled due to invalid geometry: wheel_radius=%.4f wheel_track=%.4f",
                self.cmd_vel_wheel_radius_m,
                self.cmd_vel_wheel_track_m,
            )
            return
        linear_limit = self.cmd_vel_max_linear
        angular_limit = self.cmd_vel_max_angular
        if self.cmd_vel_max_input_v > 0.0:
            linear_limit = min(linear_limit, self.cmd_vel_max_input_v)
        if self.cmd_vel_max_input_w > 0.0:
            angular_limit = min(angular_limit, self.cmd_vel_max_input_w)
        # linear = max(-linear_limit, min(linear_limit, float(msg.linear.x)))
        # angular = max(-angular_limit, min(angular_limit, float(msg.angular.z)))
        # if abs(linear) < max(0.0, self.cmd_vel_deadband_linear):
        #     linear = 0.0
        # if abs(angular) < max(0.0, self.cmd_vel_deadband_angular):
        #     angular = 0.0
        # left_mps = linear - angular * (self.cmd_vel_wheel_track_m * 0.5)

        #=============================================================================================================
        linear = max(-linear_limit, min(linear_limit, float(msg.linear.x)))
        angular = max(-angular_limit, min(angular_limit, float(msg.angular.z)))

        if abs(linear) < max(0.0, self.cmd_vel_deadband_linear):
            linear = 0.0
        if abs(angular) < max(0.0, self.cmd_vel_deadband_angular):
            angular = 0.0

        linear, angular = self._apply_cmd_vel_filter(linear, angular)

        left_mps = linear - angular * (self.cmd_vel_wheel_track_m * 0.5)
        #=================================================================================================================
        right_mps = linear + angular * (self.cmd_vel_wheel_track_m * 0.5)
        rpm_factor = 60.0 / (2.0 * math.pi * self.cmd_vel_wheel_radius_m)
        left_rpm = left_mps * rpm_factor * self.cmd_vel_gear_ratio * self.cmd_vel_scale
        right_rpm = right_mps * rpm_factor * self.cmd_vel_gear_ratio * self.cmd_vel_scale
        max_abs_rpm = abs(float(self.cmd_vel_max_abs_wheel_rpm))
        if max_abs_rpm > 0.0:
            left_rpm = max(-max_abs_rpm, min(max_abs_rpm, left_rpm))
            right_rpm = max(-max_abs_rpm, min(max_abs_rpm, right_rpm))
        left_cmd = int(round(left_rpm))
        right_cmd = int(round(right_rpm))
        mapped_left, mapped_right = self._apply_drive_direction_cmd_vel(left_cmd, right_cmd)
        with self._command_guard():
            self.command.left_wheel_speed = mapped_left
            self.command.right_wheel_speed = mapped_right
            self.last_command_time = rospy.Time.now()
        if self.enabled:
            self._write_motion_registers()

    @motion_action()
    def _handle_task_enable_cmd(self, msg):
        with self._command_guard():
            was_enabled = bool(self.task_enable)
            self.task_enable = bool(msg.data)
        if not self.task_enable:
            self._reset_cmd_vel_filter()
            self._last_cmd_vel_recv_monotonic = 0.0
            with self._command_guard():
                already_stopped = self.command.left_wheel_speed == 0 and self.command.right_wheel_speed == 0
                self.command.left_wheel_speed = 0
                self.command.right_wheel_speed = 0
                self.last_command_time = rospy.Time.now()
            if was_enabled and not already_stopped and self.enabled:
                try:
                    self._write_all_outputs(force=True)
                except ModbusTransportError as exc:
                    self._handle_transport_error(exc)
        rospy.loginfo("chassis task_enable updated: %s", str(self.task_enable))

    def _handle_emergency_stop_cmd(self, msg):
        if not bool(msg.data):
            return
        self._motion.invalidate(emergency=True)
        self._indicator_emergency.observe()
        rospy.logwarn("Chassis emergency stop command received. Issuing safe stop immediately.")
        with self._lock:
            self.task_enable = False
            self._last_cmd_vel_recv_monotonic = 0.0
        self._reset_cmd_vel_filter()
        self._issue_safe_stop()
        self._motion.persist()

    def _apply_drive_direction(self, left_speed, right_speed):
        left = int(left_speed)
        right = int(right_speed)
        if self.drive_swap_lr:
            left, right = right, left
        left = int(left * self.drive_left_sign)
        right = int(right * self.drive_right_sign)
        return (
            clamp(left, self.left_speed_min, self.left_speed_max),
            clamp(right, self.right_speed_min, self.right_speed_max),
        )

    def _filter_manual_wheel_input(self, value, prev_value):
        v = int(value)
        prev = int(prev_value)
        deadband = max(0, int(self.manual_wheel_deadband_rpm))
        enter_th = max(deadband, int(self.manual_wheel_hyst_enter_rpm))
        exit_th = max(deadband, int(self.manual_wheel_hyst_exit_rpm))
        # Hard deadband near zero.
        if abs(v) <= deadband:
            return 0
        # Hysteresis around zero crossing:
        # - from stop to move: require |v| >= enter_th
        # - from move to stop: require |v| <= exit_th
        if prev == 0:
            if abs(v) < enter_th:
                return 0
            return v
        if abs(v) <= exit_th:
            return 0
        return v

    def _apply_drive_direction_cmd_vel(self, left_speed, right_speed):
        left = int(left_speed)
        right = int(right_speed)
        if self.cmd_vel_drive_swap_lr:
            left, right = right, left
        left = int(left * self.cmd_vel_drive_left_sign)
        right = int(right * self.cmd_vel_drive_right_sign)
        return (
            clamp(left, self.left_speed_min, self.left_speed_max),
            clamp(right, self.right_speed_min, self.right_speed_max),
        )

    @motion_action()
    def _handle_disc_speed_cmd(self, msg):
        with self._command_guard():
            self.command.disc_speed = clamp(msg.data, self.disc_speed_min, self.disc_speed_max)
        if self.enabled:
            self._write_single_output(REGISTER_DISC_SPEED, self.command.disc_speed, signed=True)

    @motion_action()
    def _handle_disc_enable_cmd(self, msg):
        with self._command_guard():
            self.command.disc_enable = DISC_ENABLE_ON if msg.data else DISC_ENABLE_OFF
        if self.enabled:
            self._write_single_output(REGISTER_DISC_ENABLE, self.command.disc_enable, signed=False)

    @motion_action()
    def _handle_work_mode_cmd(self, msg):
        with self._command_guard():
            self.command.work_mode = msg.data
        if self.enabled:
            self._write_single_output(REGISTER_WORK_MODE, self.command.work_mode, signed=False)

    @motion_action()
    def _handle_disc_lift_cmd(self, msg):
        if not self.disc_lift_supported:
            rospy.logwarn_throttle(5.0, "Disc lift command ignored: this machine has no disc lift.")
            return
        with self._command_guard():
            self.command.disc_lift = msg.data
        if self.enabled:
            self._write_single_output(REGISTER_DISC_LIFT, self.command.disc_lift, signed=False)

    def _handle_light_cmd(self, msg):
        with self._lock:
            self.command.light = LIGHT_ON if msg.data else LIGHT_OFF
        if self.enabled:
            self._write_single_output(REGISTER_LIGHT, self.command.light, signed=False)

    def _handle_enable_service(self, request):
        if not request.enable:
            self.enabled = False
            ok = self._issue_safe_stop()
            return EnableChassisResponse(success=ok, message="disabled; safe write confirmed={}".format(ok))
        return self._enable_outputs()

    @motion_action(cancel_result=EnableChassisResponse(success=False, message="enable cancelled by emergency/stop"))
    def _enable_outputs(self):
        try:
            with self._command_guard():
                self.enabled = True
                self.last_command_time = rospy.Time.now()
            ok = self._write_all_outputs(force=True)
            return EnableChassisResponse(success=ok, message="enabled with safe targets")
        except (MotionCancelled, ModbusTransportError) as exc:
            return EnableChassisResponse(success=False, message=str(exc))

    def _handle_clear_fault_service(self, _request):
        return ClearFaultResponse(
            success=False,
            message="Fault clear register is not defined yet. Provide the vendor register table to implement it.",
        )

    @contextmanager
    def _command_guard(self):
        with self._motion.commit():
            if not self._outputs_ready:
                raise MotionCancelled("safe output synchronization pending")
            with self._lock:
                yield

    def _invalidate_output_cache(self):
        self._last_written_block = None
        self._last_written_registers.clear()

    def _submit_output(self, kind, address=None, value=None, signed=False, force=False, safe=False):
        # This is the only output path. Re-check after acquiring the bus gate;
        # a callback prepared before emergency/timeout cannot replay afterwards.
        with self._io_lock:
            with self._motion.lock:
                if not safe:
                    self._motion.check()
                    if not self._outputs_ready:
                        raise MotionCancelled("safe output synchronization pending")
                generation = self._motion.generation
                with self._lock:
                    if kind == "motion":
                        left = self.command.left_wheel_speed if self.enabled else 0
                        right = self.command.right_wheel_speed if self.enabled else 0
                        # A released joystick is a stop, even with ramp limiting
                        # enabled. One zero command must not leave residual RPM.
                        block = ([0, 0] if left == 0 and right == 0 else
                                 [self._apply_ramp_limit(left, self.last_sent_left_wheel_speed),
                                  self._apply_ramp_limit(right, self.last_sent_right_wheel_speed)])
                        address = REGISTER_LEFT_WHEEL_SPEED
                        if not force and tuple(block) == (self.last_sent_left_wheel_speed, self.last_sent_right_wheel_speed):
                            return True
                    elif kind == "single":
                        block = [int(value)]
                        if not force and self._last_written_registers.get(address) == int(value):
                            return True
                    else:
                        block = self.command.as_register_block()
                        if safe or not self.enabled:
                            block[:4] = [0, 0, 0, DISC_ENABLE_OFF]
                        address = READ_BLOCK_START
                        if not force and self._last_written_block == tuple(block):
                            return True
                    moving = (any(block[:4]) if kind == "all" else any(block) if kind == "motion"
                              else address in (REGISTER_LEFT_WHEEL_SPEED, REGISTER_RIGHT_WHEEL_SPEED,
                                               REGISTER_DISC_SPEED, REGISTER_DISC_ENABLE) and bool(block[0]))
                    if safe or moving:
                        self._safe_write_confirmed = False
            serial_started = time.monotonic()
            if kind == "motion":
                self._motion_submit_delay_ms = (serial_started - getattr(self._motion.local, "callback_arrival", serial_started)) * 1000.0
            # No state lock held during serial I/O. Emergency may latch now;
            # its forced stop follows this transaction, never precedes it.
            try:
                if kind == "single":
                    writer = self.transport.write_int16 if signed else self.transport.write_uint16
                    writer(address, block[0])
                elif kind == "all" and not self.disc_lift_supported:
                    self.transport.write_register_block(address, block[:5])
                    self.transport.write_uint16(REGISTER_LIGHT, block[6])
                else:
                    self.transport.write_register_block(address, block)
            except ModbusTransportError:
                with self._lock:
                    self._invalidate_output_cache()
                    self._safe_write_confirmed = False
                raise
            if kind == "motion":
                self._motion_write_ack_ms = (time.monotonic() - serial_started) * 1000.0
            with self._motion.lock, self._lock:
                if generation != self._motion.generation:
                    self._invalidate_output_cache()
                    return False
                self._last_written_block = tuple(block) if kind == "all" else None
                for index, item in enumerate(block):
                    register = address + index
                    if kind != "all" or self.disc_lift_supported or register != REGISTER_DISC_LIFT:
                        self._last_written_registers[register] = to_int16(item) if register in (
                            REGISTER_LEFT_WHEEL_SPEED, REGISTER_RIGHT_WHEEL_SPEED, REGISTER_DISC_SPEED) else int(item)
                if kind in ("motion", "all"):
                    self.last_sent_left_wheel_speed, self.last_sent_right_wheel_speed = map(to_int16, block[:2])
                if safe:
                    self._safe_write_confirmed = True
                    self._outputs_ready = True
            self._mark_connected()
            return True

    def _write_motion_registers(self):
        try:
            return self._submit_output("motion")
        except MotionCancelled:
            return False
        except ModbusTransportError as exc:
            self._handle_transport_error(exc)
            return False

    def _write_single_output(self, register_address, value, signed):
        try:
            return self._submit_output("single", register_address, value, signed)
        except MotionCancelled:
            return False
        except ModbusTransportError as exc:
            self._handle_transport_error(exc)
            return False

    def _write_all_outputs(self, force=False):
        return self._submit_output("all", force=force)

    def _issue_safe_stop(self):
        self._motion.invalidate()
        with self._lock:
            self._safe_write_confirmed = False
            self.command.left_wheel_speed = 0
            self.command.right_wheel_speed = 0
            self.command.disc_speed = 0
            self.command.disc_enable = DISC_ENABLE_OFF
            self.task_enable = False
            self._last_cmd_vel_recv_monotonic = 0.0
            self._manual_last_left_input = self._manual_last_right_input = 0
        self._reset_cmd_vel_filter()
        try:
            return self._submit_output("all", force=True, safe=True)
        except ModbusTransportError as exc:
            self._handle_transport_error(exc)
            return False

    def _reset_emergency_stop(self, _request):
        if not self._reset_service_lock.acquire(False):
            return TriggerResponse(success=False, message="emergency reset already in progress")
        try:
            result = self._verify_emergency_reset()
            self._reset_error = "" if result.success else result.message
            return result
        finally:
            self._reset_service_lock.release()

    def _verify_emergency_reset(self):
        if not self._motion.latched:
            self._motion.invalidate(emergency=True)
            self._motion.persist()
        generation = self._motion.generation + 1  # Safe stop invalidates once.
        self._reset_error = ""
        if not self.connected:
            return TriggerResponse(success=False, message="chassis disconnected; emergency remains latched")
        if not self._issue_safe_stop():
            return TriggerResponse(success=False, message="safe output write not confirmed")
        if generation != self._motion.generation:
            return TriggerResponse(success=False, message="new emergency interrupted safe write")
        started = time.monotonic()
        last_sample, count = started, 0
        while time.monotonic() - started < 3.0 and not rospy.is_shutdown():
            with self._motion.lock, self._lock:
                if generation != self._motion.generation:
                    return TriggerResponse(success=False, message="new emergency or output failure interrupted reset")
                sample = min(self._last_actual_read_monotonic, self._last_status_read_monotonic)
                valid = (self.connected and self.actual_speed_valid and self._safe_write_confirmed
                         and sample > last_sample and time.monotonic() - sample < 1.0)
                if valid:
                    # The switch readback can remain on at zero speed. Reset
                    # verifies speed readbacks; safe writes still turn it off.
                    stopped = (self.actual_left_motor_speed == 0 and self.actual_right_motor_speed == 0
                               and self.snapshot.disc_speed == 0)
                    count = count + 1 if stopped else 0
                    last_sample = sample
                if count >= 2:
                    # No old targets are restored by reset.
                    self.command.left_wheel_speed = self.command.right_wheel_speed = 0
                    self.command.disc_speed = 0
                    self.command.disc_enable = DISC_ENABLE_OFF
                    self.task_enable = False
                    self._reset_cmd_vel_filter()
            if count >= 2:
                if self._motion.reset(generation):
                    return TriggerResponse(success=True, message="software emergency released; outputs remain stopped")
                return TriggerResponse(success=False, message=self._motion.error or "reset superseded")
            time.sleep(0.02)
        self._reset_error = "two fresh stopped readbacks not confirmed within 3 seconds"
        return TriggerResponse(success=False, message=self._reset_error)

    def _poll_once(self, _event):
        if not self._outputs_ready:
            self._issue_safe_stop()
        self._enforce_command_timeout()
        now_monotonic = time.monotonic()
        status_due = now_monotonic >= self._next_status_poll_monotonic
        diagnostics_due = now_monotonic >= self._next_diagnostics_monotonic
        if status_due:
            status_interval = 1.0 / max(0.1, min(self.status_poll_rate_hz, float(self.poll_rate_hz)))
            self._next_status_poll_monotonic = now_monotonic + status_interval
        if diagnostics_due:
            diagnostics_interval = 1.0 / max(0.1, self.diagnostics_rate_hz)
            self._next_diagnostics_monotonic = now_monotonic + diagnostics_interval

        status_fresh = False
        read_started = time.monotonic()
        try:
            status_registers = None
            if status_due and self.status_combined_read:
                try:
                    actual_read_start = time.monotonic()
                    actual_read_timing = []
                    combined_registers = self.transport.read_register_block(
                        STATUS_READ_START,
                        STATUS_AND_ACTUAL_READ_COUNT,
                        signed_indices=(0, 5, 6),
                        timing=actual_read_timing,
                    )
                except ModbusExceptionResponse as exc:
                    if exc.exception_code not in (0x02, 0x03):
                        raise
                    self.status_combined_read = False
                    rospy.logwarn(
                        "Combined chassis status read is unsupported; falling back to separate reads: %s",
                        exc,
                    )
                else:
                    actual_read_end = time.monotonic()
                    actual_read_end_ros = rospy.Time.now()
                    status_registers = combined_registers[:STATUS_READ_COUNT]
                    actual_speed_registers = combined_registers[STATUS_READ_COUNT:]

            if status_due and status_registers is None:
                status_registers = self.transport.read_register_block(
                    STATUS_READ_START,
                    STATUS_READ_COUNT,
                    signed_indices=(0,),
                )
            if not status_due or not self.status_combined_read:
                actual_read_start = time.monotonic()
                actual_read_timing = []
                actual_speed_registers = self.transport.read_register_block(
                    REGISTER_LEFT_MOTOR_ACTUAL_SPEED,
                    ACTUAL_SPEED_READ_COUNT,
                    signed_indices=(0, 1),
                    timing=actual_read_timing,
                )
                actual_read_end = time.monotonic()
                actual_read_end_ros = rospy.Time.now()
            with self._lock:
                if status_registers is not None:
                    self.snapshot = RegisterSnapshot.from_status_registers(status_registers)
                    self._last_status_read_monotonic = read_started
                    status_fresh = True
                self.actual_left_motor_speed = int(actual_speed_registers[0])
                self.actual_right_motor_speed = int(actual_speed_registers[1])
                self.actual_speed_valid = True
                # Transaction midpoint is only a software estimate: upstream
                # servo/controller cache age is unknown without source timestamps.
                self._actual_feedback_stamp = actual_read_end_ros - rospy.Duration.from_sec(
                    actual_read_end - (sum(actual_read_timing) * 0.5 if actual_read_timing else
                                       (actual_read_start + actual_read_end) * 0.5)
                    + self.wheel_feedback_delay_sec)
                self._last_actual_read_monotonic = read_started
            self._mark_connected()
        except ModbusTransportError as exc:
            with self._lock:
                self.actual_left_motor_speed = 0
                self.actual_right_motor_speed = 0
                self.actual_speed_valid = False
            self._handle_transport_error(exc)

        self._publish_state(publish_status=status_due, status_fresh=status_fresh)
        if diagnostics_due:
            self._publish_diagnostics()

    def _enforce_command_timeout(self):
        if not self.stop_on_timeout or not self.enabled:
            return
        if (rospy.Time.now() - self.last_command_time).to_sec() <= self.command_timeout:
            return
        with self._lock:
            already_stopped = self.command.left_wheel_speed == 0 and self.command.right_wheel_speed == 0
            self.command.left_wheel_speed = 0
            self.command.right_wheel_speed = 0
        if not already_stopped:
            rospy.logwarn_throttle(2.0, "Wheel speed command timeout reached. Stopping chassis motion.")
            self._motion.invalidate()
            try:
                self._submit_output("all", force=True, safe=False)
            except (MotionCancelled, ModbusTransportError) as exc:
                if isinstance(exc, ModbusTransportError):
                    self._handle_transport_error(exc)

    def _publish_state(self, publish_status=True, status_fresh=True):
        self.safety_pub.publish(String(data=json.dumps({
            "emergency_latched": self._motion.latched,
            "generation": self._motion.generation,
            "safe_write_confirmed": self._safe_write_confirmed,
            "outputs_ready": self._outputs_ready,
            "reset_error": self._reset_error or self._motion.error,
        })))
        now = rospy.Time.now()
        with self._lock:
            command = ChassisCommand(**self.command.__dict__)
            snapshot = RegisterSnapshot(**self.snapshot.__dict__)
            connected = self.connected
            actual_left_motor_speed = self.actual_left_motor_speed
            actual_right_motor_speed = self.actual_right_motor_speed
            actual_speed_valid = self.actual_speed_valid
            feedback_stamp = self._actual_feedback_stamp
            enabled = self.enabled
            last_error = self.last_error
            consecutive_failures = self.consecutive_failures

        wheel_msg = WheelSpeedState()
        # A cached sample must retain its measurement age for wheel reversal confirmation.
        wheel_msg.header.stamp = feedback_stamp if feedback_stamp is not None else now
        wheel_msg.target_left_wheel_speed = command.left_wheel_speed
        wheel_msg.target_right_wheel_speed = command.right_wheel_speed
        wheel_msg.feedback_left_wheel_speed = actual_left_motor_speed
        wheel_msg.feedback_right_wheel_speed = actual_right_motor_speed
        wheel_msg.feedback_valid = connected and actual_speed_valid and feedback_stamp is not None
        self.wheel_speed_pub.publish(wheel_msg)

        self._publish_wheel_odom(
            now,
            actual_left_motor_speed,
            actual_right_motor_speed,
            connected and actual_speed_valid,
        )
        if connected and actual_speed_valid and feedback_stamp is not None:
            self._publish_wheel_twist(feedback_stamp, actual_left_motor_speed, actual_right_motor_speed)

        if publish_status:
            status_msg = ChassisStatus()
            status_msg.header.stamp = now
            status_msg.connected = connected and status_fresh
            status_msg.enabled = enabled
            status_msg.work_mode = snapshot.work_mode
            status_msg.disc_speed_target = command.disc_speed
            status_msg.disc_speed_feedback = snapshot.disc_speed
            status_msg.disc_enabled = snapshot.disc_enable == DISC_ENABLE_ON
            status_msg.disc_lift_state = snapshot.disc_lift
            status_msg.light_enabled = snapshot.light == LIGHT_ON
            status_msg.consecutive_failures = consecutive_failures
            status_msg.last_error = last_error or ("status read failed" if not status_fresh else "")
            self.status_pub.publish(status_msg)

    def _publish_wheel_odom(self, stamp, left_motor_rpm, right_motor_rpm, feedback_valid):
        linear_mps = 0.0
        angular_radps = 0.0
        if feedback_valid:
            try:
                linear_mps, angular_radps = motor_rpm_to_base_twist(
                    left_motor_rpm,
                    right_motor_rpm,
                    self.cmd_vel_wheel_radius_m,
                    self.cmd_vel_wheel_track_m,
                    self.cmd_vel_gear_ratio,
                    left_sign=self.actual_speed_left_sign,
                    right_sign=self.actual_speed_right_sign,
                    swap_lr=self.actual_speed_swap_lr,
                    motor_speed_scale=self.actual_motor_speed_scale,
                )
            except ValueError as exc:
                rospy.logwarn_throttle(5.0, "Wheel odometry disabled due to invalid geometry: %s", exc)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.wheel_odom_frame_id
        odom_msg.child_frame_id = self.wheel_odom_child_frame_id
        odom_msg.pose.pose.orientation.w = 1.0
        odom_msg.twist.twist.linear.x = linear_mps
        odom_msg.twist.twist.angular.z = angular_radps
        # This message supplies velocity to RPP. Aurora remains the source of
        # the authoritative pose/TF, so the wheel-only pose is intentionally
        # marked unavailable instead of publishing a competing TF tree.
        odom_msg.pose.covariance[0] = 1e6
        odom_msg.pose.covariance[7] = 1e6
        odom_msg.pose.covariance[14] = 1e6
        odom_msg.pose.covariance[21] = 1e6
        odom_msg.pose.covariance[28] = 1e6
        odom_msg.pose.covariance[35] = 1e6
        odom_msg.twist.covariance[0] = 0.04
        odom_msg.twist.covariance[35] = 0.04
        self.wheel_odom_pub.publish(odom_msg)

    def _publish_wheel_twist(self, stamp, left_motor_rpm, right_motor_rpm):
        try:
            linear, angular = motor_rpm_to_base_twist(
                left_motor_rpm, right_motor_rpm, self.cmd_vel_wheel_radius_m,
                self.cmd_vel_wheel_track_m, self.cmd_vel_gear_ratio,
                left_sign=self.actual_speed_left_sign, right_sign=self.actual_speed_right_sign,
                swap_lr=self.actual_speed_swap_lr, motor_speed_scale=self.actual_motor_speed_scale)
            if not all(math.isfinite(value) for value in (linear, angular)) or stamp.to_sec() <= 0.0:
                return
        except (ValueError, OverflowError):
            return
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "base_link"
        msg.twist.twist.linear.x = linear
        msg.twist.twist.angular.z = angular
        for index in (7, 14, 21, 28):
            msg.twist.covariance[index] = 1e6  # Unmeasured, never a zero-motion constraint.
        msg.twist.covariance[0] = self.wheel_twist_linear_variance
        msg.twist.covariance[35] = self.wheel_twist_angular_variance
        msg.twist.covariance[5] = msg.twist.covariance[30] = self.wheel_twist_cross_covariance
        self.wheel_twist_pub.publish(msg)

    def _publish_diagnostics(self):
        now = rospy.Time.now()
        now_monotonic = time.monotonic()
        with self._lock:
            connected = self.connected
            level = DiagnosticStatus.OK if connected else DiagnosticStatus.ERROR
            message = "connected" if connected else (self.last_error or "disconnected")
            consecutive_failures = self.consecutive_failures
            actual_age = now_monotonic - self._last_actual_read_monotonic if self._last_actual_read_monotonic else -1.0
            status_age = now_monotonic - self._last_status_read_monotonic if self._last_status_read_monotonic else -1.0

        diag = DiagnosticStatus()
        diag.name = "grinder_chassis_driver"
        diag.hardware_id = "rs485_modbus_slave_{}".format(self.slave_id)
        diag.level = level
        diag.message = message
        diag.values = [
            KeyValue(key="emergency_latched", value=str(self._motion.latched)),
            KeyValue(key="safe_write_confirmed", value=str(self._safe_write_confirmed)),
            KeyValue(key="outputs_ready", value=str(self._outputs_ready)),
            KeyValue(key="reset_error", value=self._reset_error or self._motion.error),
            KeyValue(key="port", value=str(self.port)),
            KeyValue(key="baudrate", value=str(self.baudrate)),
            KeyValue(key="slave_id", value=str(self.slave_id)),
            KeyValue(key="consecutive_failures", value=str(consecutive_failures)),
            KeyValue(key="actual_speed_age_sec", value="{:.3f}".format(actual_age)),
            KeyValue(key="status_age_sec", value="{:.3f}".format(status_age)),
            KeyValue(key="status_combined_read", value=str(self.status_combined_read)),
            KeyValue(key="motion_submit_delay_ms", value=str(getattr(self, "_motion_submit_delay_ms", -1))),
            KeyValue(key="motion_write_ack_ms", value=str(getattr(self, "_motion_write_ack_ms", -1))),
        ]

        array = DiagnosticArray()
        array.header.stamp = now
        array.status = [diag]
        self.diagnostics_pub.publish(array)

    def _mark_connected(self):
        with self._lock:
            self.connected = True
            self.last_error = ""
            self.consecutive_failures = 0

    def _handle_transport_error(self, exc):
        self._motion.invalidate()
        with self._lock:
            self._outputs_ready = False
            self._safe_write_confirmed = False
            self._invalidate_output_cache()
            self.command.left_wheel_speed = self.command.right_wheel_speed = 0
            self.command.disc_speed = 0
            self.command.disc_enable = DISC_ENABLE_OFF
            self.task_enable = False
            self.connected = False
            self.last_error = str(exc)
            self.consecutive_failures += 1
            failures = self.consecutive_failures
        rospy.logwarn_throttle(2.0, "Modbus transport error: %s", exc)
        if failures >= self.max_retries and self.stop_on_timeout and self.enable_safe_stop_on_comm_error:
            self._issue_safe_stop_on_error()

    def _issue_safe_stop_on_error(self):
        now = time.monotonic()
        if now - self._last_safe_stop_on_error_time < max(0.0, self.safe_stop_on_error_interval):
            return
        self._last_safe_stop_on_error_time = now
        try:
            with self._io_lock:
                self.transport.reconnect()
                self._submit_output("all", force=True, safe=True)
        except ModbusTransportError:
            pass

    def _apply_ramp_limit(self, target, last_value):
        if not self.enable_cmd_ramp_limit:
            return int(target)
        delta = int(target) - int(last_value)
        if delta > self.max_cmd_step_rpm:
            return int(last_value) + self.max_cmd_step_rpm
        if delta < -self.max_cmd_step_rpm:
            return int(last_value) - self.max_cmd_step_rpm
        return int(target)

    def _on_shutdown(self):
        try:
            self._issue_safe_stop()
            time.sleep(0.03)
            self._issue_safe_stop()
        except Exception:
            pass
        self.transport.close()


def main():
    rospy.init_node("grinder_chassis_driver")
    ChassisDriverNode()
    rospy.spin()
