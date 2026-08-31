#!/usr/bin/env python3

import os
import threading
import time
import math

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, Int16, UInt16

from grinder_chassis_driver.msg import ChassisStatus, WheelSpeedCommand, WheelSpeedState
from grinder_chassis_driver.srv import (
    ClearFault,
    ClearFaultResponse,
    EnableChassis,
    EnableChassisResponse,
)
from grinder_chassis_driver.modbus_transport import ModbusTransport, ModbusTransportError
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
    READ_BLOCK_COUNT,
    READ_BLOCK_START,
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
)


class ChassisDriverNode:
    def __init__(self):
        self._lock = threading.Lock()
        self.command = ChassisCommand()
        self.snapshot = RegisterSnapshot()
        self.actual_left_motor_speed = 0
        self.actual_right_motor_speed = 0
        self.actual_speed_valid = False
        self.enabled = True
        self.connected = False
        self.last_error = ""
        self.consecutive_failures = 0
        self.last_command_time = rospy.Time.now()
        self.last_sent_left_wheel_speed = 0
        self.last_sent_right_wheel_speed = 0
        self.echo_failure_count = 0
        self._last_written_block = None
        self._last_written_registers = {}
        self._last_safe_stop_on_error_time = 0.0

        self.config_file = rospy.get_param("~config_file", "")
        self.port = rospy.get_param("~port", "/dev/ttyS0")
        self.baudrate = rospy.get_param("~baudrate", 115200)
        self.slave_id = rospy.get_param("~slave_id", 1)
        self.timeout = rospy.get_param("~timeout", 0.1)
        self.poll_rate_hz = rospy.get_param("~poll_rate_hz", 20.0)
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
        self.max_echo_deviation = int(rospy.get_param("~max_echo_deviation", 200))
        self.max_echo_failures = int(rospy.get_param("~max_echo_failures", 2))
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
        self.status_pub = rospy.Publisher("/chassis/status", ChassisStatus, queue_size=10)
        self.diagnostics_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=10)

        rospy.Subscriber("/chassis/wheel_speed_cmd", WheelSpeedCommand, self._handle_wheel_speed_cmd, queue_size=10)
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

        if self.startup_zero_output:
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
            self._write_all_outputs()
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

    def _handle_wheel_speed_cmd(self, msg):
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
        with self._lock:
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

    def _handle_cmd_vel(self, msg):
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
        with self._lock:
            self.command.left_wheel_speed = mapped_left
            self.command.right_wheel_speed = mapped_right
            self.last_command_time = rospy.Time.now()
        if self.enabled:
            self._write_motion_registers()

    def _handle_task_enable_cmd(self, msg):
        was_enabled = bool(self.task_enable)
        self.task_enable = bool(msg.data)
        if not self.task_enable:
            self._reset_cmd_vel_filter()
            self._last_cmd_vel_recv_monotonic = 0.0
            with self._lock:
                already_stopped = self.command.left_wheel_speed == 0 and self.command.right_wheel_speed == 0
                self.command.left_wheel_speed = 0
                self.command.right_wheel_speed = 0
                self.last_command_time = rospy.Time.now()
            if was_enabled and not already_stopped and self.enabled:
                self._write_motion_registers()
        rospy.loginfo("chassis task_enable updated: %s", str(self.task_enable))

    def _handle_emergency_stop_cmd(self, msg):
        if not bool(msg.data):
            return
        rospy.logwarn("Chassis emergency stop command received. Issuing safe stop immediately.")
        with self._lock:
            self.task_enable = False
            self._last_cmd_vel_recv_monotonic = 0.0
        self._reset_cmd_vel_filter()
        self._issue_safe_stop()

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

    def _handle_disc_speed_cmd(self, msg):
        with self._lock:
            self.command.disc_speed = clamp(msg.data, self.disc_speed_min, self.disc_speed_max)
        if self.enabled:
            self._write_single_output(REGISTER_DISC_SPEED, self.command.disc_speed, signed=True)

    def _handle_disc_enable_cmd(self, msg):
        with self._lock:
            self.command.disc_enable = DISC_ENABLE_ON if msg.data else DISC_ENABLE_OFF
        if self.enabled:
            self._write_single_output(REGISTER_DISC_ENABLE, self.command.disc_enable, signed=False)

    def _handle_work_mode_cmd(self, msg):
        with self._lock:
            self.command.work_mode = msg.data
        if self.enabled:
            self._write_single_output(REGISTER_WORK_MODE, self.command.work_mode, signed=False)

    def _handle_disc_lift_cmd(self, msg):
        if not self.disc_lift_supported:
            rospy.logwarn_throttle(5.0, "Disc lift command ignored: this machine has no disc lift.")
            return
        with self._lock:
            self.command.disc_lift = msg.data
        if self.enabled:
            self._write_single_output(REGISTER_DISC_LIFT, self.command.disc_lift, signed=False)

    def _handle_light_cmd(self, msg):
        with self._lock:
            self.command.light = LIGHT_ON if msg.data else LIGHT_OFF
        if self.enabled:
            self._write_single_output(REGISTER_LIGHT, self.command.light, signed=False)

    def _handle_enable_service(self, request):
        self.enabled = request.enable
        if not self.enabled:
            self._issue_safe_stop()
            return EnableChassisResponse(success=True, message="Chassis output disabled and motion stopped.")
        self.last_command_time = rospy.Time.now()
        try:
            self._write_all_outputs()
        except ModbusTransportError as exc:
            self._handle_transport_error(exc)
            return EnableChassisResponse(success=False, message="Failed to re-enable outputs: {}".format(exc))
        return EnableChassisResponse(success=True, message="Chassis output enabled.")

    def _handle_clear_fault_service(self, _request):
        return ClearFaultResponse(
            success=False,
            message="Fault clear register is not defined yet. Provide the vendor register table to implement it.",
        )

    def _write_motion_registers(self):
        with self._lock:
            desired_left = self.command.left_wheel_speed if self.enabled else 0
            desired_right = self.command.right_wheel_speed if self.enabled else 0
            left_speed = self._apply_ramp_limit(desired_left, self.last_sent_left_wheel_speed)
            right_speed = self._apply_ramp_limit(desired_right, self.last_sent_right_wheel_speed)
            if (
                int(left_speed) == int(self.last_sent_left_wheel_speed)
                and int(right_speed) == int(self.last_sent_right_wheel_speed)
            ):
                return
        write_started = time.monotonic()
        write_detail = "register=0x{:04X} left={} right={}".format(
            REGISTER_LEFT_WHEEL_SPEED,
            int(left_speed),
            int(right_speed),
        )
        self.transport.log_event("MOTION_WRITE_START", write_detail)
        try:
            # Keep left/right wheel command atomic in one Modbus frame
            # to avoid transient "single wheel moves first" behavior.
            self.transport.write_register_block(
                REGISTER_LEFT_WHEEL_SPEED,
                [left_speed, right_speed],
            )
            with self._lock:
                self.last_sent_left_wheel_speed = left_speed
                self.last_sent_right_wheel_speed = right_speed
                self._last_written_registers[REGISTER_LEFT_WHEEL_SPEED] = int(left_speed)
                self._last_written_registers[REGISTER_RIGHT_WHEEL_SPEED] = int(right_speed)
            self._mark_connected()
            self.transport.log_event(
                "MOTION_WRITE_OK",
                "{} elapsed_ms={:.3f}".format(
                    write_detail,
                    (time.monotonic() - write_started) * 1000.0,
                ),
            )
        except ModbusTransportError as exc:
            self.transport.log_event(
                "MOTION_WRITE_ERROR",
                "{} elapsed_ms={:.3f} error={}".format(
                    write_detail,
                    (time.monotonic() - write_started) * 1000.0,
                    exc,
                ),
            )
            self._handle_transport_error(exc)

    def _write_single_output(self, register_address, value, signed):
        with self._lock:
            if register_address in self._last_written_registers and int(self._last_written_registers[register_address]) == int(value):
                return
        try:
            if signed:
                self.transport.write_int16(register_address, value)
            else:
                self.transport.write_uint16(register_address, value)
            with self._lock:
                self._last_written_registers[register_address] = int(value)
            self._mark_connected()
        except ModbusTransportError as exc:
            self._handle_transport_error(exc)

    def _write_all_outputs(self, force=False):
        with self._lock:
            block = self.command.as_register_block()
            if not self.enabled:
                block[0] = 0
                block[1] = 0
                block[2] = 0
                block[3] = DISC_ENABLE_OFF
            block_tuple = tuple(int(x) for x in block)
            if not force and self._last_written_block == block_tuple:
                return
        if self.disc_lift_supported:
            self.transport.write_register_block(READ_BLOCK_START, block)
        else:
            # Do not touch the lift register on machines without lift hardware.
            self.transport.write_register_block(READ_BLOCK_START, block[:5])
            self.transport.write_uint16(REGISTER_LIGHT, block[6])
        with self._lock:
            self._last_written_block = block_tuple
            self.last_sent_left_wheel_speed = int(block[0])
            self.last_sent_right_wheel_speed = int(block[1])
            for index, value in enumerate(block):
                register = READ_BLOCK_START + index
                if self.disc_lift_supported or register != REGISTER_DISC_LIFT:
                    self._last_written_registers[register] = int(value)
        self._mark_connected()

    def _issue_safe_stop(self):
        with self._lock:
            self.command.left_wheel_speed = 0
            self.command.right_wheel_speed = 0
            self.command.disc_speed = 0
            self.command.disc_enable = DISC_ENABLE_OFF
        try:
            self._write_all_outputs()
        except ModbusTransportError as exc:
            self._handle_transport_error(exc)

    def _poll_once(self, _event):
        self._enforce_command_timeout()
        try:
            registers = self.transport.read_register_block(
                READ_BLOCK_START,
                READ_BLOCK_COUNT,
                signed_indices=(0, 1, 2),
            )
            actual_speed_registers = self.transport.read_register_block(
                REGISTER_LEFT_MOTOR_ACTUAL_SPEED,
                ACTUAL_SPEED_READ_COUNT,
                signed_indices=(0, 1),
            )
            with self._lock:
                self.snapshot = RegisterSnapshot.from_registers(registers)
                snapshot = RegisterSnapshot(**self.snapshot.__dict__)
                self.actual_left_motor_speed = int(actual_speed_registers[0])
                self.actual_right_motor_speed = int(actual_speed_registers[1])
                self.actual_speed_valid = True
            self._mark_connected()
            self._check_echo_deviation(snapshot)
        except ModbusTransportError as exc:
            with self._lock:
                self.actual_left_motor_speed = 0
                self.actual_right_motor_speed = 0
                self.actual_speed_valid = False
            self._handle_transport_error(exc)

        self._publish_state()
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
            self._write_motion_registers()

    def _publish_state(self):
        now = rospy.Time.now()
        with self._lock:
            command = ChassisCommand(**self.command.__dict__)
            snapshot = RegisterSnapshot(**self.snapshot.__dict__)
            connected = self.connected
            actual_left_motor_speed = self.actual_left_motor_speed
            actual_right_motor_speed = self.actual_right_motor_speed
            actual_speed_valid = self.actual_speed_valid
            enabled = self.enabled
            last_error = self.last_error
            consecutive_failures = self.consecutive_failures

        wheel_msg = WheelSpeedState()
        wheel_msg.header.stamp = now
        wheel_msg.target_left_wheel_speed = command.left_wheel_speed
        wheel_msg.target_right_wheel_speed = command.right_wheel_speed
        wheel_msg.feedback_left_wheel_speed = actual_left_motor_speed
        wheel_msg.feedback_right_wheel_speed = actual_right_motor_speed
        wheel_msg.feedback_valid = connected and actual_speed_valid
        self.wheel_speed_pub.publish(wheel_msg)

        self._publish_wheel_odom(
            now,
            actual_left_motor_speed,
            actual_right_motor_speed,
            connected and actual_speed_valid,
        )

        status_msg = ChassisStatus()
        status_msg.header.stamp = now
        status_msg.connected = connected
        status_msg.enabled = enabled
        status_msg.work_mode = snapshot.work_mode
        status_msg.disc_speed_target = command.disc_speed
        status_msg.disc_speed_feedback = snapshot.disc_speed
        status_msg.disc_enabled = snapshot.disc_enable == DISC_ENABLE_ON
        status_msg.disc_lift_state = snapshot.disc_lift
        status_msg.light_enabled = snapshot.light == LIGHT_ON
        status_msg.consecutive_failures = consecutive_failures
        status_msg.last_error = last_error
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

    def _publish_diagnostics(self):
        now = rospy.Time.now()
        with self._lock:
            connected = self.connected
            level = DiagnosticStatus.OK if connected else DiagnosticStatus.ERROR
            message = "connected" if connected else (self.last_error or "disconnected")
            consecutive_failures = self.consecutive_failures

        diag = DiagnosticStatus()
        diag.name = "grinder_chassis_driver"
        diag.hardware_id = "rs485_modbus_slave_{}".format(self.slave_id)
        diag.level = level
        diag.message = message
        diag.values = [
            KeyValue(key="port", value=str(self.port)),
            KeyValue(key="baudrate", value=str(self.baudrate)),
            KeyValue(key="slave_id", value=str(self.slave_id)),
            KeyValue(key="consecutive_failures", value=str(consecutive_failures)),
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
        with self._lock:
            self.connected = False
            self.last_error = str(exc)
            self.consecutive_failures += 1
            failures = self.consecutive_failures
        rospy.logwarn_throttle(2.0, "Modbus transport error: %s", exc)
        if failures >= self.max_retries and self.stop_on_timeout and self.enable_safe_stop_on_comm_error:
            self._issue_safe_stop_on_error()

    def _issue_safe_stop_on_error(self):
        now = time.monotonic()
        with self._lock:
            if now - self._last_safe_stop_on_error_time < max(0.0, self.safe_stop_on_error_interval):
                return
            self._last_safe_stop_on_error_time = now
        rospy.logwarn_throttle(1.0, "Safe-stop on communication error triggered.")
        with self._lock:
            self.command.left_wheel_speed = 0
            self.command.right_wheel_speed = 0
            self.command.disc_speed = 0
            self.command.disc_enable = DISC_ENABLE_OFF
        try:
            self.transport.reconnect()
            self._write_all_outputs(force=True)
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

    def _check_echo_deviation(self, snapshot):
        with self._lock:
            if not self.enabled:
                self.echo_failure_count = 0
                return
            expected_left = self.last_sent_left_wheel_speed
            expected_right = self.last_sent_right_wheel_speed
            max_dev = self.max_echo_deviation
            max_fail = max(1, self.max_echo_failures)

        left_diff = abs(int(snapshot.left_wheel_speed) - int(expected_left))
        right_diff = abs(int(snapshot.right_wheel_speed) - int(expected_right))
        if left_diff <= max_dev and right_diff <= max_dev:
            with self._lock:
                self.echo_failure_count = 0
            return

        with self._lock:
            self.echo_failure_count += 1
            echo_failures = self.echo_failure_count
        rospy.logwarn_throttle(
            1.0,
            "Modbus echo deviation warning: left_diff=%d right_diff=%d max=%d count=%d/%d",
            left_diff,
            right_diff,
            max_dev,
            echo_failures,
            max_fail,
        )
        if echo_failures >= max_fail:
            rospy.logwarn("Echo deviation protection triggered. Issuing safe stop.")
            self._issue_safe_stop_on_error()
            with self._lock:
                self.echo_failure_count = 0

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
