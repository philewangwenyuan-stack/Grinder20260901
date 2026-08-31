#!/usr/bin/env python3
"""Hardware-compatible chassis facade backed by Gazebo differential drive."""

import math
import threading
import time

import rospy
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


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def motor_rpm_to_twist(left_rpm, right_rpm, wheel_radius, wheel_track, gear_ratio):
    if wheel_radius <= 0.0 or wheel_track <= 0.0 or gear_ratio <= 0.0:
        raise ValueError("wheel geometry and gear ratio must be positive")
    factor = (2.0 * math.pi * wheel_radius) / (60.0 * gear_ratio)
    left_mps = float(left_rpm) * factor
    right_mps = float(right_rpm) * factor
    return (left_mps + right_mps) * 0.5, (right_mps - left_mps) / wheel_track


def twist_to_motor_rpm(linear, angular, wheel_radius, wheel_track, gear_ratio):
    if wheel_radius <= 0.0 or wheel_track <= 0.0 or gear_ratio <= 0.0:
        raise ValueError("wheel geometry and gear ratio must be positive")
    left_mps = float(linear) - float(angular) * wheel_track * 0.5
    right_mps = float(linear) + float(angular) * wheel_track * 0.5
    factor = 60.0 * gear_ratio / (2.0 * math.pi * wheel_radius)
    return left_mps * factor, right_mps * factor


class ChassisSimNode:
    def __init__(self):
        self._lock = threading.RLock()
        self._wheel_track = float(rospy.get_param("~wheel_track_m", 0.70))
        self._wheel_radius = float(rospy.get_param("~wheel_radius_m", 0.1475))
        self._gear_ratio = float(rospy.get_param("~gear_ratio", 60.0))
        self._max_motor_rpm = abs(float(rospy.get_param("~max_motor_rpm", 3000.0)))
        self._command_timeout = max(0.05, float(rospy.get_param("~command_timeout", 0.5)))
        self._output_rate = max(5.0, float(rospy.get_param("~output_rate_hz", 30.0)))
        self._status_rate = max(1.0, float(rospy.get_param("~status_rate_hz", 10.0)))
        self._drive_left_sign = -1 if int(rospy.get_param("~drive_left_sign", -1)) < 0 else 1
        self._drive_right_sign = -1 if int(rospy.get_param("~drive_right_sign", -1)) < 0 else 1
        self._drive_swap_lr = bool(rospy.get_param("~drive_swap_lr", False))
        self._sim_drive_axis_sign = -1 if int(rospy.get_param("~sim_drive_axis_sign", -1)) < 0 else 1
        self._sim_command_topic = rospy.get_param("~sim_command_topic", "/grinder/sim/cmd_vel")
        self._sim_odom_topic = rospy.get_param("~sim_odom_topic", "/odom")

        if self._wheel_track <= 0.0 or self._wheel_radius <= 0.0 or self._gear_ratio <= 0.0:
            raise ValueError("Invalid chassis geometry")

        self._enabled = True
        self._task_enabled = False
        self._emergency_stop = False
        self._work_mode = 1
        self._disc_speed = 0
        self._disc_enabled = False
        self._disc_lift_state = 0
        self._light_enabled = False
        self._manual_speed_limit = 1.0

        self._auto_command = Twist()
        self._manual_left_rpm = 0.0
        self._manual_right_rpm = 0.0
        self._auto_stamp = 0.0
        self._manual_stamp = 0.0
        self._last_output = Twist()
        self._last_odom = None

        self._sim_cmd_pub = rospy.Publisher(self._sim_command_topic, Twist, queue_size=10)
        self._status_pub = rospy.Publisher("/chassis/status", ChassisStatus, queue_size=10)
        self._wheel_state_pub = rospy.Publisher(
            "/chassis/wheel_speed_state", WheelSpeedState, queue_size=10
        )
        self._wheel_odom_pub = rospy.Publisher("/odom_wheel", Odometry, queue_size=10)

        rospy.Subscriber("/cmd_vel", Twist, self._on_cmd_vel, queue_size=10)
        rospy.Subscriber("/chassis/wheel_speed_cmd", WheelSpeedCommand, self._on_wheel_speed, queue_size=10)
        rospy.Subscriber("/chassis/task_enable", Bool, self._on_task_enable, queue_size=10)
        rospy.Subscriber("/chassis/disc_speed_cmd", Int16, self._on_disc_speed, queue_size=10)
        rospy.Subscriber("/chassis/disc_enable_cmd", Bool, self._on_disc_enable, queue_size=10)
        rospy.Subscriber("/chassis/work_mode_cmd", UInt16, self._on_work_mode, queue_size=10)
        rospy.Subscriber("/chassis/disc_lift_cmd", UInt16, self._on_disc_lift, queue_size=10)
        rospy.Subscriber("/chassis/light_cmd", Bool, self._on_light, queue_size=10)
        rospy.Subscriber("/chassis/manual_speed_limit", Float32, self._on_manual_limit, queue_size=1)
        rospy.Subscriber("/chassis/emergency_stop", Bool, self._on_emergency_stop, queue_size=1)
        rospy.Subscriber(self._sim_odom_topic, Odometry, self._on_odom, queue_size=20)

        rospy.Service("/chassis/enable", EnableChassis, self._handle_enable)
        rospy.Service("/chassis/clear_fault", ClearFault, self._handle_clear_fault)

        self._output_timer = rospy.Timer(rospy.Duration(1.0 / self._output_rate), self._output_tick)
        self._status_timer = rospy.Timer(rospy.Duration(1.0 / self._status_rate), self._status_tick)
        rospy.on_shutdown(self._shutdown)
        rospy.loginfo(
            "X920 chassis simulator ready: track=%.3fm radius=%.4fm gear=%.1f command=%s",
            self._wheel_track,
            self._wheel_radius,
            self._gear_ratio,
            self._sim_command_topic,
        )

    @staticmethod
    def _copy_twist(message):
        result = Twist()
        result.linear.x = float(message.linear.x)
        result.linear.y = float(message.linear.y)
        result.linear.z = float(message.linear.z)
        result.angular.x = float(message.angular.x)
        result.angular.y = float(message.angular.y)
        result.angular.z = float(message.angular.z)
        return result

    def _on_cmd_vel(self, message):
        with self._lock:
            self._auto_command = self._copy_twist(message)
            self._auto_stamp = time.monotonic()

    def _on_wheel_speed(self, message):
        with self._lock:
            self._manual_left_rpm = clamp(
                float(message.left_wheel_speed), -self._max_motor_rpm, self._max_motor_rpm
            )
            self._manual_right_rpm = clamp(
                float(message.right_wheel_speed), -self._max_motor_rpm, self._max_motor_rpm
            )
            self._manual_stamp = time.monotonic()

    def _on_task_enable(self, message):
        with self._lock:
            self._task_enabled = bool(message.data)
            if not self._task_enabled:
                self._auto_stamp = 0.0

    def _on_disc_speed(self, message):
        with self._lock:
            self._disc_speed = int(clamp(int(message.data), -32768, 32767))

    def _on_disc_enable(self, message):
        with self._lock:
            self._disc_enabled = bool(message.data)

    def _on_work_mode(self, message):
        with self._lock:
            self._work_mode = int(message.data)

    def _on_disc_lift(self, message):
        with self._lock:
            self._disc_lift_state = int(message.data)

    def _on_light(self, message):
        with self._lock:
            self._light_enabled = bool(message.data)

    def _on_manual_limit(self, message):
        with self._lock:
            self._manual_speed_limit = clamp(float(message.data), 0.0, 1.0)

    def _on_emergency_stop(self, message):
        with self._lock:
            self._emergency_stop = bool(message.data)
            if self._emergency_stop:
                self._task_enabled = False
                self._auto_stamp = 0.0
                self._manual_stamp = 0.0
                self._disc_enabled = False

    def _on_odom(self, message):
        with self._lock:
            self._last_odom = message
        wheel_odom = Odometry()
        wheel_odom.header = message.header
        wheel_odom.child_frame_id = message.child_frame_id
        wheel_odom.pose = message.pose
        wheel_odom.twist = message.twist
        self._wheel_odom_pub.publish(wheel_odom)

    def _handle_enable(self, request):
        with self._lock:
            self._enabled = bool(request.enable)
            if not self._enabled:
                self._task_enabled = False
                self._auto_stamp = 0.0
                self._manual_stamp = 0.0
                self._disc_enabled = False
        return EnableChassisResponse(success=True, message="sim_chassis_enabled" if request.enable else "sim_chassis_disabled")

    def _handle_clear_fault(self, _request):
        with self._lock:
            self._emergency_stop = False
        return ClearFaultResponse(success=True, message="sim_faults_cleared")

    def _select_output_locked(self, now):
        if not self._enabled or self._emergency_stop:
            return Twist(), "safety_stop"

        if self._task_enabled and self._auto_stamp > 0.0:
            if now - self._auto_stamp <= self._command_timeout:
                return self._copy_twist(self._auto_command), "navigation"
            return Twist(), "navigation_timeout"

        if (not self._task_enabled) and self._manual_stamp > 0.0:
            if now - self._manual_stamp <= self._command_timeout:
                left_rpm = self._manual_left_rpm * self._manual_speed_limit
                right_rpm = self._manual_right_rpm * self._manual_speed_limit
                if self._drive_swap_lr:
                    left_rpm, right_rpm = right_rpm, left_rpm
                # Match chassis_driver_node._apply_drive_direction(): manual APP
                # wheel signs are controller-facing, not Gazebo joint signs.
                linear, angular = motor_rpm_to_twist(
                    left_rpm * self._drive_left_sign,
                    right_rpm * self._drive_right_sign,
                    self._wheel_radius,
                    self._wheel_track,
                    self._gear_ratio,
                )
                command = Twist()
                command.linear.x = linear
                command.angular.z = angular
                return command, "manual"
            return Twist(), "manual_timeout"

        return Twist(), "idle"

    def _output_tick(self, _event):
        with self._lock:
            output, source = self._select_output_locked(time.monotonic())
            self._last_output = self._copy_twist(output)
        sim_output = self._copy_twist(output)
        if source == "navigation":
            # Gazebo wheel joints use the real motor's -Y installation axis.
            # Convert ROS body-frame commands back to that joint convention;
            # manual wheel commands already received drive_left/right_sign above.
            sim_output.linear.x *= self._sim_drive_axis_sign
            sim_output.angular.z *= self._sim_drive_axis_sign
        self._sim_cmd_pub.publish(sim_output)

    def _feedback_rpm_locked(self):
        if self._last_odom is not None:
            linear = float(self._last_odom.twist.twist.linear.x)
            angular = float(self._last_odom.twist.twist.angular.z)
        else:
            linear = float(self._last_output.linear.x)
            angular = float(self._last_output.angular.z)
        left, right = twist_to_motor_rpm(
            linear, angular, self._wheel_radius, self._wheel_track, self._gear_ratio
        )
        return int(round(clamp(left, -32768, 32767))), int(round(clamp(right, -32768, 32767)))

    def _target_rpm_locked(self):
        left, right = twist_to_motor_rpm(
            self._last_output.linear.x,
            self._last_output.angular.z,
            self._wheel_radius,
            self._wheel_track,
            self._gear_ratio,
        )
        return int(round(clamp(left, -32768, 32767))), int(round(clamp(right, -32768, 32767)))

    def _status_tick(self, _event):
        now = rospy.Time.now()
        with self._lock:
            target_left, target_right = self._target_rpm_locked()
            feedback_left, feedback_right = self._feedback_rpm_locked()

            wheel_state = WheelSpeedState()
            wheel_state.header.stamp = now
            wheel_state.header.frame_id = "base_footprint"
            wheel_state.target_left_wheel_speed = target_left
            wheel_state.target_right_wheel_speed = target_right
            wheel_state.feedback_left_wheel_speed = feedback_left
            wheel_state.feedback_right_wheel_speed = feedback_right
            wheel_state.feedback_valid = bool(self._enabled)

            status = ChassisStatus()
            status.header.stamp = now
            status.header.frame_id = "base_footprint"
            status.connected = True
            status.enabled = bool(self._enabled and not self._emergency_stop)
            status.work_mode = int(self._work_mode)
            status.disc_speed_target = int(self._disc_speed)
            status.disc_speed_feedback = int(self._disc_speed if self._disc_enabled else 0)
            status.disc_enabled = bool(self._disc_enabled and not self._emergency_stop)
            status.disc_lift_state = int(self._disc_lift_state)
            status.light_enabled = bool(self._light_enabled)
            status.consecutive_failures = 0
            status.last_error = "sim_emergency_stop" if self._emergency_stop else ""

        self._wheel_state_pub.publish(wheel_state)
        self._status_pub.publish(status)

    def _shutdown(self):
        self._sim_cmd_pub.publish(Twist())


def main():
    rospy.init_node("grinder_chassis_sim")
    ChassisSimNode()
    rospy.spin()


if __name__ == "__main__":
    main()
