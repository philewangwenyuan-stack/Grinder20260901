#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace grinder_chassis_driver {

inline double monotonicSeconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

constexpr uint16_t REGISTER_LEFT_WHEEL_SPEED = 0x4010;
constexpr uint16_t REGISTER_RIGHT_WHEEL_SPEED = 0x4011;
constexpr uint16_t REGISTER_DISC_SPEED = 0x4012;
constexpr uint16_t REGISTER_DISC_ENABLE = 0x4013;
constexpr uint16_t REGISTER_WORK_MODE = 0x4014;
constexpr uint16_t REGISTER_DISC_LIFT = 0x4015;
constexpr uint16_t REGISTER_LIGHT = 0x4016;
constexpr uint16_t REGISTER_LEFT_MOTOR_ACTUAL_SPEED = 0x4017;
constexpr uint16_t REGISTER_RIGHT_MOTOR_ACTUAL_SPEED = 0x4018;

// Defaults deliberately match config/chassis_driver.yaml.
struct Config {
  std::string port = "/dev/ttyS0";
  int baudrate = 19200;
  int slave_id = 1;
  double timeout = 0.1;
  double transaction_timeout = 0.1;
  double command_watchdog_period = 0.01;
  double poll_rate_hz = 10.0;
  double status_poll_rate_hz = 2.0;
  double diagnostics_rate_hz = 1.0;
  bool status_combined_read = true;
  double command_timeout = 0.5;
  bool write_verify = false;
  bool use_functioncode_16 = true;
  int max_retries = 3;
  bool enable_safe_stop_on_comm_error = false;
  bool stop_on_timeout = true;
  bool startup_zero_output = true;
  int left_speed_min = -32768, left_speed_max = 32767;
  int right_speed_min = -32768, right_speed_max = 32767;
  int disc_speed_min = -32768, disc_speed_max = 32767;
  bool disc_lift_supported = true;
  int max_cmd_step_rpm = 50;
  bool enable_cmd_ramp_limit = false;
  double manual_drive_max_wheel_speed = 0.2;
  // Internal marker: configuration copies must not convert legacy RPM twice.
  bool manual_limit_normalized = false;
  int manual_wheel_deadband_rpm = 30;
  int manual_wheel_hyst_enter_rpm = 60;
  int manual_wheel_hyst_exit_rpm = 30;
  double safe_stop_on_error_interval = 2.0;
  bool rs485_raw_log_enabled = true;
  std::string rs485_raw_log_file = "temp/chassis_rs485_raw.log";
  int rs485_log_queue_capacity = 2048;
  bool cmd_vel_enabled = true;
  std::string cmd_vel_topic = "/cmd_vel";
  bool task_enable_default = false;
  double cmd_vel_wheel_track_m = 0.70;
  double cmd_vel_wheel_radius_m = 0.1475;
  double cmd_vel_gear_ratio = 60.0;
  std::string wheel_odom_topic = "/odom_wheel";
  std::string wheel_twist_topic = "/chassis/wheel_twist";
  std::string wheel_odom_frame_id = "odom";
  std::string wheel_odom_child_frame_id = "base_link";
  double wheel_feedback_delay_sec = 0.0;
  double wheel_twist_linear_variance = 0.04;
  double wheel_twist_angular_variance = 0.04;
  double wheel_twist_cross_covariance = 0.0;
  double actual_motor_speed_scale = 1.0;
  int actual_speed_left_sign = 1, actual_speed_right_sign = 1;
  bool actual_speed_swap_lr = false;
  double cmd_vel_scale = 1.0;
  double cmd_vel_max_linear = 0.2;
  double cmd_vel_max_angular = 0.15;
  double cmd_vel_max_input_v = 0.2;
  double cmd_vel_max_input_w = 0.15;
  double cmd_vel_max_abs_wheel_rpm = 3000.0;
  double cmd_vel_deadband_linear = 0.0;
  double cmd_vel_deadband_angular = 0.0;
  double cmd_source_hold_sec = 0.2;
  int drive_left_sign = -1, drive_right_sign = -1;
  bool drive_swap_lr = false;
  int cmd_vel_drive_left_sign = 1, cmd_vel_drive_right_sign = 1;
  bool cmd_vel_drive_swap_lr = false;
  bool cmd_vel_filter_enabled = true;
  double cmd_vel_filter_linear_accel = 2.0;
  double cmd_vel_filter_angular_accel = 2.0;
  double cmd_vel_filter_angular_deadband = 0.045;
  int cmd_vel_filter_sign_confirm_count = 2;
  std::string config_file;
  std::string motion_emergency_file = "~/.ros/grinder_safety/chassis_emergency.json";
  std::string indicator_emergency_file = "~/.ros/grinder_indicator/chassis_emergency.json";
};

struct Command {
  int left_wheel_speed = 0, right_wheel_speed = 0, disc_speed = 0;
  uint16_t disc_enable = 0, work_mode = 2, disc_lift = 2, light = 0;
};

struct RegisterSnapshot {
  int disc_speed = 0;
  uint16_t disc_enable = 0, work_mode = 2, disc_lift = 2, light = 0;
};

struct Timing { double tx = 0.0, rx = 0.0; };
struct Result { bool success = false; std::string message; };

struct Snapshot {
  Command command;
  RegisterSnapshot status;
  int actual_left_motor_speed = 0, actual_right_motor_speed = 0;
  bool actual_speed_valid = false, connected = false, enabled = true;
  bool task_enable = false, outputs_ready = false, safe_write_confirmed = false;
  bool emergency_latched = false, indicator_emergency = false;
  bool status_combined_read = true;
  uint64_t generation = 0, command_sequence = 0, wheel_sequence = 0, status_sequence = 0;
  unsigned consecutive_failures = 0;
  std::string last_error, reset_error;
  double last_actual_read = 0.0, last_status_read = 0.0;
  Timing actual_timing;
  double manual_speed_limit = 0.2;
  double command_compute_ms = 0.0, command_wait_ms = 0.0, transaction_ms = 0.0;
  double watchdog_lateness_ms = 0.0, stop_tx_ms = 0.0, stop_ack_ms = 0.0;
  uint64_t timeout_count = 0, dropped_logs = 0;
};

}  // namespace grinder_chassis_driver
