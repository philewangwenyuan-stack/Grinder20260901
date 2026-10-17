#include "grinder_chassis_driver/core.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace grinder_chassis_driver {
namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;

void finite(double value, const char* name) {
  if (!std::isfinite(value)) throw std::invalid_argument(std::string(name) + " must be finite");
}

void positive(double value, const char* name) {
  finite(value, name);
  if (value <= 0.0) throw std::invalid_argument(std::string(name) + " must be positive");
}

void nonnegative(double value, const char* name) {
  finite(value, name);
  if (value < 0.0) throw std::invalid_argument(std::string(name) + " must be nonnegative");
}

void range(int minimum, int maximum, const char* name) {
  if (minimum < -32768 || maximum > 32767 || minimum > 0 || maximum < 0 || minimum > maximum)
    throw std::invalid_argument(std::string(name) + " must be an int16 range containing zero");
}

int sign(int value) { return value < 0 ? -1 : 1; }
int velocitySign(double value, double deadband) {
  return value > deadband ? 1 : (value < -deadband ? -1 : 0);
}

double rateLimit(double target, double previous, double rate, double dt) {
  const double step = std::max(0.0, rate) * std::max(0.0, dt);
  return previous + std::max(-step, std::min(step, target - previous));
}

int mappedWheel(int value, int direction, int minimum, int maximum) {
  // Negating int16's -32768 must happen before narrowing or clamping.
  const int64_t mapped = static_cast<int64_t>(value) * sign(direction);
  return static_cast<int>(std::max<int64_t>(minimum, std::min<int64_t>(maximum, mapped)));
}
}  // namespace

int clamp(int value, int minimum, int maximum) {
  return std::max(minimum, std::min(maximum, value));
}

int toInt16(uint16_t value) {
  return value & 0x8000 ? static_cast<int>(value) - 0x10000 : static_cast<int>(value);
}

uint16_t toUint16(int value) { return static_cast<uint16_t>(value); }

int roundEven(double value) {
  finite(value, "rounded value");
  const double lower = std::floor(value);
  const double fraction = value - lower;
  const double rounded = fraction < 0.5 ? lower :
      (fraction > 0.5 ? lower + 1.0 : (std::fmod(lower, 2.0) == 0.0 ? lower : lower + 1.0));
  if (rounded < std::numeric_limits<int>::min() || rounded > std::numeric_limits<int>::max())
    throw std::overflow_error("rounded value is outside int range");
  return static_cast<int>(rounded);
}

void validateConfig(Config& c) {
  if (c.port.empty()) throw std::invalid_argument("port must not be empty");
  if (c.baudrate <= 0 || c.slave_id < 1 || c.slave_id > 247)
    throw std::invalid_argument("invalid Modbus baudrate or slave_id");
  positive(c.timeout, "timeout");
  positive(c.transaction_timeout, "transaction_timeout");
  positive(c.command_watchdog_period, "command_watchdog_period");
  positive(c.poll_rate_hz, "poll_rate_hz");
  c.poll_rate_hz = std::max(1.0, c.poll_rate_hz);
  positive(c.status_poll_rate_hz, "status_poll_rate_hz");
  positive(c.diagnostics_rate_hz, "diagnostics_rate_hz");
  nonnegative(c.command_timeout, "command_timeout");
  finite(c.safe_stop_on_error_interval, "safe_stop_on_error_interval");
  c.safe_stop_on_error_interval = std::max(0.0, c.safe_stop_on_error_interval);
  if (c.max_retries < 1 || c.rs485_log_queue_capacity < 1)
    throw std::invalid_argument("max_retries and rs485_log_queue_capacity must be positive");
  range(c.left_speed_min, c.left_speed_max, "left speed limits");
  range(c.right_speed_min, c.right_speed_max, "right speed limits");
  range(c.disc_speed_min, c.disc_speed_max, "disc speed limits");
  positive(c.cmd_vel_wheel_track_m, "cmd_vel_wheel_track_m");
  positive(c.cmd_vel_wheel_radius_m, "cmd_vel_wheel_radius_m");
  positive(c.cmd_vel_gear_ratio, "cmd_vel_gear_ratio");
  positive(c.actual_motor_speed_scale, "actual_motor_speed_scale");
  positive(c.cmd_vel_scale, "cmd_vel_scale");
  nonnegative(c.wheel_feedback_delay_sec, "wheel_feedback_delay_sec");
  positive(c.wheel_twist_linear_variance, "wheel_twist_linear_variance");
  positive(c.wheel_twist_angular_variance, "wheel_twist_angular_variance");
  finite(c.wheel_twist_cross_covariance, "wheel_twist_cross_covariance");
  // Linux long double keeps double products finite and compares the singular
  // boundary directly, without sqrt/division rounding making it look valid.
  const long double cross = c.wheel_twist_cross_covariance;
  if (cross * cross >= static_cast<long double>(c.wheel_twist_linear_variance) *
      c.wheel_twist_angular_variance)
    throw std::invalid_argument("wheel twist covariance must be positive definite");
  nonnegative(c.cmd_vel_max_linear, "cmd_vel_max_linear");
  nonnegative(c.cmd_vel_max_angular, "cmd_vel_max_angular");
  finite(c.cmd_vel_max_input_v, "cmd_vel_max_input_v");
  finite(c.cmd_vel_max_input_w, "cmd_vel_max_input_w");
  finite(c.cmd_vel_max_abs_wheel_rpm, "cmd_vel_max_abs_wheel_rpm");
  finite(c.cmd_vel_deadband_linear, "cmd_vel_deadband_linear");
  finite(c.cmd_vel_deadband_angular, "cmd_vel_deadband_angular");
  finite(c.cmd_source_hold_sec, "cmd_source_hold_sec");
  finite(c.cmd_vel_filter_linear_accel, "cmd_vel_filter_linear_accel");
  finite(c.cmd_vel_filter_angular_accel, "cmd_vel_filter_angular_accel");
  finite(c.cmd_vel_filter_angular_deadband, "cmd_vel_filter_angular_deadband");
  finite(c.manual_drive_max_wheel_speed, "manual_drive_max_wheel_speed");
  c.cmd_vel_deadband_linear = std::max(0.0, c.cmd_vel_deadband_linear);
  c.cmd_vel_deadband_angular = std::max(0.0, c.cmd_vel_deadband_angular);
  c.cmd_source_hold_sec = std::max(0.0, c.cmd_source_hold_sec);
  c.cmd_vel_filter_linear_accel = std::max(0.0, c.cmd_vel_filter_linear_accel);
  c.cmd_vel_filter_angular_accel = std::max(0.0, c.cmd_vel_filter_angular_accel);
  c.cmd_vel_filter_angular_deadband = std::max(0.0, c.cmd_vel_filter_angular_deadband);
  c.cmd_vel_filter_sign_confirm_count = std::max(1, c.cmd_vel_filter_sign_confirm_count);
  c.manual_wheel_deadband_rpm = std::max(0, c.manual_wheel_deadband_rpm);
  c.manual_wheel_hyst_exit_rpm = std::max(0, c.manual_wheel_hyst_exit_rpm);
  c.manual_wheel_hyst_enter_rpm = std::max(c.manual_wheel_hyst_exit_rpm, c.manual_wheel_hyst_enter_rpm);
  c.max_cmd_step_rpm = std::max(0, c.max_cmd_step_rpm);
  c.drive_left_sign = sign(c.drive_left_sign);
  c.drive_right_sign = sign(c.drive_right_sign);
  c.cmd_vel_drive_left_sign = sign(c.cmd_vel_drive_left_sign);
  c.cmd_vel_drive_right_sign = sign(c.cmd_vel_drive_right_sign);
  c.actual_speed_left_sign = sign(c.actual_speed_left_sign);
  c.actual_speed_right_sign = sign(c.actual_speed_right_sign);
  if (!c.manual_limit_normalized && c.manual_drive_max_wheel_speed > 10.0) {
    c.manual_drive_max_wheel_speed = (c.manual_drive_max_wheel_speed /
        c.cmd_vel_gear_ratio / c.cmd_vel_scale) * (2.0 * kPi * c.cmd_vel_wheel_radius_m) / 60.0;
  }
  c.manual_drive_max_wheel_speed = std::max(0.0, c.manual_drive_max_wheel_speed);
  finite(c.manual_drive_max_wheel_speed, "converted manual_drive_max_wheel_speed");
  c.manual_limit_normalized = true;
  if (c.cmd_vel_topic.empty() || c.wheel_odom_topic.empty() || c.wheel_twist_topic.empty() ||
      c.wheel_odom_frame_id.empty() || c.wheel_odom_child_frame_id.empty() ||
      c.motion_emergency_file.empty() || c.indicator_emergency_file.empty())
    throw std::invalid_argument("topic, frame and emergency file names must not be empty");
}

std::vector<uint16_t> registerBlock(const Command& c) {
  return {toUint16(c.left_wheel_speed), toUint16(c.right_wheel_speed), toUint16(c.disc_speed),
          c.disc_enable, c.work_mode, c.disc_lift, c.light};
}

RegisterSnapshot decodeStatus(const std::vector<uint16_t>& registers) {
  if (registers.size() < 5) throw std::invalid_argument("expected at least five status registers");
  RegisterSnapshot result;
  result.disc_speed = toInt16(registers[0]);
  result.disc_enable = registers[1];
  result.work_mode = registers[2];
  result.disc_lift = registers[3];
  result.light = registers[4];
  return result;
}

std::pair<double, double> motorRpmToBaseTwist(int left, int right, const Config& c) {
  positive(c.cmd_vel_wheel_radius_m, "cmd_vel_wheel_radius_m");
  positive(c.cmd_vel_wheel_track_m, "cmd_vel_wheel_track_m");
  positive(c.cmd_vel_gear_ratio, "cmd_vel_gear_ratio");
  positive(c.actual_motor_speed_scale, "actual_motor_speed_scale");
  double l = static_cast<double>(left) * c.actual_motor_speed_scale * sign(c.actual_speed_left_sign);
  double r = static_cast<double>(right) * c.actual_motor_speed_scale * sign(c.actual_speed_right_sign);
  // Feedback signs belong to the raw motors, before exchanging left/right.
  if (c.actual_speed_swap_lr) std::swap(l, r);
  const double factor = (2.0 * kPi * c.cmd_vel_wheel_radius_m) / (60.0 * c.cmd_vel_gear_ratio);
  const double linear = 0.5 * (l * factor + r * factor);
  const double angular = (r * factor - l * factor) / c.cmd_vel_wheel_track_m;
  finite(linear, "feedback linear velocity");
  finite(angular, "feedback angular velocity");
  return {linear, angular};
}

int manualSpeedLimitRpm(double limit, const Config& c) {
  finite(limit, "manual speed limit");
  const double speed = std::abs(limit);
  if (speed == 0.0 || c.cmd_vel_wheel_radius_m <= 0.0) return 0;
  positive(c.cmd_vel_wheel_radius_m, "cmd_vel_wheel_radius_m");
  positive(c.cmd_vel_gear_ratio, "cmd_vel_gear_ratio");
  positive(c.cmd_vel_scale, "cmd_vel_scale");
  finite(c.cmd_vel_max_abs_wheel_rpm, "cmd_vel_max_abs_wheel_rpm");
  // Saturate before integer conversion, including extremely large finite caps.
  const double cap = std::min(32767.0, std::abs(c.cmd_vel_max_abs_wheel_rpm));
  const int safe_cap = std::min(std::max(1, static_cast<int>(cap)),
      std::min(std::abs(c.left_speed_max), std::abs(c.right_speed_max)));
  const double rpm = speed * 60.0 / (2.0 * kPi * c.cmd_vel_wheel_radius_m) *
      c.cmd_vel_gear_ratio * c.cmd_vel_scale;
  if (rpm >= safe_cap) return std::max(1, safe_cap);
  return std::max(1, std::min(safe_cap, roundEven(rpm)));
}

int filterManualInput(int value, int previous, const Config& c) {
  const int deadband = std::max(0, c.manual_wheel_deadband_rpm);
  const int enter = std::max(deadband, c.manual_wheel_hyst_enter_rpm);
  const int exit = std::max(deadband, c.manual_wheel_hyst_exit_rpm);
  const int64_t magnitude = std::abs(static_cast<int64_t>(value));
  if (magnitude <= deadband || (previous == 0 && magnitude < enter) ||
      (previous != 0 && magnitude <= exit)) return 0;
  return value;
}

std::pair<int, int> driveDirection(int left, int right, const Config& c, bool navigation) {
  if (navigation ? c.cmd_vel_drive_swap_lr : c.drive_swap_lr) std::swap(left, right);
  return {mappedWheel(left, navigation ? c.cmd_vel_drive_left_sign : c.drive_left_sign,
                      c.left_speed_min, c.left_speed_max),
          mappedWheel(right, navigation ? c.cmd_vel_drive_right_sign : c.drive_right_sign,
                      c.right_speed_min, c.right_speed_max)};
}

void VelocityFilter::reset(double now) {
  finite(now, "filter time");
  linear_ = angular_ = 0.0;
  last_time_ = now;
  pending_sign_ = pending_count_ = 0;
}

std::pair<double, double> VelocityFilter::apply(double linear, double angular, const Config& c, double now) {
  finite(linear, "linear velocity");
  finite(angular, "angular velocity");
  finite(now, "filter time");
  if (!c.cmd_vel_filter_enabled) {
    linear_ = linear;
    angular_ = angular;
    last_time_ = now;
    return {linear, angular};
  }
  double dt = now - last_time_;
  if (dt <= 0.0) dt = 0.02;
  dt = std::max(0.02, std::min(0.20, dt));
  const double deadband = std::max(0.0, c.cmd_vel_filter_angular_deadband);
  if (std::abs(angular) < deadband) angular = 0.0;
  const int requested_sign = velocitySign(angular, deadband);
  const int last_sign = velocitySign(angular_, deadband);
  const int confirm = std::max(1, c.cmd_vel_filter_sign_confirm_count);
  if (confirm > 1 && requested_sign != 0 && last_sign != 0 && requested_sign != last_sign) {
    if (requested_sign == pending_sign_) ++pending_count_;
    else { pending_sign_ = requested_sign; pending_count_ = 1; }
    if (pending_count_ < confirm) angular = 0.0;
    else { pending_sign_ = 0; pending_count_ = 0; }
  } else {
    pending_sign_ = 0;
    pending_count_ = 0;
  }
  linear = rateLimit(linear, linear_, c.cmd_vel_filter_linear_accel, dt);
  angular = rateLimit(angular, angular_, c.cmd_vel_filter_angular_accel, dt);
  if (std::abs(angular) < deadband) angular = 0.0;
  linear_ = linear;
  angular_ = angular;
  last_time_ = now;
  return {linear, angular};
}

std::pair<int, int> navigationWheels(double linear, double angular, const Config& c,
                                     VelocityFilter& filter, double now) {
  finite(linear, "linear velocity");
  finite(angular, "angular velocity");
  positive(c.cmd_vel_wheel_radius_m, "cmd_vel_wheel_radius_m");
  positive(c.cmd_vel_wheel_track_m, "cmd_vel_wheel_track_m");
  double linear_limit = c.cmd_vel_max_linear, angular_limit = c.cmd_vel_max_angular;
  if (c.cmd_vel_max_input_v > 0.0) linear_limit = std::min(linear_limit, c.cmd_vel_max_input_v);
  if (c.cmd_vel_max_input_w > 0.0) angular_limit = std::min(angular_limit, c.cmd_vel_max_input_w);
  linear = std::max(-linear_limit, std::min(linear_limit, linear));
  angular = std::max(-angular_limit, std::min(angular_limit, angular));
  if (std::abs(linear) < std::max(0.0, c.cmd_vel_deadband_linear)) linear = 0.0;
  if (std::abs(angular) < std::max(0.0, c.cmd_vel_deadband_angular)) angular = 0.0;
  const auto filtered = filter.apply(linear, angular, c, now);
  linear = filtered.first;
  angular = filtered.second;
  const double factor = 60.0 / (2.0 * kPi * c.cmd_vel_wheel_radius_m);
  double l = (linear - angular * (c.cmd_vel_wheel_track_m * 0.5)) * factor * c.cmd_vel_gear_ratio * c.cmd_vel_scale;
  double r = (linear + angular * (c.cmd_vel_wheel_track_m * 0.5)) * factor * c.cmd_vel_gear_ratio * c.cmd_vel_scale;
  const double cap = std::abs(c.cmd_vel_max_abs_wheel_rpm);
  if (cap > 0.0) {
    l = std::max(-cap, std::min(cap, l));
    r = std::max(-cap, std::min(cap, r));
  }
  return driveDirection(roundEven(l), roundEven(r), c, true);
}
}  // namespace grinder_chassis_driver
