#include "grinder_chassis_driver/core.h"

#include <gtest/gtest.h>
#include <cfenv>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace grinder_chassis_driver {
namespace {
constexpr double kPi = 3.14159265358979323846;

TEST(RegisterCore, SignedEncodingAndStatusAreCompatible) {
  EXPECT_EQ(0xffff, toUint16(-1));
  EXPECT_EQ(0x8000, toUint16(-32768));
  EXPECT_EQ(-32768, toInt16(0x8000));
  EXPECT_EQ(-1, toInt16(0xffff));
  EXPECT_EQ(32767, toInt16(0x7fff));
  Command c;
  c.left_wheel_speed = -100;
  c.right_wheel_speed = 200;
  c.disc_speed = -32768;
  c.disc_enable = 1;
  EXPECT_EQ((std::vector<uint16_t>{0xff9c, 200, 0x8000, 1, 2, 2, 0}), registerBlock(c));
  const auto status = decodeStatus({0xff9c, 1, 2, 1, 1, 500, 600});
  EXPECT_EQ(-100, status.disc_speed);
  EXPECT_EQ(1, status.disc_enable);
  EXPECT_EQ(2, status.work_mode);
  EXPECT_EQ(1, status.disc_lift);
  EXPECT_EQ(1, status.light);
  EXPECT_THROW(decodeStatus({0, 0, 0, 0}), std::invalid_argument);
}

TEST(RegisterCore, RoundingMatchesPythonTiesEvenRegardlessOfFloatingPointMode) {
  const int previous_mode = std::fegetround();
  const double inputs[] = {-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5};
  const int expected[] = {-4, -2, -2, 0, 0, 2, 2, 4};
  for (int mode : {FE_DOWNWARD, FE_UPWARD, FE_TONEAREST}) {
    ASSERT_EQ(0, std::fesetround(mode));
    for (std::size_t i = 0; i < 8; ++i) EXPECT_EQ(expected[i], roundEven(inputs[i]));
  }
  std::fesetround(previous_mode);
  EXPECT_EQ(3, roundEven(std::nextafter(2.5, 3.0)));
  EXPECT_EQ(2, roundEven(std::nextafter(2.5, 2.0)));
  EXPECT_THROW(roundEven(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW(roundEven(std::numeric_limits<double>::infinity()), std::invalid_argument);
  EXPECT_THROW(roundEven(2147483648.0), std::overflow_error);
}

TEST(ConfigCore, YamlDefaultsAreValidAndLegacyRpmIsConvertedOnceAtLoad) {
  Config c;
  EXPECT_NO_THROW(validateConfig(c));
  EXPECT_EQ(19200, c.baudrate);
  EXPECT_DOUBLE_EQ(10.0, c.poll_rate_hz);
  EXPECT_DOUBLE_EQ(0.5, c.command_timeout);
  EXPECT_EQ("/chassis/wheel_twist", c.wheel_twist_topic);
  c = Config{};  // A separately loaded legacy configuration starts unnormalized.
  c.manual_drive_max_wheel_speed = 600.0;
  c.cmd_vel_wheel_radius_m = 0.1;
  c.cmd_vel_gear_ratio = 60.0;
  c.cmd_vel_scale = 2.0;
  validateConfig(c);
  EXPECT_NEAR(kPi / 60.0, c.manual_drive_max_wheel_speed, 1e-14);
  EXPECT_EQ(600, manualSpeedLimitRpm(c.manual_drive_max_wheel_speed, c));
}

TEST(ConfigCore, SignsAndNonnegativeThresholdsFollowPythonNormalization) {
  Config c;
  c.drive_left_sign = -9;
  c.drive_right_sign = 0;
  c.cmd_vel_drive_left_sign = 100;
  c.cmd_vel_drive_right_sign = -100;
  c.actual_speed_left_sign = 0;
  c.actual_speed_right_sign = -2;
  c.manual_wheel_hyst_enter_rpm = 10;
  c.manual_wheel_hyst_exit_rpm = 50;
  c.manual_wheel_deadband_rpm = -1;
  c.cmd_vel_filter_sign_confirm_count = 0;
  c.cmd_vel_deadband_linear = -0.1;
  validateConfig(c);
  EXPECT_EQ(-1, c.drive_left_sign);
  EXPECT_EQ(1, c.drive_right_sign);
  EXPECT_EQ(1, c.cmd_vel_drive_left_sign);
  EXPECT_EQ(-1, c.cmd_vel_drive_right_sign);
  EXPECT_EQ(1, c.actual_speed_left_sign);
  EXPECT_EQ(-1, c.actual_speed_right_sign);
  EXPECT_EQ(50, c.manual_wheel_hyst_enter_rpm);
  EXPECT_EQ(0, c.manual_wheel_deadband_rpm);
  EXPECT_EQ(1, c.cmd_vel_filter_sign_confirm_count);
  EXPECT_DOUBLE_EQ(0.0, c.cmd_vel_deadband_linear);
}

TEST(ConfigCore, RepeatedValidationDoesNotReconvertLegacyLimitAboveTenMetersPerSecond) {
  Config c;
  c.cmd_vel_wheel_radius_m = 1.0;
  c.cmd_vel_gear_ratio = 1.0;
  c.manual_drive_max_wheel_speed = 600.0;
  validateConfig(c);
  ASSERT_GT(c.manual_drive_max_wheel_speed, 10.0);
  EXPECT_NEAR(20.0 * kPi, c.manual_drive_max_wheel_speed, 1e-12);
  const double converted = c.manual_drive_max_wheel_speed;
  validateConfig(c);
  EXPECT_DOUBLE_EQ(converted, c.manual_drive_max_wheel_speed);
  EXPECT_TRUE(c.manual_limit_normalized);
  c.cmd_vel_wheel_radius_m = 0.0;
  EXPECT_THROW(validateConfig(c), std::invalid_argument);  // Normalization never bypasses validation.
}

TEST(ConfigCore, WheelPollingRetainsPythonMinimumOneHertzWithoutClampingStatusRate) {
  Config c;
  c.poll_rate_hz = 0.05;
  c.status_poll_rate_hz = 0.05;
  c.diagnostics_rate_hz = 0.02;
  validateConfig(c);
  EXPECT_DOUBLE_EQ(1.0, c.poll_rate_hz);
  EXPECT_DOUBLE_EQ(0.05, c.status_poll_rate_hz);
  EXPECT_DOUBLE_EQ(0.02, c.diagnostics_rate_hz);
}

TEST(ConfigCore, InvalidGeometryTimingCovarianceAndNonfiniteValuesAreRejected) {
  const std::vector<double Config::*> positive_fields = {
      &Config::timeout, &Config::transaction_timeout, &Config::command_watchdog_period,
      &Config::poll_rate_hz, &Config::status_poll_rate_hz, &Config::diagnostics_rate_hz,
      &Config::cmd_vel_wheel_radius_m, &Config::cmd_vel_wheel_track_m, &Config::cmd_vel_gear_ratio,
      &Config::cmd_vel_scale, &Config::actual_motor_speed_scale,
      &Config::wheel_twist_linear_variance, &Config::wheel_twist_angular_variance};
  for (auto field : positive_fields) {
    Config zero;
    zero.*field = 0.0;
    EXPECT_THROW(validateConfig(zero), std::invalid_argument);
  }
  const std::vector<double Config::*> finite_fields = {
      &Config::command_timeout, &Config::wheel_feedback_delay_sec, &Config::wheel_twist_cross_covariance,
      &Config::manual_drive_max_wheel_speed, &Config::cmd_vel_max_input_v, &Config::cmd_vel_max_input_w,
      &Config::cmd_vel_max_abs_wheel_rpm, &Config::cmd_vel_max_linear, &Config::cmd_vel_max_angular,
      &Config::cmd_vel_deadband_linear, &Config::cmd_vel_deadband_angular, &Config::cmd_source_hold_sec,
      &Config::cmd_vel_filter_linear_accel, &Config::cmd_vel_filter_angular_accel,
      &Config::cmd_vel_filter_angular_deadband, &Config::safe_stop_on_error_interval};
  for (auto field : finite_fields) {
    Config c;
    c.*field = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(validateConfig(c), std::invalid_argument);
    c.*field = std::numeric_limits<double>::infinity();
    EXPECT_THROW(validateConfig(c), std::invalid_argument);
  }
  Config c;
  c.wheel_twist_cross_covariance = 0.04;
  EXPECT_THROW(validateConfig(c), std::invalid_argument);
  c.wheel_twist_cross_covariance = -0.003;
  EXPECT_NO_THROW(validateConfig(c));
  c.wheel_feedback_delay_sec = -0.001;
  EXPECT_THROW(validateConfig(c), std::invalid_argument);
  c = Config{};
  c.left_speed_min = 1;
  EXPECT_THROW(validateConfig(c), std::invalid_argument);
}

TEST(DirectionCore, CommandsSwapBeforeSignAndClampEachPhysicalMotor) {
  Config c;
  c.drive_swap_lr = true;
  c.drive_left_sign = -1;
  c.drive_right_sign = 1;
  c.left_speed_min = -200;
  c.right_speed_max = 50;
  EXPECT_EQ(std::make_pair(-200, 50), driveDirection(100, 300, c, false));
  EXPECT_EQ(std::make_pair(100, 50), driveDirection(100, 300, c, true));
  c = Config{};
  EXPECT_EQ(std::make_pair(32767, 0), driveDirection(-32768, 0, c, false));
}

TEST(DirectionCore, FeedbackSignsAreAppliedBeforeSwapping) {
  Config c;
  c.cmd_vel_wheel_radius_m = 0.1;
  c.cmd_vel_wheel_track_m = 0.5;
  c.actual_motor_speed_scale = 2.0;
  c.actual_speed_left_sign = -1;
  c.actual_speed_right_sign = 1;
  c.actual_speed_swap_lr = true;
  const auto twist = motorRpmToBaseTwist(120, 60, c);
  EXPECT_NEAR(-0.01047197551196598, twist.first, 1e-14);
  EXPECT_NEAR(-0.1256637061435917, twist.second, 1e-14);
  c.cmd_vel_wheel_track_m = 0.0;
  EXPECT_THROW(motorRpmToBaseTwist(0, 0, c), std::invalid_argument);
}

TEST(ManualCore, DeadbandAndHysteresisBoundaryBehaviorIsPreserved) {
  Config c;
  EXPECT_EQ(0, filterManualInput(30, 100, c));
  EXPECT_EQ(0, filterManualInput(59, 0, c));
  EXPECT_EQ(60, filterManualInput(60, 0, c));
  EXPECT_EQ(-60, filterManualInput(-60, 0, c));
  EXPECT_EQ(31, filterManualInput(31, 100, c));
  EXPECT_EQ(-31, filterManualInput(-31, 100, c));
  EXPECT_EQ(0, filterManualInput(0, 100, c));
  EXPECT_EQ(0, manualSpeedLimitRpm(0.0, c));
  EXPECT_EQ(1, manualSpeedLimitRpm(1e-20, c));
  EXPECT_EQ(777, manualSpeedLimitRpm(0.2, c));
  EXPECT_EQ(3000, manualSpeedLimitRpm(10.0, c));
  EXPECT_THROW(manualSpeedLimitRpm(std::numeric_limits<double>::quiet_NaN(), c), std::invalid_argument);
}

TEST(VelocityCore, FilterUsesMinimumAndMaximumDtAndRetainsZeroInputDeceleration) {
  Config c;
  VelocityFilter filter;
  filter.reset(0.0);
  auto value = filter.apply(1.0, 0.1, c, 0.0);
  EXPECT_DOUBLE_EQ(0.04, value.first);
  EXPECT_DOUBLE_EQ(0.0, value.second);  // 0.04 is below the configured angular deadband.
  value = filter.apply(1.0, 0.1, c, 10.0);
  EXPECT_NEAR(0.44, value.first, 1e-15);
  EXPECT_DOUBLE_EQ(0.1, value.second);
  value = filter.apply(0.0, 0.0, c, 10.02);
  EXPECT_NEAR(0.4, value.first, 1e-14);
  EXPECT_NEAR(0.06, value.second, 1e-14);
  filter.reset(10.02);
  EXPECT_DOUBLE_EQ(0.0, filter.apply(0.0, 0.0, c, 10.04).first);
}

TEST(VelocityCore, OppositeAngularSignRequiresConsecutiveConfirmation) {
  Config c;
  c.cmd_vel_filter_angular_deadband = 0.0;
  c.cmd_vel_filter_angular_accel = 1.0;
  VelocityFilter filter;
  filter.reset();
  EXPECT_NEAR(0.2, filter.apply(0.0, 0.2, c, 0.2).second, 1e-14);
  EXPECT_NEAR(0.18, filter.apply(0.0, -0.2, c, 0.22).second, 1e-14);
  EXPECT_NEAR(-0.02, filter.apply(0.0, -0.2, c, 0.42).second, 1e-14);
}

TEST(VelocityCore, NavigationMatchesGoldenRpmAndUsesPythonRounding) {
  Config c;
  c.cmd_vel_filter_enabled = false;
  VelocityFilter filter;
  EXPECT_EQ(std::make_pair(777, 777), navigationWheels(0.2, 0.0, c, filter, 1.0));
  EXPECT_EQ(std::make_pair(-204, 204), navigationWheels(0.0, 0.15, c, filter, 2.0));
  EXPECT_EQ(std::make_pair(573, 981), navigationWheels(0.2, 0.15, c, filter, 3.0));
  c.cmd_vel_wheel_radius_m = 60.0 / (2.0 * kPi);
  c.cmd_vel_gear_ratio = 1.0;
  c.cmd_vel_max_linear = c.cmd_vel_max_input_v = 10.0;
  EXPECT_EQ(std::make_pair(2, 2), navigationWheels(1.5, 0.0, c, filter, 4.0));
  EXPECT_EQ(std::make_pair(-2, -2), navigationWheels(-2.5, 0.0, c, filter, 5.0));
  EXPECT_THROW(navigationWheels(std::numeric_limits<double>::infinity(), 0.0, c, filter, 6.0), std::invalid_argument);
  EXPECT_THROW(navigationWheels(0.0, std::numeric_limits<double>::quiet_NaN(), c, filter, 6.0), std::invalid_argument);
  EXPECT_EQ(std::make_pair(0, 0), navigationWheels(0.0, 0.0, c, filter, 6.0));
}
}  // namespace
}  // namespace grinder_chassis_driver
