// Grinder low-speed steering compensation, extracted from the controller.
#ifndef REGULATED_PURE_PURSUIT_CONTROLLER__LOW_SPEED_STEERING_HPP_
#define REGULATED_PURE_PURSUIT_CONTROLLER__LOW_SPEED_STEERING_HPP_

#include <cmath>

namespace regulated_pure_pursuit_controller
{
// At 0.10 m/s the current robot tracks correctly. Use that speed only as a
// classifier: if the same path curvature would create a useful angular
// command at 0.10 m/s, the curvature is considered a real correction rather
// than straight-line noise.
constexpr double kLowSpeedSteeringReferenceLinearVel = 0.10;

// Never create a steering command when RPP has intentionally reduced the
// translational command to a near-stop (goal approach, collision, etc.).
constexpr double kLowSpeedSteeringMinimumLinearVel = 0.015;

// Schmitt thresholds evaluated at the known-good 0.10 m/s reference speed.
// A correction must be clearly meaningful before compensation starts, while
// an already active correction may decay farther before it is released. This
// hysteresis reduces command chatter; tracking stability still depends on the
// configured output floor and the measured chassis response.
constexpr double kReferenceAngularEnter = 0.050;
constexpr double kReferenceAngularExit = 0.030;

inline double compensateLowSpeedAngularDeadband(
  double linear_vel, double path_curvature, double raw_angular_vel,
  double previous_angular_command, double angular_deadband,
  double minimum_angular_vel)
{
  // Zero disables compensation. Never lower a command or pass invalid settings.
  if (!std::isfinite(angular_deadband) || !std::isfinite(minimum_angular_vel) ||
    angular_deadband <= 0.0 || minimum_angular_vel < angular_deadband) {
    return raw_angular_vel;
  }
  const double linear_speed = std::abs(linear_vel);
  const double raw_angular_speed = std::abs(raw_angular_vel);

  if (linear_speed < kLowSpeedSteeringMinimumLinearVel ||
    linear_speed >= kLowSpeedSteeringReferenceLinearVel ||
    raw_angular_speed < 1e-9 ||
    raw_angular_speed >= angular_deadband)
  {
    return raw_angular_vel;
  }

  const double reference_linear_vel = std::copysign(
    kLowSpeedSteeringReferenceLinearVel, linear_vel);
  const double reference_angular_vel = reference_linear_vel * path_curvature;
  const double reference_angular_speed = std::abs(reference_angular_vel);
  const bool same_turn_direction =
    raw_angular_vel * previous_angular_command > 0.0;
  const bool previous_effective_correction =
    same_turn_direction &&
    std::abs(previous_angular_command) >= minimum_angular_vel;

  // Leave one raw sub-deadband cycle before reversing a compensated turn.
  // It becomes zero only if the downstream deadband matches this configuration.
  const bool reversing_effective_correction =
    raw_angular_vel * previous_angular_command < 0.0 &&
    std::abs(previous_angular_command) >= minimum_angular_vel;
  if (reversing_effective_correction) {
    return raw_angular_vel;
  }

  const bool enter_correction =
    reference_angular_speed >= kReferenceAngularEnter;
  const bool hold_correction =
    previous_effective_correction &&
    reference_angular_speed >= kReferenceAngularExit;

  if (!enter_correction && !hold_correction) {
    return raw_angular_vel;
  }

  return std::copysign(minimum_angular_vel, raw_angular_vel);
}

}  // namespace regulated_pure_pursuit_controller

#endif
