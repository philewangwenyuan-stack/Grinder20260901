#ifndef REGULATED_PURE_PURSUIT_CONTROLLER__COMMAND_VELOCITY_MEMORY_HPP_
#define REGULATED_PURE_PURSUIT_CONTROLLER__COMMAND_VELOCITY_MEMORY_HPP_

#include <cmath>

namespace regulated_pure_pursuit_controller
{

inline bool isCommandVelocityMemoryFresh(const double age, const double timeout)
{
  return std::isfinite(age) && std::isfinite(timeout) &&
         age >= 0.0 && timeout > 0.0 && age <= timeout;
}

inline double retainCommandVelocityIfFresh(
  const double command_velocity, const double age, const double timeout)
{
  return isCommandVelocityMemoryFresh(age, timeout) ? command_velocity : 0.0;
}

}  // namespace regulated_pure_pursuit_controller

#endif  // REGULATED_PURE_PURSUIT_CONTROLLER__COMMAND_VELOCITY_MEMORY_HPP_
