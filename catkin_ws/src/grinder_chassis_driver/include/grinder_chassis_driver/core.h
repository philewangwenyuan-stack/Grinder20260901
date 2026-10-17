#pragma once
#include "grinder_chassis_driver/types.h"
#include <utility>

namespace grinder_chassis_driver {
int clamp(int value, int minimum, int maximum);
int toInt16(uint16_t value);
uint16_t toUint16(int value);
int roundEven(double value);
void validateConfig(Config& config);
std::vector<uint16_t> registerBlock(const Command& command);
RegisterSnapshot decodeStatus(const std::vector<uint16_t>& registers);
std::pair<double, double> motorRpmToBaseTwist(int left, int right, const Config& config);
int manualSpeedLimitRpm(double limit, const Config& config);
int filterManualInput(int value, int previous, const Config& config);
std::pair<int, int> driveDirection(int left, int right, const Config& config, bool navigation);

class VelocityFilter {
 public:
  void reset(double now = 0.0);
  std::pair<double, double> apply(double linear, double angular, const Config& config, double now);
 private:
  double linear_ = 0.0, angular_ = 0.0, last_time_ = 0.0;
  int pending_sign_ = 0, pending_count_ = 0;
};
std::pair<int, int> navigationWheels(double linear, double angular, const Config& config,
                                     VelocityFilter& filter, double now);
}  // namespace grinder_chassis_driver
