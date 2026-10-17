#pragma once
#include "grinder_chassis_driver/modbus_transport.h"
#include <memory>

namespace grinder_chassis_driver {
class Controller {
 public:
  explicit Controller(Config config, std::unique_ptr<Transport> transport);
  ~Controller();
  void start();
  void stop();
  bool commandVelocity(double linear, double angular);
  bool commandWheels(int left, int right);
  bool taskEnable(bool enable);
  bool setOutput(uint16_t address, int value);
  void emergencyStop();
  Result enable(bool enable);
  Result resetEmergency();
  Result clearIndicator();
  bool setManualSpeedLimit(double speed);
  Snapshot snapshot() const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace grinder_chassis_driver
