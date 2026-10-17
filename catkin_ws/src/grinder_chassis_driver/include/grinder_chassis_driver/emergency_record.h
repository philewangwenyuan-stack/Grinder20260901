#pragma once
#include <cstdint>
#include <memory>
#include <string>

namespace grinder_chassis_driver {
std::string expandUserPath(const std::string& path);
bool persistManualSpeedLimit(const std::string& path, double limit, std::string& error);

class EmergencyRecord {
 public:
  explicit EmergencyRecord(const std::string& path);
  ~EmergencyRecord();
  bool exists() const;
  bool value() const;
  uint64_t revision() const;
  void observe();
  bool flush(std::string& error);
  bool clear(uint64_t expected_revision, std::string& error);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace grinder_chassis_driver
