#pragma once
#include "grinder_chassis_driver/types.h"
#include <memory>
#include <stdexcept>

namespace grinder_chassis_driver {
class TransportError : public std::runtime_error {
 public:
  explicit TransportError(const std::string& error) : std::runtime_error(error) {}
};
class ModbusException : public TransportError {
 public:
  ModbusException(int function, int code);
  int function, code;
};
class Transport {
 public:
  virtual ~Transport() = default;
  virtual void reconnect() = 0;
  virtual void close() = 0;
  virtual std::vector<uint16_t> read(uint16_t address, unsigned count, Timing* timing = nullptr) = 0;
  virtual void writeBlock(uint16_t address, const std::vector<uint16_t>& values) = 0;
  virtual void writeSingle(uint16_t address, uint16_t value) = 0;
  virtual Timing lastWriteTiming() const { return {}; }
  virtual uint64_t droppedLogs() const { return 0; }
};
uint16_t crc16(const std::vector<uint8_t>& bytes);
std::unique_ptr<Transport> makeModbusTransport(const Config& config);
}  // namespace grinder_chassis_driver
