#include "grinder_chassis_driver/modbus_transport.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <fcntl.h>
#include <iomanip>
#include <limits>
#include <mutex>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <sstream>
#include <sys/stat.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace grinder_chassis_driver {
namespace {

std::string errorText(const std::string& operation) {
  return operation + ": " + std::strerror(errno);
}

// Only this worker touches the log file. A full queue (or a busy queue lock)
// drops a record rather than making the serial owner wait for disk I/O.
class RawLogger {
 public:
  explicit RawLogger(const Config& config)
      : enabled_(config.rs485_raw_log_enabled && !config.rs485_raw_log_file.empty()),
        path_(config.rs485_raw_log_file),
        capacity_(static_cast<size_t>(std::max(0, config.rs485_log_queue_capacity))) {
    if (enabled_) worker_ = std::thread(&RawLogger::run, this);
  }

  ~RawLogger() {
    {
      std::lock_guard<std::mutex> guard(mutex_);
      stopping_ = true;
    }
    wake_.notify_one();
    if (worker_.joinable()) worker_.join();
  }

  void frame(const char* direction, const std::vector<uint8_t>& bytes) noexcept {
    if (!enabled_) return;
    try {
      std::unique_lock<std::mutex> guard(mutex_, std::try_to_lock);
      if (!guard.owns_lock() || queue_.size() >= capacity_) {
        ++dropped_;
        return;
      }
      queue_.push_back({direction, bytes, std::chrono::system_clock::now()});
      guard.unlock();
      wake_.notify_one();
    } catch (...) {
      ++dropped_;
    }
  }

  uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

 private:
  struct Entry {
    const char* direction;
    std::vector<uint8_t> bytes;
    std::chrono::system_clock::time_point timestamp;
  };

  static void ensureDirectory(const std::string& path) {
    const auto slash = path.find_last_of('/');
    if (slash == std::string::npos) return;
    const std::string parent = path.substr(0, slash);
    for (size_t i = 1; i <= parent.size(); ++i) {
      if (i != parent.size() && parent[i] != '/') continue;
      const auto part = parent.substr(0, i);
      if (!part.empty() && ::mkdir(part.c_str(), 0755) != 0 && errno != EEXIST)
        throw TransportError(errorText("create raw log directory"));
    }
  }

  static std::string format(const Entry& entry) {
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        entry.timestamp.time_since_epoch()).count();
    const auto seconds = static_cast<time_t>(milliseconds / 1000);
    struct tm local {};
    ::localtime_r(&seconds, &local);
    char timestamp[32];
    ::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &local);
    std::ostringstream output;
    output << '[' << timestamp << '.' << std::setfill('0') << std::setw(3)
           << (milliseconds % 1000) << "] " << entry.direction;
    for (const auto byte : entry.bytes)
      output << ' ' << std::uppercase << std::hex << std::setw(2) << static_cast<unsigned>(byte);
    output << '\n';
    return output.str();
  }

  void run() noexcept {
    // A disappearing FIFO reader must not terminate the robot process. Block
    // SIGPIPE only in the logger thread; failed writes are counted below.
    sigset_t blocked;
    ::sigemptyset(&blocked);
    ::sigaddset(&blocked, SIGPIPE);
    ::pthread_sigmask(SIG_BLOCK, &blocked, nullptr);
    int descriptor = -1;
    try {
      // Match the Python expanduser convention without running a shell.
      if (path_.compare(0, 2, "~/") == 0) {
        const char* home = ::getenv("HOME");
        if (home) path_ = std::string(home) + path_.substr(1);
      }
      ensureDirectory(path_);
      // Nonblocking also makes named-pipe/error-path tests safe to shut down.
      descriptor = ::open(path_.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_NONBLOCK | O_CLOEXEC, 0644);
    } catch (...) {
      descriptor = -1;
    }
    for (;;) {
      Entry entry;
      {
        std::unique_lock<std::mutex> guard(mutex_);
        wake_.wait(guard, [this] { return stopping_ || !queue_.empty(); });
        if (queue_.empty() && stopping_) break;
        entry = std::move(queue_.front());
        queue_.pop_front();
      }
      try {
        const std::string line = format(entry);
        size_t offset = 0;
        while (descriptor >= 0 && offset < line.size()) {
          const ssize_t written = ::write(descriptor, line.data() + offset, line.size() - offset);
          if (written > 0) offset += static_cast<size_t>(written);
          else if (written < 0 && errno == EINTR) continue;
          else break;
        }
        if (offset != line.size()) ++dropped_;
      } catch (...) {
        ++dropped_;
      }
    }
    if (descriptor >= 0) ::close(descriptor);
  }

  bool enabled_;
  std::string path_;
  size_t capacity_;
  std::atomic<uint64_t> dropped_{0};
  std::mutex mutex_;
  std::condition_variable wake_;
  std::deque<Entry> queue_;
  bool stopping_ = false;
  std::thread worker_;
};

speed_t baudConstant(int baudrate) {
  switch (baudrate) {
    case 1200: return B1200;
    case 1800: return B1800;
    case 2400: return B2400;
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: throw TransportError("Unsupported baudrate: " + std::to_string(baudrate));
  }
}

void append16(std::vector<uint8_t>& bytes, uint16_t value) {
  bytes.push_back(static_cast<uint8_t>(value >> 8));
  bytes.push_back(static_cast<uint8_t>(value & 0xff));
}

uint16_t read16(const std::vector<uint8_t>& bytes, size_t index) {
  return static_cast<uint16_t>((static_cast<uint16_t>(bytes.at(index)) << 8) | bytes.at(index + 1));
}

class ModbusTransport final : public Transport {
 public:
  explicit ModbusTransport(const Config& config) : config_(config), logger_(config) {
    if (!(std::isfinite(config_.timeout) && config_.timeout > 0.0 &&
          std::isfinite(config_.transaction_timeout) && config_.transaction_timeout > 0.0))
      throw TransportError("Serial and transaction timeouts must be finite and positive");
    if (config_.slave_id < 1 || config_.slave_id > 247)
      throw TransportError("Modbus slave_id must be between 1 and 247");
    baudConstant(config_.baudrate);
    // No serial device is opened until the owning worker calls an operation.
  }

  ~ModbusTransport() override { close(); }

  void close() override {
    if (fd_ >= 0) {
      // Do not carry buffered fragments of a failed motion frame into the
      // reconnect barrier. Bytes already on the physical wire cannot be undone.
      ::tcflush(fd_, TCIOFLUSH);
      ::close(fd_);
    }
    fd_ = -1;
    last_activity_ = 0.0;
  }

  void reconnect() override {
    close();
    try {
      openAndSynchronize(monotonicSeconds() + config_.transaction_timeout);
    } catch (...) {
      close();
      throw;
    }
  }

  std::vector<uint16_t> read(uint16_t address, unsigned count, Timing* timing) override {
    if (timing) *timing = Timing{};
    validateRange(address, count, 125);
    const double deadline = monotonicSeconds() + config_.transaction_timeout;
    try {
      return readInternal(address, count, deadline, timing);
    } catch (...) {
      close();
      throw;
    }
  }

  void writeBlock(uint16_t address, const std::vector<uint16_t>& values) override {
    last_write_timing_ = Timing{};
    validateRange(address, static_cast<unsigned>(values.size()), 123);
    const double deadline = monotonicSeconds() + config_.transaction_timeout;
    try {
      writeInternal(address, values, true, deadline);
      last_write_timing_.rx = monotonicSeconds();
    } catch (...) {
      close();
      throw;
    }
  }

  void writeSingle(uint16_t address, uint16_t value) override {
    last_write_timing_ = Timing{};
    const double deadline = monotonicSeconds() + config_.transaction_timeout;
    try {
      writeInternal(address, {value}, config_.use_functioncode_16, deadline);
      if (config_.write_verify) {
        const auto values = readInternal(address, 1, deadline, nullptr);
        if (values[0] != value) throw TransportError("Modbus write verification mismatch");
      }
      last_write_timing_.rx = monotonicSeconds();
    } catch (...) {
      close();
      throw;
    }
  }

  uint64_t droppedLogs() const override { return logger_.dropped(); }
  Timing lastWriteTiming() const override { return last_write_timing_; }

 private:
  static void validateRange(uint16_t address, unsigned count, unsigned maximum) {
    if (count == 0 || count > maximum || static_cast<unsigned>(address) + count > 0x10000U)
      throw TransportError("Invalid Modbus register range");
  }

  void checkDeadline(double deadline) const {
    if (monotonicSeconds() >= deadline) throw TransportError("Modbus transaction deadline exceeded");
  }

  void waitFor(short events, double deadline) {
    for (;;) {
      const double remaining = deadline - monotonicSeconds();
      if (remaining <= 0.0) throw TransportError("Modbus transaction deadline exceeded");
      const double budget = std::min(remaining, config_.timeout);
      const int milliseconds = static_cast<int>(std::min<double>(
          std::numeric_limits<int>::max(), std::ceil(budget * 1000.0)));
      struct pollfd descriptor {fd_, events, 0};
      const int result = ::poll(&descriptor, 1, std::max(1, milliseconds));
      if (result < 0 && errno == EINTR) continue;
      if (result < 0) throw TransportError(errorText("Serial poll failed"));
      if (result == 0) {
        checkDeadline(deadline);
        throw TransportError("Serial I/O wait timeout");
      }
      if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))
        throw TransportError("Serial port disconnected or poll failed");
      if (descriptor.revents & events) {
        checkDeadline(deadline);
        return;
      }
    }
  }

  void openAndSynchronize(double deadline) {
    checkDeadline(deadline);
    if (fd_ < 0) {
      fd_ = ::open(config_.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
      if (fd_ < 0) throw TransportError(errorText("Unable to open serial port " + config_.port));
      struct termios attributes {};
      if (::tcgetattr(fd_, &attributes) != 0) throw TransportError(errorText("Serial tcgetattr failed"));
      ::cfmakeraw(&attributes);
      attributes.c_cflag = CLOCAL | CREAD | CS8;  // 8N1; no hardware flow control.
      attributes.c_cc[VMIN] = 0;
      attributes.c_cc[VTIME] = 0;
      const auto speed = baudConstant(config_.baudrate);
      if (::cfsetispeed(&attributes, speed) != 0 || ::cfsetospeed(&attributes, speed) != 0 ||
          ::tcsetattr(fd_, TCSANOW, &attributes) != 0)
        throw TransportError(errorText("Serial configuration failed"));
      if (::tcflush(fd_, TCIOFLUSH) != 0) throw TransportError(errorText("Serial buffer flush failed"));
      last_activity_ = monotonicSeconds();
    }
    waitForSilence(deadline);
  }

  void waitForSilence(double deadline) {
    const double silence = 3.5 * 10.0 / config_.baudrate;
    std::array<uint8_t, 256> discarded;
    for (;;) {
      checkDeadline(deadline);
      // Drain buffered/late replies before accepting a fresh transaction.
      const ssize_t size = ::read(fd_, discarded.data(), discarded.size());
      if (size > 0) {
        last_activity_ = monotonicSeconds();
        continue;
      }
      if (size < 0 && errno == EINTR) continue;
      if (size < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        throw TransportError(errorText("Serial drain failed"));
      const double now = monotonicSeconds();
      if (now - last_activity_ >= silence) return;
      const double wait = std::min({silence - (now - last_activity_), deadline - now, config_.timeout});
      struct pollfd descriptor {fd_, POLLIN, 0};
      const int result = ::poll(&descriptor, 1, std::max(1, static_cast<int>(std::ceil(wait * 1000.0))));
      if (result < 0 && errno == EINTR) continue;
      if (result < 0) throw TransportError(errorText("Serial silence poll failed"));
      if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))
        throw TransportError("Serial disconnected while synchronizing");
    }
  }

  std::vector<uint8_t> request(uint8_t function, uint16_t address) const {
    std::vector<uint8_t> frame{static_cast<uint8_t>(config_.slave_id), function};
    append16(frame, address);
    return frame;
  }

  void send(std::vector<uint8_t>& frame, double deadline, Timing* write_timing = nullptr) {
    const uint16_t checksum = crc16(frame);
    frame.push_back(static_cast<uint8_t>(checksum & 0xff));
    frame.push_back(static_cast<uint8_t>(checksum >> 8));
    size_t offset = 0;
    while (offset < frame.size()) {
      waitFor(POLLOUT, deadline);
      if (write_timing && write_timing->tx == 0.0) write_timing->tx = monotonicSeconds();
      const ssize_t written = ::write(fd_, frame.data() + offset, frame.size() - offset);
      if (written < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
      if (written <= 0) throw TransportError(errorText("Serial write failed"));
      offset += static_cast<size_t>(written);
      last_activity_ = monotonicSeconds();
    }
    // No tcdrain: its wait has no deadline. Waiting for the ACK covers TX time.
    logger_.frame("TX", frame);
  }

  void receive(std::vector<uint8_t>& bytes, size_t size, double deadline) {
    const size_t target = bytes.size() + size;
    while (bytes.size() < target) {
      waitFor(POLLIN, deadline);
      uint8_t buffer[256];
      const size_t wanted = std::min(sizeof(buffer), target - bytes.size());
      const ssize_t received = ::read(fd_, buffer, wanted);
      if (received < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
      if (received <= 0) throw TransportError("Serial returned no data");
      bytes.insert(bytes.end(), buffer, buffer + received);
      last_activity_ = monotonicSeconds();
    }
    checkDeadline(deadline);
  }

  void validateResponse(const std::vector<uint8_t>& frame, uint8_t function) const {
    if (frame.size() < 5) throw TransportError("Modbus response too short");
    if (frame[0] != config_.slave_id) throw TransportError("Unexpected Modbus slave id");
    const uint16_t received = static_cast<uint16_t>(frame[frame.size() - 2] |
        (static_cast<uint16_t>(frame.back()) << 8));
    const std::vector<uint8_t> payload(frame.begin(), frame.end() - 2);
    if (received != crc16(payload)) throw TransportError("Modbus CRC mismatch");
    if (frame[1] == static_cast<uint8_t>(function | 0x80)) {
      if (frame.size() != 5) throw TransportError("Invalid Modbus exception length");
      throw ModbusException(frame[1], frame[2]);
    }
    if (frame[1] != function) throw TransportError("Unexpected Modbus function code");
  }

  std::vector<uint16_t> readInternal(uint16_t address, unsigned count, double deadline, Timing* timing) {
    openAndSynchronize(deadline);
    auto frame = request(3, address);
    append16(frame, static_cast<uint16_t>(count));
    const double tx = monotonicSeconds();
    send(frame, deadline);
    std::vector<uint8_t> response;
    receive(response, 3, deadline);
    if (response[1] & 0x80) {
      receive(response, 2, deadline);
    } else {
      if (response[2] != count * 2) throw TransportError("Unexpected Modbus read byte count");
      receive(response, count * 2 + 2, deadline);
    }
    const double rx = monotonicSeconds();
    logger_.frame("RX", response);
    validateResponse(response, 3);
    std::vector<uint16_t> values;
    values.reserve(count);
    for (unsigned index = 0; index < count; ++index) values.push_back(read16(response, 3 + index * 2));
    if (timing) *timing = Timing{tx, rx};
    return values;
  }

  void writeInternal(uint16_t address, const std::vector<uint16_t>& values, bool multiple, double deadline) {
    openAndSynchronize(deadline);
    const uint8_t function = multiple ? 16 : 6;
    auto frame = request(function, address);
    if (multiple) {
      append16(frame, static_cast<uint16_t>(values.size()));
      frame.push_back(static_cast<uint8_t>(values.size() * 2));
      for (const auto value : values) append16(frame, value);
    } else {
      append16(frame, values.front());
    }
    send(frame, deadline, &last_write_timing_);
    std::vector<uint8_t> response;
    receive(response, 2, deadline);
    receive(response, (response[1] & 0x80) ? 3 : 6, deadline);
    logger_.frame("RX", response);
    validateResponse(response, function);
    if (read16(response, 2) != address || read16(response, 4) !=
        (multiple ? static_cast<uint16_t>(values.size()) : values.front()))
      throw TransportError("Unexpected Modbus write acknowledgement address or value/count");
  }

  Config config_;
  RawLogger logger_;
  int fd_ = -1;
  double last_activity_ = 0.0;
  Timing last_write_timing_;
};

}  // namespace

ModbusException::ModbusException(int function_value, int code_value)
    : TransportError("Modbus exception function=" + std::to_string(function_value) +
                     " code=" + std::to_string(code_value)),
      function(function_value), code(code_value) {}

uint16_t crc16(const std::vector<uint8_t>& bytes) {
  uint16_t result = 0xffff;
  for (const auto byte : bytes) {
    result ^= byte;
    for (unsigned bit = 0; bit < 8; ++bit)
      result = static_cast<uint16_t>((result >> 1) ^ ((result & 1) ? 0xa001 : 0));
  }
  return result;
}

std::unique_ptr<Transport> makeModbusTransport(const Config& config) {
  return std::unique_ptr<Transport>(new ModbusTransport(config));
}

}  // namespace grinder_chassis_driver
