#include "grinder_chassis_driver/modbus_transport.h"
#include <gtest/gtest.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <fcntl.h>
#include <fstream>
#include <future>
#include <poll.h>
#include <pty.h>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace grinder_chassis_driver {
namespace {

using Bytes = std::vector<uint8_t>;
using namespace std::chrono;

class Pty {
 public:
  Pty() {
    char name[128];
    if (::openpty(&master, &slave, name, nullptr, nullptr) != 0)
      throw std::runtime_error("openpty failed");
    path = name;
    struct termios attributes {};
    if (::tcgetattr(slave, &attributes) != 0) throw std::runtime_error("tcgetattr failed");
    ::cfmakeraw(&attributes);
    if (::tcsetattr(slave, TCSANOW, &attributes) != 0) throw std::runtime_error("tcsetattr failed");
    ::fcntl(master, F_SETFL, ::fcntl(master, F_GETFL) | O_NONBLOCK);
  }

  ~Pty() {
    if (master >= 0) ::close(master);
    if (slave >= 0) ::close(slave);
  }

  Config config() const {
    Config result;
    result.port = path;
    result.rs485_raw_log_enabled = false;
    result.timeout = 0.1;
    result.transaction_timeout = 0.1;
    return result;
  }

  int master = -1, slave = -1;
  std::string path;
};

Bytes exact(int fd, size_t count, double timeout = 0.7) {
  Bytes result;
  const auto deadline = steady_clock::now() + duration_cast<steady_clock::duration>(duration<double>(timeout));
  while (result.size() < count) {
    if (steady_clock::now() >= deadline) throw std::runtime_error("fake slave request timeout");
    struct pollfd descriptor {fd, POLLIN, 0};
    const int ready = ::poll(&descriptor, 1, 20);
    if (ready < 0 && errno == EINTR) continue;
    if (ready < 0) throw std::runtime_error("fake slave poll failed");
    if (ready == 0) continue;
    uint8_t buffer[260];
    const ssize_t received = ::read(fd, buffer, std::min(count - result.size(), sizeof(buffer)));
    if (received < 0 && (errno == EAGAIN || errno == EINTR)) continue;
    if (received <= 0) throw std::runtime_error("fake slave disconnected");
    result.insert(result.end(), buffer, buffer + received);
  }
  return result;
}

Bytes request(int fd) {
  auto result = exact(fd, 6);
  if (result[1] == 16) {
    const auto length = exact(fd, 1);
    result.push_back(length[0]);
    const auto remaining = exact(fd, length[0] + 2);
    result.insert(result.end(), remaining.begin(), remaining.end());
  } else {
    const auto checksum = exact(fd, 2);
    result.insert(result.end(), checksum.begin(), checksum.end());
  }
  const Bytes body(result.begin(), result.end() - 2);
  if (crc16(body) != (result[result.size() - 2] | (static_cast<uint16_t>(result.back()) << 8)))
    throw std::runtime_error("request CRC invalid");
  return result;
}

Bytes checked(Bytes body) {
  const auto crc = crc16(body);
  body.push_back(static_cast<uint8_t>(crc));
  body.push_back(static_cast<uint8_t>(crc >> 8));
  return body;
}

Bytes acknowledgement(const Bytes& query) {
  return checked(Bytes(query.begin(), query.begin() + 6));
}

void reply(int fd, const Bytes& bytes) {
  size_t offset = 0;
  while (offset < bytes.size()) {
    const ssize_t size = ::write(fd, bytes.data() + offset, bytes.size() - offset);
    if (size < 0 && errno == EINTR) continue;
    if (size < 0 && errno == EAGAIN) {
      struct pollfd descriptor {fd, POLLOUT, 0};
      if (::poll(&descriptor, 1, 100) > 0) continue;
    }
    if (size <= 0) throw std::runtime_error("fake slave response write failed");
    offset += static_cast<size_t>(size);
  }
}

template <typename Function>
std::future<void> slave(Pty& pty, Function function) {
  return std::async(std::launch::async, [&pty, function] { function(pty.master); });
}

TEST(ModbusTransport, ConstructionNeverOpensSerialPort) {
  Config config;
  config.port = "/this-device-does-not-exist/grinder";
  config.rs485_raw_log_enabled = false;
  std::unique_ptr<Transport> transport;
  ASSERT_NO_THROW(transport = makeModbusTransport(config));
  EXPECT_THROW(transport->read(0x4017, 2), TransportError);
}

TEST(ModbusTransport, CrcGoldenFrameAndInvalidRanges) {
  EXPECT_EQ(0xcdc5, crc16({1, 3, 0, 0, 0, 10}));
  Config config;
  config.port = "/this-device-does-not-exist/grinder";
  config.rs485_raw_log_enabled = false;
  auto transport = makeModbusTransport(config);
  EXPECT_THROW(transport->read(0x4017, 0), TransportError);
  EXPECT_THROW(transport->read(0x4017, 126), TransportError);
  EXPECT_THROW(transport->read(0xffff, 2), TransportError);
  EXPECT_THROW(transport->writeBlock(0x4010, {}), TransportError);
  EXPECT_THROW(transport->writeBlock(0x4010, std::vector<uint16_t>(124, 0)), TransportError);
}

TEST(ModbusTransport, ReadsFragmentedUnsignedRegistersAndTransactionTiming) {
  Pty pty;
  auto transport = makeModbusTransport(pty.config());
  auto server = slave(pty, [](int fd) {
    const auto query = request(fd);
    EXPECT_EQ((Bytes{1, 3, 0x40, 0x17, 0, 2}), Bytes(query.begin(), query.begin() + 6));
    const auto response = checked({1, 3, 4, 0xff, 0xff, 0x80, 0});
    for (const auto byte : response) {
      reply(fd, {byte});
      std::this_thread::sleep_for(milliseconds(2));
    }
  });
  Timing timing;
  const auto before = monotonicSeconds();
  EXPECT_EQ((std::vector<uint16_t>{0xffff, 0x8000}), transport->read(0x4017, 2, &timing));
  EXPECT_GE(timing.tx, before);
  EXPECT_GT(timing.rx, timing.tx);
  EXPECT_LE(timing.rx, monotonicSeconds());
  EXPECT_NO_THROW(server.get());
}

TEST(ModbusTransport, BlockWriteHasCorrectWireEncodingAndAck) {
  Pty pty;
  auto transport = makeModbusTransport(pty.config());
  auto server = slave(pty, [](int fd) {
    const auto query = request(fd);
    EXPECT_EQ((Bytes{1, 16, 0x40, 0x10, 0, 2, 4, 0xff, 0xff, 0x80, 0}),
              Bytes(query.begin(), query.end() - 2));
    reply(fd, acknowledgement(query));
  });
  EXPECT_NO_THROW(transport->writeBlock(0x4010, {0xffff, 0x8000}));
  EXPECT_NO_THROW(server.get());
}

TEST(ModbusTransport, SingleWriteSupportsBothFunctionsAndOptionalReadback) {
  for (const bool multiple : {false, true}) {
    Pty pty;
    auto config = pty.config();
    config.use_functioncode_16 = multiple;
    config.write_verify = true;
    auto transport = makeModbusTransport(config);
    auto server = slave(pty, [multiple](int fd) {
      const auto query = request(fd);
      EXPECT_EQ(multiple ? 16 : 6, query[1]);
      EXPECT_EQ(0x40, query[2]);
      EXPECT_EQ(0x12, query[3]);
      reply(fd, acknowledgement(query));
      const auto verify = request(fd);
      EXPECT_EQ((Bytes{1, 3, 0x40, 0x12, 0, 1}), Bytes(verify.begin(), verify.begin() + 6));
      reply(fd, checked({1, 3, 2, 0xff, 0xff}));
    });
    EXPECT_NO_THROW(transport->writeSingle(0x4012, 0xffff));
    EXPECT_NO_THROW(server.get());
  }
}

TEST(ModbusTransport, WriteVerificationMismatchFails) {
  Pty pty;
  auto config = pty.config();
  config.write_verify = true;
  auto transport = makeModbusTransport(config);
  auto server = slave(pty, [](int fd) {
    reply(fd, acknowledgement(request(fd)));
    request(fd);
    reply(fd, checked({1, 3, 2, 0, 2}));
  });
  EXPECT_THROW(transport->writeSingle(0x4012, 1), TransportError);
  EXPECT_NO_THROW(server.get());
}

TEST(ModbusTransport, WriteTimingIncludesVerificationAndReadDoesNotOverwriteIt) {
  Pty pty;
  auto config = pty.config();
  config.write_verify = true;
  auto transport = makeModbusTransport(config);
  auto server = slave(pty, [](int fd) {
    reply(fd, acknowledgement(request(fd)));
    request(fd);
    std::this_thread::sleep_for(milliseconds(15));
    reply(fd, checked({1, 3, 2, 0, 7}));
    request(fd);
    reply(fd, checked({1, 3, 2, 0, 7}));
    auto wrong = request(fd);
    wrong.resize(6);
    wrong[3] ^= 1;
    reply(fd, checked(wrong));
  });
  EXPECT_EQ(0, transport->lastWriteTiming().tx);
  const auto before = monotonicSeconds();
  ASSERT_NO_THROW(transport->writeSingle(0x4012, 7));
  const auto good = transport->lastWriteTiming();
  EXPECT_GE(good.tx, before);
  EXPECT_GE(good.rx - good.tx, 0.015);
  EXPECT_LE(good.rx, monotonicSeconds());
  EXPECT_EQ((std::vector<uint16_t>{7}), transport->read(0x4012, 1));
  EXPECT_EQ(good.tx, transport->lastWriteTiming().tx);
  EXPECT_EQ(good.rx, transport->lastWriteTiming().rx);
  EXPECT_THROW(transport->writeBlock(0x4010, {0, 0}), TransportError);
  const auto failed = transport->lastWriteTiming();
  EXPECT_GT(failed.tx, good.rx);
  EXPECT_EQ(0, failed.rx);
  EXPECT_NO_THROW(server.get());
}

TEST(ModbusTransport, VerificationSharesWriteTransactionDeadline) {
  Pty pty;
  auto config = pty.config();
  config.write_verify = true;
  auto transport = makeModbusTransport(config);
  auto server = slave(pty, [](int fd) {
    const auto query = request(fd);
    std::this_thread::sleep_for(milliseconds(60));
    reply(fd, acknowledgement(query));
    request(fd);
    std::this_thread::sleep_for(milliseconds(60));
    reply(fd, checked({1, 3, 2, 0, 1}));
  });
  const auto begin = monotonicSeconds();
  EXPECT_THROW(transport->writeSingle(0x4012, 1), TransportError);
  const auto elapsed = monotonicSeconds() - begin;
  EXPECT_GE(elapsed, 0.09);
  EXPECT_LT(elapsed, 0.16);
  EXPECT_NO_THROW(server.get());
}

TEST(ModbusTransport, ContinuousSlowBytesCannotExtendTotalDeadline) {
  Pty pty;
  auto config = pty.config();
  config.timeout = 0.06;
  auto transport = makeModbusTransport(config);
  auto server = slave(pty, [](int fd) {
    request(fd);
    for (const auto byte : checked({1, 3, 4, 0, 0, 0, 0})) {
      std::this_thread::sleep_for(milliseconds(25));
      reply(fd, {byte});
    }
  });
  const auto begin = monotonicSeconds();
  EXPECT_THROW(transport->read(0x4017, 2), TransportError);
  const auto elapsed = monotonicSeconds() - begin;
  EXPECT_GE(elapsed, 0.09);
  EXPECT_LT(elapsed, 0.16);
  EXPECT_NO_THROW(server.get());
}

TEST(ModbusTransport, NoReplyUsesIoTimeoutWithoutAutomaticRetry) {
  Pty pty;
  auto config = pty.config();
  config.timeout = 0.025;
  config.transaction_timeout = 0.2;
  auto transport = makeModbusTransport(config);
  auto server = slave(pty, [](int fd) {
    request(fd);
    struct pollfd descriptor {fd, POLLIN, 0};
    EXPECT_EQ(0, ::poll(&descriptor, 1, 100));
  });
  const auto begin = monotonicSeconds();
  EXPECT_THROW(transport->read(0x4017, 2), TransportError);
  EXPECT_LT(monotonicSeconds() - begin, 0.15);
  EXPECT_NO_THROW(server.get());
}

TEST(ModbusTransport, BackpressuredSerialOutputHasBoundedWriteWait) {
  Pty pty;
  ::fcntl(pty.slave, F_SETFL, ::fcntl(pty.slave, F_GETFL) | O_NONBLOCK);
  const std::string filler(4096, 'x');
  while (::write(pty.slave, filler.data(), filler.size()) > 0) {}
  auto config = pty.config();
  config.timeout = 0.08;
  config.transaction_timeout = 0.08;
  auto transport = makeModbusTransport(config);
  const auto begin = monotonicSeconds();
  // The fake slave deliberately does not drain its receive side. A blocking
  // tcdrain here would never return; both partial writes and POLLOUT share the
  // same budget instead.
  EXPECT_THROW(transport->writeBlock(0x4010, std::vector<uint16_t>(123, 0)), TransportError);
  EXPECT_LT(monotonicSeconds() - begin, 0.15);
}

TEST(ModbusTransport, RejectsCrcSlaveFunctionAndByteCountAndReopens) {
  for (unsigned error = 0; error < 4; ++error) {
    Pty pty;
    auto transport = makeModbusTransport(pty.config());
    auto server = slave(pty, [error](int fd) {
      request(fd);
      Bytes body{1, 3, 4, 0, 0, 0, 0};
      if (error == 1) body[0] = 2;
      if (error == 2) body[1] = 4;
      if (error == 3) body[2] = 2;
      auto response = checked(body);
      if (error == 0) response.back() ^= 1;
      reply(fd, response);
      request(fd);
      reply(fd, checked({1, 3, 4, 0, 1, 0, 2}));
    });
    Timing timing{10, 20};
    EXPECT_THROW(transport->read(0x4017, 2, &timing), TransportError);
    EXPECT_EQ(0, timing.tx);
    EXPECT_EQ((std::vector<uint16_t>{1, 2}), transport->read(0x4017, 2));
    EXPECT_NO_THROW(server.get());
  }
}

TEST(ModbusTransport, ExceptionResponseRetainsFunctionAndCode) {
  for (const uint8_t code : {2, 3}) {
    Pty pty;
    auto transport = makeModbusTransport(pty.config());
    auto server = slave(pty, [code](int fd) {
      request(fd);
      reply(fd, checked({1, 0x83, code}));
    });
    try {
      transport->read(0x4012, 7);
      FAIL() << "expected ModbusException";
    } catch (const ModbusException& error) {
      EXPECT_EQ(0x83, error.function);
      EXPECT_EQ(code, error.code);
    }
    EXPECT_NO_THROW(server.get());
  }
}

TEST(ModbusTransport, RejectsWrongWriteAcknowledgementAddressCountAndFunction) {
  for (unsigned error = 0; error < 3; ++error) {
    Pty pty;
    auto transport = makeModbusTransport(pty.config());
    auto server = slave(pty, [error](int fd) {
      auto body = request(fd);
      body.resize(6);
      if (error == 0) body[3] ^= 1;
      if (error == 1) body[5] ^= 1;
      if (error == 2) body[1] = 6;
      reply(fd, checked(body));
    });
    EXPECT_THROW(transport->writeBlock(0x4010, {0, 0}), TransportError);
    EXPECT_NO_THROW(server.get());
  }
}

TEST(ModbusTransport, MasterDisconnectFailsRatherThanHanging) {
  Pty pty;
  auto transport = makeModbusTransport(pty.config());
  auto server = slave(pty, [&pty](int fd) {
    request(fd);
    ::close(fd);
    pty.master = -1;
  });
  const auto begin = monotonicSeconds();
  EXPECT_THROW(transport->read(0x4017, 2), TransportError);
  EXPECT_LT(monotonicSeconds() - begin, 0.16);
  EXPECT_NO_THROW(server.get());
}

TEST(ModbusTransport, ReconnectDrainsOldInputAndWaitsForFrameSilence) {
  Pty pty;
  auto transport = makeModbusTransport(pty.config());
  reply(pty.master, checked({1, 3, 4, 0, 90, 0, 91}));
  const auto begin = monotonicSeconds();
  EXPECT_NO_THROW(transport->reconnect());
  EXPECT_GE(monotonicSeconds() - begin, 3.5 * 10.0 / pty.config().baudrate);
  auto server = slave(pty, [](int fd) {
    request(fd);
    reply(fd, checked({1, 3, 4, 0, 1, 0, 2}));
  });
  EXPECT_EQ((std::vector<uint16_t>{1, 2}), transport->read(0x4017, 2));
  EXPECT_NO_THROW(server.get());
}

TEST(ModbusTransport, ContinuousInputPreventsReconnectWithinTotalBudget) {
  Pty pty;
  auto config = pty.config();
  config.baudrate = 1200;  // 29 ms silence, continuously disturbed below.
  auto transport = makeModbusTransport(config);
  auto server = slave(pty, [](int fd) {
    for (unsigned i = 0; i < 25; ++i) {
      reply(fd, {0xaa});
      std::this_thread::sleep_for(milliseconds(5));
    }
  });
  const auto begin = monotonicSeconds();
  EXPECT_THROW(transport->reconnect(), TransportError);
  EXPECT_LT(monotonicSeconds() - begin, 0.16);
  EXPECT_NO_THROW(server.get());
}

TEST(ModbusTransport, RawLogFormatAndFailuresNeverAffectTransactions) {
  char directory[] = "/tmp/grinder-transport-test-XXXXXX";
  ASSERT_NE(nullptr, ::mkdtemp(directory));
  const std::string path = std::string(directory) + "/raw.log";
  Pty pty;
  auto config = pty.config();
  config.rs485_raw_log_enabled = true;
  config.rs485_raw_log_file = path;
  {
    auto transport = makeModbusTransport(config);
    auto server = slave(pty, [](int fd) {
      request(fd);
      reply(fd, checked({1, 3, 4, 0, 1, 0, 2}));
    });
    EXPECT_NO_THROW(transport->read(0x4017, 2));
    EXPECT_NO_THROW(server.get());
  }
  std::ifstream input(path);
  const std::string text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  EXPECT_NE(std::string::npos, text.find("] TX 01 03 40 17 00 02"));
  EXPECT_NE(std::string::npos, text.find("] RX 01 03 04 00 01 00 02"));
  ::unlink(path.c_str());
  ::rmdir(directory);

  config.rs485_raw_log_file = "/dev/full";
  auto transport = makeModbusTransport(config);
  auto server = slave(pty, [](int fd) {
    request(fd);
    reply(fd, checked({1, 3, 4, 0, 0, 0, 0}));
  });
  EXPECT_NO_THROW(transport->read(0x4017, 2));
  EXPECT_NO_THROW(server.get());
  for (unsigned i = 0; i < 50 && transport->droppedLogs() == 0; ++i)
    std::this_thread::sleep_for(milliseconds(1));
  EXPECT_GT(transport->droppedLogs(), 0U);
}

TEST(ModbusTransport, ZeroCapacityQueueDropsInsteadOfBlockingSerialOwner) {
  Pty pty;
  auto config = pty.config();
  config.rs485_raw_log_enabled = true;
  config.rs485_raw_log_file = "/dev/null";
  config.rs485_log_queue_capacity = 0;
  auto transport = makeModbusTransport(config);
  auto server = slave(pty, [](int fd) {
    request(fd);
    reply(fd, checked({1, 3, 4, 0, 0, 0, 0}));
  });
  EXPECT_NO_THROW(transport->read(0x4017, 2));
  EXPECT_NO_THROW(server.get());
  EXPECT_EQ(2U, transport->droppedLogs());
}

TEST(ModbusTransport, FullNonblockingLogPipeCannotHangTransactionsOrDestruction) {
  char directory[] = "/tmp/grinder-log-pipe-test-XXXXXX";
  ASSERT_NE(nullptr, ::mkdtemp(directory));
  const std::string path = std::string(directory) + "/log.pipe";
  ASSERT_EQ(0, ::mkfifo(path.c_str(), 0600));
  const int reader = ::open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
  ASSERT_GE(reader, 0);
  const int filler = ::open(path.c_str(), O_WRONLY | O_NONBLOCK | O_CLOEXEC);
  ASSERT_GE(filler, 0);
  const std::string data(4096, 'x');
  while (::write(filler, data.data(), data.size()) > 0) {}
  Pty pty;
  auto config = pty.config();
  config.rs485_raw_log_enabled = true;
  config.rs485_raw_log_file = path;
  config.rs485_log_queue_capacity = 1;
  const auto begin = monotonicSeconds();
  {
    auto transport = makeModbusTransport(config);
    auto server = slave(pty, [](int fd) {
      request(fd);
      reply(fd, checked({1, 3, 4, 0, 0, 0, 0}));
    });
    EXPECT_NO_THROW(transport->read(0x4017, 2));
    EXPECT_NO_THROW(server.get());
    for (unsigned i = 0; i < 50 && transport->droppedLogs() == 0; ++i)
      std::this_thread::sleep_for(milliseconds(1));
    EXPECT_GT(transport->droppedLogs(), 0U);
  }
  EXPECT_LT(monotonicSeconds() - begin, 0.2);
  ::close(filler);
  ::close(reader);
  ::unlink(path.c_str());
  ::rmdir(directory);
}

}  // namespace
}  // namespace grinder_chassis_driver
