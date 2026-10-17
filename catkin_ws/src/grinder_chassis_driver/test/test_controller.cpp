#include "grinder_chassis_driver/chassis_controller.h"
#include "grinder_chassis_driver/core.h"
#include "grinder_chassis_driver/emergency_record.h"
#include <gtest/gtest.h>
#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <fstream>
#include <functional>
#include <future>
#include <limits>
#include <map>
#include <mutex>
#include <thread>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

namespace grinder_chassis_driver {
namespace {
bool waitFor(const std::function<bool()>& predicate, double seconds = 2.0) {
  const double until = monotonicSeconds() + seconds;
  while (monotonicSeconds() < until) {
    if (predicate()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return predicate();
}

struct Operation { bool write; uint16_t address; std::vector<uint16_t> values; };
struct FakeState {
  std::mutex mutex;
  std::condition_variable wake;
  std::map<uint16_t, uint16_t> registers;
  std::vector<Operation> operations;
  std::thread::id owner;
  bool owner_violation = false, block_motion = false, motion_entered = false;
  bool block_read = false, read_entered = false, freeze_actual = false;
  bool reject_combined = false, fail_read = false, fail_reconnect = false;
  bool fail_light = false, drop_safe_write = false;
  double timing_age = 0.0, write_delay = 0.0;
  unsigned combined_rejections = 0;
};

class FakeTransport : public Transport {
 public:
  explicit FakeTransport(std::shared_ptr<FakeState> state) : state_(std::move(state)) {}
  void reconnect() override {
    std::lock_guard<std::mutex> lock(state_->mutex);
    owner();
    if (state_->fail_reconnect) throw TransportError("injected reconnect failure");
  }
  void close() override { std::lock_guard<std::mutex> lock(state_->mutex); owner(); }
  std::vector<uint16_t> read(uint16_t address, unsigned count, Timing* timing) override {
    const double tx = monotonicSeconds();
    std::unique_lock<std::mutex> lock(state_->mutex);
    owner();
    if (state_->block_read) {
      state_->read_entered = true;
      state_->wake.notify_all();
      state_->wake.wait(lock, [&] { return !state_->block_read; });
    }
    if (state_->fail_read) { state_->fail_read = false; throw TransportError("injected read failure"); }
    if (state_->reject_combined && address == REGISTER_DISC_SPEED && count == 7) {
      ++state_->combined_rejections;
      throw ModbusException(3, 2);
    }
    std::vector<uint16_t> result;
    for (unsigned i = 0; i < count; ++i) result.push_back(state_->registers[static_cast<uint16_t>(address + i)]);
    state_->operations.push_back({false, address, result});
    if (timing) *timing = {tx - state_->timing_age, monotonicSeconds()};
    return result;
  }
  void writeBlock(uint16_t address, const std::vector<uint16_t>& values) override { write(address, values); }
  void writeSingle(uint16_t address, uint16_t value) override { write(address, {value}); }
 private:
  void owner() {
    if (state_->owner == std::thread::id()) state_->owner = std::this_thread::get_id();
    if (state_->owner != std::this_thread::get_id()) state_->owner_violation = true;
  }
  void write(uint16_t address, const std::vector<uint16_t>& values) {
    std::unique_lock<std::mutex> lock(state_->mutex);
    owner();
    const double delay = state_->write_delay;
    if (delay > 0.0) {
      lock.unlock();
      std::this_thread::sleep_for(std::chrono::duration<double>(delay));
      lock.lock();
    }
    if (address == REGISTER_LEFT_WHEEL_SPEED && values.size() >= 2 &&
        (values[0] || values[1]) && state_->block_motion) {
      state_->motion_entered = true;
      state_->wake.notify_all();
      state_->wake.wait(lock, [&] { return !state_->block_motion; });
    }
    if (address == REGISTER_LIGHT && state_->fail_light) {
      state_->fail_light = false;
      throw TransportError("injected second safety write failure");
    }
    state_->operations.push_back({true, address, values});
    if (address == REGISTER_LEFT_WHEEL_SPEED && values.size() >= 4 &&
        !values[0] && !values[1] && !values[2] && !values[3] && state_->drop_safe_write) return;
    for (std::size_t i = 0; i < values.size(); ++i) state_->registers[static_cast<uint16_t>(address + i)] = values[i];
    if (!state_->freeze_actual && address == REGISTER_LEFT_WHEEL_SPEED && values.size() >= 2) {
      state_->registers[REGISTER_LEFT_MOTOR_ACTUAL_SPEED] = values[0];
      state_->registers[REGISTER_RIGHT_MOTOR_ACTUAL_SPEED] = values[1];
    }
  }
  std::shared_ptr<FakeState> state_;
};

class ControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    char directory[] = "/tmp/grinder-controller-XXXXXX";
    const char* made = ::mkdtemp(directory);
    ASSERT_NE(nullptr, made);
    path = made;
    config.motion_emergency_file = path + "/motion.json";
    config.indicator_emergency_file = path + "/indicator.json";
    config.config_file = path + "/driver.yaml";
    config.poll_rate_hz = 100.0;
    config.status_poll_rate_hz = 50.0;
    config.command_timeout = 1.0;
    config.cmd_vel_filter_enabled = false;
    config.manual_wheel_deadband_rpm = 0;
    config.manual_wheel_hyst_enter_rpm = 0;
    config.manual_wheel_hyst_exit_rpm = 0;
    config.drive_left_sign = config.drive_right_sign = 1;
    fake = std::make_shared<FakeState>();
  }
  void TearDown() override {
    {
      std::lock_guard<std::mutex> lock(fake->mutex);
      fake->block_motion = fake->block_read = false;
    }
    fake->wake.notify_all();
    controller.reset();
    for (const auto& name : {"motion.json", "indicator.json", "driver.yaml", "motion.json.tmp", "indicator.json.tmp", "driver.yaml.tmp"})
      std::remove((path + "/" + name).c_str());
    std::remove((path + "/bad").c_str());
    ::rmdir(path.c_str());
  }
  void start(bool ready = true) {
    controller.reset(new Controller(config, std::unique_ptr<Transport>(new FakeTransport(fake))));
    controller->start();
    if (ready) { ASSERT_TRUE(waitFor([&] { return controller->snapshot().outputs_ready; })); }
  }
  std::vector<Operation> writes() {
    std::lock_guard<std::mutex> lock(fake->mutex);
    std::vector<Operation> result;
    for (const auto& operation : fake->operations) if (operation.write) result.push_back(operation);
    return result;
  }
  bool hasWheelWrite(int left, int right) {
    for (const auto& operation : writes())
      if (operation.address == REGISTER_LEFT_WHEEL_SPEED && operation.values.size() >= 2 &&
          toInt16(operation.values[0]) == left && toInt16(operation.values[1]) == right) return true;
    return false;
  }
  void unblockMotion() {
    { std::lock_guard<std::mutex> lock(fake->mutex); fake->block_motion = false; }
    fake->wake.notify_all();
  }
  void unblockRead() {
    { std::lock_guard<std::mutex> lock(fake->mutex); fake->block_read = false; }
    fake->wake.notify_all();
  }
  Config config;
  std::string path;
  std::shared_ptr<FakeState> fake;
  std::unique_ptr<Controller> controller;
};

TEST_F(ControllerTest, StartupSafetyPrecedesFeedbackAndUsesOneSerialOwner) {
  start();
  const auto operations = writes();
  ASSERT_FALSE(operations.empty());
  ASSERT_GE(operations.front().values.size(), 4u);
  EXPECT_EQ(REGISTER_LEFT_WHEEL_SPEED, operations.front().address);
  for (unsigned i = 0; i < 4; ++i) EXPECT_EQ(0u, operations.front().values[i]);
  const auto state = controller->snapshot();
  EXPECT_TRUE(state.safe_write_confirmed);
  EXPECT_GE(state.wheel_sequence, 2u);
  EXPECT_GE(state.status_sequence, 2u);
  EXPECT_GT(state.actual_timing.tx, 0.0);
  EXPECT_GE(state.actual_timing.rx, state.actual_timing.tx);
  controller->stop();
  std::lock_guard<std::mutex> lock(fake->mutex);
  EXPECT_FALSE(fake->owner_violation);
  EXPECT_NE(std::this_thread::get_id(), fake->owner);
}

TEST_F(ControllerTest, SlowNormalPollingStillSynchronizesAtLeastTwicePerSecond) {
  config.poll_rate_hz = 0.2;
  config.status_poll_rate_hz = 0.2;
  start(false);
  // Legacy configuration normalization may raise the ordinary rate to 1 Hz;
  // synchronization itself must still use a <=500 ms poll period.
  ASSERT_TRUE(waitFor([&] { return controller->snapshot().outputs_ready; }, 0.85));
  EXPECT_GE(controller->snapshot().wheel_sequence, 2u);
  EXPECT_GE(controller->snapshot().status_sequence, 2u);
}

TEST_F(ControllerTest, MailboxKeepsLatestTargetWhileTransactionIsBusy) {
  start();
  { std::lock_guard<std::mutex> lock(fake->mutex); fake->block_motion = true; }
  ASSERT_TRUE(controller->commandWheels(100, 100));
  ASSERT_TRUE(waitFor([&] { std::lock_guard<std::mutex> lock(fake->mutex); return fake->motion_entered; }));
  for (int speed = 101; speed <= 500; ++speed) EXPECT_TRUE(controller->commandWheels(speed, speed));
  unblockMotion();
  ASSERT_TRUE(waitFor([&] { return hasWheelWrite(500, 500); }));
  unsigned nonzero = 0;
  for (const auto& write : writes()) if (write.address == REGISTER_LEFT_WHEEL_SPEED && write.values[0]) ++nonzero;
  EXPECT_EQ(2u, nonzero);
}

TEST_F(ControllerTest, EmergencyLatchesWithoutWaitingForInflightNonzeroWrite) {
  start();
  { std::lock_guard<std::mutex> lock(fake->mutex); fake->block_motion = true; }
  ASSERT_TRUE(controller->commandWheels(100, 100));
  ASSERT_TRUE(waitFor([&] { std::lock_guard<std::mutex> lock(fake->mutex); return fake->motion_entered; }));
  ASSERT_TRUE(controller->commandWheels(300, 300));
  const auto before = monotonicSeconds();
  controller->emergencyStop();
  EXPECT_LT(monotonicSeconds() - before, 0.1);
  EXPECT_TRUE(controller->snapshot().emergency_latched);
  EXPECT_FALSE(controller->commandWheels(400, 400));
  unblockMotion();
  ASSERT_TRUE(waitFor([&] { return controller->snapshot().safe_write_confirmed; }));
  const auto operations = writes();
  ASSERT_GE(operations.back().values.size(), 4u);
  for (unsigned i = 0; i < 4; ++i) EXPECT_EQ(0u, operations.back().values[i]);
  EXPECT_FALSE(hasWheelWrite(300, 300));
  ASSERT_TRUE(waitFor([&] { return EmergencyRecord(config.motion_emergency_file).value(); }));
}

TEST_F(ControllerTest, ZeroStopSurvivesCoalescingAndBypassesRamp) {
  config.enable_cmd_ramp_limit = true;
  config.max_cmd_step_rpm = 50;
  start();
  ASSERT_TRUE(controller->commandWheels(500, 500));
  ASSERT_TRUE(waitFor([&] { return hasWheelWrite(50, 50); }));
  const auto before = writes().size();
  ASSERT_TRUE(controller->commandWheels(0, 0));
  ASSERT_TRUE(waitFor([&] { return writes().size() > before; }));
  const auto operations = writes();
  EXPECT_EQ(0u, operations.back().values[0]);
  EXPECT_EQ(0u, operations.back().values[1]);
}

TEST_F(ControllerTest, ContinuousZeroCommandsDoNotStarveWheelFeedback) {
  start();
  ASSERT_TRUE(controller->commandWheels(100, 100));
  ASSERT_TRUE(waitFor([&] { return controller->snapshot().actual_left_motor_speed == 100; }));
  { std::lock_guard<std::mutex> lock(fake->mutex); fake->write_delay = 0.003; }
  const auto before = controller->snapshot().wheel_sequence;
  std::thread zeros([&] {
    for (unsigned i = 0; i < 200; ++i) {
      controller->commandWheels(0, 0);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_GE(controller->snapshot().wheel_sequence, before + 4);
  zeros.join();
  EXPECT_EQ(0, controller->snapshot().actual_left_motor_speed);
}

TEST_F(ControllerTest, WatchdogOnlyStopsWheelsAndKeepsDiscPolicy) {
  config.command_timeout = 0.06;
  start();
  ASSERT_TRUE(controller->setOutput(REGISTER_DISC_SPEED, 1000));
  ASSERT_TRUE(controller->setOutput(REGISTER_DISC_ENABLE, 1));
  ASSERT_TRUE(controller->commandWheels(100, 100));
  ASSERT_TRUE(waitFor([&] { return hasWheelWrite(100, 100); }));
  ASSERT_TRUE(waitFor([&] { return controller->snapshot().timeout_count > 0; }));
  ASSERT_TRUE(waitFor([&] {
    std::lock_guard<std::mutex> lock(fake->mutex);
    return fake->registers[REGISTER_LEFT_WHEEL_SPEED] == 0 && fake->registers[REGISTER_DISC_ENABLE] == 1;
  }));
  const auto state = controller->snapshot();
  EXPECT_EQ(0, state.command.left_wheel_speed);
  EXPECT_EQ(1000, state.command.disc_speed);
  EXPECT_EQ(1u, state.command.disc_enable);
  EXPECT_FALSE(state.emergency_latched);
}

TEST_F(ControllerTest, WatchdogStillInvalidatesCommandsWhenSerialReadIsBlocked) {
  config.command_timeout = 0.08;
  start();
  ASSERT_TRUE(controller->commandWheels(100, 100));
  ASSERT_TRUE(waitFor([&] { return hasWheelWrite(100, 100); }));
  { std::lock_guard<std::mutex> lock(fake->mutex); fake->block_read = true; }
  ASSERT_TRUE(waitFor([&] { std::lock_guard<std::mutex> lock(fake->mutex); return fake->read_entered; }));
  const auto generation = controller->snapshot().generation;
  ASSERT_TRUE(waitFor([&] { return controller->snapshot().timeout_count > 0; }, 0.5));
  EXPECT_GT(controller->snapshot().generation, generation);
  EXPECT_EQ(0, controller->snapshot().command.left_wheel_speed);
  unblockRead();
}

TEST_F(ControllerTest, TaskDisableStopsWheelsButPreservesDisc) {
  start();
  ASSERT_TRUE(controller->taskEnable(true));
  ASSERT_TRUE(controller->setOutput(REGISTER_DISC_SPEED, 1200));
  ASSERT_TRUE(controller->setOutput(REGISTER_DISC_ENABLE, 1));
  ASSERT_TRUE(controller->commandVelocity(0.1, 0.0));
  ASSERT_TRUE(waitFor([&] { return controller->snapshot().actual_left_motor_speed != 0; }));
  ASSERT_TRUE(controller->taskEnable(false));
  ASSERT_TRUE(waitFor([&] { return controller->snapshot().actual_left_motor_speed == 0; }));
  const auto state = controller->snapshot();
  EXPECT_EQ(1200, state.command.disc_speed);
  EXPECT_EQ(1u, state.command.disc_enable);
  EXPECT_FALSE(state.task_enable);
  EXPECT_FALSE(controller->commandVelocity(0.1, 0.0));
}

TEST_F(ControllerTest, CommunicationFailureRecoversWithSafetyAndNeverReplaysTarget) {
  start();
  ASSERT_TRUE(controller->commandWheels(100, 100));
  ASSERT_TRUE(waitFor([&] { return hasWheelWrite(100, 100); }));
  const auto before = writes().size();
  { std::lock_guard<std::mutex> lock(fake->mutex); fake->fail_read = true; }
  ASSERT_TRUE(waitFor([&] { return writes().size() > before && controller->snapshot().outputs_ready; }));
  const auto operations = writes();
  ASSERT_GT(operations.size(), before);
  ASSERT_GE(operations[before].values.size(), 4u);
  for (unsigned i = 0; i < 4; ++i) EXPECT_EQ(0u, operations[before].values[i]);
  EXPECT_EQ(0, controller->snapshot().command.left_wheel_speed);
  EXPECT_FALSE(controller->snapshot().task_enable);
}

TEST_F(ControllerTest, SimilarSafetyAckCannotOpenRecoveryGateWithoutStoppedReadback) {
  start();
  ASSERT_TRUE(controller->commandWheels(100, 100));
  ASSERT_TRUE(waitFor([&] { return controller->snapshot().actual_left_motor_speed == 100; }));
  {
    std::lock_guard<std::mutex> lock(fake->mutex);
    fake->drop_safe_write = true;  // Old/similar ACK, without applying the new safe target.
    fake->fail_read = true;
  }
  ASSERT_TRUE(waitFor([&] { return !controller->snapshot().outputs_ready; }));
  ASSERT_TRUE(waitFor([&] { return controller->snapshot().safe_write_confirmed; }));
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  EXPECT_FALSE(controller->snapshot().outputs_ready);
  EXPECT_FALSE(controller->commandWheels(200, 200));
  { std::lock_guard<std::mutex> lock(fake->mutex); fake->drop_safe_write = false; }
  controller->emergencyStop();
  ASSERT_TRUE(waitFor([&] { return controller->snapshot().outputs_ready; }));
  EXPECT_TRUE(controller->snapshot().emergency_latched);
}

TEST_F(ControllerTest, RecoveryRetriesSafetyWhenStoppedFeedbackNeverArrives) {
  {
    std::lock_guard<std::mutex> lock(fake->mutex);
    fake->freeze_actual = true;
    fake->registers[REGISTER_LEFT_MOTOR_ACTUAL_SPEED] = 5;
  }
  start(false);
  ASSERT_TRUE(waitFor([&] { return controller->snapshot().safe_write_confirmed; }));
  const auto before = writes().size();
  ASSERT_TRUE(waitFor([&] { return writes().size() > before; }, 4.0));
  EXPECT_FALSE(controller->snapshot().outputs_ready);
  EXPECT_FALSE(controller->commandWheels(100, 100));
  { std::lock_guard<std::mutex> lock(fake->mutex); fake->registers[REGISTER_LEFT_MOTOR_ACTUAL_SPEED] = 0; }
  ASSERT_TRUE(waitFor([&] { return controller->snapshot().outputs_ready; }));
}

TEST_F(ControllerTest, RecoveryRejectsReadbacksOlderThanOneSecond) {
  {
    std::lock_guard<std::mutex> lock(fake->mutex);
    fake->freeze_actual = true;
    fake->registers[REGISTER_LEFT_MOTOR_ACTUAL_SPEED] = 5;
  }
  start(false);
  ASSERT_TRUE(waitFor([&] { return controller->snapshot().safe_write_confirmed; }));
  std::this_thread::sleep_for(std::chrono::milliseconds(1250));
  const auto before = controller->snapshot().wheel_sequence;
  {
    std::lock_guard<std::mutex> lock(fake->mutex);
    fake->registers[REGISTER_LEFT_MOTOR_ACTUAL_SPEED] = 0;
    fake->timing_age = 1.1;
  }
  ASSERT_TRUE(waitFor([&] { return controller->snapshot().wheel_sequence >= before + 3; }));
  EXPECT_FALSE(controller->snapshot().outputs_ready);
  { std::lock_guard<std::mutex> lock(fake->mutex); fake->timing_age = 0.0; }
  ASSERT_TRUE(waitFor([&] { return controller->snapshot().outputs_ready; }));
}

TEST_F(ControllerTest, CombinedIllegalAddressFallsBackWithoutDeclaringCommunicationFailure) {
  { std::lock_guard<std::mutex> lock(fake->mutex); fake->reject_combined = true; }
  start();
  EXPECT_FALSE(controller->snapshot().status_combined_read);
  EXPECT_EQ(0u, controller->snapshot().consecutive_failures);
  std::lock_guard<std::mutex> lock(fake->mutex);
  EXPECT_EQ(1u, fake->combined_rejections);
}

TEST_F(ControllerTest, LiftlessPartialSafetyFailureDoesNotConfirmAndRepeatsWholeSafety) {
  config.disc_lift_supported = false;
  { std::lock_guard<std::mutex> lock(fake->mutex); fake->fail_light = true; fake->fail_reconnect = false; }
  start();
  unsigned full_starts = 0;
  for (const auto& operation : writes()) {
    if (operation.address == REGISTER_LEFT_WHEEL_SPEED) {
      EXPECT_EQ(5u, operation.values.size());
      ++full_starts;
    }
    for (std::size_t i = 0; i < operation.values.size(); ++i)
      EXPECT_NE(REGISTER_DISC_LIFT, operation.address + i);
  }
  EXPECT_GE(full_starts, 2u);
}

TEST_F(ControllerTest, RepeatedEmergencyAlwaysWritesAndSurvivesRestart) {
  start();
  controller->emergencyStop();
  ASSERT_TRUE(waitFor([&] { return controller->snapshot().safe_write_confirmed; }));
  const auto before = writes().size();
  controller->emergencyStop();
  ASSERT_TRUE(waitFor([&] { return writes().size() > before; }));
  ASSERT_TRUE(waitFor([&] { return EmergencyRecord(config.motion_emergency_file).value(); }));
  controller.reset();
  start();
  EXPECT_TRUE(controller->snapshot().emergency_latched);
  EXPECT_FALSE(controller->enable(true).success);
  EXPECT_FALSE(controller->setOutput(REGISTER_DISC_ENABLE, 1));
}

TEST_F(ControllerTest, CorruptPersistentMotionRecordFailsClosed) {
  { std::ofstream file(config.motion_emergency_file); file << "broken json"; }
  start();
  EXPECT_TRUE(controller->snapshot().emergency_latched);
  EXPECT_FALSE(controller->commandWheels(100, 100));
}

TEST_F(ControllerTest, IndicatorClearDoesNotReleaseMotionLatch) {
  start();
  controller->emergencyStop();
  ASSERT_TRUE(waitFor([&] { return controller->snapshot().safe_write_confirmed; }));
  EXPECT_TRUE(controller->clearIndicator().success);
  EXPECT_FALSE(controller->snapshot().indicator_emergency);
  EXPECT_TRUE(controller->snapshot().emergency_latched);
  EXPECT_TRUE(EmergencyRecord(config.motion_emergency_file).value());
}

TEST_F(ControllerTest, ResetWaitsForFreshFeedbackAndAllowsMotorSpinDown) {
  start();
  {
    std::lock_guard<std::mutex> lock(fake->mutex);
    fake->freeze_actual = true;
    fake->registers[REGISTER_LEFT_MOTOR_ACTUAL_SPEED] = 5;
  }
  controller->emergencyStop();
  auto reset = std::async(std::launch::async, [&] { return controller->resetEmergency(); });
  ASSERT_TRUE(waitFor([&] { return controller->snapshot().actual_left_motor_speed == 5; }));
  EXPECT_EQ(std::future_status::timeout, reset.wait_for(std::chrono::milliseconds(70)));
  { std::lock_guard<std::mutex> lock(fake->mutex); fake->registers[REGISTER_LEFT_MOTOR_ACTUAL_SPEED] = 0; }
  ASSERT_EQ(std::future_status::ready, reset.wait_for(std::chrono::seconds(2)));
  const auto result = reset.get();
  EXPECT_TRUE(result.success) << result.message;
  EXPECT_FALSE(controller->snapshot().emergency_latched);
  EXPECT_FALSE(controller->snapshot().task_enable);
  EXPECT_EQ(0, controller->snapshot().command.left_wheel_speed);
  EXPECT_FALSE(EmergencyRecord(config.motion_emergency_file).value());
}

TEST_F(ControllerTest, NewEmergencyCancelsConcurrentReset) {
  start();
  {
    std::lock_guard<std::mutex> lock(fake->mutex);
    fake->freeze_actual = true;
    fake->registers[REGISTER_LEFT_MOTOR_ACTUAL_SPEED] = 5;
  }
  controller->emergencyStop();
  const auto before = controller->snapshot().generation;
  auto reset = std::async(std::launch::async, [&] { return controller->resetEmergency(); });
  ASSERT_TRUE(waitFor([&] { return controller->snapshot().generation > before; }));
  EXPECT_FALSE(controller->resetEmergency().success);
  controller->emergencyStop();
  ASSERT_EQ(std::future_status::ready, reset.wait_for(std::chrono::seconds(1)));
  EXPECT_FALSE(reset.get().success);
  EXPECT_TRUE(controller->snapshot().emergency_latched);
}

TEST_F(ControllerTest, ResetFailsAfterThreeSecondFeedbackWindowWithoutStoppedReadback) {
  start();
  {
    std::lock_guard<std::mutex> lock(fake->mutex);
    fake->freeze_actual = true;
    fake->registers[REGISTER_LEFT_MOTOR_ACTUAL_SPEED] = 5;
  }
  controller->emergencyStop();
  const auto begin = monotonicSeconds();
  const auto result = controller->resetEmergency();
  const double elapsed = monotonicSeconds() - begin;
  EXPECT_FALSE(result.success);
  EXPECT_GE(elapsed, 2.9);
  EXPECT_LT(elapsed, 4.5);
  EXPECT_TRUE(controller->snapshot().emergency_latched);
}

TEST_F(ControllerTest, FailedPersistentResetKeepsMotionLatched) {
  { std::ofstream file(path + "/bad"); file << "not a directory"; }
  config.motion_emergency_file = path + "/bad/motion.json";
  start();
  EXPECT_TRUE(controller->snapshot().emergency_latched);
  const auto result = controller->resetEmergency();
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(controller->snapshot().emergency_latched);
  EXPECT_FALSE(controller->enable(true).success);
}

TEST_F(ControllerTest, BlockedFileIoDoesNotBlockNewEmergencyOrWatchdog) {
  config.command_timeout = 0.06;
  start();
  const auto fifo = config.motion_emergency_file + ".tmp";
  ASSERT_EQ(0, ::mkfifo(fifo.c_str(), 0600));
  controller->emergencyStop();
  // The record writer now waits for a FIFO reader; no motion mutex is held.
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  const auto begin = monotonicSeconds();
  controller->emergencyStop();
  EXPECT_LT(monotonicSeconds() - begin, 0.1);
  EXPECT_TRUE(controller->snapshot().emergency_latched);
  std::thread reader([&] {
    const int fd = ::open(fifo.c_str(), O_RDONLY);
    if (fd >= 0) {
      char buffer[128];
      while (::read(fd, buffer, sizeof(buffer)) > 0) {}
      ::close(fd);
    }
  });
  reader.join();
  EXPECT_TRUE(controller->snapshot().emergency_latched);
}

TEST_F(ControllerTest, OldDisplayLatchIsImportedOnlyWhenMotionRecordIsAbsent) {
  {
    EmergencyRecord display(config.indicator_emergency_file);
    display.observe();
    std::string error;
    ASSERT_TRUE(display.flush(error)) << error;
  }
  start();
  EXPECT_TRUE(controller->snapshot().emergency_latched);
  ASSERT_TRUE(controller->resetEmergency().success);
  EXPECT_TRUE(controller->snapshot().indicator_emergency);
  controller.reset();
  start();
  EXPECT_FALSE(controller->snapshot().emergency_latched);
  EXPECT_TRUE(controller->snapshot().indicator_emergency);
}

TEST_F(ControllerTest, SourceHoldRejectsManualMotionButAlwaysAcceptsManualZero) {
  start();
  ASSERT_TRUE(controller->taskEnable(true));
  ASSERT_TRUE(controller->commandVelocity(0.1, 0.0));
  EXPECT_FALSE(controller->commandWheels(100, 100));
  EXPECT_TRUE(controller->commandWheels(0, 0));
  EXPECT_EQ(0, controller->snapshot().command.left_wheel_speed);
}

TEST_F(ControllerTest, DuplicateSingleOutputsAreNotWrittenAgain) {
  start();
  ASSERT_TRUE(controller->setOutput(REGISTER_DISC_SPEED, 500));
  ASSERT_TRUE(waitFor([&] {
    std::lock_guard<std::mutex> lock(fake->mutex);
    return fake->registers[REGISTER_DISC_SPEED] == 500;
  }));
  ASSERT_TRUE(controller->setOutput(REGISTER_DISC_SPEED, 500));
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  unsigned count = 0;
  for (const auto& operation : writes()) if (operation.address == REGISTER_DISC_SPEED) ++count;
  EXPECT_EQ(1u, count);
}

TEST_F(ControllerTest, InvalidInputsDoNotRefreshWatchdogAndManualLimitPersists) {
  config.command_timeout = 0.06;
  { std::ofstream file(config.config_file); file << "manual_drive_max_wheel_speed: 0.2\nother: keep\n"; }
  start();
  EXPECT_TRUE(controller->setManualSpeedLimit(0.1));
  EXPECT_FALSE(controller->setManualSpeedLimit(std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(controller->setManualSpeedLimit(-0.1));
  ASSERT_TRUE(controller->commandWheels(100, 100));
  for (unsigned i = 0; i < 20; ++i) {
    EXPECT_FALSE(controller->commandVelocity(std::numeric_limits<double>::quiet_NaN(), 0.0));
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_GT(controller->snapshot().timeout_count, 0u);
  ASSERT_TRUE(waitFor([&] {
    std::ifstream file(config.config_file);
    const std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    return text.find("0.1") != std::string::npos && text.find("other: keep") != std::string::npos;
  }));
}

TEST_F(ControllerTest, ExtremeFiniteConversionIsRejectedWithoutRefreshingCommand) {
  config.cmd_vel_gear_ratio = 1e308;
  config.cmd_vel_max_abs_wheel_rpm = 0.0;
  config.command_timeout = 0.06;
  start();
  ASSERT_TRUE(controller->taskEnable(true));
  ASSERT_TRUE(controller->commandWheels(100, 100));
  const auto target = controller->snapshot().command.left_wheel_speed;
  ASSERT_NE(0, target);
  const auto sequence = controller->snapshot().command_sequence;
  for (unsigned i = 0; i < 20; ++i) {
    EXPECT_FALSE(controller->commandVelocity(0.2, 0.0));
    if (controller->snapshot().timeout_count == 0) {
      EXPECT_EQ(sequence, controller->snapshot().command_sequence);
      EXPECT_EQ(target, controller->snapshot().command.left_wheel_speed);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_GT(controller->snapshot().timeout_count, 0u);
}

TEST_F(ControllerTest, EnablingAfterDisableDiscardsTargetsReceivedWhileDisabled) {
  start();
  ASSERT_TRUE(controller->enable(false).success);
  ASSERT_TRUE(waitFor([&] { return controller->snapshot().outputs_ready; }));
  ASSERT_TRUE(controller->commandWheels(200, 200));
  ASSERT_TRUE(controller->setOutput(REGISTER_DISC_SPEED, 1000));
  ASSERT_TRUE(controller->setOutput(REGISTER_DISC_ENABLE, 1));
  ASSERT_TRUE(controller->taskEnable(true));
  ASSERT_TRUE(controller->enable(true).success);
  const auto state = controller->snapshot();
  EXPECT_EQ(0, state.command.left_wheel_speed);
  EXPECT_EQ(0, state.command.disc_speed);
  EXPECT_EQ(0u, state.command.disc_enable);
  EXPECT_FALSE(state.task_enable);
  EXPECT_FALSE(hasWheelWrite(200, 200));
  ASSERT_TRUE(controller->commandWheels(300, 300));
  ASSERT_TRUE(waitFor([&] { return hasWheelWrite(300, 300); }));
}

TEST_F(ControllerTest, RepeatedEnableTrueDoesNotInterruptActiveTaskOrClearDiscTargets) {
  start();
  ASSERT_TRUE(controller->taskEnable(true));
  ASSERT_TRUE(controller->setOutput(REGISTER_DISC_SPEED, 1000));
  ASSERT_TRUE(controller->setOutput(REGISTER_DISC_ENABLE, 1));
  ASSERT_TRUE(controller->commandVelocity(0.1, 0.0));
  ASSERT_TRUE(waitFor([&] { return controller->snapshot().actual_left_motor_speed != 0; }));
  const auto target = controller->snapshot().command.left_wheel_speed;
  ASSERT_TRUE(controller->enable(true).success);
  const auto state = controller->snapshot();
  EXPECT_EQ(target, state.command.left_wheel_speed);
  EXPECT_EQ(1000, state.command.disc_speed);
  EXPECT_EQ(1u, state.command.disc_enable);
  EXPECT_TRUE(state.task_enable);
}
}  // namespace
}  // namespace grinder_chassis_driver
