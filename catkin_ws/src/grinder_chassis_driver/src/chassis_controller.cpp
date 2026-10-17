#include "grinder_chassis_driver/chassis_controller.h"
#include "grinder_chassis_driver/core.h"
#include "grinder_chassis_driver/emergency_record.h"

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <future>
#include <map>
#include <mutex>
#include <thread>

namespace grinder_chassis_driver {
namespace {
using Clock = std::chrono::steady_clock;
Clock::time_point at(double seconds) {
  return Clock::time_point(std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds)));
}
bool stopped(const Snapshot& s) {
  return s.actual_speed_valid && s.actual_left_motor_speed == 0 && s.actual_right_motor_speed == 0 &&
         s.status.disc_speed == 0 && s.status.disc_enable == 0;
}
}  // namespace

struct Controller::Impl {
  struct Pending { uint64_t generation = 0, sequence = 0; int value = 0; double received = 0.0; };
  enum class FileKind { ClearMotion, ClearIndicator };
  struct FileJob {
    FileKind kind = FileKind::ClearIndicator;
    uint64_t generation = 0, revision = 0;
    std::shared_ptr<std::promise<Result>> result;
  };
  enum class Action { None, Reconnect, Safe, WheelStop, All, Motion, Single, Poll };
  struct Work {
    Action action = Action::None;
    uint64_t generation = 0, sequence = 0;
    Command command;
    uint16_t address = 0;
    int value = 0;
    double received = 0.0;
    bool status = false, all = false;
  };

  Config config;
  std::unique_ptr<Transport> transport;
  std::unique_ptr<EmergencyRecord> motion_record, indicator_record;
  mutable std::mutex mutex;
  std::condition_variable wake;
  Snapshot state;
  VelocityFilter filter;
  bool running = false, stopping = false;
  std::thread worker, watchdog, files;
  std::mutex lifecycle_mutex, reset_mutex;
  double last_command = 0.0, last_cmd_vel = 0.0;
  int manual_left = 0, manual_right = 0, sent_left = 0, sent_right = 0;
  bool motion_pending = false, all_pending = false, safe_pending = true, wheel_stop_pending = false;
  bool wheel_stop_inflight = false, wheel_zero_confirmed = false;
  uint64_t zero_confirmed_generation = 0;
  bool wheel_stop_all = false, reconnect_pending = true, recovering = true;
  Pending motion, all;
  std::map<uint16_t, Pending> outputs;
  std::map<uint16_t, uint16_t> written;
  uint64_t next_stop = 0, safe_requested = 0, safe_completed = 0, safe_failed = 0;
  uint64_t wheel_stop_sequence = 0, recovered_generation = 0;
  unsigned recovery_samples = 0;
  double safe_at = 0.0, stop_requested_at = 0.0;
  double next_wheel = 0.0, next_status = 0.0, reconnect_at = 0.0;
  double last_extra_error_stop = 0.0;
  bool previous_write = false;

  std::mutex file_mutex;
  std::condition_variable file_wake;
  std::deque<FileJob> file_jobs;
  bool files_stopping = false, flush_motion = true, flush_indicator = true, manual_pending = false;
  double manual_to_save = 0.0;

  Impl(Config c, std::unique_ptr<Transport> t) : config(std::move(c)), transport(std::move(t)) {
    validateConfig(config);
    if (!transport) throw std::invalid_argument("controller requires a transport");
    motion_record.reset(new EmergencyRecord(config.motion_emergency_file));
    indicator_record.reset(new EmergencyRecord(config.indicator_emergency_file));
    if (!motion_record->exists() && indicator_record->value()) motion_record->observe();
    state.emergency_latched = motion_record->value();
    state.indicator_emergency = indicator_record->value();
    if (state.emergency_latched) {
      indicator_record->observe();
      state.indicator_emergency = true;
    }
    state.status_combined_read = config.status_combined_read;
    state.manual_speed_limit = config.manual_drive_max_wheel_speed;
    state.task_enable = config.task_enable_default;
    last_command = monotonicSeconds();
    filter.reset(last_command);
    fullStopLocked();
  }

  bool acceptsLocked() const {
    return running && !stopping && !state.emergency_latched && state.outputs_ready;
  }

  void resetFilterLocked() {
    filter.reset(monotonicSeconds());
    last_cmd_vel = 0.0;
    manual_left = manual_right = 0;
  }

  // Called with the state mutex only. In particular, no disk or serial I/O here.
  uint64_t fullStopLocked() {
    ++state.generation;
    ++state.command_sequence;
    state.command.left_wheel_speed = state.command.right_wheel_speed = 0;
    state.command.disc_speed = 0;
    state.command.disc_enable = 0;
    state.task_enable = false;
    state.outputs_ready = false;
    state.safe_write_confirmed = false;
    motion_pending = all_pending = wheel_stop_pending = false;
    wheel_stop_inflight = false;
    wheel_zero_confirmed = false;
    outputs.clear();
    resetFilterLocked();
    recovering = true;
    recovery_samples = 0;
    safe_at = 0.0;
    safe_pending = true;
    safe_requested = ++next_stop;
    stop_requested_at = monotonicSeconds();
    return safe_requested;
  }

  void wheelStopLocked(bool all_outputs) {
    ++state.generation;
    ++state.command_sequence;
    motion_pending = all_pending = false;
    wheel_stop_inflight = false;
    wheel_zero_confirmed = false;
    outputs.clear();
    state.command.left_wheel_speed = state.command.right_wheel_speed = 0;
    wheel_stop_pending = true;
    wheel_stop_all = all_outputs;
    wheel_stop_sequence = state.command_sequence;
    stop_requested_at = monotonicSeconds();
  }

  void requestFlush(bool motion_file, bool indicator_file) {
    {
      std::lock_guard<std::mutex> lock(file_mutex);
      flush_motion = flush_motion || motion_file;
      flush_indicator = flush_indicator || indicator_file;
    }
    file_wake.notify_one();
  }

  void start() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex);
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (running) return;
      if (stopping) throw std::logic_error("a stopped controller cannot be restarted");
      running = true;
    }
    files = std::thread(&Impl::fileLoop, this);
    worker = std::thread(&Impl::workerLoop, this);
    watchdog = std::thread(&Impl::watchdogLoop, this);
  }

  void stop() {
    std::lock_guard<std::mutex> lifecycle(lifecycle_mutex);
    {
      std::unique_lock<std::mutex> lock(mutex);
      if (!running) return;
      const auto request = fullStopLocked();
      wake.notify_all();
      // Wait for an attempted final safe write, never for physical spin-down.
      wake.wait_for(lock, std::chrono::duration<double>(0.5 + 3.0 * config.transaction_timeout), [&] {
        return safe_completed >= request || safe_failed >= request;
      });
      stopping = true;
      ++state.generation;
    }
    wake.notify_all();
    if (watchdog.joinable()) watchdog.join();
    if (worker.joinable()) worker.join();
    {
      std::lock_guard<std::mutex> lock(file_mutex);
      files_stopping = true;
      flush_motion = flush_indicator = true;
    }
    file_wake.notify_all();
    if (files.joinable()) files.join();
    std::lock_guard<std::mutex> lock(mutex);
    running = false;
  }

  void watchdogLoop() {
    std::unique_lock<std::mutex> lock(mutex);
    while (!stopping) {
      const double now = monotonicSeconds();
      const double deadline = last_command + config.command_timeout;
      if (config.stop_on_timeout && state.enabled && now > deadline &&
          (state.command.left_wheel_speed != 0 || state.command.right_wheel_speed != 0)) {
        state.watchdog_lateness_ms = std::max(0.0, (now - deadline) * 1000.0);
        ++state.timeout_count;
        wheelStopLocked(true);  // Preserve the current disc target and enable state.
        wake.notify_all();
      }
      wake.wait_until(lock, at(now + config.command_watchdog_period));
    }
  }

  bool velocity(double linear, double angular) {
    if (!std::isfinite(linear) || !std::isfinite(angular)) return false;
    const double begin = monotonicSeconds();
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!config.cmd_vel_enabled || !acceptsLocked()) return false;
      if (!state.task_enable) { filter.reset(begin); return false; }
      // Reject a failed conversion atomically: a valid finite input can still
      // overflow with an extreme (but finite) installation scale or ratio.
      auto next_filter = filter;
      std::pair<int, int> wheels;
      try {
        wheels = navigationWheels(linear, angular, config, next_filter, begin);
      } catch (const std::exception& error) {
        state.last_error = std::string("invalid velocity conversion: ") + error.what();
        return false;
      }
      filter = next_filter;
      state.command.left_wheel_speed = wheels.first;
      state.command.right_wheel_speed = wheels.second;
      last_command = last_cmd_vel = begin;
      motion = {state.generation, ++state.command_sequence, 0, begin};
      motion_pending = state.enabled;
      state.command_compute_ms = (monotonicSeconds() - begin) * 1000.0;
    }
    wake.notify_all();
    return true;
  }

  bool wheels(int left, int right) {
    const double begin = monotonicSeconds();
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!acceptsLocked()) return false;
      const bool zero = left == 0 && right == 0;
      if (config.cmd_vel_enabled && state.task_enable && !zero && last_cmd_vel > 0.0 &&
          begin - last_cmd_vel <= config.cmd_source_hold_sec) return false;
      const int limit = manualSpeedLimitRpm(state.manual_speed_limit, config);
      left = filterManualInput(clamp(left, config.left_speed_min, config.left_speed_max), manual_left, config);
      right = filterManualInput(clamp(right, config.right_speed_min, config.right_speed_max), manual_right, config);
      manual_left = left = clamp(left, -limit, limit);
      manual_right = right = clamp(right, -limit, limit);
      const auto mapped = driveDirection(left, right, config, false);
      if (mapped.first == 0 && mapped.second == 0) {
        const bool already_stopping = state.command.left_wheel_speed == 0 && state.command.right_wheel_speed == 0 &&
            !motion_pending && !all_pending &&
            (wheel_stop_pending || wheel_stop_inflight ||
             (wheel_zero_confirmed && zero_confirmed_generation == state.generation));
        if (!already_stopping) wheelStopLocked(false);
      }
      state.command.left_wheel_speed = mapped.first;
      state.command.right_wheel_speed = mapped.second;
      last_command = begin;
      if (mapped.first != 0 || mapped.second != 0) {
        motion = {state.generation, ++state.command_sequence, 0, begin};
        motion_pending = state.enabled;
      }
      state.command_compute_ms = (monotonicSeconds() - begin) * 1000.0;
    }
    wake.notify_all();
    return true;
  }

  bool taskEnable(bool enable) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!acceptsLocked()) return false;
      const bool was_enabled = state.task_enable;
      state.task_enable = enable;
      if (!enable) {
        const bool moving = state.command.left_wheel_speed || state.command.right_wheel_speed;
        resetFilterLocked();
        last_command = monotonicSeconds();
        if (moving || was_enabled) wheelStopLocked(true);
      }
    }
    wake.notify_all();
    return true;
  }

  bool output(uint16_t address, int value) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      // The legacy light callback may update its target while motion is latched.
      if (!acceptsLocked() && address != REGISTER_LIGHT) return false;
      if (!running || stopping) return false;
      switch (address) {
        case REGISTER_DISC_SPEED:
          value = clamp(value, config.disc_speed_min, config.disc_speed_max);
          state.command.disc_speed = value; break;
        case REGISTER_DISC_ENABLE:
          value = value ? 1 : 0; state.command.disc_enable = static_cast<uint16_t>(value); break;
        case REGISTER_WORK_MODE:
          state.command.work_mode = toUint16(value); value = state.command.work_mode; break;
        case REGISTER_DISC_LIFT:
          if (!config.disc_lift_supported) return false;
          state.command.disc_lift = toUint16(value); value = state.command.disc_lift; break;
        case REGISTER_LIGHT:
          value = value ? 1 : 0; state.command.light = static_cast<uint16_t>(value); break;
        default: return false;
      }
      const Pending pending{state.generation, ++state.command_sequence, value, monotonicSeconds()};
      if (state.enabled && acceptsLocked()) outputs[address] = pending;
    }
    wake.notify_all();
    return true;
  }

  void emergency() {
    {
      std::lock_guard<std::mutex> lock(mutex);
      state.emergency_latched = true;
      state.indicator_emergency = true;
      motion_record->observe();
      indicator_record->observe();
      fullStopLocked();
    }
    wake.notify_all();
    requestFlush(true, true);
  }

  Result enable(bool enable_value) {
    uint64_t generation, request = 0, sequence = 0;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!running || stopping) return {false, "driver is not running"};
      if (!enable_value) {
        state.enabled = false;
        request = fullStopLocked();
      } else {
        if (!acceptsLocked()) return {false, "emergency latched or safe synchronization pending"};
        if (!state.enabled) {
          // Targets received while disabled must not turn into a delayed start
          // when the service releases the output enable. New commands are
          // required; repeated enable(true) keeps an already active task.
          ++state.generation;
          state.command.left_wheel_speed = state.command.right_wheel_speed = 0;
          state.command.disc_speed = 0;
          state.command.disc_enable = 0;
          state.task_enable = false;
          motion_pending = all_pending = wheel_stop_pending = false;
          wheel_stop_inflight = false;
          wheel_zero_confirmed = false;
          outputs.clear();
          resetFilterLocked();
        }
        state.enabled = true;
        last_command = monotonicSeconds();
        sequence = ++state.command_sequence;
        all = {state.generation, sequence, 0, last_command};
        all_pending = true;
      }
      generation = state.generation;
    }
    wake.notify_all();
    std::unique_lock<std::mutex> lock(mutex);
    const bool completed = wake.wait_for(lock, std::chrono::duration<double>(0.5 + 3.0 * config.transaction_timeout), [&] {
      return stopping || state.generation != generation || (request ? safe_completed >= request || safe_failed >= request
                                                                   : completed_all >= sequence);
    });
    const bool success = completed && !stopping && state.generation == generation &&
                         (request ? safe_completed >= request : completed_all >= sequence);
    return {success, success ? (enable_value ? "enabled with safe targets" : "disabled; safe write confirmed=true")
                             : "enable/disable cancelled or safe write not confirmed"};
  }
  uint64_t completed_all = 0;

  Result reset() {
    std::unique_lock<std::mutex> reset_lock(reset_mutex, std::try_to_lock);
    if (!reset_lock.owns_lock()) return {false, "emergency reset already in progress"};
    uint64_t generation, request;
    bool persist = false;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!running || stopping) return {false, "driver is not running"};
      if (!state.emergency_latched) {
        state.emergency_latched = true;
        motion_record->observe();
        persist = true;
      }
      state.reset_error.clear();
      if (!state.connected) {
        state.reset_error = "chassis disconnected; emergency remains latched";
        if (persist) requestFlush(true, false);
        return {false, state.reset_error};
      }
      request = fullStopLocked();
      generation = state.generation;
    }
    if (persist) requestFlush(true, false);
    wake.notify_all();
    std::unique_lock<std::mutex> lock(mutex);
    auto fail = [&](const std::string& text) {
      state.reset_error = text;
      return Result{false, text};
    };
    const bool written_safe = wake.wait_for(lock, std::chrono::duration<double>(0.5 + 3.0 * config.transaction_timeout), [&] {
      return stopping || state.generation != generation || safe_completed >= request || safe_failed >= request;
    });
    if (stopping || state.generation != generation) return fail("new emergency or output failure interrupted reset");
    if (!written_safe || safe_completed < request || !state.safe_write_confirmed)
      return fail("safe output write not confirmed");
    const double deadline = safe_at + 3.0;
    const bool confirmed = wake.wait_until(lock, at(deadline), [&] {
      return stopping || state.generation != generation ||
             (state.connected && state.outputs_ready && recovered_generation == generation &&
              state.safe_write_confirmed && stopped(state) &&
              monotonicSeconds() - std::min(state.last_actual_read, state.last_status_read) < 1.0);
    });
    if (stopping || state.generation != generation) return fail("new emergency or output failure interrupted reset");
    if (!confirmed || !state.outputs_ready)
      return fail("two fresh stopped readbacks not confirmed within 3 seconds");
    auto promise = std::make_shared<std::promise<Result>>();
    auto future = promise->get_future();
    const FileJob job{FileKind::ClearMotion, generation, motion_record->revision(), promise};
    lock.unlock();
    {
      std::lock_guard<std::mutex> file_lock(file_mutex);
      file_jobs.push_back(job);
    }
    file_wake.notify_one();
    while (future.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) {
      std::lock_guard<std::mutex> state_lock(mutex);
      // A successful file commit increments generation before fulfilling its
      // promise. Do not report that successful commit as a cancelled reset.
      if (stopping || (state.generation != generation && state.emergency_latched))
        return {false, "reset superseded"};
    }
    return future.get();
  }

  Result clearIndicator() {
    auto promise = std::make_shared<std::promise<Result>>();
    auto future = promise->get_future();
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!running || stopping) return {false, "driver is not running"};
    }
    {
      std::lock_guard<std::mutex> lock(file_mutex);
      file_jobs.push_back({FileKind::ClearIndicator, 0, indicator_record->revision(), promise});
    }
    file_wake.notify_one();
    while (future.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) {
      std::lock_guard<std::mutex> lock(mutex);
      if (stopping) return {false, "driver is stopping"};
    }
    return future.get();
  }

  bool manualLimit(double value) {
    if (!std::isfinite(value) || value < 0.0) return false;
    {
      std::lock_guard<std::mutex> lock(mutex);
      state.manual_speed_limit = value;
    }
    {
      std::lock_guard<std::mutex> lock(file_mutex);
      manual_to_save = value;
      manual_pending = !config.config_file.empty();
    }
    file_wake.notify_one();
    return true;
  }

  void persistenceFailure(const std::string& error, bool motion_file) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (motion_file) {
        state.reset_error = "cannot persist motion interlock: " + error;
        if (!state.emergency_latched) {
          state.emergency_latched = true;
          fullStopLocked();
        }
      } else state.last_error = error;
    }
    wake.notify_all();
  }

  void fileLoop() {
    for (;;) {
      FileJob job;
      bool has_job = false, save_motion = false, save_indicator = false, save_manual = false, quitting = false;
      double limit = 0.0;
      {
        std::unique_lock<std::mutex> lock(file_mutex);
        const bool signalled = file_wake.wait_for(lock, std::chrono::milliseconds(500), [&] {
          return files_stopping || flush_motion || flush_indicator || !file_jobs.empty() || manual_pending;
        });
        quitting = files_stopping;
        // Retry dirty records after an I/O failure, without a busy retry loop.
        save_motion = flush_motion || !signalled; flush_motion = false;
        save_indicator = flush_indicator || !signalled; flush_indicator = false;
        if (!file_jobs.empty()) { job = file_jobs.front(); file_jobs.pop_front(); has_job = true; }
        if (!has_job && !save_motion && !save_indicator && manual_pending) {
          save_manual = true; manual_pending = false; limit = manual_to_save;
        }
      }
      std::string error;
      if (save_motion && !motion_record->flush(error)) persistenceFailure(error, true);
      if (save_indicator && !indicator_record->flush(error)) persistenceFailure(error, false);
      if (save_manual && !persistManualSpeedLimit(config.config_file, limit, error)) persistenceFailure(error, false);
      if (has_job) {
        Result result;
        if (job.kind == FileKind::ClearIndicator) {
          result.success = indicator_record->clear(job.revision, error);
          result.message = result.success ? "display record cleared; motion unchanged" :
                           (error.empty() ? "new emergency observed during clear" : "cannot persist display reset: " + error);
          std::lock_guard<std::mutex> lock(mutex);
          state.indicator_emergency = indicator_record->value();
        } else {
          bool current;
          {
            std::lock_guard<std::mutex> lock(mutex);
            current = !stopping && state.generation == job.generation && state.safe_write_confirmed &&
                      state.outputs_ready && state.connected && stopped(state) &&
                      monotonicSeconds() - std::min(state.last_actual_read, state.last_status_read) < 1.0;
          }
          const bool cleared = current && motion_record->clear(job.revision, error);
          bool restore = false;
          {
            std::lock_guard<std::mutex> lock(mutex);
            if (cleared && !stopping && state.generation == job.generation && state.connected &&
                state.safe_write_confirmed && state.outputs_ready && stopped(state) &&
                monotonicSeconds() - std::min(state.last_actual_read, state.last_status_read) < 1.0) {
              state.emergency_latched = false;
              ++state.generation;
              state.reset_error.clear();
              resetFilterLocked();
              result = {true, "software emergency released; outputs remain stopped"};
            } else {
              if (state.emergency_latched && !motion_record->value()) { motion_record->observe(); restore = true; }
              state.reset_error = error.empty() ? "reset superseded" : "cannot persist motion reset: " + error;
              result = {false, state.reset_error};
            }
          }
          if (restore && !motion_record->flush(error)) persistenceFailure(error, true);
        }
        job.result->set_value(result);
        wake.notify_all();
      }
      if (quitting) {
        std::lock_guard<std::mutex> lock(file_mutex);
        if (file_jobs.empty() && !flush_motion && !flush_indicator && !manual_pending) break;
      }
    }
  }

  void failure(const std::string& error, uint64_t attempted_safe = 0) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (attempted_safe) safe_failed = std::max(safe_failed, attempted_safe);
      state.connected = false;
      state.actual_speed_valid = false;
      state.actual_left_motor_speed = state.actual_right_motor_speed = 0;
      state.last_error = error;
      ++state.consecutive_failures;
      fullStopLocked();
      reconnect_pending = true;
      const double now = monotonicSeconds();
      reconnect_at = now + 1.0 / config.poll_rate_hz;
      // The legacy optional escalation requests an extra immediate reconnect,
      // bounded by its own interval. The mandatory safe recovery never depends
      // on this flag or on reaching the consecutive-failure threshold.
      if (config.enable_safe_stop_on_comm_error && config.stop_on_timeout &&
          state.consecutive_failures >= static_cast<unsigned>(config.max_retries) &&
          now - last_extra_error_stop >= config.safe_stop_on_error_interval) {
        reconnect_at = now;
        last_extra_error_stop = now;
      }
    }
    written.clear();
    wake.notify_all();
  }

  Work chooseLocked(double now) {
    if (recovering && safe_at > 0.0 && now - safe_at >= 3.0) {
      // A safety ACK without two stopped readbacks is insufficient. Start a
      // new bounded synchronization attempt; ordinary targets remain blocked.
      state.last_error = "safe synchronization stopped readback timed out";
      fullStopLocked();
    }
    Work work;
    work.generation = state.generation;
    work.command = state.command;
    if (reconnect_pending) {
      if (now >= reconnect_at) work.action = Action::Reconnect;
      return work;
    }
    if (safe_pending) {
      work.action = Action::Safe;
      work.sequence = safe_requested;
      work.received = stop_requested_at;
      safe_pending = false;
    } else if (wheel_stop_pending) {
      work.action = Action::WheelStop;
      work.all = wheel_stop_all;
      work.sequence = wheel_stop_sequence;
      work.received = stop_requested_at;
      work.command.left_wheel_speed = work.command.right_wheel_speed = 0;
      wheel_stop_pending = false;
      wheel_stop_inflight = true;
    } else if ((recovering || (previous_write && now >= next_wheel)) && now >= next_wheel) {
      work.action = Action::Poll;
    } else if (state.outputs_ready && !state.emergency_latched && state.enabled) {
      if (all_pending) {
        work.action = Action::All; work.sequence = all.sequence; work.received = all.received;
        all_pending = false;
      } else {
        auto first = outputs.end();
        for (auto item = outputs.begin(); item != outputs.end(); ++item)
          if (first == outputs.end() || item->second.sequence < first->second.sequence) first = item;
        if (motion_pending && (first == outputs.end() || motion.sequence < first->second.sequence)) {
          work.action = Action::Motion; work.sequence = motion.sequence; work.received = motion.received;
          motion_pending = false;
        } else if (first != outputs.end()) {
          work.action = Action::Single; work.address = first->first;
          work.value = first->second.value; work.sequence = first->second.sequence; work.received = first->second.received;
          outputs.erase(first);
        }
      }
    }
    if (work.action == Action::None && now >= next_wheel) work.action = Action::Poll;
    if (work.action == Action::Poll) {
      work.status = recovering || now >= next_status;
      // Even a deliberately slow normal poll rate must provide two fresh
      // stopped samples inside the reset/synchronization feedback window.
      const double period = 1.0 / config.poll_rate_hz;
      next_wheel = now + (recovering ? std::min(period, 0.5) : period);
      if (work.status) next_status = now + 1.0 / std::min(config.poll_rate_hz, config.status_poll_rate_hz);
      previous_write = false;
    }
    return work;
  }

  bool current(const Work& work, bool safe = false) {
    std::lock_guard<std::mutex> lock(mutex);
    return !stopping && work.generation == state.generation &&
           (safe || (state.outputs_ready && !state.emergency_latched && state.enabled));
  }

  bool readCurrent(const Work& work) {
    std::lock_guard<std::mutex> lock(mutex);
    return !stopping && work.generation == state.generation && !safe_pending && !wheel_stop_pending;
  }

  void write(const Work& work) {
    const bool safe = work.action == Action::Safe;
    const bool wheel_stop = work.action == Action::WheelStop;
    if (!current(work, safe || wheel_stop)) return;
    auto block = registerBlock(work.command);
    if (safe || !stateEnabled()) block[0] = block[1] = block[2] = block[3] = 0;
    uint16_t address = REGISTER_LEFT_WHEEL_SPEED;
    if (work.action == Action::Single) { address = work.address; block = {toUint16(work.value)}; }
    else if (work.action == Action::Motion || (wheel_stop && !work.all)) {
      block.resize(2);
      if (!wheel_stop && (block[0] || block[1]) && config.enable_cmd_ramp_limit) {
        block[0] = toUint16(sent_left + clamp(toInt16(block[0]) - sent_left, -config.max_cmd_step_rpm, config.max_cmd_step_rpm));
        block[1] = toUint16(sent_right + clamp(toInt16(block[1]) - sent_right, -config.max_cmd_step_rpm, config.max_cmd_step_rpm));
      }
    }
    bool same = !safe && !wheel_stop && work.action != Action::All;
    for (std::size_t i = 0; same && i < block.size(); ++i) {
      const auto found = written.find(static_cast<uint16_t>(address + i));
      same = found != written.end() && found->second == block[i];
    }
    if (same) return;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (work.generation != state.generation || stopping) return;
      bool moving = false;
      for (std::size_t i = 0; i < block.size(); ++i)
        if (address + i <= REGISTER_DISC_ENABLE && block[i]) moving = true;
      if (safe || moving) state.safe_write_confirmed = false;
      state.command_wait_ms = std::max(0.0, monotonicSeconds() - work.received) * 1000.0;
      if (safe || wheel_stop) state.stop_tx_ms = state.command_wait_ms;
    }
    const double begin = monotonicSeconds();
    Timing first_write;
    if (work.action == Action::Single) transport->writeSingle(address, block[0]);
    else if (block.size() == 7 && !config.disc_lift_supported) {
      transport->writeBlock(address, std::vector<uint16_t>(block.begin(), block.begin() + 5));
      first_write = transport->lastWriteTiming();
      if (!current(work, safe || wheel_stop)) { written.clear(); return; }
      transport->writeSingle(REGISTER_LIGHT, block[6]);
    } else transport->writeBlock(address, block);
    const double end = monotonicSeconds();
    const auto last_write = transport->lastWriteTiming();
    if (!(first_write.tx > 0.0)) first_write = last_write;
    {
      std::lock_guard<std::mutex> lock(mutex);
      state.transaction_ms = (end - begin) * 1000.0;
      if (work.generation != state.generation) { written.clear(); return; }
      for (std::size_t i = 0; i < block.size(); ++i)
        if (config.disc_lift_supported || address + i != REGISTER_DISC_LIFT)
          written[static_cast<uint16_t>(address + i)] = block[i];
      if (address == REGISTER_LEFT_WHEEL_SPEED && block.size() >= 2) {
        sent_left = toInt16(block[0]); sent_right = toInt16(block[1]);
        wheel_zero_confirmed = sent_left == 0 && sent_right == 0;
        zero_confirmed_generation = state.generation;
      }
      if (wheel_stop) wheel_stop_inflight = false;
      state.connected = true;
      state.last_error.clear();
      state.consecutive_failures = 0;
      if (safe) {
        safe_completed = std::max(safe_completed, work.sequence);
        state.safe_write_confirmed = true;
        safe_at = end;
        recovering = true;
        recovery_samples = 0;
        next_wheel = 0.0;
      }
      if (work.action == Action::All) completed_all = std::max(completed_all, work.sequence);
      if (safe || wheel_stop) {
        if (first_write.tx > 0.0)
          state.stop_tx_ms = std::max(0.0, first_write.tx - work.received) * 1000.0;
        state.stop_ack_ms = std::max(0.0, (last_write.rx > 0.0 ? last_write.rx : end) - work.received) * 1000.0;
      }
      previous_write = true;
    }
    wake.notify_all();
  }

  bool stateEnabled() {
    std::lock_guard<std::mutex> lock(mutex);
    return state.enabled;
  }

  std::vector<uint16_t> readBlock(uint16_t address, unsigned count, Timing* timing) {
    const double begin = monotonicSeconds();
    auto values = transport->read(address, count, timing);
    const double end = monotonicSeconds();
    if (values.size() != count) throw TransportError("unexpected register read length");
    if (timing && (!(timing->tx > 0.0) || timing->rx < timing->tx)) *timing = {begin, end};
    std::lock_guard<std::mutex> lock(mutex);
    state.transaction_ms = (end - begin) * 1000.0;
    return values;
  }

  void poll(const Work& work) {
    std::vector<uint16_t> actual, status;
    Timing actual_timing, status_timing;
    bool combined;
    { std::lock_guard<std::mutex> lock(mutex); combined = state.status_combined_read; }
    if (!readCurrent(work)) return;
    if (work.status && combined) {
      try {
        const auto both = readBlock(REGISTER_DISC_SPEED, 7, &actual_timing);
        status.assign(both.begin(), both.begin() + 5);
        actual.assign(both.begin() + 5, both.end());
        status_timing = actual_timing;
      } catch (const ModbusException& error) {
        if (error.code != 2 && error.code != 3) throw;
        std::lock_guard<std::mutex> lock(mutex);
        state.status_combined_read = false;
      }
    }
    if (work.status && status.empty()) {
      if (!readCurrent(work)) return;
      status = readBlock(REGISTER_DISC_SPEED, 5, &status_timing);
    }
    if (actual.empty()) {
      if (!readCurrent(work)) return;
      actual = readBlock(REGISTER_LEFT_MOTOR_ACTUAL_SPEED, 2, &actual_timing);
    }
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (work.generation != state.generation || stopping) return;
      state.actual_left_motor_speed = toInt16(actual[0]);
      state.actual_right_motor_speed = toInt16(actual[1]);
      state.actual_speed_valid = true;
      state.actual_timing = actual_timing;
      state.last_actual_read = actual_timing.tx;
      ++state.wheel_sequence;
      if (!status.empty()) {
        state.status = decodeStatus(status);
        state.last_status_read = status_timing.tx;
        ++state.status_sequence;
      }
      state.connected = true;
      state.last_error.clear();
      state.consecutive_failures = 0;
      if (recovering && state.safe_write_confirmed && safe_at > 0.0 && !status.empty() &&
          actual_timing.tx >= safe_at && status_timing.tx >= safe_at) {
        const bool fresh = monotonicSeconds() - std::min(actual_timing.tx, status_timing.tx) < 1.0;
        recovery_samples = stopped(state) && fresh ? recovery_samples + 1 : 0;
        if (recovery_samples >= 2) {
          recovering = false;
          state.outputs_ready = true;
          recovered_generation = state.generation;
        }
      }
    }
    wake.notify_all();
  }

  void workerLoop() {
    for (;;) {
      Work work;
      {
        std::unique_lock<std::mutex> lock(mutex);
        if (stopping) break;
        work = chooseLocked(monotonicSeconds());
        if (work.action == Action::None) {
          double next = reconnect_pending ? reconnect_at : next_wheel;
          if (recovering && safe_at > 0.0) next = std::min(next, safe_at + 3.0);
          wake.wait_until(lock, at(std::max(monotonicSeconds() + 0.001, next)));
          continue;
        }
      }
      try {
        if (work.action == Action::Reconnect) {
          transport->reconnect();
          written.clear();
          std::lock_guard<std::mutex> lock(mutex);
          reconnect_pending = false;
        } else if (work.action == Action::Poll) poll(work);
        else write(work);
      } catch (const std::exception& error) {
        failure(error.what(), work.action == Action::Safe ? work.sequence : 0);
      } catch (...) {
        failure("unknown transport failure", work.action == Action::Safe ? work.sequence : 0);
      }
      const auto dropped = transport->droppedLogs();
      { std::lock_guard<std::mutex> lock(mutex); state.dropped_logs = dropped; }
    }
    try { transport->close(); } catch (...) {}
  }
};

Controller::Controller(Config config, std::unique_ptr<Transport> transport)
    : impl_(new Impl(std::move(config), std::move(transport))) {}
Controller::~Controller() { impl_->stop(); }
void Controller::start() { impl_->start(); }
void Controller::stop() { impl_->stop(); }
bool Controller::commandVelocity(double linear, double angular) { return impl_->velocity(linear, angular); }
bool Controller::commandWheels(int left, int right) { return impl_->wheels(left, right); }
bool Controller::taskEnable(bool enable) { return impl_->taskEnable(enable); }
bool Controller::setOutput(uint16_t address, int value) { return impl_->output(address, value); }
void Controller::emergencyStop() { impl_->emergency(); }
Result Controller::enable(bool enable) { return impl_->enable(enable); }
Result Controller::resetEmergency() { return impl_->reset(); }
Result Controller::clearIndicator() { return impl_->clearIndicator(); }
bool Controller::setManualSpeedLimit(double speed) { return impl_->manualLimit(speed); }
Snapshot Controller::snapshot() const { std::lock_guard<std::mutex> lock(impl_->mutex); return impl_->state; }
}  // namespace grinder_chassis_driver
