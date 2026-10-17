#include "grinder_chassis_driver/chassis_controller.h"
#include "grinder_chassis_driver/core.h"
#include "grinder_chassis_driver/emergency_record.h"
#include <grinder_chassis_driver/ChassisStatus.h>
#include <grinder_chassis_driver/ClearFault.h>
#include <grinder_chassis_driver/EnableChassis.h>
#include <grinder_chassis_driver/WheelSpeedCommand.h>
#include <grinder_chassis_driver/WheelSpeedState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_srvs/Trigger.h>
#include <json/json.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <mutex>
#include <sstream>

namespace g = grinder_chassis_driver;

namespace {
g::Config loadConfig(ros::NodeHandle& nh) {
  g::Config c;
#define PARAM(name) nh.param(#name, c.name, c.name)
  PARAM(port); PARAM(baudrate); PARAM(slave_id); PARAM(timeout); PARAM(transaction_timeout);
  PARAM(command_watchdog_period); PARAM(poll_rate_hz); PARAM(status_poll_rate_hz);
  PARAM(diagnostics_rate_hz); PARAM(status_combined_read); PARAM(command_timeout);
  PARAM(write_verify); PARAM(use_functioncode_16); PARAM(max_retries);
  PARAM(enable_safe_stop_on_comm_error); PARAM(stop_on_timeout); PARAM(startup_zero_output);
  PARAM(left_speed_min); PARAM(left_speed_max); PARAM(right_speed_min); PARAM(right_speed_max);
  PARAM(disc_speed_min); PARAM(disc_speed_max); PARAM(disc_lift_supported);
  PARAM(max_cmd_step_rpm); PARAM(enable_cmd_ramp_limit); PARAM(manual_drive_max_wheel_speed);
  PARAM(manual_wheel_deadband_rpm); PARAM(manual_wheel_hyst_enter_rpm); PARAM(manual_wheel_hyst_exit_rpm);
  PARAM(safe_stop_on_error_interval); PARAM(rs485_raw_log_enabled); PARAM(rs485_raw_log_file);
  PARAM(rs485_log_queue_capacity); PARAM(cmd_vel_enabled); PARAM(cmd_vel_topic); PARAM(task_enable_default);
  PARAM(cmd_vel_wheel_track_m); PARAM(cmd_vel_wheel_radius_m); PARAM(cmd_vel_gear_ratio);
  PARAM(wheel_odom_topic); PARAM(wheel_twist_topic); PARAM(wheel_odom_frame_id);
  PARAM(wheel_odom_child_frame_id); PARAM(wheel_feedback_delay_sec);
  PARAM(wheel_twist_linear_variance); PARAM(wheel_twist_angular_variance); PARAM(wheel_twist_cross_covariance);
  PARAM(actual_motor_speed_scale); PARAM(actual_speed_left_sign); PARAM(actual_speed_right_sign);
  PARAM(actual_speed_swap_lr); PARAM(cmd_vel_scale); PARAM(cmd_vel_max_linear); PARAM(cmd_vel_max_angular);
  PARAM(cmd_vel_max_input_v); PARAM(cmd_vel_max_input_w); PARAM(cmd_vel_max_abs_wheel_rpm);
  PARAM(cmd_vel_deadband_linear); PARAM(cmd_vel_deadband_angular); PARAM(cmd_source_hold_sec);
  PARAM(drive_left_sign); PARAM(drive_right_sign); PARAM(drive_swap_lr);
  PARAM(cmd_vel_drive_left_sign); PARAM(cmd_vel_drive_right_sign); PARAM(cmd_vel_drive_swap_lr);
  PARAM(cmd_vel_filter_enabled); PARAM(cmd_vel_filter_linear_accel); PARAM(cmd_vel_filter_angular_accel);
  PARAM(cmd_vel_filter_angular_deadband); PARAM(cmd_vel_filter_sign_confirm_count);
  PARAM(config_file); PARAM(motion_emergency_file); PARAM(indicator_emergency_file);
#undef PARAM
  g::validateConfig(c);
  if (c.rs485_raw_log_enabled && !c.rs485_raw_log_file.empty()) {
    const std::time_t now = std::time(nullptr);
    std::tm tm{};
    localtime_r(&now, &tm);
    std::ostringstream suffix;
    suffix << '_' << std::put_time(&tm, "%Y%m%d_%H%M%S");
    const auto slash = c.rs485_raw_log_file.find_last_of("/\\");
    const auto dot = c.rs485_raw_log_file.find_last_of('.');
    const bool extension = dot != std::string::npos && (slash == std::string::npos || dot > slash);
    c.rs485_raw_log_file.insert(extension ? dot : c.rs485_raw_log_file.size(), suffix.str());
    if (!extension) c.rs485_raw_log_file += ".log";
    nh.setParam("rs485_raw_log_file", c.rs485_raw_log_file);
  }
  return c;
}

std::string safetyJson(const g::Snapshot& s) {
  Json::Value v(Json::objectValue);
  v["emergency_latched"] = s.emergency_latched;
  v["generation"] = Json::UInt64(s.generation);
  v["safe_write_confirmed"] = s.safe_write_confirmed;
  v["outputs_ready"] = s.outputs_ready;
  v["reset_error"] = s.reset_error;
  Json::StreamWriterBuilder writer;
  writer["indentation"] = "";
  return Json::writeString(writer, v);
}

class ChassisNode {
 public:
  ChassisNode() : private_nh_("~"), command_nh_(nh_), safety_nh_(nh_), service_nh_(nh_),
                  telemetry_nh_(nh_), config_(loadConfig(private_nh_)),
                  controller_(config_, g::makeModbusTransport(config_)),
                  command_spinner_(1, &command_queue_), safety_spinner_(1, &safety_queue_),
                  service_spinner_(2, &service_queue_), telemetry_spinner_(1, &telemetry_queue_) {
    command_nh_.setCallbackQueue(&command_queue_);
    safety_nh_.setCallbackQueue(&safety_queue_);
    service_nh_.setCallbackQueue(&service_queue_);
    telemetry_nh_.setCallbackQueue(&telemetry_queue_);
    wheel_pub_ = nh_.advertise<g::WheelSpeedState>("/chassis/wheel_speed_state", 10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(config_.wheel_odom_topic, 10);
    twist_pub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>(config_.wheel_twist_topic, 10);
    status_pub_ = nh_.advertise<g::ChassisStatus>("/chassis/status", 10);
    diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);
    safety_pub_ = nh_.advertise<std_msgs::String>("/chassis/motion_safety", 1, true);
    indicator_pub_ = nh_.advertise<std_msgs::Bool>("/chassis/indicator_emergency", 1, true);
    const auto hints = ros::TransportHints().tcpNoDelay();
    subscribers_.push_back(command_nh_.subscribe<g::WheelSpeedCommand>("/chassis/wheel_speed_cmd", 1,
        [this](const g::WheelSpeedCommand::ConstPtr& m) {
          timedCommand([&] { controller_.commandWheels(m->left_wheel_speed, m->right_wheel_speed); });
        }, ros::VoidConstPtr(), hints));
    if (config_.cmd_vel_enabled) {
      subscribers_.push_back(command_nh_.subscribe<geometry_msgs::Twist>(config_.cmd_vel_topic, 1,
          [this](const geometry_msgs::Twist::ConstPtr& m) {
            if (!std::isfinite(m->linear.x) || !std::isfinite(m->angular.z)) {
              ROS_WARN_THROTTLE(2.0, "Ignoring non-finite cmd_vel; command timeout remains active");
              return;
            }
            timedCommand([&] { controller_.commandVelocity(m->linear.x, m->angular.z); });
          }, ros::VoidConstPtr(), hints));
      subscribers_.push_back(command_nh_.subscribe<std_msgs::Bool>("/chassis/task_enable", 10,
          [this](const std_msgs::Bool::ConstPtr& m) { controller_.taskEnable(m->data); }));
    }
    subscribers_.push_back(command_nh_.subscribe<std_msgs::Int16>("/chassis/disc_speed_cmd", 10,
        [this](const std_msgs::Int16::ConstPtr& m) { controller_.setOutput(g::REGISTER_DISC_SPEED, m->data); }));
    subscribers_.push_back(command_nh_.subscribe<std_msgs::Bool>("/chassis/disc_enable_cmd", 10,
        [this](const std_msgs::Bool::ConstPtr& m) { controller_.setOutput(g::REGISTER_DISC_ENABLE, m->data ? 1 : 0); }));
    subscribers_.push_back(command_nh_.subscribe<std_msgs::UInt16>("/chassis/work_mode_cmd", 10,
        [this](const std_msgs::UInt16::ConstPtr& m) { controller_.setOutput(g::REGISTER_WORK_MODE, m->data); }));
    subscribers_.push_back(command_nh_.subscribe<std_msgs::UInt16>("/chassis/disc_lift_cmd", 10,
        [this](const std_msgs::UInt16::ConstPtr& m) { controller_.setOutput(g::REGISTER_DISC_LIFT, m->data); }));
    subscribers_.push_back(command_nh_.subscribe<std_msgs::Bool>("/chassis/light_cmd", 10,
        [this](const std_msgs::Bool::ConstPtr& m) { controller_.setOutput(g::REGISTER_LIGHT, m->data ? 1 : 0); }));
    subscribers_.push_back(service_nh_.subscribe<std_msgs::Float32>("/chassis/manual_speed_limit", 1,
        [this](const std_msgs::Float32::ConstPtr& m) {
          if (controller_.setManualSpeedLimit(m->data)) {
            private_nh_.setParam("manual_drive_max_wheel_speed", controller_.snapshot().manual_speed_limit);
          } else { ROS_WARN("Invalid manual speed limit ignored"); }
        }));
    subscribers_.push_back(safety_nh_.subscribe<std_msgs::Bool>("/chassis/emergency_stop", 1,
        [this](const std_msgs::Bool::ConstPtr& m) {
          if (m->data) { controller_.emergencyStop(); publishSafety(controller_.snapshot()); }
        }, ros::VoidConstPtr(), hints));
    services_.push_back(service_nh_.advertiseService("/chassis/enable", &ChassisNode::enable, this));
    services_.push_back(service_nh_.advertiseService("/chassis/clear_fault", &ChassisNode::clearFault, this));
    services_.push_back(service_nh_.advertiseService("/chassis/reset_emergency_stop", &ChassisNode::reset, this));
    services_.push_back(service_nh_.advertiseService("/chassis/clear_emergency_indicator", &ChassisNode::clearIndicator, this));
    wheel_timer_ = telemetry_nh_.createWallTimer(ros::WallDuration(1.0 / config_.poll_rate_hz), &ChassisNode::publishWheels, this);
    status_timer_ = telemetry_nh_.createWallTimer(ros::WallDuration(1.0 / config_.status_poll_rate_hz), &ChassisNode::publishStatus, this);
    diagnostics_timer_ = telemetry_nh_.createWallTimer(ros::WallDuration(1.0 / config_.diagnostics_rate_hz), &ChassisNode::publishDiagnostics, this);
    indicator_timer_ = telemetry_nh_.createWallTimer(ros::WallDuration(0.5), [this](const ros::WallTimerEvent&) { publishSafety(controller_.snapshot()); });
    publishSafety(controller_.snapshot());
    controller_.start();
    command_spinner_.start(); safety_spinner_.start(); service_spinner_.start(); telemetry_spinner_.start();
    ROS_INFO("C++ chassis driver started: %s, %d baud; transaction deadline %.3fs", config_.port.c_str(), config_.baudrate, config_.transaction_timeout);
  }

  ~ChassisNode() {
    command_spinner_.stop();
    controller_.stop();
    safety_spinner_.stop(); service_spinner_.stop(); telemetry_spinner_.stop();
  }

 private:
  template <typename Function>
  void timedCommand(Function&& function) {
    const double begin = g::monotonicSeconds();
    try { function(); }
    catch (const std::exception& e) { ROS_ERROR_THROTTLE(2.0, "Invalid chassis command: %s", e.what()); }
    const double elapsed = (g::monotonicSeconds() - begin) * 1000.0;
    std::lock_guard<std::mutex> guard(metrics_mutex_);
    callback_samples_[callback_sample_count_++ % callback_samples_.size()] = elapsed;
  }
  bool enable(g::EnableChassis::Request& req, g::EnableChassis::Response& res) {
    const auto result = controller_.enable(req.enable);
    res.success = result.success; res.message = result.message; return true;
  }
  bool clearFault(g::ClearFault::Request&, g::ClearFault::Response& res) {
    res.success = false;
    res.message = "Fault clear register is not defined yet. Provide the vendor register table to implement it.";
    return true;
  }
  bool reset(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    const auto result = controller_.resetEmergency();
    res.success = result.success; res.message = result.message;
    publishSafety(controller_.snapshot()); return true;
  }
  bool clearIndicator(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    const auto result = controller_.clearIndicator();
    res.success = result.success; res.message = result.message;
    publishSafety(controller_.snapshot()); return true;
  }
  void publishSafety(const g::Snapshot& s) {
    // The safety queue and telemetry queue can both publish; serialize snapshots
    // against acquiring a newer state so a pre-trip snapshot cannot latch last.
    std::lock_guard<std::mutex> guard(safety_publish_mutex_);
    const auto current = controller_.snapshot();
    (void)s;
    std_msgs::String safety; safety.data = safetyJson(current); safety_pub_.publish(safety);
    std_msgs::Bool indicator; indicator.data = current.indicator_emergency; indicator_pub_.publish(indicator);
  }
  void publishWheels(const ros::WallTimerEvent&) {
    const auto s = controller_.snapshot();
    const double now_mono = g::monotonicSeconds();
    const double max_age = std::max(1.0, 2.0 / config_.poll_rate_hz + config_.transaction_timeout);
    bool valid = s.connected && s.actual_speed_valid && now_mono - s.last_actual_read < max_age;
    const auto now = ros::Time::now();
    g::WheelSpeedState wheels;
    wheels.header.stamp = now;
    wheels.target_left_wheel_speed = s.command.left_wheel_speed;
    wheels.target_right_wheel_speed = s.command.right_wheel_speed;
    wheels.feedback_left_wheel_speed = valid ? s.actual_left_motor_speed : 0;
    wheels.feedback_right_wheel_speed = valid ? s.actual_right_motor_speed : 0;
    wheels.feedback_valid = valid;
    std::pair<double, double> velocity{0.0, 0.0};
    if (valid) {
      try { velocity = g::motorRpmToBaseTwist(s.actual_left_motor_speed, s.actual_right_motor_speed, config_); }
      catch (const std::exception& e) {
        valid = false;
        ROS_ERROR_THROTTLE(2.0, "Invalid wheel feedback conversion: %s", e.what());
      }
    }
    wheels.feedback_valid = valid;
    wheel_pub_.publish(wheels);
    nav_msgs::Odometry odom;
    odom.header.stamp = now; odom.header.frame_id = config_.wheel_odom_frame_id;
    odom.child_frame_id = config_.wheel_odom_child_frame_id; odom.pose.pose.orientation.w = 1.0;
    odom.twist.twist.linear.x = velocity.first; odom.twist.twist.angular.z = velocity.second;
    for (int i : {0, 7, 14, 21, 28, 35}) odom.pose.covariance[i] = 1e6;
    odom.twist.covariance[0] = odom.twist.covariance[35] = 0.04;
    odom_pub_.publish(odom);
    if (valid && s.wheel_sequence != last_wheel_sequence_ && s.actual_timing.tx > 0.0 &&
        s.actual_timing.rx >= s.actual_timing.tx) {
      const double age = now_mono - 0.5 * (s.actual_timing.tx + s.actual_timing.rx) + config_.wheel_feedback_delay_sec;
      if (std::isfinite(age) && age >= 0.0 && now.toSec() > age) {
        geometry_msgs::TwistWithCovarianceStamped twist;
        twist.header.stamp = now - ros::Duration(age); twist.header.frame_id = "base_link";
        twist.twist.twist.linear.x = velocity.first; twist.twist.twist.angular.z = velocity.second;
        for (int i : {7, 14, 21, 28}) twist.twist.covariance[i] = 1e6;
        twist.twist.covariance[0] = config_.wheel_twist_linear_variance;
        twist.twist.covariance[35] = config_.wheel_twist_angular_variance;
        twist.twist.covariance[5] = twist.twist.covariance[30] = config_.wheel_twist_cross_covariance;
        twist_pub_.publish(twist);
      }
    }
    last_wheel_sequence_ = s.wheel_sequence;
    publishSafety(s);
  }
  void publishStatus(const ros::WallTimerEvent&) {
    const auto s = controller_.snapshot();
    g::ChassisStatus msg; msg.header.stamp = ros::Time::now();
    const double max_age = std::max(1.0, 2.0 / config_.status_poll_rate_hz + config_.transaction_timeout);
    msg.connected = s.connected && s.actual_speed_valid && s.last_status_read > 0.0 &&
                    g::monotonicSeconds() - s.last_status_read < max_age;
    msg.enabled = s.enabled; msg.work_mode = s.status.work_mode;
    msg.disc_speed_target = s.command.disc_speed; msg.disc_speed_feedback = s.status.disc_speed;
    msg.disc_enabled = s.status.disc_enable == 1; msg.disc_lift_state = s.status.disc_lift;
    msg.light_enabled = s.status.light == 1; msg.consecutive_failures = s.consecutive_failures;
    msg.last_error = s.last_error.empty() && !msg.connected ? "status read failed" : s.last_error;
    status_pub_.publish(msg);
  }
  void publishDiagnostics(const ros::WallTimerEvent&) {
    const auto s = controller_.snapshot();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "grinder_chassis_driver";
    status.hardware_id = "rs485_modbus_slave_" + std::to_string(config_.slave_id);
    status.level = s.connected ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::ERROR;
    status.message = s.connected ? "connected" : (s.last_error.empty() ? "disconnected" : s.last_error);
    const auto add = [&status](const std::string& key, const std::string& value) {
      diagnostic_msgs::KeyValue kv; kv.key = key; kv.value = value; status.values.push_back(kv);
    };
    const auto boolean = [](bool value) { return value ? "True" : "False"; };
    const auto age = [](double value) { return value > 0.0 ? g::monotonicSeconds() - value : -1.0; };
    add("emergency_latched", boolean(s.emergency_latched));
    add("safe_write_confirmed", boolean(s.safe_write_confirmed)); add("outputs_ready", boolean(s.outputs_ready));
    add("reset_error", s.reset_error); add("port", config_.port); add("baudrate", std::to_string(config_.baudrate));
    add("slave_id", std::to_string(config_.slave_id)); add("consecutive_failures", std::to_string(s.consecutive_failures));
    add("actual_speed_age_sec", std::to_string(age(s.last_actual_read)));
    add("status_age_sec", std::to_string(age(s.last_status_read)));
    add("status_combined_read", boolean(s.status_combined_read)); add("driver_backend", "cpp");
    add("command_compute_ms", std::to_string(s.command_compute_ms)); add("command_wait_ms", std::to_string(s.command_wait_ms));
    add("transaction_ms", std::to_string(s.transaction_ms)); add("watchdog_lateness_ms", std::to_string(s.watchdog_lateness_ms));
    add("stop_tx_ms", std::to_string(s.stop_tx_ms)); add("stop_ack_ms", std::to_string(s.stop_ack_ms));
    add("timeout_count", std::to_string(s.timeout_count)); add("dropped_logs", std::to_string(s.dropped_logs));
    std::vector<double> callbacks;
    {
      std::lock_guard<std::mutex> guard(metrics_mutex_);
      callbacks.assign(callback_samples_.begin(), callback_samples_.begin() +
          std::min(callback_sample_count_, callback_samples_.size()));
    }
    if (!callbacks.empty()) {
      std::sort(callbacks.begin(), callbacks.end());
      add("command_callback_p99_ms", std::to_string(callbacks[static_cast<std::size_t>(std::ceil(callbacks.size() * 0.99)) - 1]));
      add("command_callback_max_ms", std::to_string(callbacks.back()));
    }
    diagnostic_msgs::DiagnosticArray array; array.header.stamp = ros::Time::now(); array.status.push_back(status);
    diagnostics_pub_.publish(array);
    if (!s.last_error.empty()) ROS_WARN_THROTTLE(2.0, "Chassis: %s", s.last_error.c_str());
  }

  ros::NodeHandle nh_, private_nh_, command_nh_, safety_nh_, service_nh_, telemetry_nh_;
  ros::CallbackQueue command_queue_, safety_queue_, service_queue_, telemetry_queue_;
  g::Config config_;
  g::Controller controller_;
  ros::Publisher wheel_pub_, odom_pub_, twist_pub_, status_pub_, diagnostics_pub_, safety_pub_, indicator_pub_;
  std::vector<ros::Subscriber> subscribers_;
  std::vector<ros::ServiceServer> services_;
  ros::WallTimer wheel_timer_, status_timer_, diagnostics_timer_, indicator_timer_;
  ros::AsyncSpinner command_spinner_, safety_spinner_, service_spinner_, telemetry_spinner_;
  std::mutex safety_publish_mutex_;
  std::mutex metrics_mutex_;
  std::array<double, 256> callback_samples_{};
  std::size_t callback_sample_count_ = 0;
  uint64_t last_wheel_sequence_ = 0;
};
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "chassis_driver");
  try {
    ChassisNode node;
    ros::waitForShutdown();
  } catch (const std::exception& error) {
    ROS_FATAL("Cannot start chassis driver: %s", error.what());
    return 1;
  }
  return 0;
}
