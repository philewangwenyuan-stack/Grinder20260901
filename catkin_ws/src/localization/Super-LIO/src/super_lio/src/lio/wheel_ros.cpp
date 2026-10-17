#include "lio/ROSWrapper.h"
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <stdexcept>
#include <iomanip>
#include <sstream>

namespace LI2Sup {
ROSWrapper::~ROSWrapper() {
  // These wheel handles precede the private queue in member declaration order.
  // Release them while the queue and node handle still exist.
  wheel_diagnostics_timer_.stop();wheel_sub_.shutdown();
  wheel_diagnostics_pub_.shutdown();wheel_system_diagnostics_pub_.shutdown();
  self_queue_.clear();
}

void ROSWrapper::setupWheelFusion() {
  nh_.param<std::string>("/lio/wheel/mode",wheel_mode_,"unspecified");
  nh_.param("/lio/wheel/enabled",wheel_enabled_,false);
  nh_.param("/lio/wheel/fuse_linear_velocity",wheel_linear_,true);
  nh_.param("/lio/wheel/fuse_angular_velocity",wheel_angular_,false);
  nh_.param("/lio/wheel/max_age_sec",wheel_max_age_,0.25);
  nh_.param("/lio/wheel/single_nis_gate",wheel_single_gate_,9.0);
  nh_.param("/lio/wheel/joint_nis_gate",wheel_joint_gate_,11.83);
  std::string topic;
  nh_.param<std::string>("/lio/wheel/twist_topic",topic,"/chassis/wheel_twis");
  if (wheel_enabled_ && (g_legacy_tf || (!wheel_linear_ && !wheel_angular_) || topic.empty() ||
      !std::isfinite(wheel_max_age_) || wheel_max_age_<=0.0 ||
      !std::isfinite(wheel_single_gate_) || wheel_single_gate_<=0.0 ||
      !std::isfinite(wheel_joint_gate_) || wheel_joint_gate_<=0.0))
    throw std::invalid_argument("Invalid wheel fusion configuration (requires standard TF and selected observations)");
  if (wheel_enabled_) wheel_sub_=nh_.subscribe(topic,200,&ROSWrapper::wheelHandler,this,
      ros::TransportHints().tcpNoDelay());
  wheel_diagnostics_pub_=nh_.advertise<diagnostic_msgs::DiagnosticArray>("/super_lio/wheel_fusion_diagnostics",1);
  wheel_system_diagnostics_pub_=nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics",1);
  wheel_diagnostics_timer_=nh_.createTimer(ros::Duration(0.1),&ROSWrapper::wheelDiagnostics,this);
}

void ROSWrapper::resetWheelFusion() {
  wheel_buffer_.clear(); wheel_have_left_=false; wheel_last_time_=-1.0;
  wheel_result_=wheel_fusion::Result();
  if (eskf_) eskf_->EnableWheelFusion(wheel_enabled_);
}

void ROSWrapper::wheelHandler(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg) {
  const double now=ros::Time::now().toSec();
  if (wheel_last_clock_>=0.0 && now<wheel_last_clock_) resetWheelFusion();
  wheel_last_clock_=now;
  if (!eskf_ || !eskf_->IsWheelInitialized()) { wheel_buffer_.clear(); return; }
  wheel_fusion::Sample sample;
  sample.time=msg->header.stamp.toSec();
  if (msg->header.frame_id!=g_base_frame || !std::isfinite(sample.time) || sample.time<=0.0 ||
      now-sample.time>wheel_max_age_ || sample.time>now+1e-3) {
    ++wheel_buffer_.invalid; return;
  }
  if (sample.time<=eskf_->GetTime()+1e-9) { ++wheel_buffer_.late; return; }
  const auto& t=msg->twist;
  sample.velocity << t.twist.linear.x,t.twist.angular.z;
  sample.covariance << t.covariance[0],t.covariance[5],t.covariance[30],t.covariance[35];
  for(int i=0;i<2;++i) sample.valid[i]=std::isfinite(sample.velocity[i]) &&
      std::isfinite(sample.covariance(i,i)) && sample.covariance(i,i)>0.0 && sample.covariance(i,i)<1e6;
  if (wheel_linear_ && wheel_angular_ && sample.valid[0] && sample.valid[1] &&
      (!sample.covariance.allFinite() ||
       std::abs(sample.covariance(0,1)-sample.covariance(1,0))>1e-9 ||
       !wheel_fusion::positive(sample.covariance))) {
    ++wheel_buffer_.invalid; return;
  }
  // Inactive/invalid rows must not contaminate the remaining scalar update.
  for(int i=0;i<2;++i) if (!sample.valid[i]) {
    sample.velocity[i]=0.0; sample.covariance.row(i).setZero();
    sample.covariance.col(i).setZero(); sample.covariance(i,i)=1.0;
  }
  if (!(wheel_linear_ && sample.valid[0]) && !(wheel_angular_ && sample.valid[1])) {
    ++wheel_buffer_.invalid; return;
  }
  wheel_buffer_.push(sample);
}

bool ROSWrapper::propagateWheel(MeasureGroup& meas,std::vector<DynamicState>& states) {
  if (!wheel_enabled_) return false;
  if (!eskf_ || !eskf_->IsWheelInitialized()) { resetWheelFusion(); return false; }
  states.clear(); states.push_back(eskf_->GetDynamicState());
  eskf_->SetObsTime(meas.lidar.end_time);
  for(const auto& right:meas.imu) {
    if (right.secs<=eskf_->GetTime()+1e-9) { wheel_left_=right; wheel_have_left_=true; continue; }
    if (!wheel_have_left_) {
      // Initialization needs one real left bracket; never invent an IMU sample.
      eskf_->Predict(right); wheel_left_=right;
      // Legacy prediction may clip a lookahead sample to scan end. That sample
      // is not yet a usable left bracket; retry bootstrap in the next scan.
      wheel_have_left_=right.secs<=eskf_->GetTime()+1e-9;
      if (states.back().time<eskf_->GetTime()) states.push_back(eskf_->GetDynamicState());
      continue;
    }
    const double target=std::min(right.secs,meas.lidar.end_time);
    while(!wheel_buffer_.samples.empty() && wheel_buffer_.samples.front().time<=target) {
      const auto sample=wheel_buffer_.samples.front(); wheel_buffer_.samples.pop_front();
      if (sample.time<=eskf_->GetTime()+1e-9) { ++wheel_buffer_.late; continue; }
      if (meas.lidar.end_time-sample.time>wheel_max_age_) { ++wheel_buffer_.expired; continue; }
      if (!eskf_->PredictWheelInterval(wheel_left_,right,sample.time)) { ++wheel_buffer_.late; continue; }
      wheel_result_=eskf_->UpdateWheel(sample,wheel_linear_,wheel_angular_,wheel_single_gate_,wheel_joint_gate_);
      if(wheel_result_.applied) { ++wheel_buffer_.accepted; wheel_last_time_=sample.time; }
      else ++wheel_rejected_;
      if(states.back().time<eskf_->GetTime()) states.push_back(eskf_->GetDynamicState());
    }
    if(target>eskf_->GetTime()+1e-9) {
      eskf_->PredictWheelInterval(wheel_left_,right,target);
      if(states.back().time<eskf_->GetTime()) states.push_back(eskf_->GetDynamicState());
    }
    if(right.secs>=meas.lidar.end_time) break;
    wheel_left_=right;
  }
  return true;
}

void ROSWrapper::wheelDiagnostics(const ros::TimerEvent&) {
  diagnostic_msgs::DiagnosticArray report;
  report.header.stamp=ros::Time::now();
  diagnostic_msgs::DiagnosticStatus status;
  status.name="super_lio_wheel_fusion"; status.hardware_id="super_lio";
  const double age=wheel_last_time_>0.0?report.header.stamp.toSec()-wheel_last_time_:-1.0;
  status.level=wheel_enabled_ && (age<0.0 || age>wheel_max_age_)?
      diagnostic_msgs::DiagnosticStatus::WARN:diagnostic_msgs::DiagnosticStatus::OK;
  status.message=!wheel_enabled_?"disabled":(status.level?"no fresh accepted wheel update; pure LIO":"wheel fusion active");
  auto add=[&](const std::string& key,const std::string& value) {
    diagnostic_msgs::KeyValue item; item.key=key; item.value=value; status.values.push_back(item);
  };
  auto number=[](double value) {
    std::ostringstream stream;stream<<std::setprecision(17)<<value;return stream.str();
  };
  add("enabled",wheel_enabled_?"true":"false");
  add("mode",wheel_mode_);
  add("initialized",eskf_ && eskf_->IsWheelInitialized()?"true":"false");
  add("linear_enabled",wheel_linear_?"true":"false"); add("angular_enabled",wheel_angular_?"true":"false");
  add("age_sec",number(age)); add("accepted",std::to_string(wheel_buffer_.accepted));
  add("invalid",std::to_string(wheel_buffer_.invalid)); add("late",std::to_string(wheel_buffer_.late));
  add("expired",std::to_string(wheel_buffer_.expired)); add("overflow",std::to_string(wheel_buffer_.overflow));
  add("innovation_rejected",std::to_string(wheel_rejected_));
  for(int i=0;i<2;++i) {
    const std::string key=i==0?"linear":"angular";
    add(key+"_residual",number(wheel_result_.residual[i]));
    add(key+"_nis",number(wheel_result_.nis[i]));
    add(key+"_variance",number(wheel_result_.variance[i]));
    add(key+"_innovation_variance",number(wheel_result_.innovation_variance[i]));
    add(key+"_used",wheel_result_.used[i]?"true":"false");
  }
  add("joint_nis",number(wheel_result_.joint_nis));
  add("used_covariance_cross",number(wheel_result_.used[0] && wheel_result_.used[1]?
      wheel_result_.covariance_cross:0.0));
  add("innovation_covariance_cross",number(wheel_result_.innovation_covariance_cross));
  report.status.push_back(status); wheel_diagnostics_pub_.publish(report); wheel_system_diagnostics_pub_.publish(report);
}
} // namespace LI2Sup
