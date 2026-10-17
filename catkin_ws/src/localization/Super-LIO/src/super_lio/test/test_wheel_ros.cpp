// Run against an isolated roscore; no sensor/chassis node or hardware is used.
#include "lio/ROSWrapper.h"
#include <cassert>
#include <iostream>

using namespace LI2Sup;
using namespace BASIC;

static void pump(ROSWrapper& wrapper) {
  const auto until=ros::WallTime::now()+ros::WallDuration(0.08);
  while(ros::WallTime::now()<until) {
    wrapper.spinOnce();ros::spinOnce();ros::WallDuration(0.002).sleep();
  }
}

int main(int argc,char** argv) {
  ros::init(argc,argv,"wheel_ros_test");
  ros::NodeHandle nh;
  ros::Time::setNow(ros::Time(100.2));
  g_legacy_tf=false;g_base_frame="base_link";g_world_frame="map";
  g_lidar_topic="/test/cloud";g_imu_topic="/test/imu";g_lidar_type=LID_TYPE::VELO16;
  g_odom_topic="/test/lio";g_path_topic="/test/path";
  g_gravity_norm=9.8;g_imu_base=SE3(SO3(),V3::Zero());
  nh.setParam("/lio/wheel/enabled",false);
  {
    ROSWrapper disabled;
    MeasureGroup empty;
    std::vector<DynamicState> states;
    assert(!disabled.propagateWheel(empty,states));
  }
  nh.setParam("/lio/wheel/enabled",true);
  nh.setParam("/lio/wheel/fuse_linear_velocity",false);
  nh.setParam("/lio/wheel/fuse_angular_velocity",false);
  bool rejected=false;
  try { ROSWrapper invalid; } catch(const std::invalid_argument&) { rejected=true; }
  assert(rejected);
  nh.setParam("/lio/wheel/fuse_linear_velocity",true);
  g_legacy_tf=true;rejected=false;
  try { ROSWrapper invalid; } catch(const std::invalid_argument&) { rejected=true; }
  assert(rejected);g_legacy_tf=false;
  ROSWrapper wrapper;
  auto filter=std::make_shared<ESKF>();
  filter->SetInitialConditions(ESKF::Options(),V3::Zero(),V3::Zero(),1.0,V3(0,0,-9.8));
  filter->SetX(SysState(100.0));filter->SetLastObsTime(100.0);filter->init_=false;
  wrapper.setESKF(filter);
  wrapper.setWheelInitialized(true);
  auto publisher=nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/chassis/wheel_twist",10);
  for(int i=0;i<100 && publisher.getNumSubscribers()==0;++i) pump(wrapper);
  assert(publisher.getNumSubscribers()==1);
  geometry_msgs::TwistWithCovarianceStamped message;
  message.header.stamp=ros::Time(100.05);message.header.frame_id="wrong";
  message.twist.twist.linear.x=-0.1;message.twist.twist.angular.z=-0.05;
  message.twist.covariance[0]=message.twist.covariance[35]=0.04;
  publisher.publish(message);pump(wrapper);
  message.header.frame_id="base_link";
  publisher.publish(message);pump(wrapper);
  publisher.publish(message);pump(wrapper);  // Duplicate must not be re-used.
  auto frame=[](double begin,double end) {
    MeasureGroup meas;meas.lidar.start_time=begin;meas.lidar.end_time=end;
    IMUData left,right;left.secs=begin;right.secs=end+0.01;
    left.acc=right.acc=V3(0,0,9.8);left.gyr=right.gyr=V3::Zero();
    meas.imu.push_back(left);meas.imu.push_back(right);return meas;
  };
  std::vector<DynamicState> states;
  auto meas=frame(100.0,100.1);
  assert(wrapper.propagateWheel(meas,states));
  assert(std::abs(filter->GetTime()-100.1)<1e-7 && filter->GetSysState().v.x()<0.0);
  const auto covariance=filter->GetCov();
  assert(wrapper.propagateWheel(meas,states));
  assert((filter->GetCov()-covariance).norm()==0.0);
  message.header.stamp=ros::Time(100.15);
  publisher.publish(message);pump(wrapper);
  meas=frame(100.1,100.14);assert(wrapper.propagateWheel(meas,states));
  assert(std::abs(filter->GetTime()-100.14)<1e-7);
  meas=frame(100.14,100.2);assert(wrapper.propagateWheel(meas,states));
  assert(std::abs(filter->GetTime()-100.2)<1e-7);
  ros::Time::setNow(ros::Time(100.4));
  message.header.stamp=ros::Time(100.25);message.twist.twist.linear.x=0.1;
  publisher.publish(message);pump(wrapper);
  wrapper.resetWheelFusion();
  auto expected=*filter;
  meas=frame(100.2,100.3);
  assert(expected.PredictWheelInterval(meas.imu.front(),meas.imu.back(),100.3));
  assert(wrapper.propagateWheel(meas,states));
  assert((filter->GetCov()-expected.GetCov()).norm()<1e-10);
  assert((filter->GetSysState().v-expected.GetSysState().v).norm()<1e-10);
  std::cout << "ROS subscription, invalid/duplicate messages, reverse motion, future buffering and reset passed\n";
}
