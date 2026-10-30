// Run against an isolated roscore. Only synthetic Livox/IMU messages are sent;
// no hardware, navigation, chassis or grinder node is started.
#include "lio/ROSWrapper.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>

using namespace LI2Sup;
using namespace BASIC;

static void pump(ROSWrapper& wrapper) {
  const auto until = ros::WallTime::now() + ros::WallDuration(0.04);
  while (ros::WallTime::now() < until) {
    wrapper.spinOnce();
    ros::spinOnce();
    ros::WallDuration(0.002).sleep();
  }
}

static float rangeAt(const sensor_msgs::LaserScan& scan, double angle) {
  const auto index = static_cast<std::size_t>((angle - scan.angle_min) / scan.angle_increment);
  assert(index < scan.ranges.size());
  // Existing projection uses atan2(float,float) against double bin limits;
  // LaserScan metadata is float. At a boundary the reconstructed bin can
  // differ by one, so compare a single angular-resolution neighborhood.
  float range = scan.ranges[index];
  if (index > 0) range = std::min(range, scan.ranges[index - 1]);
  if (index + 1 < scan.ranges.size()) range = std::min(range, scan.ranges[index + 1]);
  return range;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_scan_ros_test");
  ros::NodeHandle nh;
  assert(g_scan_filter_livox_dust);  // C++ fallback default.
  nh.setParam("/lio/wheel/enabled", false);
  g_lidar_type = LID_TYPE::LIVOX;
  g_lidar_topic = "/test/livox_scan/lidar";
  g_imu_topic = "/test/livox_scan/imu";
  g_visual_planner_cloud_topic = "/test/livox_scan/scan";
  g_odom_topic = "/test/livox_scan/odom";
  g_path_topic = "/test/livox_scan/path";
  g_base_frame = "base_link";
  g_world_frame = "odom";
  g_legacy_tf = false;
  g_filter_rate = 3;
  g_blind2 = 0.25f;
  g_maxrange2 = 100.0f;
  g_gravity_norm = 9.8;
  g_imu_base = SE3(SO3(), V3::Zero());
  ROSWrapper wrapper;
  auto filter = std::make_shared<ESKF>();
  filter->SetInitialConditions(ESKF::Options(), V3::Zero(), V3::Zero(), 1.0, V3(0, 0, -9.8));
  filter->SetX(SysState(99.9));
  filter->SetLastObsTime(99.9);
  filter->init_ = false;
  wrapper.setESKF(filter);

  sensor_msgs::LaserScan::ConstPtr latest;
  auto scan_sub = nh.subscribe<sensor_msgs::LaserScan>(g_visual_planner_cloud_topic, 10,
      [&](const sensor_msgs::LaserScan::ConstPtr& scan) { latest = scan; });
  auto lidar_pub = nh.advertise<livox_ros_driver2::CustomMsg>(g_lidar_topic, 10);
  auto imu_pub = nh.advertise<sensor_msgs::Imu>(g_imu_topic, 10);
  CloudPtr empty(new PointCloudType());
  wrapper.pub_scan_body(empty, ros::Time(1));  // Advertise before the first replay.
  for (int i = 0; i < 100 && (lidar_pub.getNumSubscribers() == 0 ||
       imu_pub.getNumSubscribers() == 0 || scan_sub.getNumPublishers() == 0); ++i) pump(wrapper);
  assert(lidar_pub.getNumSubscribers() == 1 && imu_pub.getNumSubscribers() == 1);
  assert(scan_sub.getNumPublishers() == 1);
  // Optional external capture acknowledges receipt, so transport connection
  // setup/teardown cannot silently omit the first or last comparison frame.
  const bool require_observer = nh.param<bool>("/test/livox_scan/require_observer", false);
  auto await_observer = [&](int sequence) {
    if (!require_observer) return;
    for (int i = 0; i < 100; ++i) {
      if (nh.param<int>("/test/livox_scan/observed_frames", -1) >= sequence) return;
      if (sequence == 0) wrapper.pub_scan_body(empty, ros::Time(1));
      pump(wrapper);
    }
    assert(false && "external scan capture did not acknowledge frame");
  };
  await_observer(0);
  int frame_sequence = 0;

  livox_ros_driver2::CustomMsg scene;
  scene.header.frame_id = "base_laser_link";
  scene.header.stamp = ros::Time(100);
  const float xyz[][3] = {{0, -2, 0}, {1, 0, 0}, {3, 0, 0}, {0, 0.55f, 0},
                          {0, 2, 0}, {-2, 0.05f, 0}, {0, -1, 0},
                          {2, 0, 1}, {4, 0, 0}, {2, 2, 0}};
  const std::uint8_t tags[] = {0, 0x04, 0, 0, 0x08, 0x0c, 0x04, 0, 0x20, 0x08};
  for (unsigned i = 0; i < 10; ++i) {
    livox_ros_driver2::CustomPoint p;
    p.x = xyz[i][0]; p.y = xyz[i][1]; p.z = xyz[i][2];
    p.tag = tags[i]; p.reflectivity = 10 + i; p.offset_time = i * 1000000;
    scene.points.push_back(p);
  }
  scene.point_num = scene.points.size();

  auto replay = [&](const livox_ros_driver2::CustomMsg& message, bool enabled) {
    wrapper.clear();
    latest.reset();
    g_scan_filter_livox_dust = enabled;
    filter->SetX(SysState(99.9));
    lidar_pub.publish(message);
    pump(wrapper);
    sensor_msgs::Imu imu;
    imu.linear_acceleration.z = 9.8;
    for (double stamp : {99.999, 100.02}) {
      imu.header.stamp.fromSec(stamp);
      imu_pub.publish(imu);
      pump(wrapper);
    }
    assert(latest && latest->header.stamp == message.header.stamp);
    assert(latest->header.frame_id == "base_laser_link");
    MeasureGroup meas;
    assert(wrapper.sync_measure(meas));
    assert(meas.lidar.pc->size() == 4);
    assert(meas.lidar.start_time == 100.0);
    assert(std::abs(meas.lidar.end_time - 100.009) < 1e-9);
    await_observer(++frame_sequence);
    return meas;
  };

  const auto baseline = replay(scene, false);
  assert(std::abs(rangeAt(*latest, 0.0) - 1.0) < 1e-5);
  assert(std::abs(rangeAt(*latest, -M_PI / 2) - 1.0) < 1e-5);
  const auto filtered = replay(scene, true);
  assert(std::abs(rangeAt(*latest, 0.0) - 3.0) < 1e-5);  // Farther normal obstacle.
  assert(std::abs(rangeAt(*latest, M_PI / 2) - 0.55) < 1e-5);  // Normal near obstacle.
  assert(std::isinf(rangeAt(*latest, -M_PI / 2)));
  assert(std::abs(rangeAt(*latest, std::atan2(0.05f, -2.0f)) - std::hypot(2.0, 0.05)) < 1e-5);
  for (std::size_t i = 0; i < baseline.lidar.pc->size(); ++i) {
    const auto& before = baseline.lidar.pc->at(i);
    const auto& after = filtered.lidar.pc->at(i);
    assert(before.x == after.x && before.y == after.y && before.z == after.z);
    assert(before.intensity == after.intensity && before.offset_time == after.offset_time);
    assert(std::abs(after.offset_time - i * 0.003) < 1e-9);
  }
  assert(baseline.lidar.start_time == filtered.lidar.start_time);
  assert(baseline.lidar.end_time == filtered.lidar.end_time);

  auto all_dust = scene;
  for (auto& p : all_dust.points) p.tag = 0x04;
  replay(all_dust, true);
  for (float range : latest->ranges) assert(std::isinf(range));

  // Exercise the unchanged LaserScan projection boundaries directly.
  CloudPtr bounds(new PointCloudType());
  const float positions[][3] = {{0.1999f, 0, 0}, {0.2f, 0, 0}, {0, 40, 0},
                                {30, 30, 0}, {-2, 0.2f, std::nextafter(0.2f, 0.0f)},
                                {-1, -1, 0.2001f}, {-2, -0.001f, 0},
                                {-1, -0.0f, 0}, {-1, 0.0f, 0}};
  for (const auto& pos : positions) {
    PointType p; p.x = pos[0]; p.y = pos[1]; p.z = pos[2]; p.intensity = 0;
    bounds->push_back(p);
  }
  latest.reset();
  wrapper.pub_scan_body(bounds, ros::Time(101));
  pump(wrapper);
  assert(latest && latest->header.stamp == ros::Time(101));
  assert(std::abs(rangeAt(*latest, 0.0) - 0.2) < 1e-5);
  assert(rangeAt(*latest, M_PI / 2) == 40.0f);
  assert(std::isinf(rangeAt(*latest, M_PI / 4)));
  assert(std::isinf(rangeAt(*latest, -3 * M_PI / 4)));
  assert(std::abs(rangeAt(*latest, std::atan2(0.2f, -2.0f)) - std::hypot(2.0, 0.2)) < 1e-5);
  assert(std::abs(latest->ranges.front() - 2.0f) < 1e-5 && std::isinf(latest->ranges.back()));
  // Legacy function-static publishers outlive wrapper's private callback
  // queue. Stop ROS while that queue is alive, before local/static teardown.
  ros::shutdown();
  std::cout << "Livox ROS replay: scan filtering/projection and unchanged LIO points/timing passed" << std::endl;
}
