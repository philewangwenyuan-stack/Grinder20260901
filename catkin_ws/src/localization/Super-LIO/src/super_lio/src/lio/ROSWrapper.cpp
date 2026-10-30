
#include "lio/ROSWrapper.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "lio/base_kinematics.h"
#include "lio/livox_scan_filter.h"
#include "super_lio/CloudPose.h"
#include "super_lio/CloudPose2.h"
#include "super_lio/DoubleCloudPose.h"

using namespace BASIC;

namespace LI2Sup{

void LoadParamFromRos(ros::NodeHandle& nh){
  nh.getParam("/lio/map/save_map", g_save_map);
  LOG(INFO) << GREEN << " ---> [Param] map/save_map: " << (g_save_map ? "true" : "false") << RESET;
  nh.getParam("/lio/map/if_filter", g_if_filter);
  nh.getParam("/lio/map/save_map_dir", g_save_map_dir);
  // Keep deployment map directories usable without copying maps into the
  // source tree. Relative paths remain package-root relative for backward
  // compatibility; absolute paths are used as provided.
  if (g_save_map_dir.empty() || g_save_map_dir.front() != '/') {
    g_save_map_dir = g_root_dir + g_save_map_dir;
  }
  nh.getParam("/lio/map/map_name", g_map_name);
  nh.getParam("/lio/map/ds_size", g_map_ds_size);
  nh.getParam("/lio/map/save_interval", g_pcd_save_interval);
  nh.param<double>("/lio/map/save_min_translation", g_map_save_min_translation, 0.10);
  double save_min_rotation_deg = 2.0;
  nh.param<double>("/lio/map/save_min_rotation_deg", save_min_rotation_deg, 2.0);
  g_map_save_min_rotation_rad = std::max(0.0, save_min_rotation_deg) * M_PI / 180.0;
  nh.param<double>("/lio/map/save_min_interval_sec", g_map_save_min_interval_sec, 0.5);
  nh.param<int>("/lio/map/save_min_points", g_map_save_min_points, 500);
  nh.param<int>("/lio/map/save_motion_confirm_frames", g_map_save_motion_confirm_frames, 2);
  g_map_save_min_translation = std::max(0.0, g_map_save_min_translation);
  g_map_save_min_interval_sec = std::max(0.0, g_map_save_min_interval_sec);
  g_map_save_min_points = std::max(1, g_map_save_min_points);
  g_map_save_motion_confirm_frames = std::max(1, g_map_save_motion_confirm_frames);
  
  // ROS Topic input
  nh.getParam("/lio/ros/lidar_topic",  g_lidar_topic);
  nh.getParam("/lio/ros/imu_topic",    g_imu_topic);
  nh.getParam("/lio/ros/odom_topic",   g_odom_topic);
  nh.getParam("/lio/ros/visual_cloud_topic", g_visual_cloud_topic);
  nh.getParam("/lio/ros/visual_planner_cloud_topic", g_visual_planner_cloud_topic);
  nh.param<std::string>("/lio/ros/world_frame", g_world_frame, "map");
  nh.param<std::string>("/lio/ros/base_frame", g_base_frame, "base_laser_link");
  nh.param<bool>("/lio/ros/legacy_tf", g_legacy_tf, true);
  nh.param<bool>("/lio/scan/filter_livox_dust", g_scan_filter_livox_dust, true);
  ROS_INFO("Livox /scan dust tag filter: %s (LIO input unchanged)",
           g_scan_filter_livox_dust ? "enabled" : "disabled");
  nh.getParam("/lio/ros/init_pose_topic", g_init_pose_topic);
  nh.getParam("/lio/ros/map_topic", g_map_topic);
  nh.getParam("/lio/ros/path_topic", g_path_topic);

  // sensor cfg
  nh.getParam("/lio/sensor/lidar_type", g_lidar_type);
  double temp_range_dis;
  nh.getParam("/lio/sensor/blind", temp_range_dis);
  g_blind2 = temp_range_dis * temp_range_dis;
  nh.getParam("/lio/sensor/maxrange", temp_range_dis);
  g_maxrange2 = temp_range_dis * temp_range_dis;
  nh.getParam("/lio/sensor/filter_rate", g_filter_rate);
  nh.getParam("/lio/sensor/enable_downsample", g_enable_downsample);
  nh.getParam("/lio/sensor/voxel_fliter_size", g_voxel_fliter_size);

  nh.getParam("/lio/sensor/gravity_norm", g_gravity_norm);
  nh.getParam("/lio/sensor/imu_type", g_imu_type);
  nh.getParam("/lio/sensor/imu_na",   g_imu_na);
  nh.getParam("/lio/sensor/imu_ng",   g_imu_ng);
  nh.getParam("/lio/sensor/imu_nba",  g_imu_nba);
  nh.getParam("/lio/sensor/imu_nbg",  g_imu_nbg);

  // extrinsic
  std::vector<scalar> extrinsic_lidar_imu, extrinsic_odom_robo;
  nh.getParam("/lio/extrinsic/lidar_imu", extrinsic_lidar_imu);      // 3 + 9
  V3 __t = V3(extrinsic_lidar_imu[0], 
              extrinsic_lidar_imu[1], 
              extrinsic_lidar_imu[2]);
  M3 __R = M3(extrinsic_lidar_imu.data() + 3);
  g_lidar_imu = SE3(__R, __t);  // lidar in imu frame·
  // The same parameters drive base_link -> base_laser_link static TF.
  // Unlike odom_robo's historical degree convention, these angles are radians.
  double mounting[6];
  const char* mounting_names[] = {"x", "y", "z", "roll", "pitch", "yaw"};
  for (int i = 0; i < 6; ++i) {
    nh.param<double>(std::string("/lio/extrinsic/base_to_laser_") + mounting_names[i],
                     mounting[i], 0.0);
    if (!std::isfinite(mounting[i])) {
      throw std::invalid_argument("Non-finite base_to_laser calibration");
    }
  }
  const auto base_laser = base_kinematics::fromXyzRpy(
      Eigen::Vector3d(mounting[0], mounting[1], mounting[2]),
      Eigen::Vector3d(mounting[3], mounting[4], mounting[5]));
  base_kinematics::Transform imu_laser = base_kinematics::Transform::Identity();
  imu_laser.linear() = g_lidar_imu.R_.cast<double>();
  imu_laser.translation() = g_lidar_imu.t_.cast<double>();
  const auto imu_base = imu_laser * base_laser.inverse();
  const M3 imu_base_rotation = imu_base.linear().cast<scalar>();
  const V3 imu_base_translation = imu_base.translation().cast<scalar>();
  g_imu_base = SE3(imu_base_rotation, imu_base_translation);
  if (g_legacy_tf) {
    ROS_WARN("legacy_tf=true: odometry/TF retain historical IMU pose; base mounting compensation is disabled");
  }
  nh.getParam("/lio/extrinsic/odom_robo", extrinsic_odom_robo);     // 3 + 3 x,y,z,r,p,y
  __t = V3(extrinsic_odom_robo[0], 
           extrinsic_odom_robo[1], 
           extrinsic_odom_robo[2]);
  auto temp_R = Eigen::AngleAxisd(extrinsic_odom_robo[5]/180 * M_PI, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(extrinsic_odom_robo[4]/180 * M_PI, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(extrinsic_odom_robo[3]/180 * M_PI, Eigen::Vector3d::UnitX());
  g_odom_robo.R_ = temp_R.cast<scalar>();

  /// ATTENTION:
  /// The transpose is intentionally applied here and represents the inverse of R.
  /// Misinterpreting this will lead to incorrect transformations.
  M3 _R = g_odom_robo.R_.transpose();
  g_odom_robo.R_  = _R;
  g_odom_robo = SE3(_R, __t);  // lidar in robot frame

  auto temp_R_yaw = Eigen::AngleAxisd(extrinsic_odom_robo[5]/180 * M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  g_lidar_robo_yaw = temp_R_yaw.cast<scalar>();

  // hash_map
  int hash_capacity;
  nh.getParam("/lio/hash_map/hash_capacity", hash_capacity);
  g_ivox_capacity = hash_capacity;
  nh.getParam("/lio/hash_map/vox_resolution", g_ivox_resolution);
  nh.getParam("/lio/hash_map/insert_resolution", g_insert_resolution);
  
  // kf
  nh.getParam("/lio/kf/kf_type", g_kf_type);
  nh.getParam("/lio/kf/kf_max_iterations", g_kf_max_iterations);
  nh.getParam("/lio/kf/kf_align_gravity", g_kf_align_gravity);
  nh.getParam("/lio/kf/kf_quit_eps", g_kf_quit_eps);

  // submaps
  nh.getParam("/lio/submap/submap_resolution", g_submap_resolution);
  nh.getParam("/lio/submap/submap_capacity", g_submap_capacity);

  // visual
  nh.getParam("/lio/output/robot",  g_2_robot);
  nh.getParam("/lio/output/planner", g_planner_enable);
  nh.getParam("/lio/output/plan_env_world",  g_2_plan_env_world);
  nh.getParam("/lio/output/plan_env_body",  g_2_plan_env_body);
  nh.getParam("/lio/output/ml_map",         g_2_ml_map);
  nh.getParam("/lio/output/map",    g_visual_map);
  nh.getParam("/lio/output/dense",  g_visual_dense);
  nh.getParam("/lio/output/pub_step", g_pub_step);

  g_update_map = false;
  nh.getParam("/lio/relocation/update_map", g_update_map);
  std::vector<float> init_pose;
  nh.getParam("/lio/relocation/init_pose", init_pose);
  if(init_pose.size() == 6){
    g_init_px    = init_pose[0];
    g_init_py    = init_pose[1];
    g_init_pz    = init_pose[2];
    g_init_roll  = init_pose[3];
    g_init_pitch = init_pose[4];
    g_init_yaw   = init_pose[5];
  }else{
    g_init_px = 0.0f;
    g_init_py = 0.0f;
    g_init_pz = 0.0f;
    g_init_roll = 0.0f;
    g_init_pitch = 0.0f;
    g_init_yaw = 0.0f;
  }
  
}

std::tuple<float, float, float> getColorFromVelocity(float velocity, float max_velocity = 2.0f) {
  float ratio = std::clamp(velocity / max_velocity, 0.0f, 1.0f);
  float r, g, b;
  if (ratio < 0.5f) {
    float t = ratio / 0.5f;
    r = 0.0f;
    g = t;
    b = 1.0f;
  } else {
    float t = (ratio - 0.5f) / 0.5f;
    r = t;
    g = 1.0f - t;
    b = 1.0f - t;
  }
  return {r, g, b};
}


void livox2pcl(const livox_ros_driver2::CustomMsg::ConstPtr& msg, CloudPtr& point_cloud){
  point_cloud->clear();
  point_cloud->reserve(msg->points.size());
  const auto stats = livox_scan_filter::forEachPoint(
      msg->points, msg->point_num, g_scan_filter_livox_dust, g_blind2, g_maxrange2,
      [&](const auto& point) {
        PointType converted;
        converted.x = point.x;
        converted.y = point.y;
        converted.z = point.z;
        converted.intensity = point.reflectivity;
        point_cloud->push_back(converted);
      });
  // Accumulate outside the traversal. Wall time also works for paused /clock.
  static livox_scan_filter::Statistics window;
  static ros::WallTime last_report = ros::WallTime::now();
  window.input_points += stats.input_points;
  window.dust_rejected += stats.dust_rejected;
  window.reserved_dust += stats.reserved_dust;
  const auto now = ros::WallTime::now();
  if ((now - last_report).toSec() >= 5.0) {
    ROS_INFO_STREAM("Livox /scan tag filter enabled=" << g_scan_filter_livox_dust
                    << " input_points=" << window.input_points
                    << " dust_tag_rejected=" << window.dust_rejected
                    << " reserved_dust_tag_points=" << window.reserved_dust);
    window = {};
    last_report = now;
  }
}


std::string lidarTypeToString(int type) {
  if (type <= 0 || type >= static_cast<int>(LID_TYPE_NAMES.size())) return "UNKNOWN";
  return LID_TYPE_NAMES[type];
}

ROSWrapper::ROSWrapper(){
  ros::SubscribeOptions ops;
  ops.transport_hints = ros::TransportHints().tcpNoDelay();
  
  if(g_lidar_type == LID_TYPE::LIVOX){
    ops.init<livox_ros_driver2::CustomMsg>(
      g_lidar_topic, 1000, 
      boost::bind(&ROSWrapper::livoxHandler, this, _1));
  }else{
    ops.init<sensor_msgs::PointCloud2>(
      g_lidar_topic, 1000,
      boost::bind(&ROSWrapper::stdMsgHandler, this, _1));
  }

  LOG(INFO) << GREEN << " ---> Using Lidar type: " << lidarTypeToString(g_lidar_type) << RESET;

  nh_.setCallbackQueue(&self_queue_);
  setupWheelFusion();

  subLidar_ = nh_.subscribe(ops);
  subIMU_   = nh_.subscribe<sensor_msgs::Imu>(g_imu_topic, 10000,    // 100Hz x 10s
               &ROSWrapper::imuHandler, this, ros::TransportHints().tcpNoDelay());

  srv_get_map_ = nh_.advertiseService("/lio/get_mapmeta", &ROSWrapper::GetMapCallback, this);
  srv_get_3dmap_ = nh_.advertiseService("/lio/get_3dmap", &ROSWrapper::Get3DMapCallback, this);
  srv_set_init_state_ = nh_.advertiseService("/lio/set_init_state", 
                  &ROSWrapper::SetInitStateCallback, this);

  /// output
  pub_odom_      = nh_.advertise<nav_msgs::Odometry>(g_odom_topic, 100);  /// imu frame -> lidar freq
  pub_path_      = nh_.advertise<nav_msgs::Path>(g_path_topic, 1);
  pub_path_robot_ = nh_.advertise<sensor_msgs::PointCloud2>("/path_robot", 1);

  pub_odom_x_      = nh_.advertise<std_msgs::Float32>("/lio/odom_x", 100); 
  pub_odom_y_     = nh_.advertise<std_msgs::Float32>("/lio/odom_y", 100); 
  pub_odom_z_      = nh_.advertise<std_msgs::Float32>("/lio/odom_z", 100); 
  
  msg_path_point_.header.frame_id = g_world_frame;
  msg2uav_.header.frame_id = g_world_frame;
  path_.header.frame_id = g_world_frame;

  cloud_map_.reset(new PointCloudType());
}


void ROSWrapper::livoxHandler(const livox_ros_driver2::CustomMsg::ConstPtr& msg){
  if(msg->point_num < 10) return;

  BASIC::CloudPtr cloud_scan(new PointCloudType());
  livox2pcl(msg, cloud_scan);
  pub_scan_body(cloud_scan, msg->header.stamp);
  LidarData lidar_data;
  std::size_t ptsize = msg->point_num;
  lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());
  lidar_data.pc->reserve(ptsize / g_filter_rate + 1);

  double offset_time = 0.0;
  for(std::size_t _i = 0; _i < ptsize; _i++){
    if (_i % g_filter_rate != 0) {
      continue;
    }
    auto& pt = msg->points[_i];
    auto tag = pt.tag & 0x30;
    if (tag == 0x10 || tag == 0x00){
      auto dis = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
      if(dis > g_blind2 && dis < g_maxrange2){
        offset_time = pt.offset_time * 1e-9;
        lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.reflectivity, offset_time);
      }
    }
  }
  lidar_data.start_time = msg->header.stamp.toSec();
  lidar_data.end_time   = lidar_data.start_time + offset_time;
  lidar_buffer_.push_back(lidar_data);
}


void ROSWrapper::stdMsgHandler(const sensor_msgs::PointCloud2::ConstPtr& msg){
  if(msg->data.size() < 10) return;
  
  LidarData lidar_data;
  lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());

  double offset_time = 0.0;
  double dis = 0.0;

  switch (g_lidar_type) {

  case LID_TYPE::HESAI16:
  {
    pcl::PointCloud<hesai_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    const double time_begin = pl_orig.points[0].timestamp;
    lidar_data.start_time = time_begin;
    lidar_data.end_time = pl_orig.points.back().timestamp;
    for(std::size_t i = 0; i < pl_orig.size(); i++){
      if (i % g_filter_rate != 0) {
        continue;
      }
      auto& pt = pl_orig.points[i];
      if(pt.x == NAN || pt.y == NAN || pt.z == NAN){
        continue;
      }
      dis = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
      if (dis > g_blind2 && dis < g_maxrange2) {
        offset_time = pt.timestamp - time_begin;
        lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.intensity, offset_time);
      }
    }
    break;
  }
  case LID_TYPE::VEL_NCLT:
  {
    pcl::PointCloud<NCLT::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = msg->header.stamp.toSec();
    
    for(std::size_t i = 0; i < pl_orig.size(); i++){
      if (i % g_filter_rate != 0) {
        continue;
      }
      auto& pt = pl_orig.points[i];
      if(pt.x == NAN || pt.y == NAN || pt.z == NAN){
        continue;
      }
      dis = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
      if (dis > g_blind2 && dis < g_maxrange2) {
        offset_time = pt.time * 1e-6;
        lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, 1, offset_time);
      }
    }
    lidar_data.end_time = lidar_data.start_time + offset_time;
    break;
  }
  case LID_TYPE::VELO16:
  {
    break;
  }
  case LID_TYPE::VELO32:
  {
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = msg->header.stamp.toSec();
    for(std::size_t i = 0; i < pl_orig.size(); i++){
      if (i % g_filter_rate != 0) {
        continue;
      }
      auto& pt = pl_orig.points[i];
      if(pt.x == NAN || pt.y == NAN || pt.z == NAN){
        continue;
      }
      dis = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
      if (dis > g_blind2 && dis < g_maxrange2) {
        lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.intensity, pt.time);
      }
    }
    lidar_data.end_time = lidar_data.start_time + lidar_data.pc->points.back().offset_time;
    break;
  }
  case LS16:
  {
    break;
  }
  case OUSTER:
  {
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = msg->header.stamp.toSec();

    for(std::size_t i = 0; i < pl_orig.size(); i++){
      if (i % g_filter_rate != 0) {
        continue;
      }
      auto& pt = pl_orig.points[i];
      if(pt.x == NAN || pt.y == NAN || pt.z == NAN){
        continue;
      }

      dis = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
      if (dis > g_blind2 && dis < g_maxrange2) {
        offset_time = pt.t * 1e-9;  // ns to s
        lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.intensity, offset_time);
      }
    }
    lidar_data.end_time = lidar_data.start_time + offset_time;
    break;
  }
  default:
    return;
  }
  
  lidar_buffer_.push_back(lidar_data);
}


void ROSWrapper::imuHandler(const sensor_msgs::Imu::ConstPtr& msg){
  IMUData data;
  data.secs = msg->header.stamp.toSec();
  data.acc = V3(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  // if(g_imu_type == 1){
  //   data.acc = g_gravity_norm * V3(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  // }else{
  //   data.acc = V3(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
  // }
  data.gyr = V3(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  imu_buffer_.push_back(data);

  

  if (data.secs < last_timestamp_imu_) {
    LOG(WARNING) << "imu loop back, clear buffer";
    imu_buffer_.clear();
    resetWheelFusion();
  }
  last_timestamp_imu_ = data.secs;

  nav_msgs::Odometry odom, odom1;
  // static ros::Publisher pub_imu_odom  = nh_.advertise<nav_msgs::Odometry>("/lio/imu/odom", 10);    /// imu frame -> imu freq
  static ros::Publisher pub_robo_odom = nh_.advertise<nav_msgs::Odometry>("/lio/robo/odom", 10);   /// robot frame -> imu freq
  if(eskf_->Predict(data, odom, odom1)){
    // odom.header.stamp = msg->header.stamp;
    odom1.header.stamp = msg->header.stamp;
    // odom.header.frame_id = "map";
    odom1.header.frame_id = "map";
    // pub_imu_odom.publish(odom);
    pub_robo_odom.publish(odom1);
  }
}

void ROSWrapper::get3DCloud(BASIC::CloudPtr& pc){;
  *cloud_map_ = *pc;
}

bool ROSWrapper::Get3DMapCallback(super_lio::GetMap::Request& req,
                        super_lio::GetMap::Response& res)
{
  LOG(INFO) << "Get3DMap: Request received.";
  std::string loc_map_name = req.map_dir + "/loc_" + req.map_name + ".pcd";
  std::string plan_map_name = req.map_dir + "/plan_" + req.map_name + ".pcd";
  pcl::VoxelGrid<PointType> voxel_fliter;
    PointCloudType latst_map;
    voxel_fliter.setInputCloud(cloud_map_);
    voxel_fliter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
    voxel_fliter.filter(latst_map);
    if(latst_map.size() > 0){
      latst_map.width = latst_map.size();
      latst_map.height = 1;
      latst_map.is_dense = false;
    }
    if (cloud_map_->size() > 0){
      cloud_map_->width = cloud_map_->size();
      cloud_map_->height = 1;
      cloud_map_->is_dense = false;
    }
    pcl::io::savePCDFileBinary(loc_map_name, latst_map);
    pcl::io::savePCDFileBinary(plan_map_name, *cloud_map_);
    LOG(INFO) << GREEN << " ---> Save map success. File: " << loc_map_name << " " << plan_map_name << RESET;
    LOG(INFO) << GREEN << " ---> Map size: " << latst_map.size() << " " << cloud_map_->size() << RESET;
  res.success = true;
  return res.success;
}


bool ROSWrapper::GetMapCallback(super_lio::GetMap::Request& req,
                    super_lio::GetMap::Response& res)
{
  LOG(INFO) << "GetMapMeta: Request received.";

  ivox_->getMap(res.map_data);
  if(res.map_data.empty()){
    res.success = false;
    LOG(WARNING) << RED << "GetMap: Map data is empty." << RESET;
    return true;
  }
  else{
    res.success = true;
    LOG(INFO) << GREEN << "GetMap: Map data retrieved successfully." << RESET;
    return true;
  }
}


bool ROSWrapper::SetInitStateCallback(super_lio::SetInitState::Request& req,
                        super_lio::SetInitState::Response& res)
{
  LOG(INFO) << "SetInitState: Request received.";
  LOG(INFO) << "req.pose.size(): " << req.pose.size();
  LOG(INFO) << "req.map_data.size(): " << req.map_data.size();
  LOG(INFO) << "req.map_data.empty(): " << req.map_data.empty();
  
  if(req.pose.size() != 12 || req.map_data.empty()){
    res.success = false;
    LOG(WARNING) << RED << "SetInitState: Invalid request data. " 
                 << "pose.size()=" << req.pose.size() 
                 << ", map_data.empty()=" << req.map_data.empty() 
                 << ", map_data.size()=" << req.map_data.size() << RESET;
  }else{
    ivox_->resetMap(req.map_data);
    V3 _t;  M3 rot;

    _t[0] = req.pose[0];
    _t[1] = req.pose[1];
    _t[2] = req.pose[2];
    rot(0, 0) = req.pose[3];
    rot(0, 1) = req.pose[4];
    rot(0, 2) = req.pose[5];
    rot(1, 0) = req.pose[6];
    rot(1, 1) = req.pose[7];
    rot(1, 2) = req.pose[8];
    rot(2, 0) = req.pose[9];
    rot(2, 1) = req.pose[10];
    rot(2, 2) = req.pose[11];

    LOG(INFO) << "Extracted position: (" << _t[0] << ", " << _t[1] << ", " << _t[2] << ")";
    LOG(INFO) << "Extracted rotation matrix:";
    LOG(INFO) << "  [" << rot(0,0) << ", " << rot(0,1) << ", " << rot(0,2) << "]";
    LOG(INFO) << "  [" << rot(1,0) << ", " << rot(1,1) << ", " << rot(1,2) << "]";
    LOG(INFO) << "  [" << rot(2,0) << ", " << rot(2,1) << ", " << rot(2,2) << "]";

    if(rot.determinant() < 0){
      LOG(WARNING) << RED << " ---> [SuperLIO]: The rotation matrix is not valid. Resetting to identity." << RESET;
      rot.setIdentity();
    }

    /// input: robot_self：map <-> robot_other: map
    SE3 input_pose_se3(rot, _t);
    auto state = eskf_->GetSysState();
    SE3 current_pose = eskf_->GetSE3();
    SE3 final_pose = input_pose_se3 * current_pose;
    state.p = final_pose.t();
    state.R = final_pose.so3();
    eskf_->SetX(state);
    resetWheelFusion();

    res.success = true;
    LOG(INFO) << GREEN << "SetInitState: Map reset successfully." << RESET;
  }
  return true;
}


bool ROSWrapper::sync_measure(MeasureGroup& meas){
  if (lidar_buffer_.empty() || imu_buffer_.empty()) {
    return false;
  }

  /*** push a lidar scan ***/
  if (!lidar_pushed_) {
    meas.lidar = lidar_buffer_.front();
    lidar_pushed_ = true;
  }

  if(last_timestamp_lidar_ > meas.lidar.end_time){
    lidar_buffer_.pop_front();
    lidar_pushed_ = false;
    return false;
  }

  if (last_timestamp_imu_ < meas.lidar.end_time) {
    return false;
  }

  /*** push imu_ data, and pop from imu_ buffer ***/
  double imu_time = imu_buffer_.front().secs;
  meas.imu.clear();
  while ((!imu_buffer_.empty()) && (imu_time < meas.lidar.end_time)) {
    imu_time = imu_buffer_.front().secs;
    if (imu_time > meas.lidar.end_time) break;
    meas.imu.push_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }

  last_timestamp_lidar_ = meas.lidar.end_time;
  lidar_buffer_.pop_front();
  lidar_pushed_ = false;
  // Keep the first IMU sample after the scan as an interpolation bracket;
  // leave it queued for the next scan as well. Disabled path is unchanged.
  if (wheel_enabled_ && !imu_buffer_.empty()) meas.imu.push_back(imu_buffer_.front());
  return true;
}


void ROSWrapper::pub_odom(const NavState& state, bool re_path){
  nav_msgs::Odometry odom;
  odom.header.frame_id = g_world_frame;
  odom.child_frame_id = g_legacy_tf ? "odom" : g_base_frame;
  odom.header.stamp = ros::Time().fromSec(state.timestamp);
  odom.pose.pose.position.x = state.p[0];
  odom.pose.pose.position.y = state.p[1];
  odom.pose.pose.position.z = state.p[2];

  V4 temp_q = state.R.coeffs();
  odom.pose.pose.orientation.x = temp_q[0];
  odom.pose.pose.orientation.y = temp_q[1];
  odom.pose.pose.orientation.z = temp_q[2];
  odom.pose.pose.orientation.w = temp_q[3];

  odom.twist.twist.linear.x = state.v[0];
  odom.twist.twist.linear.y = state.v[1];
  odom.twist.twist.linear.z = state.v[2];

  if (!g_legacy_tf) {
    base_kinematics::Transform world_imu = base_kinematics::Transform::Identity();
    world_imu.linear() = state.R.R_.cast<double>();
    world_imu.translation() = state.p.cast<double>();
    base_kinematics::Transform imu_base = base_kinematics::Transform::Identity();
    imu_base.linear() = g_imu_base.R_.cast<double>();
    imu_base.translation() = g_imu_base.t_.cast<double>();
    base_kinematics::Covariance covariance = base_kinematics::Covariance::Zero();
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();
    Eigen::Matrix3d gyro_covariance = Eigen::Matrix3d::Identity()*g_imu_ng;
    base_kinematics::StateGyroCross state_gyro_cross = base_kinematics::StateGyroCross::Zero();
    if (eskf_) {
      covariance = eskf_->GetCov().cast<double>();
      omega = eskf_->GetBodyAngularVelocity().cast<double>();
      if (wheel_enabled_) {
        gyro_covariance = eskf_->GetGyroNoiseCovariance();
        state_gyro_cross = eskf_->GetStateGyroCross();
      }
    }
    const auto output = base_kinematics::convert(world_imu, imu_base,
        state.v.cast<double>(), omega, covariance,
        gyro_covariance, state_gyro_cross);
    odom.pose.pose.position.x = output.pose.translation().x();
    odom.pose.pose.position.y = output.pose.translation().y();
    odom.pose.pose.position.z = output.pose.translation().z();
    const Eigen::Quaterniond quaternion(output.pose.linear());
    temp_q = quaternion.coeffs().cast<scalar>();
    odom.pose.pose.orientation.x = quaternion.x();
    odom.pose.pose.orientation.y = quaternion.y();
    odom.pose.pose.orientation.z = quaternion.z();
    odom.pose.pose.orientation.w = quaternion.w();
    odom.twist.twist.linear.x = output.linear.x();
    odom.twist.twist.linear.y = output.linear.y();
    odom.twist.twist.linear.z = output.linear.z();
    odom.twist.twist.angular.x = output.angular.x();
    odom.twist.twist.angular.y = output.angular.y();
    odom.twist.twist.angular.z = output.angular.z();
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < 6; ++col) {
        odom.pose.covariance[row * 6 + col] = output.pose_covariance(row, col);
        odom.twist.covariance[row * 6 + col] = output.twist_covariance(row, col);
      }
    }
  }

  // Historical legacy covariance mapping only. Standard mode above propagates
  // the full ESKF covariance into world pose and base-frame twist coordinates.
  // ESKF error-state order is [rotation, position, velocity, gyro bias,
  // accelerometer bias, gravity]. ROS pose covariance order is
  // [x, y, z, roll, pitch, yaw], so copy the corresponding blocks instead of
  // publishing the all-zero default (which incorrectly means perfect certainty).
  if (eskf_ && g_legacy_tf) {
    const ESKF::COV covariance = eskf_->GetCov();
    auto finite_covariance = [](const BASIC::scalar value) {
      return std::isfinite(value) ? static_cast<double>(value) : 0.0;
    };
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        odom.pose.covariance[row * 6 + col] =
            finite_covariance(covariance(3 + row, 3 + col));
        odom.pose.covariance[row * 6 + (3 + col)] =
            finite_covariance(covariance(3 + row, col));
        odom.pose.covariance[(3 + row) * 6 + col] =
            finite_covariance(covariance(row, 3 + col));
        odom.pose.covariance[(3 + row) * 6 + (3 + col)] =
            finite_covariance(covariance(row, col));
        odom.twist.covariance[row * 6 + col] =
            finite_covariance(covariance(6 + row, 6 + col));
      }
    }
    // Angular velocity is not part of the nominal navigation state. Mark it as
    // deliberately uncertain rather than fabricating confidence from gyro-bias
    // covariance.
    odom.twist.covariance[3 * 6 + 3] = 1e3;
    odom.twist.covariance[4 * 6 + 4] = 1e3;
    odom.twist.covariance[5 * 6 + 5] = 1e3;
  }

  pub_odom_.publish(odom);    // imu frame -> lidar frequency
  // Legacy mode preserves the original map->odom + identity odom->base chain.
  // Standard mode publishes continuous odom->base motion so a loop backend can
  // own map->odom without creating competing TF authorities.
  tf::Transform tf_tf, tf_tf2;
  tf_tf.setOrigin(tf::Vector3(odom.pose.pose.position.x,
      odom.pose.pose.position.y, odom.pose.pose.position.z));
  tf_tf.setRotation(tf::Quaternion(temp_q[0], temp_q[1], temp_q[2], temp_q[3]));

  tf_tf2.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf_tf2.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

  if (g_legacy_tf) {
    br_.sendTransform(tf::StampedTransform(
        tf_tf, odom.header.stamp, g_world_frame, "odom"));
    br_.sendTransform(tf::StampedTransform(
        tf_tf2, odom.header.stamp, "odom", g_base_frame));
  } else {
    br_.sendTransform(tf::StampedTransform(
        tf_tf, odom.header.stamp, g_world_frame, g_base_frame));
  }


  std_msgs::Float32 odom_x;
  std_msgs::Float32 odom_y;
  std_msgs::Float32 odom_z;
  odom_x.data = odom.pose.pose.position.x;
  odom_y.data = odom.pose.pose.position.y;
  odom_z.data = odom.pose.pose.position.z;
  pub_odom_x_.publish(odom_x);
  pub_odom_y_.publish(odom_y);
  pub_odom_z_.publish(odom_z);

  V3 robo_position = state.R.R_ * ( - g_odom_robo.R_ * g_odom_robo.t_) + state.p;
  if (!g_legacy_tf) {
    robo_position = V3(odom.pose.pose.position.x, odom.pose.pose.position.y,
                       odom.pose.pose.position.z);
  }

  if(g_2_robot){
    static ros::Publisher pub_msg2uav_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    M3 robo_rotation = state.R.R_ * g_odom_robo.R_;
    if (!g_legacy_tf) robo_rotation = Quat(temp_q[3], temp_q[0], temp_q[1], temp_q[2]).toRotationMatrix();
    msg2uav_.header.stamp = odom.header.stamp;
    msg2uav_.pose.position.x = robo_position[0];
    msg2uav_.pose.position.y = robo_position[1];
    msg2uav_.pose.position.z = robo_position[2];
    Quat robo_quat(robo_rotation);
    msg2uav_.pose.orientation.w = robo_quat.w();
    msg2uav_.pose.orientation.x = robo_quat.x();
    msg2uav_.pose.orientation.y = robo_quat.y();
    msg2uav_.pose.orientation.z = robo_quat.z();
    pub_msg2uav_.publish(msg2uav_);
  }

  // if(1)
  if (re_path) {path_.poses.clear();};
  if((last_path_point_ - robo_position).norm() > 0.1)
  {
    /// nav_msgs::Path
    path_.header.stamp = odom.header.stamp;
    geometry_msgs::PoseStamped point;
    point.header = odom.header;
    point.pose = odom.pose.pose;
    path_.poses.push_back(point);
    pub_path_.publish(path_);

    /// sensor_msgs::PointCloud2
    // pcl::PointXYZRGB point_robot;
    // point_robot.x = robo_position[0];
    // point_robot.y = robo_position[1];
    // point_robot.z = robo_position[2];
    // auto [r, g, b] = getColorFromVelocity(state.v.norm());
    // point_robot.r = static_cast<uint8_t>(r * 255);
    // point_robot.g = static_cast<uint8_t>(g * 255);
    // point_robot.b = static_cast<uint8_t>(b * 255);
    // point_robot.a = 255;
    // pcl::PointCloud<pcl::PointXYZRGB> path_point_;
    // path_point_.push_back(point_robot);
    // pcl::toROSMsg(path_point_, msg_path_point_);
    // msg_path_point_.header.stamp = odom.header.stamp;
    // msg_path_point_.header.frame_id = "map";
    // pub_path_robot_.publish(msg_path_point_);

    last_path_point_ = robo_position;
  }

  // tf::Transform transform;
  // tf::Quaternion q;
  // transform.setOrigin(tf::Vector3(state.p[0], state.p[1], state.p[2]));
  // q.setX(temp_q[0]);
  // q.setY(temp_q[1]);
  // q.setZ(temp_q[2]);
  // q.setW(temp_q[3]);
  // transform.setRotation(q);
  // br_.sendTransform(tf::StampedTransform(transform, odom.header.stamp, "map", "body"));

  // // Visual: for the best field of view in rviz.
  // q.setX(0);
  // q.setY(0);
  // q.setZ(0);
  // q.setW(1);
  // transform.setRotation(q);
  // br_.sendTransform(tf::StampedTransform(transform, odom.header.stamp, "map", "god"));
}

// void ROSWrapper::pub_cloud_body(const CloudPtr& pc,double time){
//   static ros::Publisher pub_cloud_body_ = nh_.advertise<sensor_msgs::PointCloud2>("/lio/cloud_body", 10);
//   sensor_msgs::PointCloud2 cloud;
//   pcl::toROSMsg(*pc, cloud);
//   cloud.header.frame_id = "body";
//   cloud.header.stamp = ros::Time().fromSec(time);
//   pub_cloud_body_.publish(cloud);
// }


void ROSWrapper::pub_cloud_world(const CloudPtr& world_pc,double time){
  static ros::Publisher pub_cloud_world_ = nh_.advertise<sensor_msgs::PointCloud2>
                                            (g_visual_cloud_topic, 10);
  sensor_msgs::PointCloud2 cloud;
  pcl::toROSMsg(*world_pc, cloud);
  cloud.header.frame_id = g_world_frame;
  cloud.header.stamp = ros::Time().fromSec(time);
  pub_cloud_world_.publish(cloud);
}


void ROSWrapper::pub_cloud2planner(const CloudPtr& robo_pc, double time){
  static ros::Publisher pub_cloud2robot_ = nh_.advertise<sensor_msgs::PointCloud2>
                                            ("/lio/robo/cloud_world", 10);
  sensor_msgs::PointCloud2 cloud;
  pcl::toROSMsg(*robo_pc, cloud);
  cloud.header.frame_id = g_world_frame;
  cloud.header.stamp = ros::Time().fromSec(time);
  pub_cloud2robot_.publish(cloud);
}

void ROSWrapper::pub_scan_body(const CloudPtr& lidar_pc,
                               const ros::Time& stamp){
  // 3d点云提取0高度点作为laserscan发布

  // ---------------- 可调参数 ----------------
  static double g_z        = 0;              // 高度切片(m)  -> "0高度点"              // 高度切片上界(m)  -> "0高度点"
  static double g_angle_min    = -M_PI;              // 扫描起始角
  static double g_angle_max    =  M_PI;              // 扫描结束角
  static double g_angle_inc    =  M_PI / 720.0;      // 角分辨率 0.25°
  static double g_range_min    =  0.2;
  static double g_range_max    =  40.0;
  static const char* g_scan_frame = "base_laser_link";         // 与点云 frame 保持一致

  const size_t N = static_cast<size_t>(
      std::lround((g_angle_max - g_angle_min) / g_angle_inc)) + 1;

  sensor_msgs::LaserScan scan;
  scan.header.stamp    = stamp;
  scan.header.frame_id = g_scan_frame;
  scan.angle_min       = g_angle_min;
  scan.angle_max       = g_angle_max;
  scan.angle_increment = g_angle_inc;
  scan.time_increment  = 0.0;
  scan.scan_time       = 0.0;   // 若用于 move_base，建议填实际帧间隔
  scan.range_min       = g_range_min;
  scan.range_max       = g_range_max;
  scan.ranges.assign(N, std::numeric_limits<float>::infinity());
  scan.intensities.assign(N, 0.0f);

  for (const auto& p : lidar_pc->points)
  {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;

    // 1) 高度过滤：只保留 z≈0 的薄片
    if (p.z < g_z - 0.2|| p.z > g_z + 0.2)
      continue;

    // 2) 距离过滤
    const double range = std::hypot(p.x, p.y);
    if (range < g_range_min || range > g_range_max)
      continue;

    // 3) 角度过滤 + 落桶
    const double ang = std::atan2(p.y, p.x);
    if (ang < g_angle_min || ang >= g_angle_max)
      continue;

    size_t idx = static_cast<size_t>((ang - g_angle_min) / g_angle_inc);
    if (idx >= N) idx = N - 1;

    // 同一角度多个点时取最近的（模拟激光遮挡，对避障更保守）
    if (range < scan.ranges[idx])
      scan.ranges[idx] = static_cast<float>(range);
  }
  static ros::Publisher pub_cloud2planner_ = nh_.advertise<sensor_msgs::LaserScan>
                                            (g_visual_planner_cloud_topic, 10);
  pub_cloud2planner_.publish(scan);
}


void ROSWrapper::pub_cloud_body_pose(const CloudPtr& pc, 
  const NavState& state)
{
  static ros::Publisher pub_output2robot_ = nh_.advertise<super_lio::CloudPose>
                                            ("/lio/body/cloud_pose", 10);
  super_lio::CloudPose cloud_pose;
  pcl::toROSMsg(*pc, cloud_pose.cloud);
  cloud_pose.cloud.header.stamp = ros::Time().fromSec(state.timestamp);  
  cloud_pose.pose.position.x = state.p[0];
  cloud_pose.pose.position.y = state.p[1];
  cloud_pose.pose.position.z = state.p[2];
  V4 temp_q = state.R.coeffs();
  cloud_pose.pose.orientation.x = temp_q[0];
  cloud_pose.pose.orientation.y = temp_q[1];
  cloud_pose.pose.orientation.z = temp_q[2];
  cloud_pose.pose.orientation.w = temp_q[3];

  pub_output2robot_.publish(cloud_pose);
}


void ROSWrapper::pub_cloud_world_pose(const CloudPtr& pc, 
   const NavState& state)
{
  static ros::Publisher pub_output2robot_ = nh_.advertise<super_lio::CloudPose>
                                            ("/lio/world/cloud_pose", 10);
  super_lio::CloudPose cloud_pose;
  pcl::toROSMsg(*pc, cloud_pose.cloud);
  cloud_pose.cloud.header.stamp = ros::Time().fromSec(state.timestamp);  
  cloud_pose.pose.position.x = state.p[0];
  cloud_pose.pose.position.y = state.p[1];
  cloud_pose.pose.position.z = state.p[2];
  V4 temp_q = state.R.coeffs();
  cloud_pose.pose.orientation.x = temp_q[0];
  cloud_pose.pose.orientation.y = temp_q[1];
  cloud_pose.pose.orientation.z = temp_q[2];
  cloud_pose.pose.orientation.w = temp_q[3];
  
  pub_output2robot_.publish(cloud_pose);
}


void ROSWrapper::pub_double_cloud_pose(const BASIC::VV3& pc_world,
                                       const BASIC::VV3& pc_body,
                                       const NavState& state)
{
  static ros::Publisher pub_msg_ = nh_.advertise<super_lio::DoubleCloudPose>
                                            ("/lio/double_cloud_pose", 10);
  super_lio::DoubleCloudPose cloud_pose;
  cloud_pose.header.stamp = ros::Time().fromSec(state.timestamp);  

  cloud_pose.pose.reserve(12);
  cloud_pose.pose.push_back(state.p[0]);
  cloud_pose.pose.push_back(state.p[1]);
  cloud_pose.pose.push_back(state.p[2]);

  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      cloud_pose.pose.push_back(state.R.R_(r, c));

  cloud_pose.cloud_world.reserve(pc_world.size() * 3);
  for (const auto& pt : pc_world) {
    cloud_pose.cloud_world.push_back(pt[0]);
    cloud_pose.cloud_world.push_back(pt[1]);
    cloud_pose.cloud_world.push_back(pt[2]);
  }

  cloud_pose.cloud_lidar.reserve(pc_body.size() * 3);
  for (const auto& pt : pc_body) {
    cloud_pose.cloud_lidar.push_back(pt[0]);
    cloud_pose.cloud_lidar.push_back(pt[1]);
    cloud_pose.cloud_lidar.push_back(pt[2]);
  }

  pub_msg_.publish(cloud_pose);
}


void ROSWrapper::pub_cloud_body_pose( 
      const BASIC::VV3& pc_body,
      const NavState& state)
{
  static ros::Publisher pub_msg_ = nh_.advertise<super_lio::CloudPose2>
                                            ("/lio/dense/cloud_pose", 10);

  super_lio::CloudPose2 cloud_pose;
  cloud_pose.header.stamp = ros::Time().fromSec(state.timestamp);

  cloud_pose.pose.reserve(12);
  cloud_pose.pose.push_back(state.p[0]);
  cloud_pose.pose.push_back(state.p[1]);
  cloud_pose.pose.push_back(state.p[2]);

  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      cloud_pose.pose.push_back(state.R.R_(r, c));

  cloud_pose.cloud_lidar.reserve(pc_body.size() * 3);
  for (const auto& pt : pc_body) {
    cloud_pose.cloud_lidar.push_back(pt[0]);
    cloud_pose.cloud_lidar.push_back(pt[1]);
    cloud_pose.cloud_lidar.push_back(pt[2]);
  }

  pub_msg_.publish(cloud_pose);
}

} // namespace END.
