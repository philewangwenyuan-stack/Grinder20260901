
#include "lio/super_lio.h"

#include <algorithm>
#include <cmath>
#include <pthread.h>
#include <sys/resource.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>
#include <fstream>
#include <iomanip>


#define LIO_TIME_EVA

using namespace BASIC;

namespace LI2Sup{

inline bool calc_plane_coeff(const int N, const std::array<V3, 5>& points, std::array<double, 4>& abcd)
{
  Eigen::Vector3d normvec;
  if (N == 5) {
    Eigen::Matrix<double, 5, 3> A;
    Eigen::Matrix<double, 5, 1> b;
    for (int j = 0; j < 5; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }
  else {
    Eigen::Matrix<double, 4, 3> A;
    Eigen::Matrix<double, 4, 1> b;

    for (int j = 0; j < N; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }

  double n = normvec.norm();
  if (n < 1e-6f) return false;

  abcd[3] = 1.0 / n;
  normvec *= abcd[3];
  abcd[0] = normvec[0];
  abcd[1] = normvec[1];
  abcd[2] = normvec[2];
  
  for (int i = 0; i < N; ++i) {
    const V3& p = points[i];
    auto dist = abcd[0] * p(0) + abcd[1] * p(1) + abcd[2] * p(2) + abcd[3];
    if (std::abs(dist) > 0.1) return false;
  }
  return true;
}


inline bool compute_error(
  const std::array<double, 4>& abcd, const V3& point, 
  const float length, scalar& error)
{
  error = abcd[0] * point[0] + abcd[1] * point[1] + abcd[2] * point[2] + abcd[3];
  return length > 81 * error * error;
}


void SuperLIO::init(){
                  
  ivox_.reset(new OctVoxMapType(OctVoxMapType::Options{g_ivox_resolution, g_ivox_capacity}));
  kf_.reset(new ESKF());
  data_wrapper_.reset(new ROSWrapper());
  data_wrapper_->setESKF(kf_);
  data_wrapper_->setMap(ivox_);
  
  scan_undistort_full_.reset(new PointCloudType());
  ds_undistort_.reset(new PointCloudType());
  world_pc_.reset(new PointCloudType());
  ds_world_.reset(new PointCloudType());

  if(g_save_map){
    point_map_.reset(new PointCloudType());
  }
  
  points_world_v3_.reserve(21000);
  abcd_vec_.resize(20000);
  effect_knn_idxs_.resize(20000);
  voxel_grid_fliter_.setLeafSize(g_voxel_fliter_size);

  LOG(INFO) << YELLOW << " ---> Initializing..." << RESET;
  
  bool init_state_once = true;
  auto start_time = std::chrono::high_resolution_clock::now();
  ros::Rate loop(100);
  while(ros::ok() && g_flag_run){
    data_wrapper_->spinOnce();
    if(!data_wrapper_->sync_measure(measures_)){
      loop.sleep();
      continue;
    }
    if(init_state_once){
      start_time = std::chrono::high_resolution_clock::now();
      init_state_once = false;
    }
    if(kf_init()) break;
    loop.sleep();
  }
  
  if(!g_flag_run || !ros::ok()) return;
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  LOG(INFO) << GREEN << " ---> KF init success. Time: " << duration.count() << " ms." << RESET;

  init_state_once = true;

  while(ros::ok() && g_flag_run){
    data_wrapper_->spinOnce();
    if(!data_wrapper_->sync_measure(measures_)){
      loop.sleep();
      continue;
    }
    if(init_state_once){
      start_time = std::chrono::high_resolution_clock::now();
      init_state_once = false;
    }
    if(map_init()){
      kf_->init_ = true;
      break;
    }
    loop.sleep();
  }

  if(!g_flag_run || !ros::ok()) return;
  end_time = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  LOG(INFO) << GREEN << " ---> Map init success. Time: " << duration.count() << " ms." << RESET;
}


bool SuperLIO::kf_init(){
  static int imu_cout = 0;
  static V3 mean_gyro = V3::Zero();
  static V3 mean_acce = V3::Zero();

  for(auto& imu: measures_.imu){
    imu_cout ++;
    mean_gyro += (imu.gyr - mean_gyro) / imu_cout;
    mean_acce += (imu.acc - mean_acce) / imu_cout;
  }

  /// 100 Hz for 1 second.
  if(imu_cout < 50){
    return false;
  }

  V3 gravity = - mean_acce * g_gravity_norm / mean_acce.norm();
  V3 ref_gravity(0, 0, - g_gravity_norm);
  M3 init_rot = Quat::FromTwoVectors(gravity, ref_gravity).toRotationMatrix();
  V3 n = init_rot.col(0);
  double yaw = atan2(n(1), n(0));

  M3 R_yaw_inv = Eigen::AngleAxis<scalar>(-yaw, V3::UnitZ()).toRotationMatrix(); 

  // init_rot represents the IMU orientation after gravity alignment (level orientation).
  // Perform LiDAR leveling correction, then transform the orientation into the robot frame.
  M3 rot = g_lidar_robo_yaw * R_yaw_inv * init_rot;  


  ESKF::Options options;
  options.gyro_var_ = g_imu_ng;
  options.acce_var_ = g_imu_na;
  options.bias_gyro_var_ = g_imu_nbg;
  options.bias_acce_var_ = g_imu_nba;
  options.num_iterations_ = g_kf_max_iterations;
  options.quit_eps_ = g_kf_quit_eps;

  float imu_scale = g_gravity_norm / mean_acce.norm();
  kf_->SetInitialConditions(options, mean_gyro, V3::Zero(), imu_scale, ref_gravity);
  auto state = kf_->GetSysState();
  state.R = SO3(rot);
  state.p = g_odom_robo.t_;        // By default, the robot frame is used as the reference origin.
  state.timestamp = measures_.imu.back().secs;
  kf_->SetX(state);
  sys_init_pose_ = kf_->GetSE3();
  return true;
}


bool SuperLIO::map_init(){
  frame_num_++;

  std::size_t ptsize = measures_.lidar.pc->size();
  points_world_v3_.resize(ptsize);  

  const SE3 transform = sys_init_pose_ * g_lidar_imu;

  tbb::parallel_for(
    tbb::blocked_range<size_t>(0, ptsize),
    [&](const tbb::blocked_range<size_t>& r) {
      for (size_t idx = r.begin(); idx < r.end(); ++idx) {
        auto& point_pcl = measures_.lidar.pc->points[idx];
        V3 point_body(point_pcl.x, point_pcl.y, point_pcl.z);
        points_world_v3_[idx] = transform * point_body;
      }
    }
  );

  ivox_->insert(points_world_v3_);
  kf_->SetLastObsTime(measures_.lidar.end_time);

  // 20 Hz for 1.0 seconds. Integral coverage area > 70%
  if(frame_num_ > 3){
    g_flg_map_init = false;
    return true;
  }
  return false;
}


void SuperLIO::run(){
  ros::Rate loop(500);
  if(ros::ok() && g_flag_run) LOG(INFO) << GREEN << " ---> Running..." << RESET;
  while(ros::ok() && g_flag_run){
    data_wrapper_->spinOnce();
    process();
    loop.sleep();
  }
  
  LOG(INFO) << GREEN << " ---> [SuperLIO]:  Exit." << RESET;
  // saveMap();
}

// bool SuperLIO::GetMapCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
//   LOG(INFO) << GREEN << " ---> [SuperLIO]:  Get map success." << RESET;
//   saveMap();
//   return true;
// }


void SuperLIO::caceData(){
  if(!g_save_map) return;
  auto state = kf_->GetNavState();

  const PointCloudType::Ptr& source_cloud = g_if_filter ? ds_undistort_ : scan_undistort_full_;
  if(!source_cloud || source_cloud->size() < static_cast<std::size_t>(g_map_save_min_points)){
    return;
  }

  bool accept_keyframe = !map_cache_has_keyframe_;
  if(map_cache_has_keyframe_){
    const double elapsed = state.timestamp - map_cache_last_timestamp_;
    const double translation = (state.p - map_cache_last_position_).norm();
    const M3 delta_rotation = map_cache_last_rotation_.transpose() * state.R.R_;
    const double cosine = std::max(-1.0, std::min(1.0,
        (static_cast<double>(delta_rotation.trace()) - 1.0) * 0.5));
    const double rotation = std::acos(cosine);
    const bool moved = translation >= g_map_save_min_translation ||
                       rotation >= g_map_save_min_rotation_rad;

    if(!moved){
      map_cache_motion_confirm_count_ = 0;
      return;
    }
    if(elapsed < g_map_save_min_interval_sec){
      return;
    }
    map_cache_motion_confirm_count_++;
    accept_keyframe = map_cache_motion_confirm_count_ >= g_map_save_motion_confirm_frames;
  }

  if(!accept_keyframe){
    return;
  }

  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = state.R.R_.cast<float>();
  transformation.block<3, 1>(0, 3) = state.p.cast<float>();

  pcl::transformPointCloud(*source_cloud, *world_pc_, transformation);

  static int scan_wait_num = 0;
  if(!world_pc_->empty()){
    *point_map_ += *world_pc_;
    scan_wait_num++;
    map_cache_has_keyframe_ = true;
    map_cache_last_position_ = state.p;
    map_cache_last_rotation_ = state.R.R_;
    map_cache_last_timestamp_ = state.timestamp;
    map_cache_motion_confirm_count_ = 0;
  }

  data_wrapper_->get3DCloud(point_map_);




  if(g_pcd_save_interval < 0) {
    scan_wait_num = 0;
    return;
  }

  static bool rm_PCD_dir = false;
  if(!rm_PCD_dir){
    rm_PCD_dir = true;
    std::string cmd = "rm -rf " + g_save_map_dir + "/PCD";
    [[maybe_unused]] int res;
    res = system(cmd.c_str());
    cmd = "mkdir -p " + g_save_map_dir + "/PCD";
    res = system(cmd.c_str());
  }

  if (point_map_->size() > 0 && scan_wait_num >= g_pcd_save_interval) {
    pcd_index_++;
    std::string map_name(std::string(g_save_map_dir + "/PCD/scans_") + std::to_string(pcd_index_) +
                               std::string(".pcd"));
    LOG(INFO) << GREEN << " ---> current scan saved to /PCD/scans_" << pcd_index_ << "  size:  " << point_map_->size() << RESET;
    pcl::io::savePCDFileBinary(map_name, *point_map_);
    point_map_->clear();
    scan_wait_num = 0;
  }
}


void SuperLIO::ProcessCaceMap(){
  namespace fs = std::filesystem;

  std::string pcd_folder = g_save_map_dir + "/PCD";
  std::string output_map_name = g_save_map_dir + "/" + g_map_name;
  std::string merged_map_name = g_save_map_dir + "/" + "merged_" + g_map_name;

  LOG(INFO) << YELLOW << " ---> Merging PCD fragments in: " << pcd_folder << RESET;

  PointCloudType::Ptr merged_map(new PointCloudType());

  int count = 0;
  for (const auto& entry : fs::directory_iterator(pcd_folder)) {
    if (entry.path().extension() == ".pcd" &&
      entry.path().filename().string().find("scans_") != std::string::npos) {
      PointCloudType::Ptr tmp_cloud(new PointCloudType());
      if (pcl::io::loadPCDFile<PointType>(entry.path().string(), *tmp_cloud) == 0) {
        *merged_map += *tmp_cloud;
        count++;
        // LOG(INFO) << GREEN << " ---> Merged: " << entry.path().filename().string() 
        //           << "   size: " << tmp_cloud->size() << RESET;
      } else {
        LOG(WARNING) << RED << " ---> Failed to load: " << entry.path().string() << RESET;
      }
    }
  }

  LOG(INFO) << YELLOW << " ---> Total merged fragments: " << count << RESET;

  PointCloudType filtered_map;
  PointCloudType last_map;

  if(g_if_filter){
    LOG(INFO) << YELLOW << " ---> Downsampling merged map before final save..." << RESET;
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
    
    voxel_filter.setInputCloud(merged_map);
    voxel_filter.filter(filtered_map);
  }else{
    LOG(INFO) << YELLOW << " ---> Not Downsampling merged map before final save..." << RESET;
    filtered_map = *merged_map;
  }
  
  if (filtered_map.size() > 0) {
    filtered_map.width = filtered_map.size();
    filtered_map.height = 1;
    filtered_map.is_dense = false;
  }
   for (const auto& pc : map_frame_buffer_) {
      last_map += *pc;
    }
    
  pcl::io::savePCDFileBinary(merged_map_name, last_map);
  pcl::io::savePCDFileBinary(output_map_name, filtered_map);

  LOG(INFO) << GREEN << " ---> Final map saved to: " << output_map_name << " And " << merged_map_name << " " << RESET;
  LOG(INFO) << GREEN << " ---> Final map size: " << filtered_map.size() << RESET;
  LOG(INFO) << GREEN << " ---> Last map size: " << last_map.size() << RESET;
  map_frame_buffer_.clear();
}


void SuperLIO::saveMap(){
  if(!g_save_map) return;
  if(g_pcd_save_interval > 0){
    LOG(INFO) << YELLOW << " ---> Saving last cace ... " << RESET;
    if (point_map_->size() > 0) {
      pcd_index_++;
      std::string map_name(std::string(g_save_map_dir + "/PCD/scans_") + std::to_string(pcd_index_) +
                                 std::string(".pcd"));
      LOG(INFO) << GREEN << " ---> current scan saved to /PCD/scans_" << pcd_index_ << "  size:  " << point_map_->size() << RESET;
      pcl::io::savePCDFileBinary(map_name, *point_map_);
      point_map_->clear();
    }
    LOG(INFO) << GREEN << " ---> Save last cace success. " << RESET;
    LOG(INFO) << YELLOW << " ---> The final processing will take a long time." << RESET;
    LOG(INFO) << YELLOW << " ---> Please do not close the program ... " << RESET;
    ProcessCaceMap();
    LOG(INFO) << GREEN << " ---> Process cace map success. " << RESET;
    return;
  }

  LOG(INFO) << YELLOW << " ---> Saving map..... " << RESET;
  if(!point_map_->empty()){
    std::string map_name = g_save_map_dir + "/" + g_map_name;
    std::string merged_map_name = g_save_map_dir + "/" + "merged_" + g_map_name;
    LOG(INFO) << YELLOW << " ---> Save map to: " << map_name << RESET;
    pcl::VoxelGrid<PointType> voxel_fliter;
    PointCloudType latst_map;
    PointCloudType last_map;
    voxel_fliter.setInputCloud(point_map_);
    voxel_fliter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
    voxel_fliter.filter(latst_map);
    if(latst_map.size() > 0){
      latst_map.width = latst_map.size();
      latst_map.height = 1;
      latst_map.is_dense = false;
    }
    for (const auto& pc : map_frame_buffer_) {
      last_map += *pc;
    }
    pcl::io::savePCDFileBinary(merged_map_name, last_map);
    pcl::io::savePCDFileBinary(map_name, latst_map);
    LOG(INFO) << GREEN << " ---> Save map success. File: " << map_name << " And " << merged_map_name << " " << RESET;
    LOG(INFO) << GREEN << " ---> Map size: " << latst_map.size() << RESET;
    LOG(INFO) << GREEN << " ---> Last map size: " << last_map.size() << RESET;
    map_frame_buffer_.clear();
  }
}


inline double get_cpu_time_seconds() {
  struct rusage usage;
  getrusage(RUSAGE_SELF, &usage);
  return usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6 +
         usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
}

void SuperLIO::process(){
  if(!data_wrapper_->sync_measure(measures_)){
    return;
  }


#ifdef LIO_TIME_EVA
  auto __t_start = std::chrono::high_resolution_clock::now();
  auto __t0 = std::chrono::high_resolution_clock::now();
  auto __t1 = std::chrono::high_resolution_clock::now();
  auto __t2 = std::chrono::high_resolution_clock::now();
#endif


  Propagation_Undistort();


#ifdef LIO_TIME_EVA
  __t1 = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(__t1 - __t2);
  static double _sum_undistort_time_wall = 0;
  _sum_undistort_time_wall += duration.count() / 1000.0;
#endif


  DownSample();


#ifdef LIO_TIME_EVA
  __t2 = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(__t2 - __t1);
  static double _sum_downsample_time_wall = 0;
  _sum_downsample_time_wall += duration.count() / 1000.0; 
#endif


  Observe();


#ifdef LIO_TIME_EVA
  __t1 = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(__t1 - __t2);
  static double _sum_observe_time_wall = 0;
  _sum_observe_time_wall += duration.count() / 1000.0;
#endif


  UpdateMap();


#ifdef LIO_TIME_EVA

  auto __t_end = std::chrono::high_resolution_clock::now();
  auto __duration = std::chrono::duration_cast<std::chrono::microseconds>(__t_end - __t_start);
  double __current_time = __duration.count() / 1000.0;

  static std::size_t __sum_count = 0;
  static double __sum_time_wall = 0.0;
  static double __mean = 0.0;
  static double __M2 = 0.0;

  __sum_count++;
  __sum_time_wall += __current_time;

  double __delta = __current_time - __mean;
  __mean += __delta / __sum_count;
  double __delta2 = __current_time - __mean;
  __M2 += __delta * __delta2;

  double __mean_time = __mean;
  double __std_time = (__sum_count > 1) ? std::sqrt(__M2 / (__sum_count - 1)) : 0.0;

  __t2 = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(__t2 - __t1);
  static double _sum_update_map_time_wall = 0;
  _sum_update_map_time_wall += duration.count() / 1000.0;

  auto t_end = std::chrono::high_resolution_clock::now();

  duration = std::chrono::duration_cast<std::chrono::microseconds>(t_end - __t0);
  static int _count = 0;
  static double _sum_time = 0;
  _sum_time += duration.count() / 1000.0;
  _count ++;

  std::cout << "AVG(ms): " << _sum_time / _count << " undistort: " << _sum_undistort_time_wall / _count 
            << " downsample: " << _sum_downsample_time_wall / _count
            << " state_update: " << _sum_observe_time_wall / _count << " map_update: " << _sum_update_map_time_wall / _count << std::endl << std::endl;

  geometry_msgs::PoseStamped msg;
  double timestamp = kf_->GetTime();
  msg.header.stamp = ros::Time(timestamp);
  msg.pose.position.x = __current_time;
  msg.pose.position.y = __mean_time;
  msg.pose.position.z = __std_time;
  pub_processing_time_.publish(msg);

#endif


  caceData(); 

  Output();

}


void SuperLIO::Propagation_Undistort(){
  propagate_states_.clear();
  propagate_states_.emplace_back(kf_->GetDynamicState());
  kf_->SetObsTime(measures_.lidar.end_time);
  for (auto &imu : measures_.imu) {
    kf_->Predict(imu);
    propagate_states_.emplace_back(kf_->GetDynamicState());
  }

  static const M3 TLI_R = g_lidar_imu.R_;
  static const V3 TLI_t = g_lidar_imu.t_;
  const SE3 T_end = kf_->GetSE3();
  const M3  R_inv = T_end.R_.transpose();
  const V3  T_end_t = T_end.t_;
  const double start_time = measures_.lidar.start_time;
  auto& raw_pc = measures_.lidar.pc;

  std::size_t ptsize = raw_pc->points.size();
  scan_undistort_full_->resize(ptsize); 

  tbb::parallel_for(
  tbb::blocked_range<size_t>(0, ptsize),
  [&](const tbb::blocked_range<size_t>& r) {
    M3 R_h, R_t; V3 p_h, v_h, acc_t, w_t;
    for (size_t idx = r.begin(); idx < r.end(); ++idx) {  
      auto& pt_full = scan_undistort_full_->points[idx];
      const auto& pt = raw_pc->points[idx];
      pt_full.intensity = pt.intensity;
      double query_time = start_time + pt.offset_time;
      if (query_time > propagate_states_.back().time) {
        V3 raw(pt.x, pt.y, pt.z);
        V3 eigen_point = TLI_R * raw + TLI_t;
        pt_full.x = eigen_point[0];
        pt_full.y = eigen_point[1];
        pt_full.z = eigen_point[2];
        continue;
      }
      auto match_iter = propagate_states_.begin();
      for (auto iter = propagate_states_.begin(); iter != propagate_states_.end(); ++iter) {
        auto next_iter = std::next(iter);
        if (iter->time < query_time && next_iter->time >= query_time) {
          match_iter = iter;
          break;
        }
      }
      auto match_iter_n = std::next(match_iter);
      double dt = match_iter_n->time - match_iter->time;
      double s = (query_time - match_iter->time) / dt;
      R_h = match_iter->R;
      R_t = match_iter_n->R;
      p_h = match_iter->p;
      v_h = match_iter->v;
      acc_t = match_iter_n->a;
      w_t = match_iter_n->w;
      M3 R_i = Quat(R_h).slerp(s, Quat(R_t)).toRotationMatrix();
      V3 t_ei(p_h + v_h * dt + 0.5 * acc_t * dt * dt - T_end_t);
      V3 raw(pt.x, pt.y, pt.z);
      V3 eigen_point = R_inv * (R_i * (TLI_R * raw + TLI_t) + t_ei);
      pt_full.x = eigen_point[0];
      pt_full.y = eigen_point[1];
      pt_full.z = eigen_point[2];
    }
  });
}


void SuperLIO::DownSample(){
  voxel_grid_fliter_.setInputCloud(scan_undistort_full_);
  voxel_grid_fliter_.filter(ds_undistort_);
}


struct ThreadACC{
  M6d HTVH = M6d::Zero();
  V6d HTVr = V6d::Zero();
  ThreadACC(): HTVH(M6d::Zero()), HTVr(V6d::Zero()) {}
};


void SuperLIO::Observe(){
  size_t ptsize = ds_undistort_->size();
  
  static std::vector<float> _lengths;
  points_body_v3_.resize(ptsize);
  _lengths.resize(ptsize);

  effect_knn_num_ = ptsize;
  std::iota(effect_knn_idxs_.begin(), effect_knn_idxs_.begin() + ptsize, 0);

  for(size_t i = 0; i < ptsize; ++i){
    const auto& point_body_pcl = ds_undistort_->points[i];
    points_body_v3_[i] = V3(point_body_pcl.x, point_body_pcl.y, point_body_pcl.z);
    _lengths[i] = points_body_v3_[i].norm();
  }

  ivox_->reset_max_group();
  int iter_num = 0;

  kf_->UpdateObserve([&, this](const ESKF::KFState &kf_state, M6 &HTVH, V6 &HTVr) {
    const SE3 pose = kf_state.pose;
    const bool need_converge = kf_state.need_converge;
    const M3d R_transpose = (pose.R_.transpose()).cast<double>();

    tbb::enumerable_thread_specific<ThreadACC> tls_acc;

    tbb::parallel_for(
      tbb::blocked_range<size_t>(0, effect_knn_num_),
      [&](const tbb::blocked_range<size_t>& r) {
        KNNHeapType top_K;
        auto& local_acc = tls_acc.local();
        for (size_t r_s = r.begin(); r_s < r.end(); ++r_s) {
          int idx = effect_knn_idxs_[r_s];
          V3& point_body = points_body_v3_[idx];
          V3 point_world = pose * point_body;

          if(!need_converge){
            top_K.reset();
            ivox_->getTopK(point_world, top_K);
            if(top_K.count < 4){
              effect_mask_[idx] = false;
              effect_knn_mask_[idx] = false;
              continue;
            }
            effect_knn_mask_[idx] = true;
            effect_mask_[idx] = calc_plane_coeff(top_K.count, top_K.points_, abcd_vec_[idx]);
          }

          if(!effect_mask_[idx]) continue;

          auto& abcd = abcd_vec_[idx];
          scalar error;
          effect_mask_[idx] = compute_error(abcd, point_world, _lengths[idx], error);
          if(!effect_mask_[idx]) continue;
          
          {
            V3d normvec(abcd[0], abcd[1], abcd[2]);
            V3d nb = R_transpose * normvec;
            V3d point_body_d = point_body.cast<double>();
            V6d J;
            J.head<3>() = point_body_d.cross(nb);
            J.tail<3>() = normvec;
      
            local_acc.HTVH += J * 1000 * J.transpose();
            local_acc.HTVr -= J * 1000 * error;
          }
        }
    });

    M6d sum_HTVH = M6d::Zero();
    V6d sum_HTVr = V6d::Zero();
    for(const auto& local_acc : tls_acc){
      sum_HTVH += local_acc.HTVH;
      sum_HTVr += local_acc.HTVr;
    }
    HTVH = sum_HTVH.cast<scalar>();
    HTVr = sum_HTVr.cast<scalar>();

    if(need_converge) return;

    int _effect_knn_num = 0;
    for(size_t i = 0; i < effect_knn_num_; ++i){
      int idx = effect_knn_idxs_[i];
      if(!effect_knn_mask_[idx]) continue;
      effect_knn_idxs_[_effect_knn_num] = idx;
      _effect_knn_num++;
    }

    // LOG(INFO) << "effect_knn_num_: " << effect_knn_num_ << ", _effect_knn_num: " << _effect_knn_num;
    effect_knn_num_ = _effect_knn_num;

    iter_num++;
  });

  frame_num_++;
}


void SuperLIO::UpdateMap() {
  const size_t ptsize = ds_undistort_->size();
  if (ptsize == 0) return;
  
  last_pose_ = kf_->GetSE3();
  points_world_v3_.resize(ptsize);
  
  const auto R = last_pose_.R_;
  const auto t = last_pose_.t_;
  
  for (size_t i = 0; i < ptsize; ++i) {
    const auto& pt = points_body_v3_[i];
    points_world_v3_[i] = R * pt + t;
  }
  
  ivox_->insert(points_world_v3_);

}


void SuperLIO::Output(){
  auto state = kf_->GetNavState();
  data_wrapper_->pub_odom(state, false);  

  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = state.R.R_.cast<float>();
  transformation.block<3, 1>(0, 3) = state.p.cast<float>();

  CloudPtr world_pc(new PointCloudType());

  if(g_visual_map){
    static int count = -1;
    count++;
    if(count % g_pub_step != 0){
      return;
    }
    count = 0;
    if(g_visual_dense){
      pcl::transformPointCloud(*scan_undistort_full_, *world_pc, transformation);
      data_wrapper_->pub_cloud_world(world_pc, state.timestamp);
    }else{
      pcl::transformPointCloud(*ds_undistort_, *world_pc, transformation);
      data_wrapper_->pub_cloud_world(world_pc, state.timestamp);
    }
  }
  
  // if(g_visual_map){
  //   static int count = -1;
  //   count++;
  //   if(count % g_pub_step != 0){
  //     return;
  //   }

  //   CloudPtr cloud(new PointCloudType());
  //   for(auto &pt : *scan_undistort_full_){
  //       // 重置intensity，以z轴值为标准
  //       pt.intensity = pt.z / 1.6 * 1000.0f; 
  //       float r = pt.x * pt.x + pt.y * pt.y;
  //       if(pt.z > 1.3) 
  //       {
  //         cloud->push_back(pt);
  //       }
  //     }
    
    // if(g_visual_dense){
    //   pcl::transformPointCloud(*cloud, *world_pc, transformation);
    //   CloudPtr map_pc(new PointCloudType());
      // for (const auto& point : *world_pc) {
      //   if (point.x > -7.1 && point.x < 0.8 && point.y > -2.1 && point.y < 1.2 && point.intensity < 830) {
      //     map_pc->push_back(point);
      //   }
      // }
  //     map_frame_buffer_.push_back(map_pc);
  //     data_wrapper_->pub_cloud_world(map_pc, state.timestamp);
  //   }else{
  //     pcl::transformPointCloud(*ds_undistort_, *world_pc, transformation);
  //     data_wrapper_->pub_cloud_world(world_pc, state.timestamp);
  //   }
  // }
  
  // static int pcd_count = 0;
  // static std::string save_path = "/mnt/d/LivoxMid360";
  // // 保存当前帧为PCD文件
  // char bin_filename[20];
  // sprintf(bin_filename, "%06d.bin", pcd_count);
  // // 保存为二进制bin文件
  // std::string bin_file_path = save_path + "/PCD/" + std::string(bin_filename);
  // std::ofstream bin_file(bin_file_path, std::ios::binary);
  // if (bin_file.is_open()) {
  //   for (const auto& point : *world_pc) {
  //     // 保存x, y, z, intensity为float类型
  //     bin_file.write(reinterpret_cast<const char*>(&point.x), sizeof(float));
  //     bin_file.write(reinterpret_cast<const char*>(&point.y), sizeof(float));
  //     bin_file.write(reinterpret_cast<const char*>(&point.z), sizeof(float));
  //     bin_file.write(reinterpret_cast<const char*>(&point.intensity), sizeof(float));
  //   }
  //   bin_file.close();
  // }
  
  // 保存当前odom为txt文件
//   std::ofstream odom_file(save_path + "/Poses/test_pose.txt", std::ios_base::app);
//   if (odom_file.is_open()) {
//     // 获取四元数
//     auto quat = state.R.quaternion();
//     // 格式：Submap_id pos_x pos_y pos_z quat_x quat_y quat_z quat_w
//     odom_file << std::fixed << std::setprecision(6);
//     odom_file << pcd_count << " " 
//               << state.p.x() << " " 
//               << state.p.y() << " " 
//               << state.p.z() << " " 
//               << quat.x() << " " 
//               << quat.y() << " " 
//               << quat.z() << " " 
//               << quat.w() << std::endl;
//     odom_file.close();
//   }
//   pcd_count++;
}

} // namespace END.
