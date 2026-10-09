
#include "lio/super_lio_location.h"

#include <sys/resource.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <cmath>
#include <string>



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


void SuperLIOReLoc::initialpose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
  flg_init_get_guess_ = true;

  V3 init_translation(0, 0, 0);
  init_translation[0] = msg->pose.pose.position.x;
  init_translation[1] = msg->pose.pose.position.y;
  init_translation[2] = 0.2; // 根据传感器离地高度调整

  double x,y,z,w;
  x = msg->pose.pose.orientation.x;
  y = msg->pose.pose.orientation.y;
  z = msg->pose.pose.orientation.z;
  w = msg->pose.pose.orientation.w;
  Quat init_rotation = Quat(w,x,y,z);
  re_init_pose_ = SE3(SO3(init_rotation.toRotationMatrix()), init_translation);
  flg_get_init_guess_ = true;
  flg_init_reset_ = true;
  flg_re_init_ = true;  /// 运行中收到新的初始猜测, 触发重新执行重定位
  init_frame_count_ = 0;
  init_obs_data_->clear();

  LOG(INFO) << YELLOW << " ---> GET Initial guess: " << init_translation.transpose() << "  yaw:  " <<  init_rotation.toRotationMatrix().eulerAngles(0, 1, 2).transpose() << RESET; 
}


void SuperLIOReLoc::init(){
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

  point_map_.reset(new PointCloudType());
  loc_map_.reset(new PointCloudType());
  init_obs_data_.reset(new PointCloudType());
  
  points_world_v3_.reserve(21000);
  abcd_vec_.resize(20000);
  effect_knn_idxs_.resize(20000);
  voxel_grid_fliter_.setLeafSize(g_voxel_fliter_size);

  LOG(INFO) << YELLOW << " ---> Initializing..." << RESET;


  auto start_time = std::chrono::high_resolution_clock::now();
  map_init();
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  LOG(INFO) << GREEN << " ---> Map init success. Time: " << duration.count() << " ms." << RESET;

  /// 首次重定位
  if(!reloc_initialization()) return;

  /// 不释放 loc_map_ / init_obs_data_, 不退订 /initialpose,
  /// 以支持运行中收到新的初始猜测时重新执行重定位.
  flg_re_init_ = false;  /// 清除首次重定位期间残留的触发标志
}


bool SuperLIOReLoc::reloc_initialization(){
  ros::Rate loop(100);
  auto start_time = std::chrono::high_resolution_clock::now();
  int __count = -1;
  std::size_t __count_base = 50;
  bool first_sync = true;

  while(ros::ok() && g_flag_run){
    data_wrapper_->spinOnce();
    self_queue_.callAvailable();

    __count++;
    if(__count % __count_base == 0){
      msg_global_map_.header.stamp = ros::Time().now();
      pub_map_.publish(msg_global_map_);
      __count = 0;
      __count_base += 50;
    }

    if(!data_wrapper_->sync_measure(measures_)){
      loop.sleep();
      continue;
    }

    if(first_sync){
      start_time = std::chrono::high_resolution_clock::now();
      first_sync = false;
    }

    CloudPtr point_cloud_pcl = CloudPtr(new PointCloudType());
    for(std::size_t i = 0; i < measures_.lidar.pc->size(); i++){
      auto p = measures_.lidar.pc->points[i];
      PointType point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      point.intensity = p.intensity;
      point_cloud_pcl->points.push_back(point);
    }

    if(init_frame_count_ < 20){
      *init_obs_data_ += *point_cloud_pcl;
    }
    init_frame_count_++;

    if(kf_init()) {
      break;
    }
    loop.sleep();
  }

  if(!g_flag_run || !ros::ok()) return false;
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  LOG(INFO) << GREEN << " ---> KF init success. Time: " << duration.count() << " ms." << RESET;


  while(ros::ok() && g_flag_run){
    data_wrapper_->spinOnce();
    if(!data_wrapper_->sync_measure(measures_)){
      loop.sleep();
      continue;
    }
    kf_->SetLastObsTime(measures_.lidar.end_time);
    break;
  }

  return true;
}


bool SuperLIOReLoc::kf_init(){
  if(!flg_init_get_guess_) 
  {
    LOG(INFO) << YELLOW << " ---> Failed to GET Initial guess." << RESET;
    return false;
  }
  static int imu_cout = 0;
  static V3 mean_gyro = V3::Zero();
  static V3 mean_acce = V3::Zero();

  if(flg_init_reset_){
    imu_cout = 0;
    mean_gyro = V3::Zero();
    mean_acce = V3::Zero();
    flg_init_reset_ = false;
  }


  for(auto& imu: measures_.imu){
    imu_cout ++;
    mean_gyro += (imu.gyr - mean_gyro) / imu_cout;
    mean_acce += (imu.acc - mean_acce) / imu_cout;
  }

  /// 100 Hz for 1 second.
  if(imu_cout < 20){
    return false;
  }

  if(init_frame_count_ < 20){
    return false;
  }

  LOG(INFO) << YELLOW << " ---> INIT start... obs_data size: " << init_obs_data_->size() << " target size: " << loc_map_->size() << RESET;

  V3 gravity = - mean_acce * g_gravity_norm / mean_acce.norm();
  V3 ref_gravity(0, 0, - g_gravity_norm);
  M3 init_rot = Quat::FromTwoVectors(gravity, ref_gravity).toRotationMatrix();
  V3 n = init_rot.col(0);
  double yaw = atan2(n(1), n(0));

  M3 R_yaw_inv = Eigen::AngleAxis<scalar>(-yaw, V3::UnitZ()).toRotationMatrix(); 
  M3 rot = R_yaw_inv * init_rot;

  M3 init_guess_R_ = re_init_pose_.R_ * rot;
  V3 init_guess_t_ = re_init_pose_.t_;
  M4 init_guess_T = M4::Identity();
  init_guess_T.block<3, 3>(0, 0) = init_guess_R_;
  init_guess_T.block<3, 1>(0, 3) = init_guess_t_;


  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_src(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*init_obs_data_, *tmp_src, g_lidar_imu.matrix().cast<float>());

  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
  ndt.setTransformationEpsilon(1e-4);
  ndt.setEuclideanFitnessEpsilon(1e-4);
  ndt.setMaximumIterations(25);
  ndt.setResolution(1.0);
  ndt.setInputTarget(loc_map_);

  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setMaxCorrespondenceDistance(4.0);
  icp.setMaximumIterations(60);
  icp.setTransformationEpsilon(1e-4);
  icp.setEuclideanFitnessEpsilon(1e-4);
  icp.setRANSACIterations(0);
  icp.setInputTarget(loc_map_);

  ndt.setInputSource(tmp_src);
  icp.setInputSource(tmp_src);

  pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZI>());
  ndt.align(*unused_result, init_guess_T.matrix().cast<float>());
  icp.align(*unused_result, ndt.getFinalTransformation());

  bool result = true;
  // if (icp.hasConverged() == false || icp.getFitnessScore() > 0.3)
  if (icp.hasConverged() == false)
  {
    flg_init_state_ = false;
    if(flg_get_init_guess_){
      flg_get_init_guess_ = false;
    }
    imu_cout = 0;
    init_frame_count_ = 0;
    init_obs_data_.reset(new PointCloudType());
    mean_gyro = V3::Zero();
    mean_acce = V3::Zero();
    LOG(INFO) << RED << " ---> Global ICP Converged Fail! " << RESET;
    result = false;
  } else{
    init_guess_T = icp.getFinalTransformation().cast<scalar>();
    LOG(INFO) << GREEN << " ---> Global ICP Converged Succeed! " << RESET;
  }
  flg_init_get_guess_ = false;

  if(!result) return false;

  LOG(INFO) << GREEN << "\nInit Guess T: \n" << init_guess_T << RESET;

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
  /// 机器人坐标系下雷达的水平初始状态.
  state.R = SO3(init_guess_T.block<3, 3>(0, 0));
  state.p = init_guess_T.block<3, 1>(0, 3);
  state.timestamp = -1.0;
  kf_->SetX(state);
  sys_init_pose_ = kf_->GetSE3();
  return true;
}


bool SuperLIOReLoc::map_init(){
  std::string map_name = g_save_map_dir + "/" + g_map_name;

  if(pcl::io::loadPCDFile<PointType>(map_name, *loc_map_) == -1){
    LOG(ERROR) << RED << " ---> Load map failed. File: " << map_name << RESET;
    return false;
  }

  std::vector<int> useless_indices;
  pcl::removeNaNFromPointCloud(*loc_map_, *loc_map_, useless_indices);

  VV3 point_map_v3;
  point_map_v3.reserve(loc_map_->size());
  for(const auto& point: *loc_map_){
    V3 pt(point.x, point.y, point.z);
    point_map_v3.push_back(pt);
  }

  ivox_->insert(point_map_v3);
  msg_global_map_.header.frame_id = "map";
  msg_global_map_.header.stamp = ros::Time().now();
  pcl::toROSMsg(*loc_map_, msg_global_map_);

  msg_global_map_.header.frame_id = "map";
  msg_global_map_.header.stamp = ros::Time().now();
  pub_map_.publish(msg_global_map_);

  LOG(INFO) << GREEN << " ---> Load map success. File: " << map_name << RESET;
  LOG(INFO) << GREEN << " ---> Map size: " << loc_map_->size() << RESET;
  ivox_->printInfo();
  return true;
}


void SuperLIOReLoc::run(){
  ros::Rate loop(500);
  if(ros::ok() && g_flag_run) LOG(INFO) << GREEN << " ---> Running..." << RESET;
  while(ros::ok() && g_flag_run){
    data_wrapper_->spinOnce();
    self_queue_.callAvailable();  /// 处理 /initialpose 回调

    /// 运行中收到新的初始猜测: 重新执行重定位, 无需重启节点
    if(flg_re_init_){
      flg_re_path_ = true;
      flg_re_init_ = false;
      LOG(INFO) << YELLOW << " ---> Re-Relocation start with new initial guess..." << RESET;
      if(!reloc_initialization()) break;
      flg_re_init_ = false;  /// 重定位期间的新点击已在流程内生效, 清除残留触发
      LOG(INFO) << GREEN << " ---> Re-Relocation success. Resume localization." << RESET;
      continue;
    }

    process();
    loop.sleep();
  }
  
  LOG(INFO) << GREEN << " ---> [SuperLIOReLoc]:  Exit." << RESET;
  // saveMap();
}


void SuperLIOReLoc::caceData(){
  if(!g_save_map) return;
  auto state = kf_->GetNavState();
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = state.R.R_.cast<float>();
  transformation.block<3, 1>(0, 3) = state.p.cast<float>();

  if(g_if_filter){
    pcl::transformPointCloud(*ds_undistort_, *world_pc_, transformation);
  }else{
    pcl::transformPointCloud(*scan_undistort_full_, *world_pc_, transformation);
  }

  static int scan_wait_num = 0;
  if(!world_pc_->empty()){
    *point_map_ += *world_pc_;
    scan_wait_num++;
  }

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


void SuperLIOReLoc::ProcessCaceMap(){
  namespace fs = std::filesystem;

  std::string pcd_folder = g_save_map_dir + "/PCD";
  std::string output_map_name = g_save_map_dir + "/" + g_map_name;

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

  pcl::io::savePCDFileBinary(output_map_name, filtered_map);

  LOG(INFO) << GREEN << " ---> Final map saved to: " << output_map_name << RESET;
  LOG(INFO) << GREEN << " ---> Final map size: " << filtered_map.size() << RESET;
}


void SuperLIOReLoc::saveMap(){
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
    LOG(INFO) << YELLOW << " ---> Save map to: " << map_name << RESET;
    pcl::VoxelGrid<PointType> voxel_fliter;
    PointCloudType latst_map;
    voxel_fliter.setInputCloud(point_map_);
    voxel_fliter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
    voxel_fliter.filter(latst_map);
    if(latst_map.size() > 0){
      latst_map.width = latst_map.size();
      latst_map.height = 1;
      latst_map.is_dense = false;
    }
    pcl::io::savePCDFileBinary(map_name, latst_map);
    LOG(INFO) << GREEN << " ---> Save map success. File: " << map_name << RESET;
    LOG(INFO) << GREEN << " ---> Map size: " << latst_map.size() << RESET;
  }
}


inline double get_cpu_time_seconds() {
  struct rusage usage;
  getrusage(RUSAGE_SELF, &usage);
  return usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6 +
         usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
}


void SuperLIOReLoc::process(){
  if(!data_wrapper_->sync_measure(measures_)){
    return;
  }

  Propagation_Undistort();

  DownSample();

  Observe();

  if(g_update_map){
    static int __update_delay = 10;
    if(__update_delay > 0){
      __update_delay--;
      std::cout << "Update map Delay: " << 100 - __update_delay << " %" << std::endl;
    }else{
      UpdateMap();
    }
  }

  caceData();


  Output();

}


void SuperLIOReLoc::Propagation_Undistort(){
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


void SuperLIOReLoc::DownSample(){
  voxel_grid_fliter_.setInputCloud(scan_undistort_full_);
  voxel_grid_fliter_.filter(ds_undistort_);
}


struct ThreadACC{
  M6d HTVH = M6d::Zero();
  V6d HTVr = V6d::Zero();
  double squared_residual_sum = 0.0;
  std::size_t candidate_correspondences = 0;
  std::size_t valid_correspondences = 0;
  ThreadACC(): HTVH(M6d::Zero()), HTVr(V6d::Zero()) {}
};


void SuperLIOReLoc::Observe(){
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
  registration_fitness_m2_ = 0.0;
  registration_inlier_ratio_ = 0.0;
  registration_valid_correspondences_ = 0;
  registration_converged_ = false;

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
            ivox_->getTopK_VN(point_world, top_K);
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
          ++local_acc.candidate_correspondences;
          effect_mask_[idx] = compute_error(abcd, point_world, _lengths[idx], error);
          if(!effect_mask_[idx]) continue;
          ++local_acc.valid_correspondences;
          local_acc.squared_residual_sum += static_cast<double>(error) * static_cast<double>(error);
          
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
    double squared_residual_sum = 0.0;
    std::size_t candidate_correspondences = 0;
    std::size_t valid_correspondences = 0;
    for(const auto& local_acc : tls_acc){
      sum_HTVH += local_acc.HTVH;
      sum_HTVr += local_acc.HTVr;
      squared_residual_sum += local_acc.squared_residual_sum;
      candidate_correspondences += local_acc.candidate_correspondences;
      valid_correspondences += local_acc.valid_correspondences;
    }
    registration_valid_correspondences_ = valid_correspondences;
    registration_fitness_m2_ = valid_correspondences > 0
        ? squared_residual_sum / static_cast<double>(valid_correspondences)
        : 0.0;
    registration_inlier_ratio_ = candidate_correspondences > 0
        ? static_cast<double>(valid_correspondences) / static_cast<double>(candidate_correspondences)
        : 0.0;
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

    effect_knn_num_ = _effect_knn_num;

    iter_num++;
  });

  registration_converged_ = kf_->DidLastUpdateConverge();

  frame_num_++;
}


void SuperLIOReLoc::UpdateMap() {
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


void SuperLIOReLoc::Output(){
  auto state = kf_->GetNavState();
  data_wrapper_->pub_odom(state, flg_re_path_);
  diagnostic_msgs::DiagnosticArray diagnostics;
  diagnostics.header.stamp.fromSec(state.timestamp);
  diagnostic_msgs::DiagnosticStatus registration;
  registration.name = "super_lio_registration";
  registration.hardware_id = "super_lio";
  registration.level = registration_converged_ && registration_valid_correspondences_ > 0
      && std::isfinite(registration_fitness_m2_) && std::isfinite(registration_inlier_ratio_)
      ? diagnostic_msgs::DiagnosticStatus::OK
      : diagnostic_msgs::DiagnosticStatus::WARN;
  registration.message = registration.level == diagnostic_msgs::DiagnosticStatus::OK
      ? "registration converged" : "registration quality unavailable";
  const auto add_value = [&registration](const std::string& key, const std::string& value) {
    diagnostic_msgs::KeyValue item;
    item.key = key;
    item.value = value;
    registration.values.push_back(item);
  };
  add_value("fitness_m2", std::to_string(registration_fitness_m2_));
  add_value("inlier_ratio", std::to_string(registration_inlier_ratio_));
  add_value("valid_correspondences", std::to_string(registration_valid_correspondences_));
  add_value("converged", registration_converged_ ? "true" : "false");
  diagnostics.status.push_back(registration);
  pub_registration_diag_.publish(diagnostics);
  flg_re_path_ = false;

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

  static std::size_t pub_map_count = -1;
  static std::size_t pub_count_base = 200;
  pub_map_count++;
  if(pub_map_count % pub_count_base != 0){
    return;
  }
  pub_map_count = 0;
  pub_count_base += 200;
  msg_global_map_.header.stamp = ros::Time().now();
  pub_map_.publish(msg_global_map_);
}

} // namespace END.
