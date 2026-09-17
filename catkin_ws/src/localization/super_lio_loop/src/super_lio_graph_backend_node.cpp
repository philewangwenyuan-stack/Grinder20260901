#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64.h>
#include <tf2_ros/transform_broadcaster.h>
#include <super_lio_loop/GetKeyframe.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#ifdef SUPER_LIO_HAS_FAST_GICP
#include <fast_gicp/gicp/fast_vgicp.hpp>
#endif

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
using CloudPtr = Cloud::Ptr;
using gtsam::symbol_shorthand::X;

double wrapAngle(double value) {
  while (value > kPi) value -= 2.0 * kPi;
  while (value < -kPi) value += 2.0 * kPi;
  return value;
}

gtsam::Pose3 poseFromOdom(const nav_msgs::Odometry& message) {
  const auto& q = message.pose.pose.orientation;
  const auto& p = message.pose.pose.position;
  return gtsam::Pose3(gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z),
                      gtsam::Point3(p.x, p.y, p.z));
}

Eigen::Matrix4f matrixFromPose(const gtsam::Pose3& pose) {
  return pose.matrix().cast<float>();
}

geometry_msgs::Pose poseMessage(const gtsam::Pose3& pose) {
  geometry_msgs::Pose output;
  const gtsam::Point3 translation = pose.translation();
  const Eigen::Quaterniond quaternion(pose.rotation().matrix());
  output.position.x = translation.x();
  output.position.y = translation.y();
  output.position.z = translation.z();
  output.orientation.x = quaternion.x();
  output.orientation.y = quaternion.y();
  output.orientation.z = quaternion.z();
  output.orientation.w = quaternion.w();
  return output;
}

double yawOf(const gtsam::Pose3& pose) {
  return pose.rotation().yaw();
}

double translationDistance(const gtsam::Pose3& lhs, const gtsam::Pose3& rhs) {
  return (lhs.translation() - rhs.translation()).norm();
}

struct ScanDescriptor {
  std::vector<float> cells;
  std::vector<float> ring_key;
};

ScanDescriptor makeDescriptor(const Cloud& cloud, int rings, int sectors, double radius) {
  ScanDescriptor result;
  result.cells.assign(static_cast<std::size_t>(rings * sectors), 0.0F);
  result.ring_key.assign(static_cast<std::size_t>(rings), 0.0F);
  for (const Point& point : cloud) {
    const double range = std::hypot(point.x, point.y);
    if (!std::isfinite(range) || range <= 0.1 || range >= radius) continue;
    const double angle = std::atan2(point.y, point.x) + kPi;
    const int ring = std::min(rings - 1, static_cast<int>(range / radius * rings));
    const int sector = std::min(sectors - 1, static_cast<int>(angle / (2.0 * kPi) * sectors));
    const std::size_t index = static_cast<std::size_t>(ring * sectors + sector);
    // Occupancy plus height gives stable indoor descriptors without relying on intensity calibration.
    result.cells[index] = std::max(result.cells[index], 1.0F + std::max(-1.0F, point.z));
  }
  for (int ring = 0; ring < rings; ++ring) {
    float sum = 0.0F;
    for (int sector = 0; sector < sectors; ++sector) {
      sum += result.cells[static_cast<std::size_t>(ring * sectors + sector)];
    }
    result.ring_key[static_cast<std::size_t>(ring)] = sum / static_cast<float>(sectors);
  }
  return result;
}

double ringKeyDistance(const ScanDescriptor& lhs, const ScanDescriptor& rhs) {
  double squared = 0.0;
  for (std::size_t index = 0; index < lhs.ring_key.size(); ++index) {
    const double delta = lhs.ring_key[index] - rhs.ring_key[index];
    squared += delta * delta;
  }
  return std::sqrt(squared);
}

std::pair<double, int> descriptorDistance(const ScanDescriptor& lhs,
                                          const ScanDescriptor& rhs,
                                          int rings, int sectors) {
  double best = 1.0;
  int best_shift = 0;
  for (int shift = 0; shift < sectors; ++shift) {
    double similarity_sum = 0.0;
    int valid_columns = 0;
    for (int sector = 0; sector < sectors; ++sector) {
      double dot = 0.0;
      double lhs_norm = 0.0;
      double rhs_norm = 0.0;
      for (int ring = 0; ring < rings; ++ring) {
        const double a = lhs.cells[static_cast<std::size_t>(ring * sectors + sector)];
        const int shifted = (sector + shift) % sectors;
        const double b = rhs.cells[static_cast<std::size_t>(ring * sectors + shifted)];
        dot += a * b;
        lhs_norm += a * a;
        rhs_norm += b * b;
      }
      if (lhs_norm > 1e-6 && rhs_norm > 1e-6) {
        similarity_sum += dot / std::sqrt(lhs_norm * rhs_norm);
        ++valid_columns;
      }
    }
    if (valid_columns == 0) continue;
    const double distance = 1.0 - similarity_sum / valid_columns;
    if (distance < best) {
      best = distance;
      best_shift = shift;
    }
  }
  return {best, best_shift};
}

diagnostic_msgs::KeyValue diagnosticValue(const std::string& key, const std::string& value) {
  diagnostic_msgs::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

}  // namespace

class IndustrialLoopBackend {
 public:
  IndustrialLoopBackend() : nh_(), private_nh_("~"), running_(true) {
    loadParameters();
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam_.reset(new gtsam::ISAM2(parameters));

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(corrected_odom_topic_, 20);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(corrected_cloud_topic_, 2);
    path_pub_ = nh_.advertise<nav_msgs::Path>(corrected_path_topic_, 1, true);
    status_pub_ = private_nh_.advertise<std_msgs::String>("status", 1, true);
    revision_pub_ = private_nh_.advertise<std_msgs::UInt64>("revision", 1, true);
    diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 2);
    keyframe_service_ = private_nh_.advertiseService(
        "get_keyframe", &IndustrialLoopBackend::getKeyframe, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 100, &IndustrialLoopBackend::odomCallback, this,
                              ros::TransportHints().tcpNoDelay());
    cloud_sub_ = nh_.subscribe(cloud_topic_, 2, &IndustrialLoopBackend::cloudCallback, this,
                               ros::TransportHints().tcpNoDelay());
    worker_ = std::thread(&IndustrialLoopBackend::workerLoop, this);
    publishStatus("WAITING_FOR_DATA");
#ifdef SUPER_LIO_HAS_FAST_GICP
    ROS_INFO("super_lio industrial backend: GTSAM iSAM2 + Scan Context + FastVGICP");
#else
    ROS_WARN("super_lio industrial backend: fast_gicp unavailable, using PCL GICP degraded mode");
#endif
  }

  ~IndustrialLoopBackend() {
    running_.store(false);
    queue_condition_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

 private:
  struct Keyframe {
    ros::Time stamp;
    gtsam::Pose3 raw_pose;
    gtsam::Pose3 optimized_pose;
    CloudPtr cloud;
    ScanDescriptor descriptor;
  };

  struct Pending {
    ros::Time stamp;
    gtsam::Pose3 raw_pose;
    CloudPtr cloud;
  };

  struct RegistrationResult {
    bool valid = false;
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    gtsam::Matrix66 covariance = gtsam::Matrix66::Identity();
    double fitness = std::numeric_limits<double>::infinity();
    double overlap = 0.0;
    double condition = std::numeric_limits<double>::infinity();
  };

  void loadParameters() {
    private_nh_.param<std::string>("odom_topic", odom_topic_, "/lio/odom");
    private_nh_.param<std::string>("cloud_topic", cloud_topic_, "/lio/map_cloud");
    private_nh_.param<std::string>("corrected_odom_topic", corrected_odom_topic_, "/lio/loop_odom");
    private_nh_.param<std::string>("corrected_cloud_topic", corrected_cloud_topic_, "/lio/loop_cloud");
    private_nh_.param<std::string>("corrected_path_topic", corrected_path_topic_, "/lio/loop_path");
    private_nh_.param<std::string>("map_frame", map_frame_, "map");
    private_nh_.param<std::string>("odom_frame", odom_frame_, "odom");
    private_nh_.param<std::string>("base_frame", base_frame_, "base_laser_link");
    private_nh_.param("publish_tf", publish_tf_, true);
    private_nh_.param("sync_tolerance", sync_tolerance_, 0.03);
    private_nh_.param("keyframe_distance", keyframe_distance_, 0.8);
    private_nh_.param("keyframe_angle_deg", keyframe_angle_deg_, 10.0);
    private_nh_.param("keyframe_voxel_size", voxel_size_, 0.25F);
    private_nh_.param("max_points_per_keyframe", max_points_, 8000);
    private_nh_.param("max_keyframes", max_keyframes_, 2000);
    private_nh_.param("exclude_recent_keyframes", exclude_recent_, 30);
    private_nh_.param("candidate_search_radius", search_radius_, 8.0);
    private_nh_.param("history_submap_size", submap_size_, 10);
    private_nh_.param("loop_check_interval", check_interval_, 3);
    private_nh_.param("descriptor_rings", descriptor_rings_, 20);
    private_nh_.param("descriptor_sectors", descriptor_sectors_, 60);
    private_nh_.param("descriptor_max_radius", descriptor_radius_, 40.0);
    private_nh_.param("descriptor_distance_threshold", descriptor_threshold_, 0.18);
    private_nh_.param("descriptor_candidate_count", descriptor_candidates_, 10);
    private_nh_.param("registration_threads", registration_threads_, 3);
    private_nh_.param("icp_max_iterations", registration_iterations_, 60);
    private_nh_.param("icp_max_correspondence_distance", correspondence_distance_, 1.0);
    private_nh_.param("icp_fitness_threshold", fitness_threshold_, 0.35);
    private_nh_.param("min_overlap_ratio", min_overlap_, 0.45);
    private_nh_.param("max_registration_condition_number", max_condition_, 1e6);
    private_nh_.param("icp_min_source_points", min_source_points_, 100);
    private_nh_.param("icp_min_target_points", min_target_points_, 500);
    private_nh_.param("max_loop_correction_translation", max_correction_translation_, 5.0);
    private_nh_.param("max_loop_correction_yaw_deg", max_correction_yaw_deg_, 45.0);
    private_nh_.param("loop_consistency_translation", consistency_translation_, 3.0);
    private_nh_.param("loop_consistency_yaw_deg", consistency_yaw_deg_, 30.0);
    private_nh_.param("loop_confirmation_count", confirmation_required_, 2);
    private_nh_.param("odom_translation_stddev", odom_translation_stddev_, 0.05);
    private_nh_.param("odom_yaw_stddev_deg", odom_yaw_stddev_deg_, 1.0);
    private_nh_.param("loop_huber_delta", huber_delta_, 3.0);
    keyframe_angle_ = keyframe_angle_deg_ * kPi / 180.0;
    max_correction_yaw_ = max_correction_yaw_deg_ * kPi / 180.0;
    consistency_yaw_ = consistency_yaw_deg_ * kPi / 180.0;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& message) {
    {
      std::lock_guard<std::mutex> lock(latest_mutex_);
      latest_odom_ = *message;
      have_odom_ = true;
    }
    gtsam::Pose3 correction;
    gtsam::Matrix66 covariance;
    {
      std::lock_guard<std::mutex> lock(correction_mutex_);
      correction = correction_;
      covariance = correction_covariance_;
    }
    const gtsam::Pose3 corrected = correction.compose(poseFromOdom(*message));
    nav_msgs::Odometry output = *message;
    output.header.frame_id = map_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose = poseMessage(corrected);
    addCorrectionCovariance(covariance, &output);
    odom_pub_.publish(output);
    if (publish_tf_) publishTransform(message->header.stamp, correction);
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    if (message->header.stamp.isZero()) return;
    nav_msgs::Odometry odom;
    {
      std::lock_guard<std::mutex> lock(latest_mutex_);
      if (!have_odom_) return;
      odom = latest_odom_;
    }
    if (std::abs((message->header.stamp - odom.header.stamp).toSec()) > sync_tolerance_) {
      ROS_WARN_THROTTLE(2.0, "industrial loop cloud/odom timestamp mismatch");
      return;
    }
    publishCorrectedCloud(message);
    const gtsam::Pose3 raw_pose = poseFromOdom(odom);
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (have_enqueued_ && translationDistance(raw_pose, last_enqueued_) < keyframe_distance_ &&
          std::abs(wrapAngle(yawOf(raw_pose) - yawOf(last_enqueued_))) < keyframe_angle_) return;
      if (pending_.size() >= 3U) {
        ROS_WARN_THROTTLE(2.0, "industrial loop worker behind; dropping keyframe");
        return;
      }
    }
    CloudPtr world(new Cloud());
    pcl::fromROSMsg(*message, *world);
    if (world->empty()) return;
    CloudPtr local(new Cloud());
    pcl::transformPointCloud(*world, *local, matrixFromPose(raw_pose.inverse()));
    pcl::VoxelGrid<Point> filter;
    filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    filter.setInputCloud(local);
    CloudPtr compact(new Cloud());
    filter.filter(*compact);
    if (compact->size() > static_cast<std::size_t>(max_points_)) {
      CloudPtr capped(new Cloud());
      capped->reserve(static_cast<std::size_t>(max_points_));
      const double stride = static_cast<double>(compact->size()) / max_points_;
      for (int index = 0; index < max_points_; ++index)
        capped->push_back(compact->at(static_cast<std::size_t>(index * stride)));
      compact = capped;
    }
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      pending_.push_back({message->header.stamp, raw_pose, compact});
      last_enqueued_ = raw_pose;
      have_enqueued_ = true;
    }
    queue_condition_.notify_one();
  }

  void workerLoop() {
    while (running_.load() && ros::ok()) {
      Pending item;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_condition_.wait(lock, [this]() { return !running_.load() || !pending_.empty(); });
        if (!running_.load()) break;
        item = std::move(pending_.front());
        pending_.pop_front();
      }
      processKeyframe(std::move(item));
    }
  }

  gtsam::SharedNoiseModel odometryNoise() const {
    gtsam::Vector6 sigmas;
    const double yaw_sigma = odom_yaw_stddev_deg_ * kPi / 180.0;
    sigmas << yaw_sigma * 0.5, yaw_sigma * 0.5, yaw_sigma,
              odom_translation_stddev_, odom_translation_stddev_, odom_translation_stddev_ * 0.5;
    return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
  }

  void processKeyframe(Pending item) {
    std::size_t index;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      if (keyframes_.size() >= static_cast<std::size_t>(max_keyframes_)) {
        publishStatus("CAPACITY_REACHED");
        return;
      }
      index = keyframes_.size();
      Keyframe frame{item.stamp, item.raw_pose, item.raw_pose, item.cloud,
                     makeDescriptor(*item.cloud, descriptor_rings_, descriptor_sectors_, descriptor_radius_)};
      gtsam::NonlinearFactorGraph factors;
      gtsam::Values values;
      if (index == 0U) {
        gtsam::Vector6 prior_sigmas;
        prior_sigmas << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3;
        factors.add(gtsam::PriorFactor<gtsam::Pose3>(
            X(0), frame.raw_pose, gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas)));
      } else {
        frame.optimized_pose = keyframes_.back().optimized_pose.compose(
            keyframes_.back().raw_pose.between(frame.raw_pose));
        factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
            X(index - 1U), X(index), keyframes_.back().raw_pose.between(frame.raw_pose),
            odometryNoise()));
      }
      values.insert(X(index), frame.optimized_pose);
      isam_->update(factors, values);
      frame.optimized_pose = isam_->calculateEstimate<gtsam::Pose3>(X(index));
      keyframes_.push_back(std::move(frame));
    }
    updateOutputs();
    publishStatus("TRACKING");
    if (index >= static_cast<std::size_t>(exclude_recent_) &&
        index % static_cast<std::size_t>(std::max(1, check_interval_)) == 0U) {
      tryLoop(index);
    }
  }

  int findCandidate(std::size_t current, double* descriptor_distance, int* yaw_shift) {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    const std::size_t limit = current - static_cast<std::size_t>(exclude_recent_);
    std::vector<std::pair<double, std::size_t>> ring_candidates;
    ring_candidates.reserve(limit);
    for (std::size_t index = 0; index < limit; ++index) {
      ring_candidates.push_back({ringKeyDistance(keyframes_[current].descriptor,
                                                 keyframes_[index].descriptor), index});
    }
    const std::size_t count = std::min(ring_candidates.size(),
        static_cast<std::size_t>(std::max(1, descriptor_candidates_)));
    std::partial_sort(ring_candidates.begin(), ring_candidates.begin() + count,
                      ring_candidates.end());
    int best = -1;
    *descriptor_distance = std::numeric_limits<double>::infinity();
    *yaw_shift = 0;
    for (std::size_t item = 0; item < count; ++item) {
      const std::size_t index = ring_candidates[item].second;
      const auto score = descriptorDistance(keyframes_[current].descriptor,
                                            keyframes_[index].descriptor,
                                            descriptor_rings_, descriptor_sectors_);
      if (score.first < *descriptor_distance) {
        *descriptor_distance = score.first;
        *yaw_shift = score.second;
        best = static_cast<int>(index);
      }
    }
    // Spatial proximity is a secondary candidate, but it must still pass descriptor validation.
    for (std::size_t index = 0; index < limit; ++index) {
      if (translationDistance(keyframes_[current].optimized_pose,
                              keyframes_[index].optimized_pose) > search_radius_) continue;
      const auto score = descriptorDistance(keyframes_[current].descriptor,
                                            keyframes_[index].descriptor,
                                            descriptor_rings_, descriptor_sectors_);
      if (score.first < *descriptor_distance) {
        *descriptor_distance = score.first;
        *yaw_shift = score.second;
        best = static_cast<int>(index);
      }
    }
    return *descriptor_distance <= descriptor_threshold_ ? best : -1;
  }

  CloudPtr transformedFrame(std::size_t index) const {
    CloudPtr output(new Cloud());
    pcl::transformPointCloud(*keyframes_[index].cloud, *output,
                             matrixFromPose(keyframes_[index].optimized_pose));
    return output;
  }

  RegistrationResult registerClouds(const CloudPtr& source, const CloudPtr& target,
                                    const Eigen::Matrix4f& guess) const {
    RegistrationResult result;
    Cloud aligned;
#ifdef SUPER_LIO_HAS_FAST_GICP
    fast_gicp::FastVGICP<Point, Point> registration;
    registration.setNumThreads(std::max(1, registration_threads_));
    registration.setResolution(voxel_size_);
#else
    pcl::GeneralizedIterativeClosestPoint<Point, Point> registration;
#endif
    registration.setMaximumIterations(registration_iterations_);
    registration.setMaxCorrespondenceDistance(correspondence_distance_);
    registration.setTransformationEpsilon(1e-6);
    registration.setInputSource(source);
    registration.setInputTarget(target);
    registration.align(aligned, guess);
    result.fitness = registration.getFitnessScore(correspondence_distance_);
    result.transform = registration.getFinalTransformation();
    if (!registration.hasConverged() || !std::isfinite(result.fitness) ||
        result.fitness > fitness_threshold_) return result;

    pcl::KdTreeFLANN<Point> tree;
    tree.setInputCloud(target);
    Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Zero();
    double squared_error = 0.0;
    std::size_t inliers = 0U;
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    for (const Point& point : aligned) {
      if (tree.nearestKSearch(point, 1, indices, distances) != 1 ||
          distances[0] > correspondence_distance_ * correspondence_distance_) continue;
      const Eigen::Vector3d p(point.x, point.y, point.z);
      const Point& nearest = target->at(static_cast<std::size_t>(indices[0]));
      const Eigen::Vector3d residual = p - Eigen::Vector3d(nearest.x, nearest.y, nearest.z);
      Eigen::Matrix<double, 3, 6> jacobian;
      jacobian << 0.0, p.z(), -p.y(), 1.0, 0.0, 0.0,
                 -p.z(), 0.0, p.x(), 0.0, 1.0, 0.0,
                  p.y(), -p.x(), 0.0, 0.0, 0.0, 1.0;
      hessian.noalias() += jacobian.transpose() * jacobian;
      squared_error += residual.squaredNorm();
      ++inliers;
    }
    result.overlap = aligned.empty() ? 0.0 : static_cast<double>(inliers) / aligned.size();
    if (inliers < 20U || result.overlap < min_overlap_) return result;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen(hessian);
    if (eigen.info() != Eigen::Success || eigen.eigenvalues().minCoeff() <= 1e-9) return result;
    result.condition = eigen.eigenvalues().maxCoeff() / eigen.eigenvalues().minCoeff();
    if (!std::isfinite(result.condition) || result.condition > max_condition_) return result;
    const double variance = std::max(1e-6, squared_error /
        std::max<double>(1.0, 3.0 * static_cast<double>(inliers) - 6.0));
    result.covariance = variance * hessian.inverse();
    result.valid = result.covariance.allFinite();
    return result;
  }

  void tryLoop(std::size_t current) {
    double descriptor_distance = 0.0;
    int yaw_shift = 0;
    const int candidate_value = findCandidate(current, &descriptor_distance, &yaw_shift);
    if (candidate_value < 0) {
      pending_confirmation_count_ = 0;
      pending_candidate_ = -1;
      return;
    }
    const std::size_t candidate = static_cast<std::size_t>(candidate_value);
    CloudPtr source;
    CloudPtr target(new Cloud());
    gtsam::Pose3 current_pose;
    gtsam::Pose3 candidate_pose;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      source = transformedFrame(current);
      current_pose = keyframes_[current].optimized_pose;
      candidate_pose = keyframes_[candidate].optimized_pose;
      const int start = std::max(0, static_cast<int>(candidate) - submap_size_);
      const int finish = std::min(static_cast<int>(current) - exclude_recent_,
                                  static_cast<int>(candidate) + submap_size_);
      for (int index = start; index <= finish; ++index)
        *target += *transformedFrame(static_cast<std::size_t>(index));
    }
    if (source->size() < static_cast<std::size_t>(min_source_points_) ||
        target->size() < static_cast<std::size_t>(min_target_points_)) return;
    pcl::VoxelGrid<Point> filter;
    filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    filter.setInputCloud(target);
    CloudPtr compact_target(new Cloud());
    filter.filter(*compact_target);

    const double descriptor_yaw = -2.0 * kPi * yaw_shift / descriptor_sectors_;
    const Eigen::Vector3f center = matrixFromPose(current_pose).block<3, 1>(0, 3);
    Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
    guess.block<3, 3>(0, 0) = Eigen::AngleAxisf(static_cast<float>(descriptor_yaw),
                                               Eigen::Vector3f::UnitZ()).toRotationMatrix();
    guess.block<3, 1>(0, 3) = center - guess.block<3, 3>(0, 0) * center;
    const RegistrationResult registration = registerClouds(source, compact_target, guess);
    if (!registration.valid) {
      pending_confirmation_count_ = 0;
      pending_candidate_ = -1;
      publishDiagnostics("LOOP_REJECTED", descriptor_distance, registration);
      return;
    }
    const gtsam::Pose3 registration_correction(registration.transform.cast<double>());
    const gtsam::Pose3 corrected_current = registration_correction.compose(current_pose);
    const gtsam::Pose3 correction_delta = current_pose.between(corrected_current);
    if (correction_delta.translation().norm() > max_correction_translation_ ||
        std::abs(yawOf(correction_delta)) > max_correction_yaw_) {
      pending_confirmation_count_ = 0;
      pending_candidate_ = -1;
      return;
    }
    const gtsam::Pose3 predicted = candidate_pose.between(current_pose);
    const gtsam::Pose3 measured = candidate_pose.between(corrected_current);
    const gtsam::Pose3 consistency = predicted.between(measured);
    if (consistency.translation().norm() > consistency_translation_ ||
        std::abs(yawOf(consistency)) > consistency_yaw_) {
      pending_confirmation_count_ = 0;
      pending_candidate_ = -1;
      return;
    }

    if (pending_candidate_ < 0 ||
        std::abs(pending_candidate_ - static_cast<int>(candidate)) > submap_size_) {
      pending_candidate_ = static_cast<int>(candidate);
      pending_confirmation_count_ = 1;
    } else {
      ++pending_confirmation_count_;
    }
    if (pending_confirmation_count_ < std::max(1, confirmation_required_)) {
      publishStatus("LOOP_PENDING_CONFIRMATION");
      publishDiagnostics("LOOP_PENDING_CONFIRMATION", descriptor_distance, registration);
      return;
    }

    gtsam::Matrix66 loop_covariance =
        0.5 * (registration.covariance + registration.covariance.transpose());
    const double rotation_floor = std::pow(0.5 * kPi / 180.0, 2);
    const double translation_floor = 0.02 * 0.02;
    for (int axis = 0; axis < 3; ++axis) {
      loop_covariance(axis, axis) = std::max(loop_covariance(axis, axis), rotation_floor);
      loop_covariance(axis + 3, axis + 3) =
          std::max(loop_covariance(axis + 3, axis + 3), translation_floor);
    }

    // The registration Hessian uses [rotation, translation], matching Pose3 tangent ordering.
    const auto gaussian = gtsam::noiseModel::Gaussian::Covariance(loop_covariance);
    const auto robust = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(huber_delta_), gaussian);
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      gtsam::NonlinearFactorGraph factor;
      factor.add(gtsam::BetweenFactor<gtsam::Pose3>(X(candidate), X(current), measured, robust));
      isam_->update(factor, gtsam::Values());
      isam_->update();
      const gtsam::Values estimate = isam_->calculateEstimate();
      for (std::size_t index = 0; index < keyframes_.size(); ++index)
        keyframes_[index].optimized_pose = estimate.at<gtsam::Pose3>(X(index));
    }
    ++revision_;
    pending_confirmation_count_ = 0;
    pending_candidate_ = -1;
    std_msgs::UInt64 revision;
    revision.data = revision_.load();
    revision_pub_.publish(revision);
    publishStatus("LOOP_ACCEPTED");
    publishDiagnostics("LOOP_ACCEPTED", descriptor_distance, registration);
    updateOutputs();
  }

  void updateOutputs() {
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = map_frame_;
    gtsam::Pose3 correction;
    gtsam::Matrix66 covariance = gtsam::Matrix66::Identity() * 1e-3;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      if (keyframes_.empty()) return;
      const std::size_t latest = keyframes_.size() - 1U;
      correction = keyframes_[latest].optimized_pose.compose(keyframes_[latest].raw_pose.inverse());
      try {
        covariance = isam_->marginalCovariance(X(latest));
      } catch (const std::exception& error) {
        ROS_WARN_STREAM_THROTTLE(2.0, "GTSAM marginal covariance unavailable: " << error.what());
      }
      path.poses.reserve(keyframes_.size());
      for (const Keyframe& frame : keyframes_) {
        geometry_msgs::PoseStamped item;
        item.header.stamp = frame.stamp;
        item.header.frame_id = map_frame_;
        item.pose = poseMessage(frame.optimized_pose);
        path.poses.push_back(item);
      }
    }
    {
      std::lock_guard<std::mutex> lock(correction_mutex_);
      correction_ = correction;
      correction_covariance_ = covariance;
    }
    path_pub_.publish(path);
  }

  void publishCorrectedCloud(const sensor_msgs::PointCloud2ConstPtr& message) {
    gtsam::Pose3 correction;
    {
      std::lock_guard<std::mutex> lock(correction_mutex_);
      correction = correction_;
    }
    Cloud input;
    Cloud output_cloud;
    pcl::fromROSMsg(*message, input);
    pcl::transformPointCloud(input, output_cloud, matrixFromPose(correction));
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(output_cloud, output);
    output.header = message->header;
    output.header.frame_id = map_frame_;
    cloud_pub_.publish(output);
  }

  void publishTransform(const ros::Time& stamp, const gtsam::Pose3& correction) {
    geometry_msgs::TransformStamped output;
    output.header.stamp = stamp;
    output.header.frame_id = map_frame_;
    output.child_frame_id = odom_frame_;
    const geometry_msgs::Pose pose = poseMessage(correction);
    output.transform.translation.x = pose.position.x;
    output.transform.translation.y = pose.position.y;
    output.transform.translation.z = pose.position.z;
    output.transform.rotation = pose.orientation;
    tf_broadcaster_.sendTransform(output);
  }

  void addCorrectionCovariance(const gtsam::Matrix66& covariance,
                               nav_msgs::Odometry* output) const {
    // GTSAM Pose3 tangent order is rotation then translation; ROS is translation then rotation.
    const int ros_to_gtsam[6] = {3, 4, 5, 0, 1, 2};
    for (int row = 0; row < 6; ++row)
      for (int column = 0; column < 6; ++column)
        output->pose.covariance[static_cast<std::size_t>(row * 6 + column)] =
            covariance(ros_to_gtsam[row], ros_to_gtsam[column]);
  }

  void publishStatus(const std::string& value) {
    std_msgs::String message;
    message.data = value;
    status_pub_.publish(message);
  }

  bool getKeyframe(super_lio_loop::GetKeyframe::Request& request,
                   super_lio_loop::GetKeyframe::Response& response) {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    response.current_revision = revision_.load();
    response.total = static_cast<std::uint32_t>(keyframes_.size());
    if (request.revision != response.current_revision || request.index >= keyframes_.size()) {
      response.success = false;
      response.message = request.revision != response.current_revision
          ? "graph revision changed" : "keyframe index out of range";
      return true;
    }
    const Keyframe& frame = keyframes_[request.index];
    response.pose = poseMessage(frame.optimized_pose);
    pcl::toROSMsg(*frame.cloud, response.cloud);
    response.cloud.header.stamp = frame.stamp;
    response.cloud.header.frame_id = base_frame_;
    response.success = true;
    response.message = "ok";
    return true;
  }

  void publishDiagnostics(const std::string& state, double descriptor_distance,
                          const RegistrationResult& registration) {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "super_lio_loop/industrial_backend";
    status.hardware_id = "mid360";
    status.level = state == "LOOP_ACCEPTED" ? diagnostic_msgs::DiagnosticStatus::OK
                                             : diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = state;
    status.values.push_back(diagnosticValue("descriptor_distance", std::to_string(descriptor_distance)));
    status.values.push_back(diagnosticValue("registration_fitness", std::to_string(registration.fitness)));
    status.values.push_back(diagnosticValue("overlap_ratio", std::to_string(registration.overlap)));
    status.values.push_back(diagnosticValue("condition_number", std::to_string(registration.condition)));
    status.values.push_back(diagnosticValue("revision", std::to_string(revision_.load())));
    array.status.push_back(status);
    diagnostics_pub_.publish(array);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber cloud_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher cloud_pub_;
  ros::Publisher path_pub_;
  ros::Publisher status_pub_;
  ros::Publisher revision_pub_;
  ros::Publisher diagnostics_pub_;
  ros::ServiceServer keyframe_service_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::string odom_topic_, cloud_topic_, corrected_odom_topic_, corrected_cloud_topic_;
  std::string corrected_path_topic_, map_frame_, odom_frame_, base_frame_;
  bool publish_tf_ = true;
  double sync_tolerance_ = 0.03, keyframe_distance_ = 0.8, keyframe_angle_deg_ = 10.0;
  double keyframe_angle_ = 0.0, search_radius_ = 8.0, descriptor_radius_ = 40.0;
  double descriptor_threshold_ = 0.18, correspondence_distance_ = 1.0;
  double fitness_threshold_ = 0.35, min_overlap_ = 0.45, max_condition_ = 1e6;
  double max_correction_translation_ = 5.0, max_correction_yaw_deg_ = 45.0;
  double max_correction_yaw_ = 0.0, consistency_translation_ = 3.0;
  double consistency_yaw_deg_ = 30.0, consistency_yaw_ = 0.0;
  double odom_translation_stddev_ = 0.05, odom_yaw_stddev_deg_ = 1.0, huber_delta_ = 3.0;
  float voxel_size_ = 0.25F;
  int max_points_ = 8000, max_keyframes_ = 2000, exclude_recent_ = 30, submap_size_ = 10;
  int check_interval_ = 3, descriptor_rings_ = 20, descriptor_sectors_ = 60;
  int descriptor_candidates_ = 10, registration_threads_ = 3, registration_iterations_ = 60;
  int min_source_points_ = 100, min_target_points_ = 500;
  int confirmation_required_ = 2;
  int pending_candidate_ = -1;
  int pending_confirmation_count_ = 0;

  std::mutex latest_mutex_;
  nav_msgs::Odometry latest_odom_;
  bool have_odom_ = false;
  std::mutex queue_mutex_;
  std::condition_variable queue_condition_;
  std::deque<Pending> pending_;
  gtsam::Pose3 last_enqueued_;
  bool have_enqueued_ = false;
  std::thread worker_;
  std::atomic<bool> running_;

  std::mutex graph_mutex_;
  std::vector<Keyframe> keyframes_;
  std::unique_ptr<gtsam::ISAM2> isam_;
  std::mutex correction_mutex_;
  gtsam::Pose3 correction_;
  gtsam::Matrix66 correction_covariance_ = gtsam::Matrix66::Identity() * 1e-3;
  std::atomic<std::uint64_t> revision_{0U};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "super_lio_graph_backend");
  IndustrialLoopBackend node;
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
