#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
using CloudPtr = Cloud::Ptr;

double wrapAngle(double angle) {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

struct Pose2 {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

Pose2 compose(const Pose2& a, const Pose2& b) {
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  return {a.x + c * b.x - s * b.y,
          a.y + s * b.x + c * b.y,
          wrapAngle(a.yaw + b.yaw)};
}

Pose2 inverse(const Pose2& pose) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {-c * pose.x - s * pose.y,
           s * pose.x - c * pose.y,
          -pose.yaw};
}

Pose2 between(const Pose2& from, const Pose2& to) {
  return compose(inverse(from), to);
}

Eigen::Matrix4f poseMatrix(const Pose2& pose) {
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  const float c = static_cast<float>(std::cos(pose.yaw));
  const float s = static_cast<float>(std::sin(pose.yaw));
  matrix(0, 0) = c;
  matrix(0, 1) = -s;
  matrix(1, 0) = s;
  matrix(1, 1) = c;
  matrix(0, 3) = static_cast<float>(pose.x);
  matrix(1, 3) = static_cast<float>(pose.y);
  return matrix;
}

Pose2 poseFromMatrix(const Eigen::Matrix4f& matrix) {
  return {matrix(0, 3), matrix(1, 3),
          std::atan2(matrix(1, 0), matrix(0, 0))};
}

Pose2 poseFromOdometry(const nav_msgs::Odometry& odometry) {
  const auto& position = odometry.pose.pose.position;
  const auto& orientation = odometry.pose.pose.orientation;
  tf2::Quaternion quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  return {position.x, position.y, yaw};
}

double planarDistance(const Pose2& a, const Pose2& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

}  // namespace

class SuperLioLoop {
 public:
  SuperLioLoop()
      : nh_(), private_nh_("~"), running_(true), loop_revision_(0) {
    loadParameters();

    corrected_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(corrected_odom_topic_, 20);
    corrected_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(corrected_cloud_topic_, 2);
    corrected_path_pub_ = nh_.advertise<nav_msgs::Path>(corrected_path_topic_, 1, true);
    marker_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("loop_markers", 1, true);
    status_pub_ = private_nh_.advertise<std_msgs::String>("status", 1, true);
    revision_pub_ = private_nh_.advertise<std_msgs::UInt64>("revision", 1, true);

    odom_sub_ = nh_.subscribe(odom_topic_, 100, &SuperLioLoop::odomCallback, this,
                              ros::TransportHints().tcpNoDelay());
    cloud_sub_ = nh_.subscribe(cloud_topic_, 2, &SuperLioLoop::cloudCallback, this,
                               ros::TransportHints().tcpNoDelay());
    worker_ = std::thread(&SuperLioLoop::workerLoop, this);
    publishStatus("WAITING_FOR_DATA");

    ROS_INFO_STREAM("super_lio_loop ready: " << odom_topic_ << " + " << cloud_topic_
                    << " -> " << corrected_odom_topic_ << ", max keyframes "
                    << max_keyframes_);
  }

  ~SuperLioLoop() {
    running_.store(false);
    queue_condition_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

 private:
  struct Keyframe {
    ros::Time stamp;
    Pose2 raw_pose;
    Pose2 optimized_pose;
    CloudPtr local_cloud;
  };

  struct Edge {
    std::size_t from = 0;
    std::size_t to = 0;
    Pose2 measurement;
    Eigen::Vector3d information = Eigen::Vector3d::Ones();
    bool loop = false;
  };

  struct PendingKeyframe {
    ros::Time stamp;
    Pose2 raw_pose;
    CloudPtr local_cloud;
  };

  void loadParameters() {
    private_nh_.param("enabled", enabled_, true);
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
    private_nh_.param("keyframe_voxel_size", keyframe_voxel_size_, 0.25F);
    private_nh_.param("max_points_per_keyframe", max_points_per_keyframe_, 8000);
    private_nh_.param("max_keyframes", max_keyframes_, 2000);
    private_nh_.param("exclude_recent_keyframes", exclude_recent_keyframes_, 30);
    private_nh_.param("candidate_search_radius", candidate_search_radius_, 8.0);
    private_nh_.param("history_submap_size", history_submap_size_, 10);
    private_nh_.param("loop_check_interval", loop_check_interval_, 3);
    private_nh_.param("icp_max_iterations", icp_max_iterations_, 60);
    private_nh_.param("icp_max_correspondence_distance", icp_max_correspondence_distance_, 1.0);
    private_nh_.param("icp_fitness_threshold", icp_fitness_threshold_, 0.35);
    private_nh_.param("icp_min_source_points", icp_min_source_points_, 100);
    private_nh_.param("icp_min_target_points", icp_min_target_points_, 500);
    private_nh_.param("max_loop_correction_translation", max_loop_correction_translation_, 5.0);
    private_nh_.param("max_loop_correction_yaw_deg", max_loop_correction_yaw_deg_, 45.0);
    private_nh_.param("odom_translation_stddev", odom_translation_stddev_, 0.05);
    private_nh_.param("odom_yaw_stddev_deg", odom_yaw_stddev_deg_, 1.0);
    private_nh_.param("loop_translation_stddev", loop_translation_stddev_, 0.10);
    private_nh_.param("loop_yaw_stddev_deg", loop_yaw_stddev_deg_, 2.0);
    private_nh_.param("loop_huber_delta", loop_huber_delta_, 3.0);
    private_nh_.param("optimizer_iterations", optimizer_iterations_, 10);
    private_nh_.param("optimizer_damping", optimizer_damping_, 1e-6);

    keyframe_angle_ = keyframe_angle_deg_ * kPi / 180.0;
    max_loop_correction_yaw_ = max_loop_correction_yaw_deg_ * kPi / 180.0;
    const double odom_yaw_stddev = odom_yaw_stddev_deg_ * kPi / 180.0;
    const double loop_yaw_stddev = loop_yaw_stddev_deg_ * kPi / 180.0;
    odom_information_ = {1.0 / (odom_translation_stddev_ * odom_translation_stddev_),
                         1.0 / (odom_translation_stddev_ * odom_translation_stddev_),
                         1.0 / (odom_yaw_stddev * odom_yaw_stddev)};
    loop_information_ = {1.0 / (loop_translation_stddev_ * loop_translation_stddev_),
                         1.0 / (loop_translation_stddev_ * loop_translation_stddev_),
                         1.0 / (loop_yaw_stddev * loop_yaw_stddev)};
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& message) {
    {
      std::lock_guard<std::mutex> lock(latest_odom_mutex_);
      latest_odom_ = *message;
      have_odom_ = true;
    }
    if (!enabled_) return;

    Pose2 correction;
    Eigen::Matrix3d correction_covariance;
    {
      std::lock_guard<std::mutex> lock(correction_mutex_);
      correction = correction_;
      correction_covariance = correction_covariance_;
    }
    const Pose2 corrected = compose(correction, poseFromOdometry(*message));
    nav_msgs::Odometry output = *message;
    output.header.frame_id = map_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose.position.x = corrected.x;
    output.pose.pose.position.y = corrected.y;

    const auto& raw_orientation = message->pose.pose.orientation;
    tf2::Quaternion raw_quaternion(raw_orientation.x, raw_orientation.y,
                                   raw_orientation.z, raw_orientation.w);
    double roll = 0.0;
    double pitch = 0.0;
    double ignored_yaw = 0.0;
    tf2::Matrix3x3(raw_quaternion).getRPY(roll, pitch, ignored_yaw);
    tf2::Quaternion corrected_quaternion;
    corrected_quaternion.setRPY(roll, pitch, corrected.yaw);
    output.pose.pose.orientation.x = corrected_quaternion.x();
    output.pose.pose.orientation.y = corrected_quaternion.y();
    output.pose.pose.orientation.z = corrected_quaternion.z();
    output.pose.pose.orientation.w = corrected_quaternion.w();
    output.pose.covariance[0] += correction_covariance(0, 0);
    output.pose.covariance[1] += correction_covariance(0, 1);
    output.pose.covariance[5] += correction_covariance(0, 2);
    output.pose.covariance[6] += correction_covariance(1, 0);
    output.pose.covariance[7] += correction_covariance(1, 1);
    output.pose.covariance[11] += correction_covariance(1, 2);
    output.pose.covariance[30] += correction_covariance(2, 0);
    output.pose.covariance[31] += correction_covariance(2, 1);
    output.pose.covariance[35] += correction_covariance(2, 2);
    corrected_odom_pub_.publish(output);

    if (publish_tf_) {
      geometry_msgs::TransformStamped transform;
      transform.header = message->header;
      transform.header.frame_id = map_frame_;
      transform.child_frame_id = odom_frame_;
      transform.transform.translation.x = correction.x;
      transform.transform.translation.y = correction.y;
      transform.transform.rotation.z = std::sin(0.5 * correction.yaw);
      transform.transform.rotation.w = std::cos(0.5 * correction.yaw);
      tf_broadcaster_.sendTransform(transform);
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message) {
    if (!enabled_ || message->header.stamp.isZero()) return;

    nav_msgs::Odometry odometry;
    {
      std::lock_guard<std::mutex> lock(latest_odom_mutex_);
      if (!have_odom_) return;
      odometry = latest_odom_;
    }
    if (std::abs((message->header.stamp - odometry.header.stamp).toSec()) > sync_tolerance_) {
      ROS_WARN_THROTTLE(2.0, "Loop backend cloud/odom timestamp mismatch: %.3f s",
                        std::abs((message->header.stamp - odometry.header.stamp).toSec()));
      return;
    }

    publishCorrectedCloud(message);
    const Pose2 raw_pose = poseFromOdometry(odometry);
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (have_enqueued_pose_ &&
          planarDistance(raw_pose, last_enqueued_pose_) < keyframe_distance_ &&
          std::abs(wrapAngle(raw_pose.yaw - last_enqueued_pose_.yaw)) < keyframe_angle_) {
        return;
      }
      if (pending_keyframes_.size() >= 3U) {
        ROS_WARN_THROTTLE(2.0, "Loop keyframe worker is behind; dropping candidate");
        return;
      }
    }

    CloudPtr world_cloud(new Cloud());
    pcl::fromROSMsg(*message, *world_cloud);
    if (world_cloud->empty()) return;

    Eigen::Quaternionf quaternion(
        static_cast<float>(odometry.pose.pose.orientation.w),
        static_cast<float>(odometry.pose.pose.orientation.x),
        static_cast<float>(odometry.pose.pose.orientation.y),
        static_cast<float>(odometry.pose.pose.orientation.z));
    Eigen::Matrix4f world_from_sensor = Eigen::Matrix4f::Identity();
    world_from_sensor.block<3, 3>(0, 0) = quaternion.normalized().toRotationMatrix();
    world_from_sensor(0, 3) = static_cast<float>(odometry.pose.pose.position.x);
    world_from_sensor(1, 3) = static_cast<float>(odometry.pose.pose.position.y);
    world_from_sensor(2, 3) = static_cast<float>(odometry.pose.pose.position.z);

    CloudPtr local_cloud(new Cloud());
    pcl::transformPointCloud(*world_cloud, *local_cloud, world_from_sensor.inverse());
    pcl::VoxelGrid<Point> voxel_filter;
    voxel_filter.setLeafSize(keyframe_voxel_size_, keyframe_voxel_size_, keyframe_voxel_size_);
    voxel_filter.setInputCloud(local_cloud);
    CloudPtr filtered(new Cloud());
    voxel_filter.filter(*filtered);
    if (filtered->size() > static_cast<std::size_t>(max_points_per_keyframe_)) {
      CloudPtr capped(new Cloud());
      capped->reserve(max_points_per_keyframe_);
      const double stride = static_cast<double>(filtered->size()) / max_points_per_keyframe_;
      for (int index = 0; index < max_points_per_keyframe_; ++index) {
        capped->push_back(filtered->at(static_cast<std::size_t>(index * stride)));
      }
      filtered = capped;
    }

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      pending_keyframes_.push_back({message->header.stamp, raw_pose, filtered});
      last_enqueued_pose_ = raw_pose;
      have_enqueued_pose_ = true;
    }
    queue_condition_.notify_one();
  }

  void publishCorrectedCloud(const sensor_msgs::PointCloud2ConstPtr& message) {
    Pose2 correction;
    {
      std::lock_guard<std::mutex> lock(correction_mutex_);
      correction = correction_;
    }
    Cloud raw;
    pcl::fromROSMsg(*message, raw);
    Cloud corrected;
    pcl::transformPointCloud(raw, corrected, poseMatrix(correction));
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(corrected, output);
    output.header = message->header;
    output.header.frame_id = map_frame_;
    corrected_cloud_pub_.publish(output);
  }

  void workerLoop() {
    while (running_.load() && ros::ok()) {
      PendingKeyframe pending;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_condition_.wait(lock, [this]() {
          return !running_.load() || !pending_keyframes_.empty();
        });
        if (!running_.load()) break;
        pending = std::move(pending_keyframes_.front());
        pending_keyframes_.pop_front();
      }
      processKeyframe(std::move(pending));
    }
  }

  void processKeyframe(PendingKeyframe pending) {
    std::size_t current_index = 0;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      if (keyframes_.size() >= static_cast<std::size_t>(max_keyframes_)) {
        publishStatus("CAPACITY_REACHED");
        ROS_ERROR_THROTTLE(5.0, "Loop keyframe capacity reached; loop backend is frozen");
        return;
      }
      Keyframe keyframe{pending.stamp, pending.raw_pose, pending.raw_pose, pending.local_cloud};
      current_index = keyframes_.size();
      if (!keyframes_.empty()) {
        keyframe.optimized_pose = compose(
            keyframes_.back().optimized_pose,
            between(keyframes_.back().raw_pose, keyframe.raw_pose));
        edges_.push_back({current_index - 1U, current_index,
                          between(keyframes_.back().raw_pose, keyframe.raw_pose),
                          odom_information_, false});
      }
      keyframes_.push_back(std::move(keyframe));
    }

    updateCorrectionAndPublishPath();
    publishStatus("TRACKING");
    if (current_index < static_cast<std::size_t>(exclude_recent_keyframes_) ||
        current_index % static_cast<std::size_t>(std::max(1, loop_check_interval_)) != 0U) {
      return;
    }

    const int candidate = findLoopCandidate(current_index);
    if (candidate < 0) return;
    verifyAndAddLoop(static_cast<std::size_t>(candidate), current_index);
  }

  int findLoopCandidate(std::size_t current_index) {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    const Pose2 current_pose = keyframes_[current_index].optimized_pose;
    const std::size_t candidate_limit = current_index -
        static_cast<std::size_t>(exclude_recent_keyframes_);
    int best_index = -1;
    double best_distance = candidate_search_radius_;
    for (std::size_t index = 0; index < candidate_limit; ++index) {
      const double distance = planarDistance(current_pose, keyframes_[index].optimized_pose);
      if (distance < best_distance) {
        best_distance = distance;
        best_index = static_cast<int>(index);
      }
    }
    return best_index;
  }

  CloudPtr transformedKeyframe(std::size_t index) const {
    CloudPtr transformed(new Cloud());
    pcl::transformPointCloud(*keyframes_[index].local_cloud, *transformed,
                             poseMatrix(keyframes_[index].optimized_pose));
    return transformed;
  }

  void verifyAndAddLoop(std::size_t candidate_index, std::size_t current_index) {
    CloudPtr source;
    CloudPtr target(new Cloud());
    Pose2 current_pose;
    Pose2 candidate_pose;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      source = transformedKeyframe(current_index);
      current_pose = keyframes_[current_index].optimized_pose;
      candidate_pose = keyframes_[candidate_index].optimized_pose;
      const int start = std::max(0, static_cast<int>(candidate_index) - history_submap_size_);
      // Never contaminate the historical target with the current/recent source frames.
      const int finish = std::min(static_cast<int>(current_index) - exclude_recent_keyframes_,
                                  static_cast<int>(candidate_index) + history_submap_size_);
      for (int index = start; index <= finish; ++index) {
        *target += *transformedKeyframe(static_cast<std::size_t>(index));
      }
    }
    if (source->size() < static_cast<std::size_t>(icp_min_source_points_) ||
        target->size() < static_cast<std::size_t>(icp_min_target_points_)) {
      return;
    }

    pcl::VoxelGrid<Point> target_filter;
    target_filter.setLeafSize(keyframe_voxel_size_, keyframe_voxel_size_, keyframe_voxel_size_);
    target_filter.setInputCloud(target);
    CloudPtr target_filtered(new Cloud());
    target_filter.filter(*target_filtered);

    pcl::IterativeClosestPoint<Point, Point> icp;
    icp.setMaximumIterations(icp_max_iterations_);
    icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setInputSource(source);
    icp.setInputTarget(target_filtered);
    Cloud aligned;
    icp.align(aligned);
    const double fitness = icp.getFitnessScore(icp_max_correspondence_distance_);
    if (!icp.hasConverged() || !std::isfinite(fitness) || fitness > icp_fitness_threshold_) {
      ROS_INFO_STREAM("Loop rejected " << candidate_index << " <- " << current_index
                      << ", ICP fitness " << fitness);
      return;
    }

    const Pose2 correction = poseFromMatrix(icp.getFinalTransformation());
    if (std::hypot(correction.x, correction.y) > max_loop_correction_translation_ ||
        std::abs(correction.yaw) > max_loop_correction_yaw_) {
      ROS_WARN_STREAM("Loop rejected by correction gate: " << std::hypot(correction.x, correction.y)
                      << " m, " << correction.yaw * 180.0 / kPi << " deg");
      return;
    }

    const Pose2 corrected_current = compose(correction, current_pose);
    Edge loop_edge;
    loop_edge.from = candidate_index;
    loop_edge.to = current_index;
    loop_edge.measurement = between(candidate_pose, corrected_current);
    loop_edge.information = loop_information_ / std::max(1.0, fitness / 0.05);
    loop_edge.loop = true;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      edges_.push_back(loop_edge);
      optimizeGraphLocked();
    }
    ++loop_revision_;
    std_msgs::UInt64 revision;
    revision.data = loop_revision_.load();
    revision_pub_.publish(revision);
    publishStatus("LOOP_ACCEPTED");
    publishLoopMarkers(candidate_index, current_index);
    updateCorrectionAndPublishPath();
    ROS_WARN_STREAM("Loop accepted " << candidate_index << " <- " << current_index
                    << ", ICP fitness " << fitness);
  }

  Eigen::Vector3d edgeResidual(const Edge& edge) const {
    const Pose2 prediction = between(keyframes_[edge.from].optimized_pose,
                                     keyframes_[edge.to].optimized_pose);
    return {prediction.x - edge.measurement.x,
            prediction.y - edge.measurement.y,
            wrapAngle(prediction.yaw - edge.measurement.yaw)};
  }

  void optimizeGraphLocked() {
    const std::size_t node_count = keyframes_.size();
    if (node_count < 2U) return;
    const int dimension = static_cast<int>((node_count - 1U) * 3U);
    Eigen::SparseMatrix<double> final_hessian;

    for (int iteration = 0; iteration < optimizer_iterations_; ++iteration) {
      std::vector<Eigen::Triplet<double>> triplets;
      triplets.reserve(edges_.size() * 36U + static_cast<std::size_t>(dimension));
      Eigen::VectorXd gradient = Eigen::VectorXd::Zero(dimension);

      auto add_block = [&triplets](int row, int col, const Eigen::Matrix3d& block) {
        for (int r = 0; r < 3; ++r) {
          for (int c = 0; c < 3; ++c) {
            triplets.emplace_back(row + r, col + c, block(r, c));
          }
        }
      };

      for (const Edge& edge : edges_) {
        const Pose2& from = keyframes_[edge.from].optimized_pose;
        const Pose2& to = keyframes_[edge.to].optimized_pose;
        const Pose2 prediction = between(from, to);
        Eigen::Vector3d residual = edgeResidual(edge);
        double robust_weight = 1.0;
        if (edge.loop) {
          const double normalized_error = std::sqrt(
              (residual.array().square() * edge.information.array()).sum());
          if (normalized_error > loop_huber_delta_) {
            robust_weight = loop_huber_delta_ / normalized_error;
          }
        }
        const Eigen::Matrix3d information =
            robust_weight * edge.information.asDiagonal();
        const double c = std::cos(from.yaw);
        const double s = std::sin(from.yaw);
        Eigen::Matrix3d jacobian_from;
        jacobian_from << -c, -s, prediction.y,
                          s, -c, -prediction.x,
                          0.0, 0.0, -1.0;
        Eigen::Matrix3d jacobian_to;
        jacobian_to << c, s, 0.0,
                      -s, c, 0.0,
                       0.0, 0.0, 1.0;

        const bool from_variable = edge.from > 0U;
        const bool to_variable = edge.to > 0U;
        const int from_offset = static_cast<int>((edge.from - (from_variable ? 1U : 0U)) * 3U);
        const int to_offset = static_cast<int>((edge.to - (to_variable ? 1U : 0U)) * 3U);
        if (from_variable) {
          add_block(from_offset, from_offset,
                    jacobian_from.transpose() * information * jacobian_from);
          gradient.segment<3>(from_offset) +=
              jacobian_from.transpose() * information * residual;
        }
        if (to_variable) {
          add_block(to_offset, to_offset,
                    jacobian_to.transpose() * information * jacobian_to);
          gradient.segment<3>(to_offset) +=
              jacobian_to.transpose() * information * residual;
        }
        if (from_variable && to_variable) {
          const Eigen::Matrix3d cross =
              jacobian_from.transpose() * information * jacobian_to;
          add_block(from_offset, to_offset, cross);
          add_block(to_offset, from_offset, cross.transpose());
        }
      }
      for (int index = 0; index < dimension; ++index) {
        triplets.emplace_back(index, index, optimizer_damping_);
      }

      Eigen::SparseMatrix<double> hessian(dimension, dimension);
      hessian.setFromTriplets(triplets.begin(), triplets.end());
      Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
      solver.compute(hessian);
      if (solver.info() != Eigen::Success) {
        ROS_ERROR("Loop pose graph factorization failed");
        return;
      }
      const Eigen::VectorXd increment = solver.solve(-gradient);
      if (solver.info() != Eigen::Success || !increment.allFinite()) {
        ROS_ERROR("Loop pose graph solve failed");
        return;
      }
      for (std::size_t index = 1; index < node_count; ++index) {
        const int offset = static_cast<int>((index - 1U) * 3U);
        keyframes_[index].optimized_pose.x += increment[offset];
        keyframes_[index].optimized_pose.y += increment[offset + 1];
        keyframes_[index].optimized_pose.yaw = wrapAngle(
            keyframes_[index].optimized_pose.yaw + increment[offset + 2]);
      }
      final_hessian = hessian;
      if (increment.lpNorm<Eigen::Infinity>() < 1e-5) break;
    }

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> covariance_solver;
    covariance_solver.compute(final_hessian);
    if (covariance_solver.info() == Eigen::Success) {
      Eigen::Matrix3d marginal = Eigen::Matrix3d::Zero();
      const int offset = dimension - 3;
      for (int column = 0; column < 3; ++column) {
        Eigen::VectorXd unit = Eigen::VectorXd::Zero(dimension);
        unit[offset + column] = 1.0;
        const Eigen::VectorXd solution = covariance_solver.solve(unit);
        if (solution.allFinite()) {
          marginal.col(column) = solution.segment<3>(offset);
        }
      }
      graph_covariance_ = 0.5 * (marginal + marginal.transpose());
    }
  }

  void updateCorrectionAndPublishPath() {
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = map_frame_;
    Pose2 correction;
    Eigen::Matrix3d covariance;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      if (keyframes_.empty()) return;
      correction = compose(keyframes_.back().optimized_pose,
                           inverse(keyframes_.back().raw_pose));
      covariance = graph_covariance_;
      path.poses.reserve(keyframes_.size());
      for (const Keyframe& keyframe : keyframes_) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = keyframe.stamp;
        pose.header.frame_id = map_frame_;
        pose.pose.position.x = keyframe.optimized_pose.x;
        pose.pose.position.y = keyframe.optimized_pose.y;
        pose.pose.orientation.z = std::sin(0.5 * keyframe.optimized_pose.yaw);
        pose.pose.orientation.w = std::cos(0.5 * keyframe.optimized_pose.yaw);
        path.poses.push_back(pose);
      }
    }
    {
      std::lock_guard<std::mutex> lock(correction_mutex_);
      correction_ = correction;
      correction_covariance_ = covariance;
    }
    corrected_path_pub_.publish(path);
  }

  void publishLoopMarkers(std::size_t from, std::size_t to) {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker nodes;
    nodes.header.frame_id = map_frame_;
    nodes.header.stamp = ros::Time::now();
    nodes.ns = "loop_nodes";
    nodes.id = static_cast<int>(loop_revision_.load());
    nodes.type = visualization_msgs::Marker::SPHERE_LIST;
    nodes.action = visualization_msgs::Marker::ADD;
    nodes.scale.x = nodes.scale.y = nodes.scale.z = 0.4;
    nodes.color.r = 1.0;
    nodes.color.g = 0.2;
    nodes.color.a = 1.0;
    visualization_msgs::Marker edge = nodes;
    edge.ns = "loop_edges";
    edge.type = visualization_msgs::Marker::LINE_LIST;
    edge.scale.x = 0.08;
    edge.color.r = 1.0;
    edge.color.g = 1.0;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      geometry_msgs::Point first;
      first.x = keyframes_[from].optimized_pose.x;
      first.y = keyframes_[from].optimized_pose.y;
      geometry_msgs::Point second;
      second.x = keyframes_[to].optimized_pose.x;
      second.y = keyframes_[to].optimized_pose.y;
      nodes.points.push_back(first);
      nodes.points.push_back(second);
      edge.points.push_back(first);
      edge.points.push_back(second);
    }
    markers.markers.push_back(nodes);
    markers.markers.push_back(edge);
    marker_pub_.publish(markers);
  }

  void publishStatus(const std::string& status) {
    std_msgs::String message;
    message.data = status;
    status_pub_.publish(message);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber cloud_sub_;
  ros::Publisher corrected_odom_pub_;
  ros::Publisher corrected_cloud_pub_;
  ros::Publisher corrected_path_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher status_pub_;
  ros::Publisher revision_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  bool enabled_ = true;
  bool publish_tf_ = true;
  std::string odom_topic_;
  std::string cloud_topic_;
  std::string corrected_odom_topic_;
  std::string corrected_cloud_topic_;
  std::string corrected_path_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double sync_tolerance_ = 0.03;
  double keyframe_distance_ = 0.8;
  double keyframe_angle_deg_ = 10.0;
  double keyframe_angle_ = 0.0;
  float keyframe_voxel_size_ = 0.25F;
  int max_points_per_keyframe_ = 8000;
  int max_keyframes_ = 2000;
  int exclude_recent_keyframes_ = 30;
  double candidate_search_radius_ = 8.0;
  int history_submap_size_ = 10;
  int loop_check_interval_ = 3;
  int icp_max_iterations_ = 60;
  double icp_max_correspondence_distance_ = 1.0;
  double icp_fitness_threshold_ = 0.35;
  int icp_min_source_points_ = 100;
  int icp_min_target_points_ = 500;
  double max_loop_correction_translation_ = 5.0;
  double max_loop_correction_yaw_deg_ = 45.0;
  double max_loop_correction_yaw_ = 0.0;
  double odom_translation_stddev_ = 0.05;
  double odom_yaw_stddev_deg_ = 1.0;
  double loop_translation_stddev_ = 0.10;
  double loop_yaw_stddev_deg_ = 2.0;
  double loop_huber_delta_ = 3.0;
  int optimizer_iterations_ = 10;
  double optimizer_damping_ = 1e-6;
  Eigen::Vector3d odom_information_ = Eigen::Vector3d::Ones();
  Eigen::Vector3d loop_information_ = Eigen::Vector3d::Ones();

  std::mutex latest_odom_mutex_;
  nav_msgs::Odometry latest_odom_;
  bool have_odom_ = false;
  std::mutex queue_mutex_;
  std::condition_variable queue_condition_;
  std::deque<PendingKeyframe> pending_keyframes_;
  Pose2 last_enqueued_pose_;
  bool have_enqueued_pose_ = false;
  std::thread worker_;
  std::atomic<bool> running_;

  mutable std::mutex graph_mutex_;
  std::vector<Keyframe> keyframes_;
  std::vector<Edge> edges_;
  Eigen::Matrix3d graph_covariance_ = Eigen::Matrix3d::Identity() * 1e-3;
  std::mutex correction_mutex_;
  Pose2 correction_;
  Eigen::Matrix3d correction_covariance_ = Eigen::Matrix3d::Identity() * 1e-3;
  std::atomic<std::uint64_t> loop_revision_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "super_lio_loop");
  SuperLioLoop node;
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
