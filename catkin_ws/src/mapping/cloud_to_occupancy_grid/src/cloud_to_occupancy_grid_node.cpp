#include <ros/ros.h>
#include "sparse_grid.h"
#include <std_msgs/String.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/UInt64.h>
#include <std_srvs/Trigger.h>
#include <super_lio_loop/GetKeyframe.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_listener.h>

#include <boost/filesystem.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

struct VoxelKey {
  int x;
  int y;
  int z;

  bool operator==(const VoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& key) const {
    std::size_t seed = std::hash<int>{}(key.x);
    seed ^= std::hash<int>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<int>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

struct AngularHit {
  double range = std::numeric_limits<double>::infinity();
  double x = 0.0;
  double y = 0.0;
};

struct MapSnapshot {
  std::vector<int8_t> data;
  int width = 0;
  int height = 0;
  double origin_x = 0.0;
  double origin_y = 0.0;
};

double probabilityToLogOdds(double probability) {
  return std::log(probability / (1.0 - probability));
}

}  // namespace

class CloudToOccupancyGrid {
 public:
  CloudToOccupancyGrid()
      : nh_(),
        private_nh_("~"),
        tf_listener_(tf_buffer_),
        saving_(false),
        update_sequence_(0),
        saved_sequence_(0),
        replay_running_(true) {
    loadParameters();
    initializeMap();
    storage_pub_ = private_nh_.advertise<std_msgs::String>("storage_status", 1, true);

    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_topic_, 1, true);
    if (!tablet_map_topic_.empty()) {
      tablet_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(tablet_map_topic_, 1, true);
    }
    cloud_sub_ = nh_.subscribe(cloud_topic_, 1, &CloudToOccupancyGrid::cloudCallback, this);
    if (reprojection_enabled_) {
      revision_sub_ = nh_.subscribe(loop_revision_topic_, 2,
          &CloudToOccupancyGrid::revisionCallback, this);
      keyframe_client_ = nh_.serviceClient<super_lio_loop::GetKeyframe>(keyframe_service_, true);
      replay_thread_ = std::thread(&CloudToOccupancyGrid::replayLoop, this);
    }
    save_service_ = private_nh_.advertiseService("save_map", &CloudToOccupancyGrid::saveService, this);
    freeze_service_ = private_nh_.advertiseService("freeze_map", &CloudToOccupancyGrid::freezeService, this);
    reset_service_ = private_nh_.advertiseService("reset_map", &CloudToOccupancyGrid::resetService, this);

    map_publish_timer_ = nh_.createTimer(
        ros::Duration(1.0 / map_publish_frequency_),
        &CloudToOccupancyGrid::publishMapTimer, this);
    if (!tablet_map_topic_.empty() && tablet_publish_frequency_ > 0.0) {
      tablet_publish_timer_ = nh_.createTimer(
          ros::Duration(1.0 / tablet_publish_frequency_),
          &CloudToOccupancyGrid::publishTabletMapTimer, this);
    }
    if (autosave_ && autosave_interval_ > 0.0) {
      autosave_timer_ = nh_.createTimer(
          ros::Duration(autosave_interval_),
          &CloudToOccupancyGrid::autosaveTimer, this);
    }

    ROS_INFO_STREAM("cloud_to_occupancy_grid ready: " << width_cells_ << " x "
                    << height_cells_ << " cells (" << resolution_ << " m), input "
                    << cloud_topic_ << ", sensor frame " << sensor_frame_);
  }

  ~CloudToOccupancyGrid() {
    replay_running_.store(false);
    replay_condition_.notify_all();
    if (replay_thread_.joinable()) replay_thread_.join();
    if (save_thread_.joinable()) {
      save_thread_.join();
    }
  }

 private:
  void loadParameters() {
    private_nh_.param<std::string>("cloud_topic", cloud_topic_, "/lio/map_cloud");
    private_nh_.param<std::string>("map_topic", map_topic_, "/map");
    private_nh_.param<std::string>("tablet_map_topic", tablet_map_topic_, "/tablet/map");
    private_nh_.param<std::string>("global_frame", global_frame_, "map");
    private_nh_.param<std::string>("sensor_frame", sensor_frame_, "base_laser_link");
    private_nh_.param("tf_timeout", tf_timeout_, 0.1);
    private_nh_.param("reprojection_enabled", reprojection_enabled_, true);
    private_nh_.param<std::string>("loop_revision_topic", loop_revision_topic_,
                                   "/super_lio_loop/revision");
    private_nh_.param<std::string>("keyframe_service", keyframe_service_,
                                   "/super_lio_loop/get_keyframe");

    private_nh_.param("resolution", resolution_, 0.05);
    private_nh_.param("max_grid_memory_mb", max_grid_memory_mb_, 256);
    private_nh_.param("max_snapshot_cells", max_snapshot_cells_, 25000000);
    private_nh_.param("map_width", map_width_, 150.0);
    private_nh_.param("map_height", map_height_, 150.0);
    private_nh_.param("origin_x", origin_x_, -75.0);
    private_nh_.param("origin_y", origin_y_, -75.0);

    private_nh_.param("min_range", min_range_, 0.5);
    private_nh_.param("max_range", max_range_, 30.0);
    private_nh_.param("voxel_size", voxel_size_, 0.10);
    private_nh_.param("min_obstacle_height", min_obstacle_height_, 0.08);
    private_nh_.param("max_obstacle_height", max_obstacle_height_, 1.20);
    private_nh_.param("angular_resolution", angular_resolution_degrees_, 0.5);

    private_nh_.param("occupied_prob", occupied_probability_, 0.70);
    private_nh_.param("free_prob", free_probability_, 0.30);
    private_nh_.param("occupied_threshold", occupied_threshold_, 0.65);
    private_nh_.param("free_threshold", free_threshold_, 0.35);
    // map_server thresholds classify PGM intensities, not the mapper's
    // internal log-odds. Pixel value 205 represents unknown and corresponds
    // to an occupancy probability of about 0.196.
    private_nh_.param("map_server_free_threshold", map_server_free_threshold_, 0.196);
    private_nh_.param("min_log_odds", min_log_odds_, -4.0);
    private_nh_.param("max_log_odds", max_log_odds_, 4.0);

    private_nh_.param("processing_frequency", processing_frequency_, 5.0);
    private_nh_.param("map_publish_frequency", map_publish_frequency_, 1.0);
    private_nh_.param("tablet_publish_frequency", tablet_publish_frequency_, 0.2);

    private_nh_.param("autosave", autosave_, true);
    private_nh_.param("autosave_interval", autosave_interval_, 30.0);
    private_nh_.param("save_only_when_changed", save_only_when_changed_, true);
    private_nh_.param("publish_cropped_map", publish_cropped_map_, true);
    private_nh_.param("save_cropped_map", save_cropped_map_, true);
    private_nh_.param("crop_padding", crop_padding_, 1.0);
    private_nh_.param<std::string>("map_directory", map_directory_, "/data/maps/current");
    private_nh_.param<std::string>("map_name", map_name_, "grinder_map");

    const bool probabilities_valid =
        occupied_probability_ > 0.5 && occupied_probability_ < 1.0 &&
        free_probability_ > 0.0 && free_probability_ < 0.5 &&
        free_threshold_ > 0.0 && free_threshold_ < occupied_threshold_ &&
        occupied_threshold_ < 1.0;
    if (max_grid_memory_mb_ <= 0 || max_snapshot_cells_ <= 0 ||
        !std::isfinite(resolution_) || !std::isfinite(map_width_) ||
        !std::isfinite(map_height_) || !std::isfinite(origin_x_) || !std::isfinite(origin_y_) ||
        map_width_ / resolution_ > std::numeric_limits<int>::max() / 2 ||
        map_height_ / resolution_ > std::numeric_limits<int>::max() / 2 ||
        resolution_ <= 0.0 || map_width_ <= 0.0 || map_height_ <= 0.0 ||
        min_range_ < 0.0 || max_range_ <= min_range_ || voxel_size_ <= 0.0 ||
        angular_resolution_degrees_ <= 0.0 || angular_resolution_degrees_ > 360.0 ||
        processing_frequency_ <= 0.0 || map_publish_frequency_ <= 0.0 ||
        crop_padding_ < 0.0 ||
        map_server_free_threshold_ <= 0.0 ||
        map_server_free_threshold_ >= occupied_threshold_ ||
        min_obstacle_height_ > max_obstacle_height_ || min_log_odds_ >= max_log_odds_ ||
        !probabilities_valid) {
      throw std::runtime_error("invalid cloud_to_occupancy_grid parameters");
    }

    occupied_log_odds_increment_ = probabilityToLogOdds(occupied_probability_);
    free_log_odds_increment_ = probabilityToLogOdds(free_probability_);
    occupied_log_odds_threshold_ = probabilityToLogOdds(occupied_threshold_);
    free_log_odds_threshold_ = probabilityToLogOdds(free_threshold_);
    processing_period_ = 1.0 / processing_frequency_;
  }

  void initializeMap() {
    width_cells_ = static_cast<int>(std::ceil(map_width_ / resolution_));
    height_cells_ = static_cast<int>(std::ceil(map_height_ / resolution_));
    tile_budget_ = std::make_shared<mapper::TileBudget>(
        static_cast<std::size_t>(max_grid_memory_mb_) * 1024U * 1024U);
    grid_.reset(new mapper::SparseGrid(tile_budget_));
    map_load_time_ = ros::Time::now();
  }

  tf2::Transform transformFromMessage(const geometry_msgs::TransformStamped& message) const {
    const auto& translation = message.transform.translation;
    const auto& rotation = message.transform.rotation;
    return tf2::Transform(
        tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
        tf2::Vector3(translation.x, translation.y, translation.z));
  }

  bool lookupTransform(const std::string& target,
                       const std::string& source,
                       const ros::Time& stamp,
                       tf2::Transform* transform) {
    if (target == source) {
      transform->setIdentity();
      return true;
    }
    try {
      *transform = transformFromMessage(
          tf_buffer_.lookupTransform(target, source, stamp, ros::Duration(tf_timeout_)));
      return true;
    } catch (const tf2::TransformException& error) {
      ROS_WARN_THROTTLE(2.0, "Cannot transform %s <- %s at cloud time: %s",
                        target.c_str(), source.c_str(), error.what());
      return false;
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud) {
    std::lock_guard<std::recursive_mutex> input_lock(input_mutex_);
    if (frozen_) return;
    latest_cloud_ = cloud;
    if (rebuilding_.load()) return;
    if (cloud->header.stamp.isZero()) {
      ROS_WARN_THROTTLE(2.0, "Ignoring point cloud with zero timestamp");
      return;
    }
    if (!last_processed_stamp_.isZero() && cloud->header.stamp >= last_processed_stamp_ &&
        (cloud->header.stamp - last_processed_stamp_).toSec() < processing_period_) {
      return;
    }

    tf2::Transform global_from_sensor;
    if (!lookupTransform(global_frame_, sensor_frame_, cloud->header.stamp,
                         &global_from_sensor)) {
      return;
    }
    tf2::Transform global_from_cloud;
    if (!lookupTransform(global_frame_, cloud->header.frame_id, cloud->header.stamp,
                         &global_from_cloud)) {
      return;
    }

    const tf2::Vector3 sensor_origin = global_from_sensor.getOrigin();
    int sensor_cell_x = 0;
    int sensor_cell_y = 0;
    if (!worldToCell(sensor_origin.x(), sensor_origin.y(), &sensor_cell_x, &sensor_cell_y)) {
      ROS_WARN_THROTTLE(2.0, "Sensor origin is outside the configured occupancy grid");
      return;
    }

    const double angular_resolution = angular_resolution_degrees_ * kPi / 180.0;
    const std::size_t bin_count = static_cast<std::size_t>(std::ceil(2.0 * kPi / angular_resolution));
    std::vector<AngularHit> hits(bin_count);
    std::unordered_set<VoxelKey, VoxelKeyHash> occupied_voxels;
    occupied_voxels.reserve(std::min<std::size_t>(cloud->width * cloud->height, 50000U));

    try {
      sensor_msgs::PointCloud2ConstIterator<float> x_iterator(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_iterator(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z_iterator(*cloud, "z");
      for (; x_iterator != x_iterator.end(); ++x_iterator, ++y_iterator, ++z_iterator) {
        if (!std::isfinite(*x_iterator) || !std::isfinite(*y_iterator) ||
            !std::isfinite(*z_iterator)) {
          continue;
        }

        const tf2::Vector3 point_global =
            global_from_cloud * tf2::Vector3(*x_iterator, *y_iterator, *z_iterator);
        if (point_global.z() < min_obstacle_height_ ||
            point_global.z() > max_obstacle_height_) {
          continue;
        }

        const VoxelKey voxel{
            static_cast<int>(std::floor(point_global.x() / voxel_size_)),
            static_cast<int>(std::floor(point_global.y() / voxel_size_)),
            static_cast<int>(std::floor(point_global.z() / voxel_size_))};
        if (!occupied_voxels.insert(voxel).second) {
          continue;
        }

        const double dx = point_global.x() - sensor_origin.x();
        const double dy = point_global.y() - sensor_origin.y();
        const double range = std::hypot(dx, dy);
        if (range < min_range_ || range > max_range_) {
          continue;
        }

        const double angle = std::atan2(dy, dx) + kPi;
        const std::size_t bin = std::min(
            static_cast<std::size_t>(angle / angular_resolution), bin_count - 1U);
        if (range < hits[bin].range) {
          hits[bin].range = range;
          hits[bin].x = point_global.x();
          hits[bin].y = point_global.y();
        }
      }
    } catch (const std::runtime_error& error) {
      ROS_ERROR_THROTTLE(2.0, "Point cloud must contain float32 x/y/z fields: %s", error.what());
      return;
    }

    std::lock_guard<std::mutex> lock(map_mutex_);
    if (storage_failed_.load()) return;
    try {
      std::vector<std::size_t> occupied_cells;
      occupied_cells.reserve(hits.size());
      bool observed_cells = false;
      for (const AngularHit& hit : hits) {
        if (!std::isfinite(hit.range)) {
          continue;
        }
        int endpoint_x = 0;
        int endpoint_y = 0;
        worldToRawCell(hit.x, hit.y, &endpoint_x, &endpoint_y);
        raytraceFree(sensor_cell_x, sensor_cell_y, endpoint_x, endpoint_y);
        observed_cells = true;
        if (cellInBounds(endpoint_x, endpoint_y)) {
          occupied_cells.push_back(cellIndex(endpoint_x, endpoint_y));
        }
      }
      for (const std::size_t index : occupied_cells) {
        updateCell(index, occupied_log_odds_increment_);
      }
      if (observed_cells) {
        ++update_sequence_;
      }
      last_processed_stamp_ = cloud->header.stamp;
    } catch (const std::exception& error) {
      storage_failed_.store(true);
      ROS_ERROR_STREAM("2D mapping stopped: " << error.what());
    }
  }

  void revisionCallback(const std_msgs::UInt64ConstPtr& message) {
    std::lock_guard<std::recursive_mutex> input_lock(input_mutex_);
    if (frozen_ || storage_failed_.load()) return;
    if (message->data <= requested_revision_.load()) return;
    requested_revision_.store(message->data);
    replay_condition_.notify_one();
  }

  void replayLoop() {
    while (replay_running_.load() && ros::ok()) {
      std::uint64_t revision = 0U;
      {
        std::unique_lock<std::mutex> lock(replay_mutex_);
        replay_condition_.wait(lock, [this]() {
          return !replay_running_.load() || requested_revision_.load() > completed_revision_.load();
        });
        if (!replay_running_.load()) break;
        revision = requested_revision_.load();
      }
      rebuildFromKeyframes(revision);
    }
  }

  void rebuildFromKeyframes(std::uint64_t revision) {
    rebuilding_.store(true);
    mapper::SparseGrid staged(tile_budget_);
    try {
      std::uint32_t total = 0U;
      bool success = true;
      for (std::uint32_t index = 0U; success && (index == 0U || index < total); ++index) {
        if (!replay_running_.load() || requested_revision_.load() != revision) {
          success = false;
          break;
        }
        super_lio_loop::GetKeyframe service;
        service.request.revision = revision;
        service.request.index = index;
        if (!keyframe_client_.call(service) || !service.response.success) {
          ROS_WARN_STREAM("2D map reprojection aborted: " << service.response.message);
          success = false;
          break;
        }
        total = service.response.total;
        applyReplayCloud(service.response.cloud, service.response.pose,
                         &staged);
      }
      if (success && total > 0U && requested_revision_.load() == revision) {
        {
          std::lock_guard<std::mutex> lock(map_mutex_);
          grid_->swap(staged);
          map_load_time_ = ros::Time::now();
          ++update_sequence_;
        }
        completed_revision_.store(revision);
        ROS_INFO_STREAM("2D map reprojected from " << total
                        << " optimized keyframes at loop revision " << revision);
      } else if (requested_revision_.load() == revision) {
        // Keep the previous coherent grid and wait for the next graph revision;
        // do not spin on a missing or transiently failed service.
        completed_revision_.store(revision);
      }
    } catch (const std::exception& error) {
      storage_failed_.store(true);
      completed_revision_.store(revision);
      ROS_ERROR_STREAM("2D reprojection failed: " << error.what());
    }
    rebuilding_.store(false);
  }

  void applyReplayCloud(const sensor_msgs::PointCloud2& cloud,
                        const geometry_msgs::Pose& pose,
                        mapper::SparseGrid* grid) {
    const tf2::Transform global_from_sensor(
        tf2::Quaternion(pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w),
        tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
    int sensor_x = 0;
    int sensor_y = 0;
    if (!worldToCell(pose.position.x, pose.position.y, &sensor_x, &sensor_y)) return;
    const double angular_resolution = angular_resolution_degrees_ * kPi / 180.0;
    const std::size_t bin_count = static_cast<std::size_t>(std::ceil(2.0 * kPi / angular_resolution));
    std::vector<AngularHit> hits(bin_count);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) continue;
        const tf2::Vector3 point = global_from_sensor * tf2::Vector3(*x, *y, *z);
        if (point.z() < min_obstacle_height_ || point.z() > max_obstacle_height_) continue;
        const double dx = point.x() - pose.position.x;
        const double dy = point.y() - pose.position.y;
        const double range = std::hypot(dx, dy);
        if (range < min_range_ || range > max_range_) continue;
        const std::size_t bin = std::min(static_cast<std::size_t>(
            (std::atan2(dy, dx) + kPi) / angular_resolution), bin_count - 1U);
        if (range < hits[bin].range) hits[bin] = {range, point.x(), point.y()};
      }
    } catch (const std::runtime_error& error) {
      ROS_ERROR_STREAM("2D replay cloud fields invalid: " << error.what());
      return;
    }
    for (const AngularHit& hit : hits) {
      if (!std::isfinite(hit.range)) continue;
      int end_x = 0;
      int end_y = 0;
      worldToRawCell(hit.x, hit.y, &end_x, &end_y);
      raytraceFreeInto(sensor_x, sensor_y, end_x, end_y, grid);
      if (cellInBounds(end_x, end_y))
        updateCellInto(cellIndex(end_x, end_y), occupied_log_odds_increment_, grid);
    }
  }

  void raytraceFreeInto(int start_x, int start_y, int end_x, int end_y,
                        mapper::SparseGrid* grid) {
    int x = start_x;
    int y = start_y;
    const int dx = std::abs(end_x - start_x);
    const int step_x = start_x < end_x ? 1 : -1;
    const int dy = -std::abs(end_y - start_y);
    const int step_y = start_y < end_y ? 1 : -1;
    int error = dx + dy;
    while (x != end_x || y != end_y) {
      if (!cellInBounds(x, y)) break;
      updateCellInto(cellIndex(x, y), free_log_odds_increment_, grid);
      const int twice = 2 * error;
      if (twice >= dy) { error += dy; x += step_x; }
      if (twice <= dx) { error += dx; y += step_y; }
    }
  }

  void updateCellInto(std::size_t index, double increment, mapper::SparseGrid* grid) {
    grid->update(static_cast<int>(index % width_cells_), static_cast<int>(index / width_cells_),
                 increment, min_log_odds_, max_log_odds_);
  }

  void raytraceFree(int start_x, int start_y, int end_x, int end_y) {
    raytraceFreeInto(start_x, start_y, end_x, end_y, grid_.get());
  }

  void updateCell(std::size_t index, double increment) {
    updateCellInto(index, increment, grid_.get());
  }

  bool worldToCell(double x, double y, int* cell_x, int* cell_y) const {
    worldToRawCell(x, y, cell_x, cell_y);
    return cellInBounds(*cell_x, *cell_y);
  }

  void worldToRawCell(double x, double y, int* cell_x, int* cell_y) const {
    *cell_x = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    *cell_y = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  }

  bool cellInBounds(int x, int y) const {
    return x >= 0 && y >= 0 && x < width_cells_ && y < height_cells_;
  }

  std::size_t cellIndex(int x, int y) const {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(width_cells_) +
           static_cast<std::size_t>(x);
  }

  MapSnapshot occupancySnapshot(bool crop) const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    int min_x = 0;
    int min_y = 0;
    int max_x = width_cells_ - 1;
    int max_y = height_cells_ - 1;

    if (storage_failed_.load()) throw std::runtime_error("map storage failed; reset or restart scan");
    if (crop) {
      // Empty maps are a single unknown cell, never the entire capacity.
      if (grid_->max_x < 0) { max_x = max_y = 0; }
      else {
        const int padding = static_cast<int>(std::min<double>(
            std::ceil(crop_padding_ / resolution_), std::max(width_cells_, height_cells_)));
        min_x = std::max(0, grid_->min_x - padding);
        min_y = std::max(0, grid_->min_y - padding);
        max_x = std::min(width_cells_ - 1, grid_->max_x + padding);
        max_y = std::min(height_cells_ - 1, grid_->max_y + padding);
      }
    }
    if (static_cast<uint64_t>(max_x - min_x + 1) * (max_y - min_y + 1) >
        static_cast<uint64_t>(max_snapshot_cells_))
      throw std::runtime_error("map snapshot exceeds max_snapshot_cells");

    MapSnapshot snapshot;
    snapshot.width = max_x - min_x + 1;
    snapshot.height = max_y - min_y + 1;
    snapshot.origin_x = origin_x_ + static_cast<double>(min_x) * resolution_;
    snapshot.origin_y = origin_y_ + static_cast<double>(min_y) * resolution_;
    snapshot.data.assign(static_cast<std::size_t>(snapshot.width) *
                         static_cast<std::size_t>(snapshot.height), -1);

    grid_->visit([&](int x, int y, float odds) {
      if (x < min_x || x > max_x || y < min_y || y > max_y) return;
      const std::size_t index = static_cast<std::size_t>(y - min_y) * snapshot.width + x - min_x;
      if (odds >= occupied_log_odds_threshold_) snapshot.data[index] = 100;
      else if (odds <= free_log_odds_threshold_) snapshot.data[index] = 0;
    });
    return snapshot;
  }

  nav_msgs::OccupancyGrid buildMapMessage() const {
    nav_msgs::OccupancyGrid map;
    map.header.stamp = ros::Time::now();
    map.header.frame_id = global_frame_;
    map.info.map_load_time = map_load_time_;
    map.info.resolution = static_cast<float>(resolution_);
    const MapSnapshot snapshot = occupancySnapshot(publish_cropped_map_);
    map.info.width = static_cast<uint32_t>(snapshot.width);
    map.info.height = static_cast<uint32_t>(snapshot.height);
    map.info.origin.position.x = snapshot.origin_x;
    map.info.origin.position.y = snapshot.origin_y;
    map.info.origin.orientation.w = 1.0;
    map.data = snapshot.data;
    return map;
  }

  void publishStorageStatus() {
    std::lock_guard<std::mutex> lock(map_mutex_);
    std_msgs::String status;
    const int padding = static_cast<int>(std::min<double>(
        std::ceil(crop_padding_ / resolution_), std::max(width_cells_, height_cells_)));
    const uint64_t cropped_width = grid_->max_x < 0 ? 1 :
        std::min(width_cells_ - 1, grid_->max_x + padding) - std::max(0, grid_->min_x - padding) + 1;
    const uint64_t cropped_height = grid_->max_y < 0 ? 1 :
        std::min(height_cells_ - 1, grid_->max_y + padding) - std::max(0, grid_->min_y - padding) + 1;
    const uint64_t crop_cells = cropped_width * cropped_height;
    const uint64_t full_cells = static_cast<uint64_t>(width_cells_) * height_cells_;
    const bool snapshot_limit = (publish_cropped_map_ ? crop_cells : full_cells) >
        static_cast<uint64_t>(max_snapshot_cells_) ||
        (save_cropped_map_ ? crop_cells : full_cells) > static_cast<uint64_t>(max_snapshot_cells_);
    status.data = std::string(storage_failed_.load() ? "ERROR" : (snapshot_limit ? "ERROR_SNAPSHOT" : "OK")) +
        " tile_bytes=" + std::to_string(tile_budget_->used.load()) +
        " limit_bytes=" + std::to_string(tile_budget_->limit) +
        " live_tiles=" + std::to_string(grid_->tileCount());
    storage_pub_.publish(status);
  }

  void publishMapTimer(const ros::TimerEvent&) {
    publishStorageStatus();
    try { map_pub_.publish(buildMapMessage()); }
    catch (const std::exception& e) { ROS_ERROR_THROTTLE(5.0, "Map publication blocked: %s", e.what()); }
  }

  void publishTabletMapTimer(const ros::TimerEvent&) {
    try { tablet_map_pub_.publish(buildMapMessage()); }
    catch (const std::exception& e) { ROS_ERROR_THROTTLE(5.0, "Tablet map blocked: %s", e.what()); }
  }

  bool writeMapFiles(const MapSnapshot& snapshot, std::string* error_message) const {
    try {
      const boost::filesystem::path directory(map_directory_);
      boost::filesystem::create_directories(directory);
      const boost::filesystem::path pgm_path = directory / (map_name_ + ".pgm");
      const boost::filesystem::path yaml_path = directory / (map_name_ + ".yaml");
      const boost::filesystem::path pgm_temp = directory / (map_name_ + ".pgm.tmp");
      const boost::filesystem::path yaml_temp = directory / (map_name_ + ".yaml.tmp");

      std::ofstream pgm(pgm_temp.string(), std::ios::binary | std::ios::trunc);
      if (!pgm) {
        throw std::runtime_error("cannot open " + pgm_temp.string());
      }
      pgm << "P5\n# CREATOR: cloud_to_occupancy_grid\n"
          << snapshot.width << " " << snapshot.height << "\n255\n";
      for (int y = snapshot.height - 1; y >= 0; --y) {
        for (int x = 0; x < snapshot.width; ++x) {
          const std::size_t index = static_cast<std::size_t>(y) *
                                    static_cast<std::size_t>(snapshot.width) +
                                    static_cast<std::size_t>(x);
          const int8_t value = snapshot.data[index];
          const unsigned char pixel = value < 0 ? 205U : (value >= 65 ? 0U : 254U);
          pgm.write(reinterpret_cast<const char*>(&pixel), 1);
        }
      }
      pgm.close();
      if (!pgm) {
        throw std::runtime_error("failed while writing " + pgm_temp.string());
      }

      std::ofstream yaml(yaml_temp.string(), std::ios::trunc);
      if (!yaml) {
        throw std::runtime_error("cannot open " + yaml_temp.string());
      }
      yaml << "image: " << map_name_ << ".pgm\n"
           << "resolution: " << resolution_ << "\n"
           << "origin: [" << snapshot.origin_x << ", " << snapshot.origin_y << ", 0.0]\n"
           << "negate: 0\n"
           << "occupied_thresh: " << occupied_threshold_ << "\n"
           << "free_thresh: " << map_server_free_threshold_ << "\n";
      yaml.close();
      if (!yaml) {
        throw std::runtime_error("failed while writing " + yaml_temp.string());
      }

      if (boost::filesystem::exists(pgm_path)) {
        boost::filesystem::remove(pgm_path);
      }
      boost::filesystem::rename(pgm_temp, pgm_path);
      if (boost::filesystem::exists(yaml_path)) {
        boost::filesystem::remove(yaml_path);
      }
      boost::filesystem::rename(yaml_temp, yaml_path);
      ROS_INFO_STREAM("Saved occupancy map to " << yaml_path.string());
      return true;
    } catch (const std::exception& error) {
      *error_message = error.what();
      ROS_ERROR_STREAM("Failed to save occupancy map: " << *error_message);
      return false;
    }
  }

  void autosaveTimer(const ros::TimerEvent&) {
    const std::uint64_t sequence = update_sequence_.load();
    if ((save_only_when_changed_ && sequence == saved_sequence_.load()) ||
        saving_.exchange(true)) {
      return;
    }
    if (save_thread_.joinable()) {
      save_thread_.join();
    }
    MapSnapshot snapshot;
    try { snapshot = occupancySnapshot(save_cropped_map_); }
    catch (const std::exception& e) { saving_.store(false); ROS_ERROR_STREAM(e.what()); return; }
    save_thread_ = std::thread([this, snapshot = std::move(snapshot), sequence]() {
      std::string error;
      if (writeMapFiles(snapshot, &error)) {
        saved_sequence_.store(sequence);
      }
      saving_.store(false);
    });
  }

  bool saveService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    if (saving_.exchange(true)) {
      response.success = false;
      response.message = "an autosave is already running";
      return true;
    }
    if (save_thread_.joinable()) {
      save_thread_.join();
    }
    const std::uint64_t sequence = update_sequence_.load();
    MapSnapshot snapshot;
    try { snapshot = occupancySnapshot(save_cropped_map_); }
    catch (const std::exception& e) { saving_.store(false); response.success = false;
      response.message = e.what(); return true; }
    response.success = writeMapFiles(snapshot, &response.message);
    if (response.success) {
      saved_sequence_.store(sequence);
      response.message = map_directory_ + "/" + map_name_ + ".yaml";
    }
    saving_.store(false);
    return true;
  }

  bool freezeService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    std::lock_guard<std::recursive_mutex> input_lock(input_mutex_);
    if (storage_failed_.load()) {
      response.success = false; response.message = "map storage failed; reset or restart scan"; return true;
    }
    std::string target;
    private_nh_.param<std::string>("freeze_target_stamp_ns", target, "0");
    std::uint64_t target_stamp = 0;
    try { target_stamp = std::stoull(target); }
    catch (const std::exception&) {
      response.success = false;
      response.message = "invalid freeze timestamp";
      return true;
    }
    if (rebuilding_.load() || requested_revision_.load() > completed_revision_.load() ||
        !latest_cloud_ || latest_cloud_->header.stamp.toNSec() < target_stamp) {
      response.success = false;
      response.message = "waiting for final cloud / loop reprojection";
      return true;
    }
    if (!frozen_) {
      last_processed_stamp_ = ros::Time();  // Include the last scan despite rate limiting.
      cloudCallback(latest_cloud_);
      if (last_processed_stamp_ != latest_cloud_->header.stamp) {
        response.success = false;
        response.message = "final cloud transform is not ready";
        return true;
      }
      try { occupancySnapshot(save_cropped_map_); occupancySnapshot(publish_cropped_map_); }
      catch (const std::exception& e) { response.success = false; response.message = e.what(); return true; }
      frozen_ = true;
      // Keep subscriptions until node shutdown; dropping callbacks avoids
      // waiting for another AsyncSpinner thread while holding input_mutex_.
    }
    response.success = true;
    response.message = std::to_string(latest_cloud_->header.stamp.toNSec());
    return true;
  }

  bool resetService(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    std::lock_guard<std::recursive_mutex> input_lock(input_mutex_);
    if (frozen_) {
      response.success = false;
      response.message = "mapping is frozen; start a new scan to clear it";
      return true;
    }
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (rebuilding_.load() || requested_revision_.load() > completed_revision_.load()) {
      response.success = false; response.message = "loop reprojection in progress"; return true;
    }
    grid_->clear();
    storage_failed_.store(false);
    latest_cloud_.reset();
    last_processed_stamp_ = ros::Time();
    map_load_time_ = ros::Time::now();
    ++update_sequence_;
    response.success = true;
    response.message = "occupancy map cleared";
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::recursive_mutex input_mutex_;
  bool frozen_ = false;
  sensor_msgs::PointCloud2ConstPtr latest_cloud_;
  ros::ServiceServer freeze_service_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber revision_sub_;
  ros::Publisher map_pub_;
  ros::Publisher tablet_map_pub_;
  ros::ServiceServer save_service_;
  ros::ServiceServer reset_service_;
  ros::Timer map_publish_timer_;
  ros::Timer tablet_publish_timer_;
  ros::Timer autosave_timer_;
  ros::ServiceClient keyframe_client_;

  std::string cloud_topic_;
  std::string map_topic_;
  std::string tablet_map_topic_;
  std::string global_frame_;
  std::string sensor_frame_;
  std::string map_directory_;
  std::string map_name_;
  std::string loop_revision_topic_;
  std::string keyframe_service_;

  double tf_timeout_;
  double resolution_;
  double map_width_;
  double map_height_;
  double origin_x_;
  double origin_y_;
  double min_range_;
  double max_range_;
  double voxel_size_;
  double min_obstacle_height_;
  double max_obstacle_height_;
  double angular_resolution_degrees_;
  double occupied_probability_;
  double free_probability_;
  double occupied_threshold_;
  double free_threshold_;
  double map_server_free_threshold_;
  double min_log_odds_;
  double max_log_odds_;
  double occupied_log_odds_increment_;
  double free_log_odds_increment_;
  double occupied_log_odds_threshold_;
  double free_log_odds_threshold_;
  double processing_frequency_;
  double map_publish_frequency_;
  double tablet_publish_frequency_;
  double processing_period_;
  double autosave_interval_;
  double crop_padding_;
  bool autosave_;
  bool save_only_when_changed_;
  bool publish_cropped_map_;
  bool save_cropped_map_;
  bool reprojection_enabled_;

  int width_cells_;
  int height_cells_;
  mutable std::mutex map_mutex_;
  std::shared_ptr<mapper::TileBudget> tile_budget_;
  std::unique_ptr<mapper::SparseGrid> grid_;
  std::atomic<bool> storage_failed_{false};
  int max_grid_memory_mb_;
  int max_snapshot_cells_;
  ros::Publisher storage_pub_;
  ros::Time map_load_time_;
  ros::Time last_processed_stamp_;

  std::thread save_thread_;
  std::atomic<bool> saving_;
  std::atomic<std::uint64_t> update_sequence_;
  std::atomic<std::uint64_t> saved_sequence_;
  std::thread replay_thread_;
  std::mutex replay_mutex_;
  std::condition_variable replay_condition_;
  std::atomic<bool> replay_running_;
  std::atomic<bool> rebuilding_{false};
  std::atomic<std::uint64_t> requested_revision_{0U};
  std::atomic<std::uint64_t> completed_revision_{0U};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_to_occupancy_grid");
  try {
    CloudToOccupancyGrid node;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("cloud_to_occupancy_grid failed: " << error.what());
    return 1;
  }
  return 0;
}
