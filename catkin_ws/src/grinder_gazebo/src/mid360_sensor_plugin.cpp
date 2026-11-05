// Gazebo Classic CPU/GPU ray approximation of the MID-360S interface.
// Every ray is sampled at one simulator instant: offset_time MUST remain zero.
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <livox_ros_driver2/CustomMsg.h>
#include <ros/ros.h>

#include <cmath>
#include <functional>
#include <memory>
#include <vector>

namespace grinder_gazebo {
class Mid360SensorPlugin : public gazebo::SensorPlugin {
 public:
  ~Mid360SensorPlugin() override { connection_.reset(); }

  void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf) override {
    sensor_ = std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(sensor);
    gpu_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::GpuRaySensor>(sensor);
    if ((!sensor_ && !gpu_sensor_) || !ros::isInitialized()) {
      gzerr << "MID-360S requires a ray/gpu_ray sensor and gazebo_ros API plugin\n";
      return;
    }
    node_.reset(new ros::NodeHandle("/"));
    sample_offset_ = gpu_sensor_ && sdf->HasElement("sample_offset")
                         ? sdf->Get<int>("sample_offset") : 0;
    if (sample_offset_ < 0 || sample_offset_ > 1) {
      gzerr << "MID-360S sample_offset must be 0 or 1\n";
      return;
    }
    cloud_pub_ = node_->advertise<livox_ros_driver2::CustomMsg>("/livox/lidar", 2);
    if (gpu_sensor_) {
      connection_ = gpu_sensor_->ConnectUpdated([this]() { OnScan(gpu_sensor_); });
    } else {
      connection_ = sensor_->ConnectUpdated([this]() { OnScan(sensor_); });
    }
    sensor->SetActive(true);
    gzmsg << "MID-360S backend: " << (gpu_sensor_ ? "gpu_ray" : "ray") << "\n";
  }

 private:
  template <class SensorPtr>
  void OnScan(const SensorPtr &sensor) {
    if (!ros::ok()) return;
    const int width = sensor->RangeCount();
    const int height = sensor->VerticalRangeCount();
    if (width < 2 || height < 2) return;
    std::vector<double> ranges;
    sensor->Ranges(ranges);  // Snapshot after sensor update, including range noise.
    if (ranges.size() != static_cast<size_t>(width * height)) return;
    const auto stamp = sensor->LastUpdateTime();
    livox_ros_driver2::CustomMsg cloud;
    cloud.header.stamp = ros::Time(stamp.sec, stamp.nsec);
    cloud.header.frame_id = "laser_link";
    cloud.timebase = cloud.header.stamp.toNSec();
    cloud.lidar_id = 0;
    cloud.points.reserve(ranges.size());
    const double angle_min = sensor->AngleMin().Radian();
    const double angle_step = (sensor->AngleMax().Radian() - angle_min) / (width - 1);
    const double low = sensor->VerticalAngleMin().Radian();
    const double step = (sensor->VerticalAngleMax().Radian() - low) / (height - 1);
    // Classic Ogre point-raster output may be shifted by one texel on WSL/D3D12.
    // The GPU SDF adds one lower-angle guard row/column. Exclude unfilled trailing
    // texels and use the actual sampled angles, preserving the requested FOV.
    for (int v = 0; v < height - sample_offset_; ++v) {
      const double elevation = low + (v + sample_offset_) * step;
      for (int h = 0; h < width - sample_offset_; ++h) {
        const double range = ranges[v * width + h];
        if (!std::isfinite(range) || range <= sensor->RangeMin() || range >= sensor->RangeMax()) continue;
        const double azimuth = angle_min + (h + sample_offset_) * angle_step;
        livox_ros_driver2::CustomPoint point;
        point.x = range * std::cos(elevation) * std::cos(azimuth);
        point.y = range * std::cos(elevation) * std::sin(azimuth);
        point.z = range * std::sin(elevation);
        point.offset_time = 0;
        point.reflectivity = 100;  // Geometric simulation has no material model.
        point.tag = 0x10;
        point.line = v % 4;  // Compatibility field; not a physical scan channel.
        cloud.points.push_back(point);
      }
    }
    cloud.point_num = cloud.points.size();
    cloud_pub_.publish(cloud);
  }

  gazebo::sensors::RaySensorPtr sensor_;
  gazebo::sensors::GpuRaySensorPtr gpu_sensor_;
  gazebo::event::ConnectionPtr connection_;
  std::unique_ptr<ros::NodeHandle> node_;
  ros::Publisher cloud_pub_;
  int sample_offset_ = 0;
};
GZ_REGISTER_SENSOR_PLUGIN(Mid360SensorPlugin)
}  // namespace grinder_gazebo
