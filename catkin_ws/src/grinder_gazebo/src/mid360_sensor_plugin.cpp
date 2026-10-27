// Gazebo Classic CPU ray approximation of the MID-360S interface.
// Every ray is sampled at one simulator instant: offset_time MUST remain zero.
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/RaySensor.hh>
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

  void Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr) override {
    sensor_ = std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(sensor);
    if (!sensor_ || !ros::isInitialized()) {
      gzerr << "MID-360S requires a ray sensor and gazebo_ros API plugin\n";
      return;
    }
    node_.reset(new ros::NodeHandle("/"));
    cloud_pub_ = node_->advertise<livox_ros_driver2::CustomMsg>("/livox/lidar", 2);
    connection_ = sensor_->ConnectUpdated(std::bind(&Mid360SensorPlugin::OnScan, this));
    sensor_->SetActive(true);
  }

 private:
  void OnScan() {
    if (!ros::ok()) return;
    const int width = sensor_->RangeCount();
    const int height = sensor_->VerticalRangeCount();
    if (width < 2 || height < 2) return;
    std::vector<double> ranges;
    sensor_->Ranges(ranges);  // Atomic snapshot of all ranges from this update.
    if (ranges.size() != static_cast<size_t>(width * height)) return;
    const auto stamp = sensor_->LastUpdateTime();
    livox_ros_driver2::CustomMsg cloud;
    cloud.header.stamp = ros::Time(stamp.sec, stamp.nsec);
    cloud.header.frame_id = "laser_link";
    cloud.timebase = cloud.header.stamp.toNSec();
    cloud.lidar_id = 0;
    cloud.points.reserve(ranges.size());
    const double angle_min = sensor_->AngleMin().Radian();
    const double angle_step = (sensor_->AngleMax().Radian() - angle_min) / (width - 1);
    const double low = sensor_->VerticalAngleMin().Radian();
    const double step = (sensor_->VerticalAngleMax().Radian() - low) / (height - 1);
    for (int v = 0; v < height; ++v) {
      const double elevation = low + v * step;
      for (int h = 0; h < width; ++h) {
        const double range = ranges[v * width + h];
        if (!std::isfinite(range) || range <= sensor_->RangeMin() || range >= sensor_->RangeMax()) continue;
        const double azimuth = angle_min + h * angle_step;
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
  gazebo::event::ConnectionPtr connection_;
  std::unique_ptr<ros::NodeHandle> node_;
  ros::Publisher cloud_pub_;
};
GZ_REGISTER_SENSOR_PLUGIN(Mid360SensorPlugin)
}  // namespace grinder_gazebo
