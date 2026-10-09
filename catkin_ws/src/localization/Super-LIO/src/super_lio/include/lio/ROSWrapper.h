

#ifndef ROSWRAPPER_HPP_
#define ROSWRAPPER_HPP_

#include <map>
#include <tuple>
#include <deque>
#include <vector>
#include <execution>

 #include <pcl/point_types.h>
 #include <pcl/point_cloud.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ros/subscribe_options.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>

#include "params.h"
#include "basic/alias.h"
#include "basic/logs.h"
#include "basic/Manifold.h"
#include <livox_ros_driver2/CustomMsg.h>
#include "common/ds.h"

#include "ESKF.h"
#include "OctVoxMap/OctVoxMap.hpp"

#include "super_lio/GetMap.h"
#include "super_lio/SetInitState.h"

// #define SIM_GAZEBO


namespace LI2Sup{

void LoadParamFromRos(ros::NodeHandle& nh);

std::tuple<float, float, float> getColorFromVelocity(float velocity, float max_velocity);

void livox2pcl(const livox_ros_driver2::CustomMsg::ConstPtr& msg, BASIC::CloudPtr& point_cloud);

class ROSWrapper{
public:
  ROSWrapper();
  ~ROSWrapper(){};
  using Ptr = std::unique_ptr<ROSWrapper>;
  bool sync_measure(MeasureGroup&);
  void spinOnce(){
    self_queue_.callAvailable();
  }

  void setESKF(ESKF::Ptr& eskf) {
    eskf_ = eskf;
  }

  void setMap(OctVoxMap<BASIC::V3, BASIC::scalar>::Ptr& ivox) {
    ivox_ = ivox;
  }

  void clear(){
    lidar_buffer_.clear();
    imu_buffer_.clear();
    lidar_pushed_ = false;
    last_timestamp_imu_ = -1.0;
    last_timestamp_lidar_ = -1.0;
  }

  void pub_odom(const NavState& state, bool re_path);
  // void pub_cloud_body(const BASIC::CloudPtr& pc, double time);
  void pub_cloud_world(const BASIC::CloudPtr& pc, double time);
  void pub_cloud2planner(const BASIC::CloudPtr& pc, double time);
  void pub_scan_body(const BASIC::CloudPtr& lidar_pc,
                     const ros::Time& stamp);
  void pub_path();
  void pub_cloud_body_pose(const BASIC::CloudPtr& pc, 
                           const NavState& state);
  void pub_cloud_world_pose(const BASIC::CloudPtr& pc, 
                            const NavState& state);
  
  void pub_double_cloud_pose(const BASIC::VV3& pc_world,
                             const BASIC::VV3& pc_body,
                             const NavState& state);

  void pub_cloud_body_pose( const BASIC::VV3& pc_body,
                            const NavState& state);  

  void get3DCloud(BASIC::CloudPtr& pc);
 
private:
  void imuHandler(const sensor_msgs::Imu::ConstPtr&);
  void livoxHandler(const livox_ros_driver2::CustomMsg::ConstPtr&);
  void stdMsgHandler(const sensor_msgs::PointCloud2::ConstPtr&);

  bool GetMapCallback(super_lio::GetMap::Request& req,
                      super_lio::GetMap::Response& res);
  
  bool Get3DMapCallback(super_lio::GetMap::Request& req,
                      super_lio::GetMap::Response& res);
  
  bool SetInitStateCallback(super_lio::SetInitState::Request& req,
                          super_lio::SetInitState::Response& res);

  ros::NodeHandle nh_;
  ros::CallbackQueue self_queue_;
  ros::Subscriber subLidar_;
  ros::Subscriber subIMU_;
  ros::ServiceServer srv_get_map_;
  ros::ServiceServer srv_get_3dmap_;
  ros::ServiceServer srv_set_init_state_;
  std::deque<IMUData>   imu_buffer_;
  std::deque<LidarData> lidar_buffer_;
  bool lidar_pushed_ = false;
  double last_timestamp_imu_ = -1.0;
  double last_timestamp_lidar_ = -1.0;

  ESKF::Ptr eskf_ = nullptr;
  OctVoxMap<BASIC::V3, BASIC::scalar>::Ptr ivox_ = nullptr;

  /// output
  ros::Publisher pub_odom_;        // imu frame -> lidar frequency
  ros::Publisher pub_path_;        // robo path
  ros::Publisher pub_path_robot_;
  ros::Publisher pub_odom_x_;
  ros::Publisher pub_odom_y_;
  ros::Publisher pub_odom_z_;

  nav_msgs::Path path_;
  sensor_msgs::PointCloud2 msg_path_point_;
  geometry_msgs::PoseStamped msg2uav_;
  BASIC::V3 last_path_point_ = BASIC::V3(0, 0, -100);
  tf::TransformBroadcaster br_;

  BASIC::CloudPtr cloud_map_;

  
};

} // namespace END.

#endif
