/**
 * @file server_params.cpp
 * @brief Implementation of the ServerParams class for managing server parameters in the SLAMWARE ROS SDK.
 */

#include "server_params.h"

#include <cmath>

namespace slamware_ros_sdk
{

    //////////////////////////////////////////////////////////////////////////

    const float C_FLT_PI = ((float)M_PI);
    const float C_FLT_2PI = (C_FLT_PI * 2);

    //////////////////////////////////////////////////////////////////////////

    ServerParams::ServerParams()
    {
        resetToDefault();
    }

    void ServerParams::resetToDefault()
    {
        ip_address = "192.168.11.1";
        robot_port = 1445;
        reconn_wait_ms = (1000U * 3U);

        angle_compensate = true;
        ladar_data_clockwise = true;

        robot_frame = "base_link";
        laser_frame = "laser";
        map_frame = "map";
        imu_frame = "imu_link";
        camera_left = "camera_left";
        camera_right = "camera_right";
        odom_frame = "odom";

        robot_pose_pub_period = 0.05f;
        scan_pub_period = 0.1f;
        map_update_period = 0.2f;
        map_pub_period = 0.2f;

        map_sync_once_get_max_wh = 100.f;
        map_update_near_robot_half_wh = 8.0f;

        imu_raw_data_period = 0.05f;
        system_status_pub_period = 0.05f;
        // stereo_image_pub_period = 0.067f;
        stereo_image_pub_period = 0.05f;
        point_cloud_pub_period = 0.1f;
        robot_basic_state_pub_period = 0.1f;
        odometry_pub_period = 0.1f;
        use_aurora_pose_covariance = true;
        aurora_covariance_max_age = 0.5;
        odom_covariance_unavailable_variance = 1.0e6;
        odom_twist_linear_variance = 0.0025;   // (0.05 m/s)^2
        odom_twist_angular_variance = 0.01;    // (0.1 rad/s)^2
        enhanced_imaging_pub_period = 0.05f;

        scan_topic = "/slamware_ros_sdk_server_node/scan";
        robot_pose_topic = "/slamware_ros_sdk_server_node/robot_pose";
        odom_topic = "/slamware_ros_sdk_server_node/odom";
        map_topic = "/slamware_ros_sdk_server_node/map";
        map_info_topic = "/slamware_ros_sdk_server_node/map_metadata";
        system_status_topic_name = "/slamware_ros_sdk_server_node/system_status";
        relocalization_status_topic_name = "/slamware_ros_sdk_server_node/relocalization_status";
        left_image_raw_topic_name = "/slamware_ros_sdk_server_node/left_image_raw";
        right_image_raw_topic_name = "/slamware_ros_sdk_server_node/right_image_raw";
        point_cloud_topic_name = "/slamware_ros_sdk_server_node/point_cloud";
        stereo_keypoints_topic_name = "/slamware_ros_sdk_server_node/stereo_keypoints";
        imu_raw_data_topic = "/slamware_ros_sdk_server_node/imu_raw_data";

        // Enhanced imaging topics
        depth_image_raw_topic_name = "/slamware_ros_sdk_server_node/depth_image_raw";
        depth_image_colorized_topic_name = "/slamware_ros_sdk_server_node/depth_image_colorized";
        semantic_segmentation_topic_name = "/slamware_ros_sdk_server_node/semantic_segmentation";
        // 0528===========================================
        slam_pose_is_laser_frame = false;
        publish_laser_pose_tf = true;
        align_map_to_initial_yaw = false;
        aligned_map_frame = "map_aligned";
        aligned_map_topic = "/slamware_ros_sdk_server_node/map_aligned";
        aligned_map_info_topic = "/slamware_ros_sdk_server_node/map_aligned_metadata";
        base_to_laser_x = 0.0;
        base_to_laser_y = 0.0;
        base_to_laser_z = 0.0;
        base_to_laser_roll = 0.0;
        base_to_laser_pitch = 0.0;
        base_to_laser_yaw = 0.0;
        // ===========================================
        event_period = 1.0f;
        no_preview_image = false;
        raw_image_on = false;
        
    }

    void ServerParams::setBy(const ros::NodeHandle &nhRos)
    {

        nhRos.getParam("event_period", event_period);

        // new ----------------
        nhRos.getParam("ip_address", ip_address);
        nhRos.getParam("reconn_wait_ms", reconn_wait_ms);

        nhRos.getParam("angle_compensate", angle_compensate);
        nhRos.getParam("ladar_data_clockwise", ladar_data_clockwise);

        nhRos.getParam("robot_frame", robot_frame);
        nhRos.getParam("laser_frame", laser_frame);
        nhRos.getParam("map_frame", map_frame);
        nhRos.getParam("imu_frame", imu_frame);
        nhRos.getParam("camera_left", camera_left);
        nhRos.getParam("camera_right", camera_right);
        nhRos.getParam("odom_frame", odom_frame);

        nhRos.getParam("robot_pose_pub_period", robot_pose_pub_period);
        nhRos.getParam("scan_pub_period", scan_pub_period);
        nhRos.getParam("map_update_period", map_update_period);
        nhRos.getParam("map_pub_period", map_pub_period);

        nhRos.getParam("map_sync_once_get_max_wh", map_sync_once_get_max_wh);
        nhRos.getParam("map_update_near_robot_half_wh", map_update_near_robot_half_wh);

        nhRos.getParam("imu_raw_data_period", imu_raw_data_period);
        nhRos.getParam("system_status_pub_period", system_status_pub_period);
        nhRos.getParam("stereo_image_pub_period", stereo_image_pub_period);
        nhRos.getParam("point_cloud_pub_period", point_cloud_pub_period);
        nhRos.getParam("robot_basic_state_pub_period", robot_basic_state_pub_period);
        nhRos.getParam("odometry_pub_period", odometry_pub_period);
        nhRos.getParam("use_aurora_pose_covariance", use_aurora_pose_covariance);
        nhRos.getParam("aurora_covariance_max_age", aurora_covariance_max_age);
        nhRos.getParam("odom_covariance_unavailable_variance", odom_covariance_unavailable_variance);
        nhRos.getParam("odom_twist_linear_variance", odom_twist_linear_variance);
        nhRos.getParam("odom_twist_angular_variance", odom_twist_angular_variance);

        nhRos.getParam("scan_topic", scan_topic);
        nhRos.getParam("robot_pose_topic", robot_pose_topic);
        nhRos.getParam("odom_topic", odom_topic);
        nhRos.getParam("map_topic", map_topic);
        nhRos.getParam("map_info_topic", map_info_topic);
        nhRos.getParam("system_status_topic_name", system_status_topic_name);
        nhRos.getParam("relocalization_status_topic_name", relocalization_status_topic_name);
        nhRos.getParam("left_image_raw_topic_name", left_image_raw_topic_name);
        nhRos.getParam("right_image_raw_topic_name", right_image_raw_topic_name);
        nhRos.getParam("point_cloud_topic_name", point_cloud_topic_name);
        nhRos.getParam("stereo_keypoints_topic_name", stereo_keypoints_topic_name);
        nhRos.getParam("imu_raw_data_topic", imu_raw_data_topic);
        // 0528=============================================Enhanced imaging topics
        nhRos.getParam("slam_pose_is_laser_frame", slam_pose_is_laser_frame);
        nhRos.getParam("publish_laser_pose_tf", publish_laser_pose_tf);
        nhRos.getParam("align_map_to_initial_yaw", align_map_to_initial_yaw);
        nhRos.getParam("aligned_map_frame", aligned_map_frame);
        nhRos.getParam("aligned_map_topic", aligned_map_topic);
        nhRos.getParam("aligned_map_info_topic", aligned_map_info_topic);
        nhRos.getParam("base_to_laser_x", base_to_laser_x);
        nhRos.getParam("base_to_laser_y", base_to_laser_y);
        nhRos.getParam("base_to_laser_z", base_to_laser_z);
        nhRos.getParam("base_to_laser_roll", base_to_laser_roll);
        nhRos.getParam("base_to_laser_pitch", base_to_laser_pitch);
        nhRos.getParam("base_to_laser_yaw", base_to_laser_yaw);
        // ================================================
        nhRos.getParam("depth_image_raw_topic_name", depth_image_raw_topic_name);
        nhRos.getParam("depth_image_colorized_topic_name", depth_image_colorized_topic_name);
        nhRos.getParam("semantic_segmentation_topic_name", semantic_segmentation_topic_name);

        nhRos.getParam("no_preview_image", no_preview_image);
        nhRos.getParam("raw_image_on", raw_image_on);
    }

}
