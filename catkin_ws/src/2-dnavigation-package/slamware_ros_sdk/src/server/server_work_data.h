/**
 * @file server_work_data.h
 * @brief Defines the ServerWorkData structure and related types for managing server work data in the SLAMWARE ROS SDK.
 */

#pragma once

#include <atomic>
#include <ros/ros.h>

#include "server_map_holder.h"

#include <geometry_msgs/PoseStamped.h>

namespace slamware_ros_sdk
{

    struct ServerWorkData
    {
    public:
        geometry_msgs::PoseStamped robotPose;
        // Aurora device timestamp corresponding to robotPose, in nanoseconds.
        std::atomic<uint64_t> robotPoseSdkTimestampNs;

        std::atomic<bool> syncMapRequested;
        std::atomic<bool> hasMapYawAlignment;
        std::atomic<double> mapYawAlignmentYaw;
        ServerMapHolder exploreMapHolder;

    public:
        ServerWorkData();

    public:
        static inline bool sfIsDigitalSensorValueImpact(float fVal) { return fVal < FLT_EPSILON; }
    };

    typedef std::shared_ptr<ServerWorkData> ServerWorkData_Ptr;
    typedef std::shared_ptr<const ServerWorkData> ServerWorkData_ConstPtr;

}
