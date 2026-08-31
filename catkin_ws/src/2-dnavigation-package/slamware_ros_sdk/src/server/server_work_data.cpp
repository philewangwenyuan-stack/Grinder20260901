/*
 * @file server_work_data.cpp
 * @brief Implementation of the ServerWorkData class for the SLAMWARE ROS SDK.
 */

#include "server_work_data.h"

namespace slamware_ros_sdk
{

    ServerWorkData::ServerWorkData()
        : robotPoseSdkTimestampNs(0),
          syncMapRequested(true),
          hasMapYawAlignment(false),
          mapYawAlignmentYaw(0.0)
    {
        //
    }

}
