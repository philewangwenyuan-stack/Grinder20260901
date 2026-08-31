/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Eitan Marder-Eppstein, Sachin Chitta
*********************************************************************/
#include <angles/angles.h>
#include <carrot_planner/carrot_planner.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/locks.hpp>
#include <algorithm>
#include <cmath>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(carrot_planner::CarrotPlanner, nav_core::BaseGlobalPlanner)

namespace carrot_planner {

  CarrotPlanner::CarrotPlanner()
  : costmap_ros_(NULL), costmap_(NULL), world_model_(NULL), use_external_plan_(true),
    external_plan_max_age_s_(2.0), external_plan_goal_tolerance_m_(0.5),
    external_plan_wait_timeout_s_(0.2), has_external_plan_(false), initialized_(false){}

  CarrotPlanner::CarrotPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), costmap_(NULL), world_model_(NULL), use_external_plan_(true),
    external_plan_max_age_s_(2.0), external_plan_goal_tolerance_m_(0.5),
    external_plan_wait_timeout_s_(0.2), has_external_plan_(false), initialized_(false){
    initialize(name, costmap_ros);
  }

  CarrotPlanner::~CarrotPlanner() {
    if (external_plan_spinner_) {
      external_plan_spinner_->stop();
    }
    // deleting a nullptr is a noop
    delete world_model_;
  }
  
  void CarrotPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle private_nh("~/" + name);
      private_nh.param("step_size", step_size_, costmap_->getResolution());
      private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
      private_nh.param("use_external_plan", use_external_plan_, true);
      private_nh.param("external_plan_topic", external_plan_topic_, std::string("/grinder/navigation/active_segment_plan"));
      private_nh.param("external_plan_max_age_s", external_plan_max_age_s_, 2.0);
      private_nh.param("external_plan_goal_tolerance_m", external_plan_goal_tolerance_m_, 0.5);
      private_nh.param("external_plan_wait_timeout_s", external_plan_wait_timeout_s_, 0.2);
      world_model_ = new base_local_planner::CostmapModel(*costmap_); 

      if (use_external_plan_) {
        ros::NodeHandle external_nh;
        external_nh.setCallbackQueue(&external_plan_queue_);
        external_plan_sub_ = external_nh.subscribe<nav_msgs::Path>(
            external_plan_topic_, 1, &CarrotPlanner::externalPlanCB, this);
        external_plan_spinner_.reset(new ros::AsyncSpinner(1, &external_plan_queue_));
        external_plan_spinner_->start();
        ROS_INFO("CarrotPlanner external segment plan enabled: topic=%s max_age=%.3fs goal_tol=%.3fm wait=%.3fs",
            external_plan_topic_.c_str(), external_plan_max_age_s_,
            external_plan_goal_tolerance_m_, external_plan_wait_timeout_s_);
      }

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double CarrotPlanner::footprintCost(double x_i, double y_i, double theta_i){
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return -1.0;
    }

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    //if we have no footprint... do nothing
    if(footprint.size() < 3)
      return -1.0;

    //check if the footprint is legal
    double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
    return footprint_cost;
  }


  bool CarrotPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();
    costmap_ = costmap_ros_->getCostmap();

    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
          costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }

    if (use_external_plan_ && getExternalPlanForGoal(goal, plan)) {
      ROS_INFO_THROTTLE(2.0, "CarrotPlanner returned external active segment plan: points=%lu",
          static_cast<unsigned long>(plan.size()));
      return true;
    }

    ROS_WARN_THROTTLE(2.0, "CarrotPlanner external active segment unavailable or unmatched; falling back to two-point carrot plan.");
    return makeCarrotPlan(start, goal, plan);
  }

  bool CarrotPlanner::makeCarrotPlan(const geometry_msgs::PoseStamped& start,
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    plan.clear();

    const double start_yaw = tf2::getYaw(start.pose.orientation);
    const double goal_yaw = tf2::getYaw(goal.pose.orientation);

    //we want to step back along the vector created by the robot's position and the goal pose until we find a legal cell
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;

    double diff_x = goal_x - start_x;
    double diff_y = goal_y - start_y;
    double diff_yaw = angles::normalize_angle(goal_yaw-start_yaw);

    double target_x = goal_x;
    double target_y = goal_y;
    double target_yaw = goal_yaw;

    bool done = false;
    double scale = 1.0;
    double dScale = 0.01;

    while(!done)
    {
      if(scale < 0)
      {
        target_x = start_x;
        target_y = start_y;
        target_yaw = start_yaw;
        ROS_WARN("The carrot planner could not find a valid plan for this goal");
        break;
      }
      target_x = start_x + scale * diff_x;
      target_y = start_y + scale * diff_y;
      target_yaw = angles::normalize_angle(start_yaw + scale * diff_yaw);
      
      double footprint_cost = footprintCost(target_x, target_y, target_yaw);
      if(footprint_cost >= 0)
      {
          done = true;
      }
      scale -=dScale;
    }

    plan.push_back(start);
    geometry_msgs::PoseStamped new_goal = goal;
    tf2::Quaternion goal_quat;
    goal_quat.setRPY(0, 0, target_yaw);

    new_goal.pose.position.x = target_x;
    new_goal.pose.position.y = target_y;

    new_goal.pose.orientation.x = goal_quat.x();
    new_goal.pose.orientation.y = goal_quat.y();
    new_goal.pose.orientation.z = goal_quat.z();
    new_goal.pose.orientation.w = goal_quat.w();

    plan.push_back(new_goal);
    if (!done) {
      ROS_WARN_THROTTLE(2.0, "CarrotPlanner fallback could not find a valid footprint target; returning start-clamped two-point plan.");
    }
    return true;
  }

  void CarrotPlanner::externalPlanCB(const nav_msgs::Path::ConstPtr& path_msg) {
    boost::mutex::scoped_lock lock(external_plan_mutex_);
    latest_external_plan_ = *path_msg;
    latest_external_plan_receive_time_ = ros::Time::now();
    has_external_plan_ = !latest_external_plan_.poses.empty();
    external_plan_cv_.notify_all();
  }

  bool CarrotPlanner::getExternalPlanForGoal(const geometry_msgs::PoseStamped& goal,
      std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!use_external_plan_) {
      return false;
    }
    ros::Time deadline = ros::Time::now() + ros::Duration(std::max(0.0, external_plan_wait_timeout_s_));
    boost::unique_lock<boost::mutex> lock(external_plan_mutex_);
    while (ros::ok()) {
      if (copyMatchingExternalPlanLocked(goal, plan)) {
        return true;
      }
      if (ros::Time::now() >= deadline) {
        return false;
      }
      external_plan_cv_.timed_wait(lock, boost::posix_time::milliseconds(20));
    }
    return false;
  }

  bool CarrotPlanner::copyMatchingExternalPlanLocked(const geometry_msgs::PoseStamped& goal,
      std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!has_external_plan_ || latest_external_plan_.poses.size() < 2) {
      return false;
    }
    const ros::Time now = ros::Time::now();
    if (external_plan_max_age_s_ > 0.0 &&
        (now - latest_external_plan_receive_time_).toSec() > external_plan_max_age_s_) {
      ROS_WARN_THROTTLE(2.0, "External active segment plan is stale: age=%.3fs max=%.3fs",
          (now - latest_external_plan_receive_time_).toSec(), external_plan_max_age_s_);
      return false;
    }
    const std::string plan_frame = latest_external_plan_.header.frame_id.empty()
        ? latest_external_plan_.poses.front().header.frame_id
        : latest_external_plan_.header.frame_id;
    if (plan_frame != costmap_ros_->getGlobalFrameID()) {
      ROS_WARN_THROTTLE(2.0, "External active segment frame mismatch: plan=%s global=%s",
          plan_frame.c_str(), costmap_ros_->getGlobalFrameID().c_str());
      return false;
    }

    const geometry_msgs::PoseStamped& last_pose = latest_external_plan_.poses.back();
    const double dx = last_pose.pose.position.x - goal.pose.position.x;
    const double dy = last_pose.pose.position.y - goal.pose.position.y;
    const double goal_dist = std::sqrt(dx * dx + dy * dy);
    if (goal_dist > external_plan_goal_tolerance_m_) {
      ROS_WARN_THROTTLE(2.0, "External active segment endpoint mismatch: dist=%.3fm tol=%.3fm points=%lu",
          goal_dist, external_plan_goal_tolerance_m_,
          static_cast<unsigned long>(latest_external_plan_.poses.size()));
      return false;
    }

    plan.clear();
    plan.reserve(latest_external_plan_.poses.size());
    for (std::size_t i = 0; i < latest_external_plan_.poses.size(); ++i) {
      geometry_msgs::PoseStamped pose = latest_external_plan_.poses[i];
      if (pose.header.frame_id.empty()) {
        pose.header.frame_id = plan_frame;
      }
      pose.header.stamp = ros::Time(0);
      plan.push_back(pose);
    }
    return true;
  }

};
