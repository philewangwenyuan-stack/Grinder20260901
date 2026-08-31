// Copyright (c) 2022 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>
#include <tf2/utils.h>

#include "regulated_pure_pursuit_controller/path_handler.hpp"
#include "regulated_pure_pursuit_controller/geometry_utils.h"

namespace regulated_pure_pursuit_controller
{

using regulated_pure_pursuit_controller::geometry_utils::euclidean_distance;

PathHandler::PathHandler(
  ros::Duration transform_tolerance,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros)
: transform_tolerance_(transform_tolerance), tf_(tf), costmap_ros_(costmap_ros)
{
}

double PathHandler::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters = std::max(
    costmap_ros_->getCostmap()->getSizeInMetersX(),
    costmap_ros_->getCostmap()->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}

nav_msgs::Path PathHandler::transformGlobalPlan(
  const geometry_msgs::PoseStamped & pose,
  double max_robot_pose_search_dist,
  bool use_global_plan_search_fallback,
  bool reject_unit_path)
{
  std::lock_guard<std::mutex> plan_lock(plan_mutex_);

  if (global_plan_.poses.empty()) {
    throw std::runtime_error("[RPP] Received plan with zero length");
  }

  if (reject_unit_path && global_plan_.poses.size() == 1) {
    throw std::runtime_error("[RPP] Received plan with length of one");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw std::runtime_error("[RPP] Unable to transform robot pose into global plan's frame");
  }

  const double max_costmap_extent = getCostmapMaxExtent();
  bool fallback_used = false;
  const auto distance_to_robot = [&robot_pose](const geometry_msgs::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    };
  auto transformation_begin =
    regulated_pure_pursuit_controller::geometry_utils::find_closest_pose_with_search_fallback(
    global_plan_.poses.begin(), global_plan_.poses.end(),
    max_robot_pose_search_dist, max_costmap_extent,
    use_global_plan_search_fallback, distance_to_robot, fallback_used);

  auto closest_pose_upper_bound =
    regulated_pure_pursuit_controller::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist);

  if (fallback_used) {
    const auto skipped_poses = static_cast<std::size_t>(
      std::distance(global_plan_.poses.begin(), transformation_begin));
    ROS_WARN_THROTTLE(
      2.0,
      "[RPP] Bounded plan search no longer reached the robot; fallback matched "
      "the full plan at distance %.3fm and will prune %zu passed poses",
      distance_to_robot(*transformation_begin), skipped_poses);
  }

  // Make sure we always have at least 2 points on the transformed plan and that we don't prune
  // the global plan below 2 points in order to have always enough point to interpolate the
  // end of path direction
  if (std::distance(global_plan_.poses.begin(), closest_pose_upper_bound) >= 2 &&
    global_plan_.poses.size() > 1 &&
    transformation_begin == std::prev(closest_pose_upper_bound))
  {
    transformation_begin = std::prev(std::prev(closest_pose_upper_bound));
  }

  // We'll discard points on the plan that are outside the local costmap
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & global_plan_pose) {
      return euclidean_distance(global_plan_pose, robot_pose) > max_costmap_extent;
    });

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      geometry_msgs::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = robot_pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      if (!transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose)) {
        throw std::runtime_error("Unable to transform plan pose into local frame");
      }
      transformed_pose.pose.position.z = 0.0;
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  const auto newly_pruned = static_cast<std::size_t>(
    std::distance(global_plan_.poses.begin(), transformation_begin));
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  pruned_pose_count_ += newly_pruned;
  if (!last_source_plan_.poses.empty()) {
    const std::size_t max_pruned = last_source_plan_.poses.size() > 1 ?
      last_source_plan_.poses.size() - 2 : 0;
    pruned_pose_count_ = std::min(pruned_pose_count_, max_pruned);
  }

  if (transformed_plan.poses.empty()) {
    throw std::runtime_error("[RPP] Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

std::size_t PathHandler::setPlan(
  const nav_msgs::Path & path, bool preserve_progress,
  double equivalence_tolerance_m)
{
  std::lock_guard<std::mutex> plan_lock(plan_mutex_);

  const bool equivalent = preserve_progress &&
    plansHaveEquivalentGeometry(path, last_source_plan_, equivalence_tolerance_m);
  last_plan_update_equivalent_ = equivalent;
  if (!equivalent) {
    pruned_pose_count_ = 0;
  }

  last_source_plan_ = path;
  global_plan_ = path;

  std::size_t reapplied_pruned_count = 0;
  if (equivalent && global_plan_.poses.size() > 2) {
    reapplied_pruned_count = std::min(
      pruned_pose_count_, global_plan_.poses.size() - 2);
    global_plan_.poses.erase(
      global_plan_.poses.begin(),
      global_plan_.poses.begin() + static_cast<std::ptrdiff_t>(reapplied_pruned_count));
  }
  return reapplied_pruned_count;
}

bool PathHandler::lastPlanUpdateWasEquivalent() const
{
  std::lock_guard<std::mutex> plan_lock(plan_mutex_);
  return last_plan_update_equivalent_;
}

nav_msgs::Path PathHandler::getPlan() const
{
  std::lock_guard<std::mutex> plan_lock(plan_mutex_);
  return global_plan_;
}

bool PathHandler::plansHaveEquivalentGeometry(
  const nav_msgs::Path & lhs, const nav_msgs::Path & rhs,
  double tolerance_m) const
{
  if (lhs.poses.size() != rhs.poses.size() || lhs.poses.empty()) {
    return false;
  }
  if (lhs.header.frame_id != rhs.header.frame_id) {
    return false;
  }

  const double tolerance = std::max(0.0, tolerance_m);
  for (std::size_t i = 0; i < lhs.poses.size(); ++i) {
    if (euclidean_distance(lhs.poses[i], rhs.poses[i]) > tolerance) {
      return false;
    }
  }
  return true;
}

bool PathHandler::transformPose(
  const std::string frame,
  const geometry_msgs::PoseStamped & in_pose,
  geometry_msgs::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    ROS_ERROR("[RPP] Exception in transformPose: %s", ex.what());
  }
  return false;
}

}  // namespace regulated_pure_pursuit_controller
