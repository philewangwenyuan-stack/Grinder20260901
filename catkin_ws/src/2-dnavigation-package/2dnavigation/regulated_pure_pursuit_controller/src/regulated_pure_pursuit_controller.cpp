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
#include <cmath>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "angles/angles.h"
#include "regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "regulated_pure_pursuit_controller/command_velocity_memory.hpp"
#include "regulated_pure_pursuit_controller/geometry_utils.h"
#include <pluginlib/class_list_macros.h>

using std::hypot;
using std::min;
using std::max;
using std::abs;
using namespace costmap_2d;  // NOLINT

PLUGINLIB_EXPORT_CLASS(regulated_pure_pursuit_controller::RegulatedPurePursuitController, nav_core::BaseLocalPlanner)
PLUGINLIB_EXPORT_CLASS(regulated_pure_pursuit_controller::RegulatedPurePursuitController, mbf_costmap_core::CostmapController)

namespace regulated_pure_pursuit_controller
{

namespace
{

bool getFirstPathSegmentHeading(const nav_msgs::Path & path, double & heading)
{
  for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const double dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
    const double dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
    if (std::hypot(dx, dy) > 1e-6) {
      heading = std::atan2(dy, dx);
      return true;
    }
  }
  return false;
}

bool findFirstSharpCorner(
  const nav_msgs::Path & path, double min_turn_angle,
  geometry_msgs::Point & corner, double & outgoing_heading)
{
  bool have_previous_heading = false;
  double previous_heading = 0.0;
  for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const double dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
    const double dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
    if (std::hypot(dx, dy) <= 1e-6) {
      continue;
    }
    const double heading = std::atan2(dy, dx);
    if (have_previous_heading &&
      std::abs(angles::shortest_angular_distance(previous_heading, heading)) > min_turn_angle)
    {
      corner = path.poses[i].pose.position;
      outgoing_heading = heading;
      return true;
    }
    previous_heading = heading;
    have_previous_heading = true;
  }
  return false;
}

}  // namespace

void RegulatedPurePursuitController::initialize(
  std::string name, tf2_ros::Buffer* tf,
  costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!isInitialized() ){
    node_ = std::make_shared<ros::NodeHandle>("");
    private_node_ = std::make_shared<ros::NodeHandle>("~" + name);
    // move_base owns both objects. Keep non-owning shared_ptr wrappers because
    // the upstream helper APIs accept shared_ptr, but never delete these pointers.
    costmap_ros_ = std::shared_ptr<costmap_2d::Costmap2DROS>(
      costmap_ros, [](costmap_2d::Costmap2DROS*) {});
    costmap_ = costmap_ros_->getCostmap();
    tf_ = std::shared_ptr<tf2_ros::Buffer>(tf, [](tf2_ros::Buffer*) {});
    private_node_->param<std::string>("odom_topic", odom_topic_, "odom");
    odom_helper_.setOdomTopic( odom_topic_ );

    // Handles storage and dynamic configuration of parameters.
    // Returns pointer to data current param settings.
    param_handler_ = std::make_unique<ParameterHandler>(
      node_, private_node_, costmap_->getSizeInMetersX());
    params_ = param_handler_->getParams();

    // Handles global path transformations
    path_handler_ = std::make_unique<PathHandler>(
      ros::Duration(params_->transform_tolerance), tf_, costmap_ros_);

    // Checks for imminent collisions
    collision_checker_ = std::make_unique<CollisionChecker>(node_, costmap_ros_, params_);
    last_cmd_linear_vel_ = 0.0;
    last_cmd_angular_vel_ = 0.0;
    last_cmd_time_ = ros::WallTime();

    double control_frequency = 20.0;
    goal_dist_tol_ = 0.25;  // reasonable default before first update

    ros::NodeHandle move_base_private_nh("~");
    move_base_private_nh.param("controller_frequency", control_frequency, 20.0);
    if (control_frequency <= 0.0) {
      ROS_WARN("[RPP] controller_frequency must be positive. Falling back to 20 Hz.");
      control_frequency = 20.0;
    }
    control_duration_ = 1.0 / control_frequency;

    global_path_pub_ = private_node_->advertise<nav_msgs::Path>("received_global_plan", 1);
    global_path_origin_pub_ = private_node_->advertise<nav_msgs::Path>("global_plan_origin", 1);
    carrot_pub_ = private_node_->advertise<geometry_msgs::PointStamped>("lookahead_point", 1);
    vector_pub_ = private_node_->advertise<geometry_msgs::PoseStamped>("lookahead_vector", 1);
    curvature_carrot_pub_ = private_node_->advertise<geometry_msgs::PointStamped>(
      "curvature_lookahead_point", 1);
    // Keep the last safety state available for a scheduler that starts after
    // RPP. Use an absolute topic name so it is stable across plugin names.
    collision_imminent_pub_ = node_->advertise<std_msgs::Bool>(
      "/rpp/collision_imminent", 1, true);
    std_msgs::Bool collision_state;
    collision_state.data = false;
    collision_imminent_pub_.publish(collision_state);
    scan_sub_ = node_->subscribe(
      params_->collision_scan_topic, 1,
      &RegulatedPurePursuitController::scanCallback, this);
    // Keep clearing the fixed-distance collision latch while the navigation
    // action is paused after a collision. Otherwise no further controller
    // cycle may run to observe the clear scan and publish false.
    collision_clear_timer_ = node_->createTimer(
      ros::Duration(0.05),
      &RegulatedPurePursuitController::collisionClearTimerCallback, this);
    
    initialized_ = true;
    ROS_INFO("[RPP] RegulatedPurePursuitController initialized.");
  }

  else{
    ROS_WARN("[RPP] RegulatedPurePursuitController already initialized.");
  }

}

geometry_msgs::PointStamped RegulatedPurePursuitController::createCarrotMsg(
  const geometry_msgs::PoseStamped & carrot_pose)
{
  auto carrot_msg = geometry_msgs::PointStamped();
  carrot_msg.header = carrot_pose.header;
  carrot_msg.point.x = carrot_pose.pose.position.x;
  carrot_msg.point.y = carrot_pose.pose.position.y;
  carrot_msg.point.z = 0.01;  // publish right over map to stand out
  return carrot_msg;
}

double RegulatedPurePursuitController::getLookAheadDistance(
  const geometry_msgs::Twist & speed)
{
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Else, use the static look ahead distance
  double lookahead_dist = params_->lookahead_dist;
  if (params_->use_velocity_scaled_lookahead_dist) {
    lookahead_dist = fabs(speed.linear.x) * params_->lookahead_time;
    lookahead_dist = std::clamp(
      lookahead_dist, params_->min_lookahead_dist, params_->max_lookahead_dist);
  }

  return lookahead_dist;
}

double calculateCurvature(geometry_msgs::Point lookahead_point)
{
  // Find distance^2 to look ahead point (carrot) in robot base frame
  // This is the chord length of the circle
  const double carrot_dist2 =
    (lookahead_point.x * lookahead_point.x) +
    (lookahead_point.y * lookahead_point.y);

  // Find curvature of circle (k = 1 / R)
  if (carrot_dist2 > 0.001) {
    return 2.0 * lookahead_point.y / carrot_dist2;
  } else {
    return 0.0;
  }
}

double RegulatedPurePursuitController::calcTurningRadius(const geometry_msgs::PoseStamped & target_pose)
{
  // Calculate angle to lookahead point
  double target_angle = angles::normalize_angle(tf2::getYaw(target_pose.pose.orientation));
  double distance = std::hypot(target_pose.pose.position.x, target_pose.pose.position.y);
  // Compute turning radius (screw center)
  double turning_radius;
  if (params_->allow_reversing || target_pose.pose.position.x >= 0.0) {
    if (std::abs(target_pose.pose.position.y) > 1e-6) {
      double phi_1 = std::atan2(
        (2 * std::pow(target_pose.pose.position.y, 2) - std::pow(distance, 2)),
        (2 * target_pose.pose.position.x * target_pose.pose.position.y));
      double term_2 = std::pow(distance, 2) / (2 * target_pose.pose.position.y);
      double phi_2 = std::atan2(term_2, 0.0);
      double phi = angles::normalize_angle_positive(phi_1 - phi_2);
      phi = std::max(phi, 1.0e-9);
      double term_1 = (params_->k * phi) / (((params_->k - 1) * phi) + target_angle);
      turning_radius = std::abs(term_1 * term_2);
    } else {
      // Handle case when target is directly ahead
      turning_radius = std::numeric_limits<double>::max();
    }
  } else {
    // If lookahead point is behind the robot, set turning radius to minimum
    turning_radius = params_->min_turning_radius;
  }

  // Limit turning radius to avoid extremely sharp turns
  turning_radius = std::max(turning_radius, params_->min_turning_radius);
  if (target_pose.pose.position.y < 0) {
    turning_radius *= -1;
  }
  return turning_radius;
}

geometry_msgs::Quaternion RegulatedPurePursuitController::getOrientation(
  const geometry_msgs::Point & p1,
  const geometry_msgs::Point & p2)
{
  tf2::Quaternion tf2_quat;

  double yaw = std::atan2(p2.y - p1.y, p2.x - p1.x);
  tf2_quat.setRPY(0.0, 0.0, yaw);
  geometry_msgs::Quaternion quat_msg = tf2::toMsg(tf2_quat);

  return quat_msg;
}

void RegulatedPurePursuitController::getRobotVel(geometry_msgs::Twist& speed){
        nav_msgs::Odometry robot_odom;

        odom_helper_.getOdom(robot_odom);

        speed.linear.x = robot_odom.twist.twist.linear.x;
        speed.angular.z = robot_odom.twist.twist.angular.z;
    }

bool RegulatedPurePursuitController::isThetaGoalReached(double dtheta, double angle_tolerance, 
                                                        double max_angular_vel, double dt)
{
    if (fabs(dtheta) < angle_tolerance || fabs(dtheta) < max_angular_vel * dt)
    {
        return true;
    }
    return false;
}

bool RegulatedPurePursuitController::isGoalReached()
{
  if (goal_reached_){
      ROS_INFO("[RPP] Goal Reached!");
      return true;
  }
  return false;
}

bool RegulatedPurePursuitController::isGoalReached(double xy_tolerance, double yaw_tolerance)
{
  if (goal_reached_){
      ROS_INFO("[RPP] Goal Reached!");
      return true;
  }
  return false;
}

uint32_t RegulatedPurePursuitController::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                  const geometry_msgs::TwistStamped& velocity,
                                  geometry_msgs::TwistStamped& cmd_vel,
                                  std::string& message) 
{
  cmd_vel = geometry_msgs::TwistStamped();
  cmd_vel.header = pose.header;

  if(!initialized_)
  {
      ROS_ERROR("[RPP] RegulatedPurePursuitController has not been initialized");
      message = "RegulatedPurePursuitController has not been initialized";
      return mbf_msgs::ExePathResult::NOT_INITIALIZED;
  }

  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());
  costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  goal_dist_tol_ = params_->goal_dist_tol;
  angle_tolerance_ = params_->angle_tol;
  max_angular_vel_ = params_->max_angular_vel;
  theta_stopped_vel_ = params_->theta_stopped_vel;
  trans_stopped_vel_ = params_->trans_stopped_vel;
  goal_reached_ = false;

  // Transform path to robot base frame. A malformed plan or transient TF
  // failure must stop the robot instead of unwinding through move_base.
  nav_msgs::Path transformed_plan;
  try {
    transformed_plan = path_handler_->transformGlobalPlan(
      pose, params_->max_robot_pose_search_dist,
      params_->use_global_plan_search_fallback,
      params_->interpolate_curvature_after_goal);
  } catch (const std::exception& ex) {
    message = ex.what();
    ROS_ERROR_THROTTLE(1.0, "[RPP] Failed to transform global plan: %s", ex.what());
    last_cmd_linear_vel_ = 0.0;
    last_cmd_angular_vel_ = 0.0;
    last_cmd_time_ = ros::WallTime();
    resetRotateToHeadingState();
    return mbf_msgs::ExePathResult::INVALID_PATH;
  }
  global_path_pub_.publish(transformed_plan);

  if (transformed_plan.poses.empty())
  {
      ROS_WARN("[RPP] Transformed plan is empty. Cannot determine a local plan.");
      message = "Transformed plan is empty";
      last_cmd_linear_vel_ = 0.0;
      last_cmd_angular_vel_ = 0.0;
      last_cmd_time_ = ros::WallTime();
      resetRotateToHeadingState();
      return mbf_msgs::ExePathResult::INVALID_PATH;
  }

  // Get current robot velocity
  nav_msgs::Odometry base_odom;
  odom_helper_.getOdom(base_odom);
  geometry_msgs::Twist speed = base_odom.twist.twist;

  geometry_msgs::PoseStamped global_goal = transformed_plan.poses.back();
  double dx_2 = global_goal.pose.position.x * global_goal.pose.position.x;
  double dy_2 = global_goal.pose.position.y * global_goal.pose.position.y;
  double dtheta = angles::normalize_angle(tf2::getYaw(global_goal.pose.orientation));

  if(fabs(std::sqrt(dx_2 + dy_2)) < goal_dist_tol_ && isThetaGoalReached(dtheta, angle_tolerance_, max_angular_vel_, control_duration_) && base_local_planner::stopped(base_odom, theta_stopped_vel_, trans_stopped_vel_))
  {
      goal_reached_ = true;
      last_cmd_linear_vel_ = 0.0;
      last_cmd_angular_vel_ = 0.0;
      last_cmd_time_ = ros::WallTime();
      resetRotateToHeadingState();
      return mbf_msgs::ExePathResult::SUCCESS;
  }

  // Find look ahead distance and point on path and publish
  double lookahead_dist = getLookAheadDistance(speed);

  // Check for reverse driving
  if (params_->allow_reversing) {
    // Cusp check
    const double dist_to_cusp = findVelocitySignChange(transformed_plan);

    // if the lookahead distance is further than the cusp, use the cusp distance instead
    if (dist_to_cusp < lookahead_dist) {
      lookahead_dist = dist_to_cusp;
    }
  }

  // Get the particular point on the path at the lookahead distance
  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  auto rotate_to_path_carrot_pose = carrot_pose;
  if(params_->use_vector_pure_pursuit){
    vector_pub_.publish(carrot_pose);
  }
  else{
    carrot_pub_.publish(createCarrotMsg(carrot_pose));
  }

  double linear_vel, angular_vel;
  double lookahead_curvature;
  if(params_->use_vector_pure_pursuit){
    // Implement vector pure pursuit
    double turning_radius = calcTurningRadius(carrot_pose);
    lookahead_curvature = 1.0 / turning_radius;
  }
  else{
    lookahead_curvature = calculateCurvature(carrot_pose.pose.position);
  }
  
  double regulation_curvature = lookahead_curvature;
  if (params_->use_fixed_curvature_lookahead) {
    auto curvature_lookahead_pose = getLookAheadPoint(
      params_->curvature_lookahead_dist,
      transformed_plan, params_->interpolate_curvature_after_goal);
    rotate_to_path_carrot_pose = curvature_lookahead_pose;
    
    if(params_->use_vector_pure_pursuit){
      // Implement vector pure pursuit
      double turning_radius = calcTurningRadius(curvature_lookahead_pose);
      regulation_curvature = 1.0 / turning_radius;
    }
    else{
      regulation_curvature = calculateCurvature(curvature_lookahead_pose.pose.position);
    }

    curvature_carrot_pub_.publish(createCarrotMsg(curvature_lookahead_pose));
  }

  // Setting the velocity direction
  double x_vel_sign = 1.0;
  if (params_->allow_reversing) {
    x_vel_sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
  }

  linear_vel = params_->desired_linear_vel;

  // Make sure we're in compliance with basic constraints
  // For shouldRotateToPath, using x_vel_sign in order to support allow_reversing
  // and rotate_to_path_carrot_pose for the direction carrot pose:
  //        - equal to "normal" carrot_pose when curvature_lookahead_pose = false
  //        - otherwise equal to curvature_lookahead_pose (which can be interpolated after goal)
  const bool near_goal = shouldRotateToGoalHeading(carrot_pose);
  const double angle_to_goal = angles::normalize_angle(
    tf2::getYaw(transformed_plan.poses.back().pose.orientation));
  double angle_to_heading = 0.0;
  bool should_enter_path_rotation = shouldRotateToPath(
    rotate_to_path_carrot_pose, angle_to_heading, x_vel_sign);
  bool corner_approach_active = false;
  double corner_distance = std::numeric_limits<double>::infinity();

  if (!near_goal && params_->use_rotate_to_heading &&
    params_->use_corner_aware_rotate_to_heading)
  {
    geometry_msgs::Point corner;
    double outgoing_heading = 0.0;
    const double enter_angle = std::max(0.0, params_->rotate_to_heading_min_angle);
    if (findFirstSharpCorner(
        transformed_plan, enter_angle, corner, outgoing_heading) &&
      std::abs(outgoing_heading) > enter_angle)
    {
      corner_distance = std::hypot(corner.x, corner.y);
      const double rotate_distance = std::max(
        0.01, params_->rotate_to_heading_corner_distance);
      double incoming_heading = 0.0;
      const bool incoming_misaligned = getFirstPathSegmentHeading(
        transformed_plan, incoming_heading) &&
        std::abs(incoming_heading) > enter_angle;

      if (incoming_misaligned && corner_distance > rotate_distance) {
        // Before approaching the corner, first align to its incoming edge.
        angle_to_heading = incoming_heading;
        should_enter_path_rotation = true;
      } else if (corner_distance <= rotate_distance) {
        // At the corner: align directly to the outgoing edge in one rotation.
        angle_to_heading = outgoing_heading;
        should_enter_path_rotation = true;
      } else {
        // Do not rotate early merely because the lookahead point has crossed
        // onto the next edge. Pursue the corner itself and slow down near it.
        should_enter_path_rotation = false;
        corner_approach_active = true;
        regulation_curvature = calculateCurvature(corner);
      }
    }
  }

  if (!params_->use_rotate_to_heading && is_rotating_to_heading_) {
    resetRotateToHeadingState();
  }

  // Enter rotation only once, then hold an absolute target yaw. The carrot
  // moves slightly as poses are pruned or the same plan is refreshed; using it
  // as a fresh target every cycle caused the old rotate/drive/rotate chatter.
  if (!is_rotating_to_heading_ && params_->use_rotate_to_heading) {
    if (near_goal && std::abs(angle_to_goal) > angle_tolerance_) {
      startRotateToHeading(pose, angle_to_goal, angle_tolerance_, true);
    } else if (!near_goal && should_enter_path_rotation) {
      const double exit_angle = std::min(
        std::max(0.0, params_->rotate_to_heading_exit_angle),
        std::max(0.0, params_->rotate_to_heading_min_angle));
      startRotateToHeading(pose, angle_to_heading, exit_angle, false);
    }
  }

  bool hold_for_heading = false;
  if (is_rotating_to_heading_) {
    hold_for_heading = true;
    const double robot_yaw = tf2::getYaw(pose.pose.orientation);
    const double heading_error = angles::shortest_angular_distance(
      robot_yaw, rotate_to_heading_target_yaw_);
    const bool crossed_target =
      rotate_to_heading_last_error_ * heading_error < 0.0;
    const bool entered_exit_band =
      std::abs(heading_error) <= rotate_to_heading_exit_angle_latched_;
    if (!rotate_to_heading_settling_ &&
      (entered_exit_band || crossed_target))
    {
      rotate_to_heading_settling_ = true;
      rotate_to_heading_stable_count_ = 0;
    }
    rotate_to_heading_last_error_ = heading_error;

    if (std::abs(speed.linear.x) > trans_stopped_vel_) {
      // Finish stopping translation before beginning the in-place rotation.
      linear_vel = 0.0;
      const double reference_speed =
        params_->use_command_angular_velocity_for_accel_limit ?
        last_cmd_angular_vel_ : speed.angular.z;
      const double max_step = std::max(0.0, params_->max_angular_accel) *
        control_duration_;
      angular_vel = std::clamp(
        0.0, reference_speed - max_step, reference_speed + max_step);
      rotate_to_heading_stable_count_ = 0;
    } else if (rotate_to_heading_settling_) {
      // The target has been reached. Brake in the current direction and wait
      // for odometry to stop instead of reversing immediately after overshoot.
      linear_vel = 0.0;
      const double reference_speed =
        params_->use_command_angular_velocity_for_accel_limit ?
        last_cmd_angular_vel_ : speed.angular.z;
      const double max_step = std::max(0.0, params_->max_angular_accel) *
        control_duration_;
      angular_vel = std::clamp(
        0.0, reference_speed - max_step, reference_speed + max_step);
    } else {
      rotateToHeading(linear_vel, angular_vel, heading_error, speed);
    }

    const bool inside_exit_angle =
      std::abs(heading_error) <= rotate_to_heading_exit_angle_latched_;
    const bool command_stopped = std::abs(angular_vel) <= theta_stopped_vel_;
    const bool robot_stopped = std::abs(speed.angular.z) <= theta_stopped_vel_;
    const bool translation_stopped = std::abs(speed.linear.x) <= trans_stopped_vel_;
    if (rotate_to_heading_settling_ && inside_exit_angle &&
      command_stopped && robot_stopped && translation_stopped)
    {
      ++rotate_to_heading_stable_count_;
    } else {
      rotate_to_heading_stable_count_ = 0;
    }

    // If inertia carried the stopped robot outside the exit band, perform one
    // new correction from rest. This avoids rapid left/right command reversal.
    if (rotate_to_heading_settling_ && command_stopped && robot_stopped &&
      translation_stopped && !inside_exit_angle)
    {
      rotate_to_heading_settling_ = false;
      rotate_to_heading_last_error_ = heading_error;
    }

    if (rotate_to_heading_stable_count_ >=
      std::max(1, params_->rotate_to_heading_stable_cycles))
    {
      ROS_INFO(
        "[RPP] Latched rotate-to-heading complete: target=%.3f rad, error=%.3f rad; "
        "driving may resume on the next control cycle",
        rotate_to_heading_target_yaw_, heading_error);
      resetRotateToHeadingState();
      linear_vel = 0.0;
      angular_vel = 0.0;
    }
  } else if (near_goal && params_->use_rotate_to_heading) {
    // The goal orientation is already within tolerance. Hold zero until odom
    // reports stopped so the goal checker can finish instead of creeping away.
    hold_for_heading = true;
    linear_vel = 0.0;
    angular_vel = 0.0;
  }

  if (!hold_for_heading) {
    applyConstraints(
      regulation_curvature,
      speed,
      collision_checker_->costAtPose(pose.pose.position.x, pose.pose.position.y), transformed_plan,
      linear_vel, x_vel_sign);

    if (corner_approach_active) {
      const double rotate_distance = std::max(
        0.01, params_->rotate_to_heading_corner_distance);
      const double slowdown_distance = std::max(
        rotate_distance + 1e-3, params_->approach_velocity_scaling_dist);
      if (corner_distance < slowdown_distance) {
        const double ratio = std::clamp(
          (corner_distance - rotate_distance) /
          (slowdown_distance - rotate_distance), 0.0, 1.0);
        const double corner_speed_limit = std::max(
          params_->min_approach_linear_velocity,
          params_->desired_linear_vel * ratio);
        linear_vel = std::copysign(
          std::min(std::abs(linear_vel), corner_speed_limit), x_vel_sign);
      }
    }
      
    // Apply curvature to angular velocity after constraining linear velocity
    angular_vel = linear_vel * regulation_curvature;
  }

  // Collision checking on this velocity heading
  const double & carrot_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);

  // Speed-independent laser clearance check. It can use either the legacy
  // front sector or the costmap footprint expanded by separate front/side
  // clearances. The side band remains active while latched so an in-place
  // turn cannot repeatedly clear and re-trigger on the same obstacle.
  bool fixed_collision = false;
  if (params_->use_fixed_distance_collision_detection) {
    FixedObstacleStats obstacle_stats;
    std::uint64_t scan_sequence = 0;
    bool collision_latched = false;
    {
      std::lock_guard<std::mutex> collision_lock(collision_state_mutex_);
      collision_latched = fixed_collision_latched_;
    }
    const bool scan_fresh = getFixedObstacleStats(
      angular_vel, collision_latched, obstacle_stats, scan_sequence);
    const std::size_t hit_count = obstacle_stats.front_hits + obstacle_stats.side_hits;

    if (!scan_fresh) {
      // Fail safe when the fixed-distance detector is enabled but its sensor
      // data has gone stale. A fresh clear scan will release the latch.
      {
        std::lock_guard<std::mutex> collision_lock(collision_state_mutex_);
        fixed_collision_latched_ = true;
        fixed_collision_confirm_count_ = 0;
        fixed_collision_clear_count_ = 0;
      }
      ROS_ERROR_THROTTLE(1.0, "[RPP] Fixed-distance collision check has no fresh laser scan; stopping.");
      fixed_collision = true;
    } else {
      const bool robot_commanded_to_move = std::abs(linear_vel) > 0.005 ||
        std::abs(angular_vel) >=
          std::max(0.005, params_->collision_side_turning_min_angular_vel);
      const bool candidate = robot_commanded_to_move &&
        hit_count >= static_cast<std::size_t>(std::max(1, params_->collision_min_valid_points));

      bool process_scan = false;
      {
        std::lock_guard<std::mutex> collision_lock(collision_state_mutex_);
        process_scan = scan_sequence != processed_scan_sequence_;
        if (process_scan && candidate) {
          processed_scan_sequence_ = scan_sequence;
          ++fixed_collision_confirm_count_;
          fixed_collision_clear_count_ = 0;
          if (fixed_collision_confirm_count_ >=
              std::max(1, params_->collision_confirm_scans)) {
            fixed_collision_latched_ = true;
          }
        }
      }

      if (process_scan && !candidate) {
        clearCollisionLatchIfSafe(obstacle_stats, scan_sequence);
      }

      {
        std::lock_guard<std::mutex> collision_lock(collision_state_mutex_);
        fixed_collision = fixed_collision_latched_;
      }
    }

    if (fixed_collision) {
      if (obstacle_stats.footprint_mode) {
        ROS_ERROR_THROTTLE(
          1.0,
          "[RPP] Expanded-footprint collision stop: front_clearance=%.3fm "
          "(limit %.3fm, hits=%zu), side_clearance=%.3fm "
          "(limit %.3fm, hits=%zu, active=%s).",
          obstacle_stats.nearest_front_clearance,
          params_->collision_front_clearance_m, obstacle_stats.front_hits,
          obstacle_stats.nearest_side_clearance,
          params_->collision_side_clearance_m, obstacle_stats.side_hits,
          obstacle_stats.side_check_active ? "true" : "false");
      } else {
        ROS_ERROR_THROTTLE(
          1.0,
          "[RPP] Legacy sector collision stop: obstacle at %.3fm "
          "(threshold %.3fm, hits=%zu).",
          obstacle_stats.nearest_front_clearance,
          params_->collision_stop_distance_m, hit_count);
      }
      std_msgs::Bool collision_state;
      collision_state.data = true;
      collision_imminent_pub_.publish(collision_state);
      message = "Fixed-distance collision threshold reached";
      cmd_vel.twist = geometry_msgs::Twist();
      last_cmd_linear_vel_ = 0.0;
      last_cmd_angular_vel_ = 0.0;
      last_cmd_time_ = ros::WallTime();
      return mbf_msgs::ExePathResult::COLLISION;
    }
  } else {
    std::lock_guard<std::mutex> collision_lock(collision_state_mutex_);
    fixed_collision_latched_ = false;
    fixed_collision_confirm_count_ = 0;
    fixed_collision_clear_count_ = 0;
  }

  bool predicted_collision = false;
  if (params_->use_collision_detection &&
    collision_checker_->isCollisionImminent(pose, linear_vel, angular_vel, carrot_dist))
  {
    ++prediction_collision_confirm_count_;
  } else {
    prediction_collision_confirm_count_ = 0;
  }
  predicted_collision = params_->use_collision_detection &&
    prediction_collision_confirm_count_ >=
    std::max(1, params_->prediction_collision_confirm_cycles);

  if (predicted_collision)
  {
    ROS_ERROR("[RPP] RegulatedPurePursuitController detected collision ahead!");
    std_msgs::Bool collision_state;
    collision_state.data = true;
    collision_imminent_pub_.publish(collision_state);
    message = "Collision detected ahead";
    cmd_vel.twist = geometry_msgs::Twist();
    last_cmd_linear_vel_ = 0.0;
    last_cmd_angular_vel_ = 0.0;
    last_cmd_time_ = ros::WallTime();
    return mbf_msgs::ExePathResult::COLLISION;
  }

  linear_vel = std::clamp(linear_vel, -params_->max_linear_vel, params_->max_linear_vel);
  angular_vel = std::clamp(angular_vel, -params_->max_angular_vel, params_->max_angular_vel);

  // populate and return message
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;
  std_msgs::Bool collision_state;
  collision_state.data = false;
  collision_imminent_pub_.publish(collision_state);
  last_cmd_linear_vel_ = linear_vel;
  last_cmd_angular_vel_ = angular_vel;
  last_cmd_time_ = ros::WallTime::now();
  return mbf_msgs::ExePathResult::SUCCESS;
}

void RegulatedPurePursuitController::scanCallback(
  const sensor_msgs::LaserScan::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(scan_mutex_);
  latest_scan_ = msg;
  latest_scan_receive_time_ = ros::Time::now();
  ++latest_scan_sequence_;
}

void RegulatedPurePursuitController::collisionClearTimerCallback(
  const ros::TimerEvent & /*event*/)
{
  if (!initialized_ || !params_->use_fixed_distance_collision_detection) {
    return;
  }

  {
    std::lock_guard<std::mutex> collision_lock(collision_state_mutex_);
    if (!fixed_collision_latched_) {
      return;
    }
  }

  FixedObstacleStats obstacle_stats;
  std::uint64_t scan_sequence = 0;
  // While latched, the footprint detector intentionally checks the side band
  // too, so angular velocity is irrelevant for this clear check.
  if (!getFixedObstacleStats(0.0, true, obstacle_stats, scan_sequence)) {
    return;
  }

  const bool publish_clear = clearCollisionLatchIfSafe(obstacle_stats, scan_sequence);

  if (publish_clear) {
    std_msgs::Bool collision_state;
    collision_state.data = false;
    collision_imminent_pub_.publish(collision_state);
    ROS_INFO("[RPP] Fixed-distance collision latch cleared after fresh clear scans.");
  }
}

bool RegulatedPurePursuitController::clearCollisionLatchIfSafe(
  const FixedObstacleStats & stats, std::uint64_t scan_sequence)
{
  const std::size_t hit_count = stats.front_hits + stats.side_hits;
  std::lock_guard<std::mutex> collision_lock(collision_state_mutex_);
  if (!fixed_collision_latched_ || scan_sequence == processed_scan_sequence_) {
    return false;
  }

  // Mark this scan as processed so the normal controller loop and the timer
  // cannot count the same clear scan twice.
  processed_scan_sequence_ = scan_sequence;
  fixed_collision_confirm_count_ = 0;

  if (hit_count >= static_cast<std::size_t>(std::max(1, params_->collision_min_valid_points))) {
    fixed_collision_clear_count_ = 0;
    return false;
  }

  ++fixed_collision_clear_count_;
  if (fixed_collision_clear_count_ <
    std::max(1, params_->collision_clear_confirm_scans))
  {
    return false;
  }

  fixed_collision_latched_ = false;
  fixed_collision_clear_count_ = 0;
  return true;
}

bool RegulatedPurePursuitController::getFixedObstacleStats(
  double angular_vel, bool collision_latched,
  FixedObstacleStats & stats, std::uint64_t & scan_sequence) const
{
  sensor_msgs::LaserScan::ConstPtr scan;
  ros::Time received;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan = latest_scan_;
    received = latest_scan_receive_time_;
    scan_sequence = latest_scan_sequence_;
  }

  if (!scan || received.isZero() ||
      (ros::Time::now() - received).toSec() > params_->collision_scan_timeout_s)
  {
    return false;
  }

  stats = FixedObstacleStats();
  stats.footprint_mode = params_->use_footprint_expansion_collision_detection;
  if (!stats.footprint_mode) {
    const double half_angle = std::max(0.0, params_->collision_scan_half_angle_rad);
    for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
      const double angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;
      const double range = scan->ranges[i];
      if (std::abs(angle) > half_angle || !std::isfinite(range) ||
          range < scan->range_min || range > scan->range_max)
      {
        continue;
      }
      stats.nearest_front_clearance = std::min(stats.nearest_front_clearance, range);
      if (range <= params_->collision_stop_distance_m) {
        ++stats.front_hits;
      }
    }
    return true;
  }

  const auto footprint = costmap_ros_->getRobotFootprint();
  if (footprint.empty()) {
    ROS_ERROR_THROTTLE(1.0, "[RPP] Footprint collision mode has no costmap footprint.");
    return false;
  }

  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const auto & point : footprint) {
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }

  const double front_clearance = std::max(0.0, params_->collision_front_clearance_m);
  const double side_clearance = std::max(0.0, params_->collision_side_clearance_m);
  const double outer_min_x = min_x - side_clearance;
  const double outer_max_x = max_x + front_clearance;
  const double outer_min_y = min_y - side_clearance;
  const double outer_max_y = max_y + side_clearance;
  stats.side_check_active = !params_->collision_side_only_when_turning ||
    collision_latched ||
    std::abs(angular_vel) >=
      std::max(0.0, params_->collision_side_turning_min_angular_vel);

  tf2::Transform scan_to_base;
  scan_to_base.setIdentity();
  const std::string base_frame = costmap_ros_->getBaseFrameID();
  if (!scan->header.frame_id.empty() && scan->header.frame_id != base_frame) {
    try {
      const auto transform = tf_->lookupTransform(
        base_frame, scan->header.frame_id, scan->header.stamp,
        ros::Duration(params_->transform_tolerance));
      tf2::fromMsg(transform.transform, scan_to_base);
    } catch (const tf2::TransformException & ex) {
      ROS_ERROR_THROTTLE(
        1.0, "[RPP] Cannot transform collision scan from %s to %s: %s",
        scan->header.frame_id.c_str(), base_frame.c_str(), ex.what());
      return false;
    }
  }

  for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
    const double angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;
    const double range = scan->ranges[i];
    if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max)
    {
      continue;
    }

    const tf2::Vector3 scan_point(range * std::cos(angle), range * std::sin(angle), 0.0);
    const tf2::Vector3 base_point = scan_to_base * scan_point;
    const double x = base_point.x();
    const double y = base_point.y();
    if (x < outer_min_x || x > outer_max_x || y < outer_min_y || y > outer_max_y) {
      continue;
    }

    const double dx = x < min_x ? min_x - x : (x > max_x ? x - max_x : 0.0);
    const double dy = y < min_y ? min_y - y : (y > max_y ? y - max_y : 0.0);
    const double clearance = std::hypot(dx, dy);
    if (x > max_x) {
      stats.nearest_front_clearance = std::min(
        stats.nearest_front_clearance, clearance);
      ++stats.front_hits;
    } else if (stats.side_check_active) {
      stats.nearest_side_clearance = std::min(
        stats.nearest_side_clearance, clearance);
      ++stats.side_hits;
    }
  }
  return true;
}

bool RegulatedPurePursuitController::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
{
    std::string dummy_message;
    geometry_msgs::PoseStamped dummy_pose;
    geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;
    costmap_ros_->getRobotPose(dummy_pose);
    uint32_t outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);
    cmd_vel = cmd_vel_stamped.twist;
    return outcome == mbf_msgs::ExePathResult::SUCCESS;
}

bool RegulatedPurePursuitController::shouldRotateToPath(
  const geometry_msgs::PoseStamped & carrot_pose, double & angle_to_path,
  double & x_vel_sign)
{
  // getLookAheadPoint() stores the direction of the path segment containing
  // the carrot in its orientation. Using atan2(carrot.y, carrot.x) here made
  // a sharp corner look like a sequence of gradually changing headings, so
  // the robot repeatedly rotated a few degrees and crept forward. Align to
  // the actual outgoing segment direction in one latched rotation instead.
  angle_to_path = angles::normalize_angle(
    tf2::getYaw(carrot_pose.pose.orientation));
  // In case we are reversing
  if (x_vel_sign < 0.0) {
    angle_to_path = angles::normalize_angle(angle_to_path + M_PI);
  }
  return params_->use_rotate_to_heading &&
         fabs(angle_to_path) > params_->rotate_to_heading_min_angle;
}

bool RegulatedPurePursuitController::shouldRotateToGoalHeading(
  const geometry_msgs::PoseStamped & carrot_pose)
{
  // Whether we should rotate robot to goal heading
  double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  return params_->use_rotate_to_heading && dist_to_goal < goal_dist_tol_;
}

void RegulatedPurePursuitController::rotateToHeading(
  double & linear_vel, double & angular_vel,
  const double & angle_to_path, const geometry_msgs::Twist & curr_speed)
{
  // Rotate in place. Slow down early enough to stop inside the exit angle,
  // then keep commanding zero until both command and odometry are stationary.
  linear_vel = 0.0;
  const double angular_accel = std::max(0.0, params_->max_angular_accel);
  const double remaining_angle = std::max(
    0.0, std::abs(angle_to_path) - rotate_to_heading_exit_angle_latched_);
  double target_speed = 0.0;
  if (remaining_angle > 0.0) {
    const double braking_limited_speed = angular_accel > 0.0 ?
      std::sqrt(2.0 * angular_accel * remaining_angle) :
      params_->rotate_to_heading_angular_vel;
    target_speed = std::min(
      params_->rotate_to_heading_angular_vel, braking_limited_speed);
    target_speed = std::copysign(target_speed, angle_to_path);
  }

  const double reference_speed =
    params_->use_command_angular_velocity_for_accel_limit ?
    last_cmd_angular_vel_ : curr_speed.angular.z;
  const double max_step = angular_accel * control_duration_;
  angular_vel = std::clamp(
    target_speed, reference_speed - max_step, reference_speed + max_step);
}

void RegulatedPurePursuitController::startRotateToHeading(
  const geometry_msgs::PoseStamped & robot_pose,
  double relative_heading_error, double exit_angle, bool goal_heading)
{
  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  rotate_to_heading_target_yaw_ = angles::normalize_angle(
    robot_yaw + relative_heading_error);
  rotate_to_heading_exit_angle_latched_ = std::max(0.0, exit_angle);
  rotate_to_heading_settling_ = false;
  rotate_to_heading_last_error_ = relative_heading_error;
  rotate_to_heading_stable_count_ = 0;
  is_rotating_to_heading_ = true;
  ROS_INFO(
    "[RPP] Latched rotate-to-%s: target=%.3f rad, initial_error=%.3f rad, "
    "exit_angle=%.3f rad",
    goal_heading ? "goal-heading" : "path-heading",
    rotate_to_heading_target_yaw_, relative_heading_error,
    rotate_to_heading_exit_angle_latched_);
}

void RegulatedPurePursuitController::resetRotateToHeadingState()
{
  is_rotating_to_heading_ = false;
  rotate_to_heading_settling_ = false;
  rotate_to_heading_last_error_ = 0.0;
  rotate_to_heading_stable_count_ = 0;
}

geometry_msgs::Point RegulatedPurePursuitController::circleSegmentIntersection(
  const geometry_msgs::Point & p1,
  const geometry_msgs::Point & p2,
  double r)
{
  // Formula for intersection of a line with a circle centered at the origin,
  // modified to always return the point that is on the segment between the two points.
  // https://mathworld.wolfram.com/Circle-LineIntersection.html
  // This works because the poses are transformed into the robot frame.
  // This can be derived from solving the system of equations of a line and a circle
  // which results in something that is just a reformulation of the quadratic formula.
  // Interactive illustration in doc/circle-segment-intersection.ipynb as well as at
  // https://www.desmos.com/calculator/td5cwbuocd
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double D = x1 * y2 - x2 * y1;

  // Augmentation to only return point within segment
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  geometry_msgs::Point p;
  double sqrt_term = std::sqrt(r * r * dr2 - D * D);
  p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  return p;
}

geometry_msgs::PoseStamped RegulatedPurePursuitController::getLookAheadPoint(
  const double & lookahead_dist,
  const nav_msgs::Path & transformed_plan,
  bool interpolate_after_goal)
{
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });
  
  geometry_msgs::PoseStamped pose;

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    if (interpolate_after_goal) {
      auto last_pose_it = std::prev(transformed_plan.poses.end());
      auto prev_last_pose_it = std::prev(last_pose_it);

      double end_path_orientation = atan2(
        last_pose_it->pose.position.y - prev_last_pose_it->pose.position.y,
        last_pose_it->pose.position.x - prev_last_pose_it->pose.position.x);

      // Project the last segment out to guarantee it is beyond the look ahead
      // distance
      auto projected_position = last_pose_it->pose.position;
      projected_position.x += cos(end_path_orientation) * lookahead_dist;
      projected_position.y += sin(end_path_orientation) * lookahead_dist;

      // Use the circle intersection to find the position at the correct look
      // ahead distance
      const auto interpolated_position = circleSegmentIntersection(
        last_pose_it->pose.position, projected_position, lookahead_dist);

      pose.header = last_pose_it->header;
      pose.pose.position = interpolated_position;
      pose.pose.orientation = getOrientation(
        last_pose_it->pose.position, interpolated_position);
    } else {
      goal_pose_it = std::prev(transformed_plan.poses.end());
      pose = *(goal_pose_it);
      pose.pose.orientation = getOrientation(
        std::prev(goal_pose_it)->pose.position, goal_pose_it->pose.position);
    }
  } else if (goal_pose_it == transformed_plan.poses.begin()) {
    pose = *(goal_pose_it); 
    pose.pose.orientation = getOrientation(
      goal_pose_it->pose.position, std::next(goal_pose_it)->pose.position);
  } else{
    // Find the point on the line segment between the two poses
    // that is exactly the lookahead distance away from the robot pose (the origin)
    // This can be found with a closed form for the intersection of a segment and a circle
    // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
    // and goal_pose is guaranteed to be outside the circle.
    auto prev_pose_it = std::prev(goal_pose_it);
    auto point = circleSegmentIntersection(
      prev_pose_it->pose.position,
      goal_pose_it->pose.position, lookahead_dist);
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;
    pose.pose.orientation = getOrientation(prev_pose_it->pose.position, point);
  }

  return pose;
}

void RegulatedPurePursuitController::applyConstraints(
  const double & curvature,
  const geometry_msgs::Twist & curr_speed,
  const double & pose_cost, const nav_msgs::Path & path, double & linear_vel, double & sign)
{
  double curvature_vel = linear_vel, cost_vel = linear_vel, smooth_vel = linear_vel;

  // limit the linear velocity by curvature
  if (params_->use_regulated_linear_velocity_scaling) {
    curvature_vel = heuristics::curvatureConstraint(
      linear_vel, curvature, params_->regulated_linear_scaling_min_radius);
  }

  // limit the linear velocity by proximity to obstacles
  if (params_->use_cost_regulated_linear_velocity_scaling) {
    cost_vel = heuristics::costConstraint(linear_vel, pose_cost, costmap_ros_, params_);
  }

  // Use the previous RPP command by default. Set the parameter false to
  // restore the original ODOM-based acceleration reference.
  const double reference_linear_vel = params_->use_command_velocity_for_accel_limit ?
    std::abs(last_cmd_linear_vel_) : std::abs(curr_speed.linear.x);
  smooth_vel = reference_linear_vel + params_->max_linear_accel * control_duration_;

  // Use the lowest of the 2 constraints, but above the minimum translational speed
  linear_vel = std::min({cost_vel, curvature_vel, smooth_vel});
  linear_vel = std::max(linear_vel, params_->regulated_linear_scaling_min_speed);

  // A scheduler may feed many intermediate active segments. In that mode each
  // plan endpoint is not a real task goal, so endpoint slowdown would create a
  // visible speed dip before every segment switch. Keep the standard behavior
  // available for ordinary single-goal navigation.
  if (params_->use_approach_velocity_scaling) {
    linear_vel = heuristics::approachVelocityConstraint(
      linear_vel, path, params_->min_approach_linear_velocity,
      params_->approach_velocity_scaling_dist);
  }

  // Limit linear velocities to be valid
  linear_vel = std::clamp(fabs(linear_vel), 0.0, params_->desired_linear_vel);
  linear_vel = sign * linear_vel;
}

void RegulatedPurePursuitController::createPathMsg(const std::vector<geometry_msgs::PoseStamped>& plan, nav_msgs::Path& path)
{
    path.header = plan[0].header;
    for (int i = 0; i < plan.size(); i++){
        path.poses.push_back(plan[i]);
    }
}

bool RegulatedPurePursuitController::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if(!initialized_)
  {
      ROS_ERROR("[RPP] RegulatedPurePursuitController has not been initialized, please call initialize() before using this planner");
      return false;
  }

  if (plan.size() < 2)
  {
      ROS_ERROR("[RPP] A valid plan must contain at least two poses.");
      last_cmd_linear_vel_ = 0.0;
      last_cmd_angular_vel_ = 0.0;
      last_cmd_time_ = ros::WallTime();
      resetRotateToHeadingState();
      return false;
  }

  std::lock_guard<std::mutex> lock_reinit(param_handler_->getMutex());

  global_plan_.clear();
  goal_reached_ = false;
  prediction_collision_confirm_count_ = 0;
  const ros::WallTime now = ros::WallTime::now();
  const double command_age = last_cmd_time_.isZero() ?
    std::numeric_limits<double>::infinity() : (now - last_cmd_time_).toSec();
  last_cmd_linear_vel_ = retainCommandVelocityIfFresh(
    last_cmd_linear_vel_, command_age, params_->command_velocity_memory_timeout);
  last_cmd_angular_vel_ = retainCommandVelocityIfFresh(
    last_cmd_angular_vel_, command_age, params_->command_velocity_memory_timeout);
  if (last_cmd_linear_vel_ == 0.0 && last_cmd_angular_vel_ == 0.0) {
    last_cmd_time_ = ros::WallTime();
  }
  global_plan_ = plan;
  nav_msgs::Path path;
  createPathMsg(plan, path);
  global_path_origin_pub_.publish(path);
  const std::size_t preserved_pruned_count = path_handler_->setPlan(
    path, params_->preserve_plan_progress_on_update,
    params_->plan_update_equivalence_tolerance_m);
  if (!path_handler_->lastPlanUpdateWasEquivalent()) {
    // A genuinely new segment may require a different heading. Equivalent
    // planner refreshes intentionally keep the already-latched target.
    resetRotateToHeadingState();
  }
  if (preserved_pruned_count > 0) {
    ROS_INFO_THROTTLE(
      2.0,
      "[RPP] Equivalent plan refresh preserved progress and kept %zu passed poses pruned",
      preserved_pruned_count);
  }
  return true;
}

double RegulatedPurePursuitController::findVelocitySignChange(
  const nav_msgs::Path & transformed_plan)
{
  // Iterating through the transformed global path to determine the position of the cusp
  for (unsigned int pose_id = 1; pose_id < transformed_plan.poses.size() - 1; ++pose_id) {
    // We have two vectors for the dot product OA and AB. Determining the vectors.
    double oa_x = transformed_plan.poses[pose_id].pose.position.x -
      transformed_plan.poses[pose_id - 1].pose.position.x;
    double oa_y = transformed_plan.poses[pose_id].pose.position.y -
      transformed_plan.poses[pose_id - 1].pose.position.y;
    double ab_x = transformed_plan.poses[pose_id + 1].pose.position.x -
      transformed_plan.poses[pose_id].pose.position.x;
    double ab_y = transformed_plan.poses[pose_id + 1].pose.position.y -
      transformed_plan.poses[pose_id].pose.position.y;

    /* Checking for the existence of cusp, in the path, using the dot product
    and determine it's distance from the robot. If there is no cusp in the path,
    then just determine the distance to the goal location. */
    const double dot_prod = (oa_x * ab_x) + (oa_y * ab_y);
    if (dot_prod < 0.0) {
      // returning the distance if there is a cusp
      // The transformed path is in the robots frame, so robot is at the origin
      return hypot(
        transformed_plan.poses[pose_id].pose.position.x,
        transformed_plan.poses[pose_id].pose.position.y);
    }

    if (
      (hypot(oa_x, oa_y) == 0.0 &&
      transformed_plan.poses[pose_id - 1].pose.orientation !=
      transformed_plan.poses[pose_id].pose.orientation)
      ||
      (hypot(ab_x, ab_y) == 0.0 &&
      transformed_plan.poses[pose_id].pose.orientation !=
      transformed_plan.poses[pose_id + 1].pose.orientation))
    {
      // returning the distance since the points overlap
      // but are not simply duplicate points (e.g. in place rotation)
      return hypot(
        transformed_plan.poses[pose_id].pose.position.x,
        transformed_plan.poses[pose_id].pose.position.y);
    }
  }

  return std::numeric_limits<double>::max();
}
}  // namespace regulated_pure_pursuit_controller
