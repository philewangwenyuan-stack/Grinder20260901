// Copyright (c) 2019 Intel Corporation
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

#ifndef GEOMETRY_UTILS_HPP_
#define GEOMETRY_UTILS_HPP_

#include <cmath>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>

namespace regulated_pure_pursuit_controller
{

namespace geometry_utils
{

/**
 * @brief Get a geometry_msgs Quaternion from a yaw angle
 * @param angle Yaw angle to generate a quaternion from
 * @return geometry_msgs Quaternion
 */
inline geometry_msgs::Quaternion orientationAroundZAxis(double angle)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, angle);  // void returning function
  return tf2::toMsg(q);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::Points
 * @param pos1 First point
 * @param pos1 Second point
 * @return double L2 distance
 */
inline double euclidean_distance(
  const geometry_msgs::Point & pos1,
  const geometry_msgs::Point & pos2)
{
  double dx = pos1.x - pos2.x;
  double dy = pos1.y - pos2.y;
  return std::hypot(dx, dy);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::Poses
 * @param pos1 First pose
 * @param pos1 Second pose
 * @return double L2 distance
 */
inline double euclidean_distance(
  const geometry_msgs::Pose & pos1,
  const geometry_msgs::Pose & pos2)
{
  double dx = pos1.position.x - pos2.position.x;
  double dy = pos1.position.y - pos2.position.y;
  return std::hypot(dx, dy);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::PoseStamped
 * @param pos1 First pose
 * @param pos1 Second pose
 * @return double L2 distance
 */
inline double euclidean_distance(
  const geometry_msgs::PoseStamped & pos1,
  const geometry_msgs::PoseStamped & pos2)
{
  return euclidean_distance(pos1.pose, pos2.pose);
}

/**
 * @brief Get the L2 distance between 2 geometry_msgs::Pose2D
 * @param pos1 First pose
 * @param pos1 Second pose
 * @return double L2 distance
 */
inline double euclidean_distance(
  const geometry_msgs::Pose2D & pos1,
  const geometry_msgs::Pose2D & pos2)
{
  double dx = pos1.x - pos2.x;
  double dy = pos1.y - pos2.y;

  return std::hypot(dx, dy);
}

/**
 * Find element in iterator with the minimum calculated value
 */
template<typename Iter, typename Getter>
inline Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) {
    return end;
  }
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp < lowest) {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

/**
 * Find first element in iterator that is greater integrated distance than comparevalue
 */
template<typename Iter, typename Getter>
inline Iter first_after_integrated_distance(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) {
    return end;
  }
  Getter dist = 0.0;
  for (Iter it = begin; it != end - 1; it++) {
    dist += euclidean_distance(*it, *(it + 1));
    if (dist > getCompareVal) {
      return it + 1;
    }
  }
  return end;
}

/**
 * Find the closest path pose using the normal bounded arc-length window first.
 * If every pose in that window is outside the usable local area, optionally
 * search the remaining path. This is intended for refreshed plans whose stale
 * prefix has been reintroduced after the robot has already driven past it.
 */
template<typename Iter, typename Getter>
inline Iter find_closest_pose_with_search_fallback(
  Iter begin, Iter end, double max_search_dist, double max_acceptable_distance,
  bool use_fallback, Getter get_distance, bool & fallback_used)
{
  fallback_used = false;
  if (begin == end) {
    return end;
  }

  auto bounded_end = first_after_integrated_distance(begin, end, max_search_dist);
  auto bounded_closest = min_by(begin, bounded_end, get_distance);
  if (bounded_closest == end || !use_fallback ||
    get_distance(*bounded_closest) <= max_acceptable_distance)
  {
    return bounded_closest;
  }

  auto fallback_closest = min_by(bounded_end, end, get_distance);
  if (fallback_closest != end &&
    get_distance(*fallback_closest) <= max_acceptable_distance)
  {
    fallback_used = true;
    return fallback_closest;
  }

  return bounded_closest;
}

/**
 * @brief Calculate the length of the provided path, starting at the provided index
 * @param path Path containing the poses that are planned
 * @param start_index Optional argument specifying the starting index for
 * the calculation of path length. Provide this if you want to calculate length of a
 * subset of the path.
 * @return double Path length
 */
inline double calculate_path_length(const nav_msgs::Path & path, size_t start_index = 0)
{
  if (start_index + 1 >= path.poses.size()) {
    return 0.0;
  }
  double path_length = 0.0;
  for (size_t idx = start_index; idx < path.poses.size() - 1; ++idx) {
    path_length += euclidean_distance(path.poses[idx].pose, path.poses[idx + 1].pose);
  }
  return path_length;
}

}  // namespace geometry_utils

}  // namespace regulated_pure_pursuit_controller

#endif  // GEOMETRY_UTILS_HPP_
