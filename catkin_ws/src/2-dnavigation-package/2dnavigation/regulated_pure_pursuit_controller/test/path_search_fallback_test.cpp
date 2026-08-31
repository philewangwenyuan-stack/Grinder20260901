#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <geometry_msgs/PoseStamped.h>

#include "regulated_pure_pursuit_controller/geometry_utils.h"

namespace
{

std::vector<geometry_msgs::PoseStamped> makeStraightPath(int last_x)
{
  std::vector<geometry_msgs::PoseStamped> path;
  for (int x = 0; x <= last_x; ++x) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = static_cast<double>(x);
    pose.pose.orientation.w = 1.0;
    path.push_back(pose);
  }
  return path;
}

}  // namespace

TEST(PathSearchFallback, FindsRobotBeyondBoundedPrefix)
{
  auto path = makeStraightPath(10);
  const double robot_x = 6.0;
  bool fallback_used = false;
  const auto closest =
    regulated_pure_pursuit_controller::geometry_utils::find_closest_pose_with_search_fallback(
    path.begin(), path.end(), 2.5, 2.5, true,
    [robot_x](const geometry_msgs::PoseStamped & pose) {
      return std::abs(pose.pose.position.x - robot_x);
    },
    fallback_used);

  ASSERT_NE(closest, path.end());
  EXPECT_TRUE(fallback_used);
  EXPECT_DOUBLE_EQ(closest->pose.position.x, 6.0);
}

TEST(PathSearchFallback, KeepsOriginalBoundWhenFallbackDisabled)
{
  auto path = makeStraightPath(10);
  const double robot_x = 6.0;
  bool fallback_used = false;
  const auto closest =
    regulated_pure_pursuit_controller::geometry_utils::find_closest_pose_with_search_fallback(
    path.begin(), path.end(), 2.5, 2.5, false,
    [robot_x](const geometry_msgs::PoseStamped & pose) {
      return std::abs(pose.pose.position.x - robot_x);
    },
    fallback_used);

  ASSERT_NE(closest, path.end());
  EXPECT_FALSE(fallback_used);
  EXPECT_DOUBLE_EQ(closest->pose.position.x, 2.0);
}

TEST(PathSearchFallback, DoesNotFallbackWhenBoundedPoseIsUsable)
{
  auto path = makeStraightPath(10);
  const double robot_x = 1.0;
  bool fallback_used = false;
  const auto closest =
    regulated_pure_pursuit_controller::geometry_utils::find_closest_pose_with_search_fallback(
    path.begin(), path.end(), 2.5, 2.5, true,
    [robot_x](const geometry_msgs::PoseStamped & pose) {
      return std::abs(pose.pose.position.x - robot_x);
    },
    fallback_used);

  ASSERT_NE(closest, path.end());
  EXPECT_FALSE(fallback_used);
  EXPECT_DOUBLE_EQ(closest->pose.position.x, 1.0);
}

TEST(PathSearchFallback, RejectsFallbackOutsideLocalExtent)
{
  auto path = makeStraightPath(10);
  const double robot_x = 20.0;
  bool fallback_used = false;
  const auto closest =
    regulated_pure_pursuit_controller::geometry_utils::find_closest_pose_with_search_fallback(
    path.begin(), path.end(), 2.5, 2.5, true,
    [robot_x](const geometry_msgs::PoseStamped & pose) {
      return std::abs(pose.pose.position.x - robot_x);
    },
    fallback_used);

  ASSERT_NE(closest, path.end());
  EXPECT_FALSE(fallback_used);
  EXPECT_DOUBLE_EQ(closest->pose.position.x, 2.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
