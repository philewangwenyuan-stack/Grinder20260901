#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace LI2Sup {
namespace base_kinematics {

// T_A_B maps coordinates in B into A. This header has no ROS/PCL dependency.
using Transform = Eigen::Isometry3d;
using Covariance = Eigen::Matrix<double, 18, 18>;
using Jacobian = Eigen::Matrix<double, 6, 18>;
using Covariance6 = Eigen::Matrix<double, 6, 6>;
using StateGyroCross = Eigen::Matrix<double, 18, 3>;

inline Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
  Eigen::Matrix3d result;
  result << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return result;
}

inline Transform fromXyzRpy(const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy) {
  Transform result = Transform::Identity();
  result.linear() = (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                     Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
  result.translation() = xyz;
  return result;
}

inline Transform imuToBase(const Transform& world_imu, const Transform& imu_base) {
  return world_imu * imu_base;
}

struct Output {
  Transform pose;
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
  Covariance6 pose_covariance;
  Covariance6 twist_covariance;
};

inline Output convert(const Transform& world_imu, const Transform& imu_base,
                      const Eigen::Vector3d& world_velocity,
                      const Eigen::Vector3d& imu_omega,
                      const Covariance& covariance,
                      const Eigen::Matrix3d& gyro_covariance,
                      const StateGyroCross& state_gyro_cross = StateGyroCross::Zero()) {
  Output result;
  result.pose = imuToBase(world_imu, imu_base);
  const Eigen::Matrix3d R = world_imu.linear();
  const Eigen::Matrix3d A = imu_base.linear().transpose();
  const Eigen::Vector3d r = imu_base.translation();
  const Eigen::Vector3d imu_velocity = R.transpose() * world_velocity;
  result.linear = A * (imu_velocity + imu_omega.cross(r));
  result.angular = A * imu_omega;

  // ESKF errors: right-local attitude, world p/v, IMU bg/ba, world gravity.
  // ROS pose errors: world position followed by world small-angle attitude.
  Jacobian J_pose = Jacobian::Zero();
  J_pose.block<3, 3>(0, 0) = -R * hat(r);
  J_pose.block<3, 3>(0, 3).setIdentity();
  J_pose.block<3, 3>(3, 0) = R;
  result.pose_covariance = J_pose * covariance * J_pose.transpose();

  Jacobian J_twist = Jacobian::Zero();
  J_twist.block<3, 3>(0, 0) = A * hat(imu_velocity);
  J_twist.block<3, 3>(0, 6) = A * R.transpose();
  J_twist.block<3, 3>(0, 9) = A * hat(r);
  J_twist.block<3, 3>(3, 9) = -A;
  Eigen::Matrix<double, 6, 3> G;
  G.topRows<3>() = -A * hat(r);
  G.bottomRows<3>() = A;
  result.twist_covariance = J_twist * covariance * J_twist.transpose() +
                            G * gyro_covariance * G.transpose() -
                            J_twist * state_gyro_cross * G.transpose() -
                            G * state_gyro_cross.transpose() * J_twist.transpose();
  return result;
}

// Preserve the IMU gravity estimate, but align the BASE x-axis with the
// requested map yaw. Input z is the base height in the existing map frame.
inline Transform initialImuPose(const Transform& requested_base,
                                const Eigen::Matrix3d& gravity_aligned_imu,
                                const Transform& imu_base) {
  const Eigen::Matrix3d aligned_base = gravity_aligned_imu * imu_base.linear();
  const double base_yaw = std::atan2(aligned_base(1, 0), aligned_base(0, 0));
  const double requested_yaw = std::atan2(requested_base.linear()(1, 0),
                                         requested_base.linear()(0, 0));
  Transform world_base = Transform::Identity();
  world_base.linear() = Eigen::AngleAxisd(requested_yaw - base_yaw,
      Eigen::Vector3d::UnitZ()).toRotationMatrix() * aligned_base;
  world_base.translation() = requested_base.translation();
  return world_base * imu_base.inverse();
}

}  // namespace base_kinematics
}  // namespace LI2Sup
