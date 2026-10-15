#include "lio/base_kinematics.h"
#include <Eigen/Eigenvalues>
#include <cstdlib>
#include <iostream>

using namespace LI2Sup::base_kinematics;
using Eigen::Vector3d;

void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

void checkPoseAndInitialGuess() {
  const Transform identity = Transform::Identity();
  const Transform raw = fromXyzRpy(Vector3d(1, 2, 3), Vector3d(.2, -.3, .7));
  require(imuToBase(raw, identity).matrix().isApprox(raw.matrix()), "zero extrinsic");

  // Real internal sensor offset and nontrivial measured mounting rotation.
  const Transform imu_laser = fromXyzRpy(Vector3d(-.011, -.02329, .04412),
                                        Vector3d(.01, -.02, .03));
  const Transform base_laser = fromXyzRpy(Vector3d(.10, .025, .32),
                                         Vector3d(.04, -.07, .35));
  const Transform imu_base = imu_laser * base_laser.inverse();
  for (double pitch : {0.0, M_PI / 6}) {
    for (double yaw : {0.0, M_PI / 2, -2.1}) {
      const Transform base = fromXyzRpy(Vector3d(1, 2, -.17), Vector3d(0, pitch, yaw));
      const Transform imu = base * imu_base.inverse();
      const Transform recovered = imuToBase(imu, imu_base);
      require(recovered.matrix().isApprox(base.matrix(), 1e-10), "base recovery at tilt/yaw");
      require((recovered * base_laser).matrix().isApprox((imu * imu_laser).matrix(), 1e-10),
              "TF closure with internal sensor offset");

      // Gravity alignment retains tilt but has an arbitrary heading.
      const Eigen::Matrix3d aligned = Eigen::AngleAxisd(.8, Vector3d::UnitZ()).toRotationMatrix() * imu.linear();
      const Transform requested = fromXyzRpy(base.translation(), Vector3d(0, 0, yaw));
      const Transform initial_imu = initialImuPose(requested, aligned, imu_base);
      require(initial_imu.matrix().isApprox(imu.matrix(), 1e-10),
              "initial base XYZ/yaw -> IMU with gravity and mounting yaw");
      require((initial_imu.linear().transpose() * Vector3d::UnitZ()).isApprox(
                  aligned.transpose() * Vector3d::UnitZ(), 1e-10), "initial guess preserves gravity");

      const Vector3d point(2, -1, .4);
      const Vector3d world_point = imu * (imu_laser * point);
      require((recovered * (recovered.inverse() * world_point)).isApprox(world_point, 1e-10),
              "industrial keyframe reprojection");
      const Transform planar = fromXyzRpy(Vector3d(1, 2, 0), Vector3d(0, 0, yaw));
      require((planar * (planar.inverse() * world_point)).isApprox(world_point, 1e-10),
              "lightweight planar keyframe reprojection preserves tilt/height");
    }
  }
}

void checkRotationAboutBase() {
  const Transform base_imu = fromXyzRpy(Vector3d(.1, 0, .3), Vector3d(.1, .2, .4));
  const Transform imu_base = base_imu.inverse();
  for (double yaw : {0.0, .7, 2.0}) {
    const Transform base = fromXyzRpy(Vector3d(1, 2, 0), Vector3d(0, .3, yaw));
    const Transform imu = base * base_imu;
    const Vector3d world_omega(0, 0, .8);
    const Vector3d world_velocity = world_omega.cross(imu.translation() - base.translation());
    const auto output = convert(imu, imu_base, world_velocity,
        imu.linear().transpose() * world_omega, Covariance::Zero(), Eigen::Matrix3d::Zero());
    require(output.pose.translation().isApprox(base.translation(), 1e-10), "stationary rotation center");
    require(output.linear.norm() < 1e-10, "lever-arm velocity cancels IMU orbit");
    require(output.angular.isApprox(base.linear().transpose() * world_omega, 1e-10),
            "angular velocity expressed in base");
  }
}

void checkCovarianceFiniteDifferences() {
  const Transform imu = fromXyzRpy(Vector3d(2, 3, 1), Vector3d(.2, .5, -.9));
  const Transform imu_base = fromXyzRpy(Vector3d(-.1, .02, -.4), Vector3d(.1, -.2, .4));
  const Vector3d velocity(.3, -.5, .2), omega(.2, -.1, .6);
  Eigen::Matrix<double, 18, 18> seed;
  for (int i = 0; i < 18; ++i)
    for (int j = 0; j < 18; ++j) seed(i, j) = std::sin(i * 1.7 + j * .3) * .01;
  const Covariance covariance = seed * seed.transpose() + Covariance::Identity() * .001;
  const Eigen::Matrix3d gyro = Vector3d(.01, .02, .03).asDiagonal();
  const auto nominal = convert(imu, imu_base, velocity, omega, covariance, gyro);
  Jacobian pose_jacobian = Jacobian::Zero(), twist_jacobian = Jacobian::Zero();
  Eigen::Matrix<double, 6, 3> noise_jacobian;
  const double eps = 1e-7;
  for (int i = 0; i < 21; ++i) {
    Transform perturbed = imu;
    Vector3d v = velocity, w = omega;
    if (i < 3) perturbed.linear() *= Eigen::AngleAxisd(eps, Vector3d::Unit(i)).toRotationMatrix();
    else if (i < 6) perturbed.translation()[i - 3] += eps;
    else if (i < 9) v[i - 6] += eps;
    else if (i < 12) w[i - 9] -= eps;  // bias error at fixed gyro observation
    else if (i >= 18) w[i - 18] += eps;
    const auto changed = convert(perturbed, imu_base, v, w, Covariance::Zero(), Eigen::Matrix3d::Zero());
    Eigen::Matrix<double, 6, 1> pose_delta, twist_delta;
    pose_delta.head<3>() = (changed.pose.translation() - nominal.pose.translation()) / eps;
    const Eigen::AngleAxisd angle(changed.pose.linear() * nominal.pose.linear().transpose());
    pose_delta.tail<3>() = angle.axis() * angle.angle() / eps;
    twist_delta.head<3>() = (changed.linear - nominal.linear) / eps;
    twist_delta.tail<3>() = (changed.angular - nominal.angular) / eps;
    if (i < 18) {
      pose_jacobian.col(i) = pose_delta;
      twist_jacobian.col(i) = twist_delta;
    } else noise_jacobian.col(i - 18) = twist_delta;
  }
  require(nominal.pose_covariance.isApprox(pose_jacobian * covariance * pose_jacobian.transpose(), 1e-6),
          "pose covariance matches numerical SE3 Jacobian including cross terms");
  require(nominal.twist_covariance.isApprox(twist_jacobian * covariance * twist_jacobian.transpose() +
              noise_jacobian * gyro * noise_jacobian.transpose(), 1e-6), "twist covariance matches numerical Jacobian");
  require(nominal.pose_covariance.isApprox(nominal.pose_covariance.transpose(), 1e-12), "pose covariance symmetry");
  require(Eigen::SelfAdjointEigenSolver<Covariance6>(nominal.twist_covariance).eigenvalues().minCoeff() > -1e-12,
          "twist covariance positive semidefinite");
}

int main() {
  checkPoseAndInitialGuess();
  checkRotationAboutBase();
  checkCovarianceFiniteDifferences();
  std::cout << "base kinematics: all regression scenarios passed\n";
}
