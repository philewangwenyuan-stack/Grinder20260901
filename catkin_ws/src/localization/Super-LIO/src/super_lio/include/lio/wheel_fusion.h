#pragma once

// ROS-independent wheel observation math and bounded, single-consumption buffer.
#include "lio/base_kinematics.h"
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <deque>
#include <limits>

namespace LI2Sup { namespace wheel_fusion {
using M18 = Eigen::Matrix<double, 18, 18>;
using M24 = Eigen::Matrix<double, 24, 24>;
using V18 = Eigen::Matrix<double, 18, 1>;
using M18x6 = Eigen::Matrix<double, 18, 6>;

struct Sample {
  double time = 0.0;
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  std::array<bool, 2> valid{{false, false}};
};

struct Buffer {
  std::deque<Sample> samples;
  double last_received = -1.0;
  unsigned long accepted = 0, invalid = 0, late = 0, expired = 0, overflow = 0;
  void clear() { samples.clear(); last_received = -1.0; }
  bool push(const Sample& s) {
    if (!std::isfinite(s.time) || s.time <= 0.0) {
      ++invalid; return false;
    }
    const auto position = std::lower_bound(samples.begin(), samples.end(), s.time,
        [](const Sample& value, double time) { return value.time < time; });
    if (position != samples.end() && position->time == s.time) {
      ++invalid; return false;
    }
    last_received = std::max(last_received, s.time);
    samples.insert(position, s);
    while (samples.size() > 200 || samples.back().time - samples.front().time > 2.0) {
      samples.pop_front(); ++overflow;
    }
    return true;
  }
};

struct Model {
  Eigen::Vector2d prediction;
  Eigen::Matrix<double, 2, 18> H = Eigen::Matrix<double, 2, 18>::Zero();
  Eigen::Matrix<double, 2, 3> G = Eigen::Matrix<double, 2, 3>::Zero();
};

// T_I_B maps base coordinates into IMU coordinates. State error is right-local R.
inline Model model(const Eigen::Matrix3d& R, const Eigen::Vector3d& v,
                   const Eigen::Vector3d& omega, const base_kinematics::Transform& T_I_B) {
  Model m;
  const Eigen::Matrix3d A = T_I_B.linear().transpose();
  const Eigen::Vector3d r = T_I_B.translation();
  const Eigen::Vector3d iv = R.transpose() * v;
  m.prediction << (A * (iv + omega.cross(r))).x(), (A * omega).z();
  m.H.block<1,3>(0,0) = (A * base_kinematics::hat(iv)).row(0);
  m.H.block<1,3>(0,6) = (A * R.transpose()).row(0);
  m.H.block<1,3>(0,9) = (A * base_kinematics::hat(r)).row(0);
  m.H.block<1,3>(1,9) = -A.row(2);
  // d prediction / d raw gyro; actual gyro noise enters prediction negatively.
  m.G.row(0) = (-A * base_kinematics::hat(r)).row(0);
  m.G.row(1) = A.row(2);
  return m;
}

// Keep the two original gyro samples bracketing the current propagation time as
// nuisance variables. Splitting an IMU interval must NOT create independent noise.
struct GyroContext {
  std::array<double,2> times{{-1.0, -1.0}};
  Eigen::Matrix<double,6,1> mean = Eigen::Matrix<double,6,1>::Zero();
  Eigen::Matrix<double,6,6> covariance = Eigen::Matrix<double,6,6>::Zero();
  M18x6 cross = M18x6::Zero();
  void clear() { *this = GyroContext(); }
  void select(double left, double right, double variance) {
    const auto previous = *this;
    clear(); times = {{left, right}};
    for (int i=0; i<2; ++i) {
      covariance.block<3,3>(3*i,3*i).setIdentity();
      covariance.block<3,3>(3*i,3*i) *= variance;
      for (int j=0; j<2; ++j) if (times[i] == previous.times[j]) {
        mean.segment<3>(3*i) = previous.mean.segment<3>(3*j);
        cross.block<18,3>(0,3*i) = previous.cross.block<18,3>(0,3*j);
        for (int k=0; k<2; ++k) for (int l=0; l<2; ++l)
          if (times[k] == previous.times[l])
            covariance.block<3,3>(3*i,3*k) = previous.covariance.block<3,3>(3*j,3*l);
      }
    }
  }
};

struct Result {
  bool applied = false;
  std::array<bool,2> used{{false,false}};
  Eigen::Vector2d residual = Eigen::Vector2d::Zero();
  Eigen::Vector2d nis = Eigen::Vector2d::Constant(-1.0);
  Eigen::Vector2d variance = Eigen::Vector2d::Zero();
  Eigen::Vector2d innovation_variance = Eigen::Vector2d::Zero();
  double covariance_cross = 0.0, innovation_covariance_cross = 0.0;
  double joint_nis = -1.0;
};

inline bool positive(const Eigen::MatrixXd& matrix) {
  if (!matrix.allFinite()) return false;
  Eigen::LDLT<Eigen::MatrixXd> ldlt(matrix);
  return ldlt.info() == Eigen::Success && (ldlt.vectorD().array() > 0.0).all();
}

Result update(M18& P, GyroContext& gyro, const Model& m,
              const Eigen::Matrix<double,3,6>& W, const Sample& sample,
              bool linear, bool angular, V18& dx,
              double single_gate=9.0, double joint_gate=11.83);

}} // namespace LI2Sup::wheel_fusion
