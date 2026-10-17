#include "lio/wheel_fusion.h"

namespace LI2Sup { namespace wheel_fusion {
Result update(M18& P, GyroContext& gyro, const Model& m,
                     const Eigen::Matrix<double,3,6>& W, const Sample& sample,
                     bool linear, bool angular, V18& dx,
                     double single_gate, double joint_gate) {
  Result result;
  result.residual = sample.velocity - m.prediction;
  M24 prior = M24::Zero();
  prior.topLeftCorner<18,18>() = P;
  prior.topRightCorner<18,6>() = gyro.cross;
  prior.bottomLeftCorner<6,18>() = gyro.cross.transpose();
  prior.bottomRightCorner<6,6>() = gyro.covariance;
  Eigen::Matrix<double,2,24> H;
  H.leftCols<18>() = m.H;
  H.rightCols<6>() = -m.G * W;
  const Eigen::Matrix2d S = H * prior * H.transpose() + sample.covariance;
  result.innovation_variance=S.diagonal();
  result.covariance_cross=sample.covariance(0,1);
  result.innovation_covariance_cross=S(0,1);
  std::array<int,2> rows{{0,0}}; int n=0;
  for (int i=0; i<2; ++i) {
    result.variance[i] = sample.covariance(i,i);
    const bool enabled = i==0 ? linear : angular;
    if (!enabled || !sample.valid[i] || !std::isfinite(sample.velocity[i]) ||
        !std::isfinite(S(i,i)) || S(i,i)<=0.0 ||
        !std::isfinite(sample.covariance(i,i)) || sample.covariance(i,i)<=0.0) continue;
    result.nis[i] = result.residual[i]*result.residual[i]/S(i,i);
    if (result.nis[i] <= single_gate) rows[n++]=i;
  }
  if (!n) return result;
  Eigen::MatrixXd h(n,24), r(n,n); Eigen::VectorXd residual(n);
  for (int i=0; i<n; ++i) {
    h.row(i)=H.row(rows[i]); residual[i]=result.residual[rows[i]];
    for (int j=0;j<n;++j) r(i,j)=sample.covariance(rows[i],rows[j]);
  }
  if (!positive(r)) return result;
  const Eigen::MatrixXd innovation=h*prior*h.transpose()+r;
  if (!positive(innovation)) return result;
  const auto solver=innovation.ldlt();
  result.joint_nis=residual.dot(solver.solve(residual));
  if (!std::isfinite(result.joint_nis) || result.joint_nis>(n==2 ? joint_gate : single_gate)) return result;
  const Eigen::MatrixXd K=solver.solve(h*prior).transpose();
  const Eigen::Matrix<double,24,1> correction=K*residual;
  const M24 A=M24::Identity()-K*h;
  M24 posterior=A*prior*A.transpose()+K*r*K.transpose();
  posterior=(posterior+posterior.transpose()).eval()*0.5;
  if (!posterior.allFinite() || !correction.allFinite()) return result;
  Eigen::SelfAdjointEigenSolver<M24> eigen(posterior, Eigen::EigenvaluesOnly);
  if (eigen.info()!=Eigen::Success || eigen.eigenvalues().minCoeff() < -1e-9) return result;
  const V18 state_correction=correction.head<18>();
  M18 reset=M18::Identity();
  reset.topLeftCorner<3,3>()-=0.5*base_kinematics::hat(state_correction.head<3>());
  const M18 state_covariance=reset*posterior.topLeftCorner<18,18>()*reset.transpose();
  auto conditioned=gyro;
  conditioned.mean+=correction.tail<6>();
  conditioned.cross=reset*posterior.topRightCorner<18,6>();
  conditioned.covariance=posterior.bottomRightCorner<6,6>();
  if (!state_covariance.allFinite() || !conditioned.mean.allFinite() ||
      !conditioned.cross.allFinite() || !conditioned.covariance.allFinite()) return result;
  dx=state_correction;P=state_covariance;gyro=conditioned;
  for(int i=0;i<n;++i) result.used[rows[i]]=true;
  result.applied=true;
  return result;
}
}}
