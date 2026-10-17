#include "lio/ESKF.h"

namespace LI2Sup {
using namespace BASIC;

bool ESKF::PredictWheelInterval(const IMUData& left, const IMUData& right, double target) {
  const double start = current_time_;
  if (!wheel_enabled_ || !wheel_initialized_ || !std::isfinite(target) || right.secs<=left.secs ||
      target<=start || start<left.secs-1e-8 || target>right.secs+1e-8) return false;
  const double dt=target-start, span=right.secs-left.secs;
  const double a=std::clamp((start-left.secs)/span,0.0,1.0);
  const double b=std::clamp((target-left.secs)/span,0.0,1.0);
  const double weight=0.5*(a+b);
  auto gyro=wheel_gyro_;
  gyro.select(left.secs,right.secs,options_.gyro_var_);
  Eigen::Matrix<double,3,6> W=Eigen::Matrix<double,3,6>::Zero();
  W.leftCols<3>()=(1.0-weight)*Eigen::Matrix3d::Identity();
  W.rightCols<3>()=weight*Eigen::Matrix3d::Identity();
  const V3 raw=(1.0-weight)*left.gyr+weight*right.gyr;
  const V3 omega=raw-bg_-(W*gyro.mean).cast<scalar>();
  const V3 acc=imu_scale_*((1.0-weight)*left.acc+weight*right.acc)-ba_;
  const M3 R=R_.R_;
  const M3 rotation_step=SO3::Exp(omega,dt).R_;
  const V3 angle=omega*dt;
  const double norm=angle.norm();
  const M3 hat=SO3::hat(angle);
  const M3 right_jacobian=norm<1e-5 ?
      (M3::Identity()-0.5*hat+hat*hat/6.0).eval() :
      (M3::Identity()-(1.0-std::cos(norm))/(norm*norm)*hat+
       (norm-std::sin(norm))/(norm*norm*norm)*hat*hat).eval();
  F_X F=F_X::Identity();
  F.block<3,3>(0,0)=rotation_step.transpose();
  F.block<3,3>(0,9)=-right_jacobian*dt;
  F.block<3,3>(3,6)=M3::Identity()*dt;
  F.block<3,3>(6,0)=-R*SO3::hat(acc)*dt;
  F.block<3,3>(6,12)=-R*dt;
  F.block<3,3>(6,15)=M3::Identity()*dt;
  F_W L=F_W::Zero();
  L.block<3,3>(6,3)=-R*dt;
  L.block<3,3>(9,6)=M3::Identity()*dt;
  L.block<3,3>(12,9)=M3::Identity()*dt;
  // Preserve the original IMU interval's non-gyro process-noise budget when a
  // wheel timestamp splits it into smaller propagation intervals.
  L*=std::sqrt(span/dt);
  wheel_fusion::M18x6 B=wheel_fusion::M18x6::Zero();
  B.topRows<3>()=-dt*right_jacobian.cast<double>()*W;
  const wheel_fusion::M18 f=F.cast<double>();
  const wheel_fusion::M18x6 c=f*gyro.cross;
  wheel_fusion::M18 posterior=f*P_.cast<double>()*f.transpose()+
      c*B.transpose()+B*c.transpose()+B*gyro.covariance*B.transpose()+
      (L*Q_*L.transpose()).cast<double>();
  posterior=(posterior+posterior.transpose()).eval()*0.5;
  if (!posterior.allFinite()) return false;
  gyro.cross=c+B*gyro.covariance;
  wheel_gyro_=gyro;
  P_=posterior.cast<scalar>();
  global_acc_=R*acc+g_;
  p_+=v_*dt+0.5*global_acc_*dt*dt;
  v_+=global_acc_*dt;
  R_=R_*SO3::Exp(omega,dt);
  current_time_=last_imu_time_=target;
  last_imu_.secs=target;
  last_imu_.gyr=(1.0-b)*left.gyr+b*right.gyr;
  last_imu_.acc=(1.0-b)*left.acc+b*right.acc;
  wheel_raw_gyro_=last_imu_.gyr;
  wheel_gyro_weights_.leftCols<3>()=(1.0-b)*Eigen::Matrix3d::Identity();
  wheel_gyro_weights_.rightCols<3>()=b*Eigen::Matrix3d::Identity();
  raw_body_omega_=wheel_raw_gyro_-(wheel_gyro_weights_*wheel_gyro_.mean).cast<scalar>();
  body_omega_=raw_body_omega_-bg_;
  return true;
}

wheel_fusion::Result ESKF::UpdateWheel(const wheel_fusion::Sample& sample,
    bool linear,bool angular,double single_gate,double joint_gate) {
  if (!wheel_initialized_ || !wheel_enabled_ || std::abs(sample.time-current_time_)>1e-7)
    return wheel_fusion::Result();
  base_kinematics::Transform mount=base_kinematics::Transform::Identity();
  mount.linear()=g_imu_base.R_.cast<double>(); mount.translation()=g_imu_base.t_.cast<double>();
  const auto m=wheel_fusion::model(R_.R_.cast<double>(),v_.cast<double>(),
      GetBodyAngularVelocity().cast<double>(),mount);
  auto P=P_.cast<double>().eval(); auto gyro=wheel_gyro_;
  wheel_fusion::V18 correction=wheel_fusion::V18::Zero();
  auto result=wheel_fusion::update(P,gyro,m,wheel_gyro_weights_,sample,
      linear,angular,correction,single_gate,joint_gate);
  if (!result.applied) return result;
  const STATE delta=correction.cast<scalar>();
  const V3 gravity=g_+delta.segment<3>(15);
  const SO3 rotation=R_*SO3::Exp(delta.head<3>());
  if (!delta.allFinite() || !P.cast<scalar>().allFinite() || !rotation.R_.allFinite() ||
      !(p_+delta.segment<3>(3)).allFinite() || !(v_+delta.segment<3>(6)).allFinite() ||
      !(bg_+delta.segment<3>(9)).allFinite() || !(ba_+delta.segment<3>(12)).allFinite() ||
      !gravity.allFinite() || !std::isfinite(gravity.norm()) || gravity.norm()<1e-8) {
    result.applied=false;result.used={{false,false}};return result;
  }
  // Update() resets the high-frequency forward predictor; use the observation
  // time here, not the future scan-end timestamp used by the LiDAR update.
  const double saved_obs=current_obs_time_;
  current_obs_time_=current_time_;
  dx_=delta; Update(); dx_.setZero();
  current_obs_time_=saved_obs;
  P_=P.cast<scalar>(); wheel_gyro_=gyro;
  raw_body_omega_=wheel_raw_gyro_-(wheel_gyro_weights_*wheel_gyro_.mean).cast<scalar>();
  body_omega_=raw_body_omega_-bg_;
  return result;
}
} // namespace LI2Sup
