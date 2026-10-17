#include "lio/wheel_fusion.h"
#include <cassert>
#include <iostream>

using namespace LI2Sup;
using namespace wheel_fusion;

int main() {
  auto mount=base_kinematics::fromXyzRpy({0.1,-0.06,0.3},{0.12,-0.08,0.4});
  auto R=base_kinematics::fromXyzRpy({0,0,0},{0.2,-0.15,0.7}).linear().eval();
  Eigen::Vector3d v(-0.1,0.04,0.01),omega(0.02,-0.01,-0.12);
  const auto m=model(R,v,omega,mount);
  const double eps=1e-6;
  for(int col=0;col<18;++col) {
    auto r=R; auto vel=v; auto w=omega;
    if(col<3) r=R*Eigen::AngleAxisd(eps,Eigen::Vector3d::Unit(col)).toRotationMatrix();
    if(col>=6 && col<9) vel[col-6]+=eps;
    if(col>=9 && col<12) w[col-9]-=eps;
    const auto numerical=(model(r,vel,w,mount).prediction-m.prediction)/eps;
    assert((numerical-m.H.col(col)).norm()<1e-6);
  }
  for(int col=0;col<3;++col) {
    auto w=omega; w[col]+=eps;
    assert(((model(R,v,w,mount).prediction-m.prediction)/eps-m.G.col(col)).norm()<1e-6);
  }
  const auto identity=base_kinematics::Transform::Identity();
  for(double speed:{-0.15,0.0,0.15}) for(double yaw:{-0.2,0.0,0.2}) {
    const auto observed=model(Eigen::Matrix3d::Identity(),{speed,0,0},{0,0,yaw},identity);
    assert(std::abs(observed.prediction[0]-speed)<1e-12);
    assert(std::abs(observed.prediction[1]-yaw)<1e-12);
    for(int mask=1;mask<4;++mask) {
      M18 P=M18::Identity()*0.1;
      GyroContext gyro; gyro.select(1.0,1.01,0.01);
      Eigen::Matrix<double,3,6> W=Eigen::Matrix<double,3,6>::Zero(); W.rightCols<3>().setIdentity();
      gyro.cross.block<3,3>(0,3)=-0.0001*Eigen::Matrix3d::Identity();
      Sample sample; sample.time=1.01; sample.velocity=observed.prediction;
      sample.velocity[0]+=0.01; sample.velocity[1]-=0.01;
      sample.covariance << 0.04,0.005,0.005,0.04; sample.valid={{true,true}};
      V18 dx=V18::Zero();
      auto result=update(P,gyro,observed,W,sample,mask&1,mask&2,dx);
      assert(result.applied && result.used[0]==bool(mask&1) && result.used[1]==bool(mask&2));
      assert((P-P.transpose()).norm()<1e-10);
      assert(P.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff()>-1e-10);
      if(!(mask&2)) assert(std::abs(dx[11])<1e-12);
      const M18 before=P;
      sample.velocity.setConstant(1e5);
      assert(!update(P,gyro,observed,W,sample,true,true,dx).applied);
      assert((P-before).norm()==0.0);
    }
  }
  GyroContext retained; retained.select(1.0,2.0,0.1);
  retained.mean[3]=0.02; retained.cross(0,3)=0.001;
  retained.select(2.0,3.0,0.1);
  assert(retained.mean[0]==0.02 && retained.cross(0,0)==0.001);
  assert(retained.mean[3]==0.0 && retained.cross.col(3).norm()==0.0);
  // Reference joint conditioning: state and raw gyro noise are correlated.
  // Omitting either cross term or counting gyro noise twice changes this answer.
  {
    M18 P=M18::Identity()*0.1;
    GyroContext gyro;gyro.select(1.0,1.1,0.01);
    gyro.cross(11,5)=0.003;
    Eigen::Matrix<double,3,6> W=Eigen::Matrix<double,3,6>::Zero();W.rightCols<3>().setIdentity();
    const auto observed=model(Eigen::Matrix3d::Identity(),{0.1,0,0},{0,0,0.05},identity);
    Sample s;s.velocity=observed.prediction;s.covariance=Eigen::Matrix2d::Identity()*0.04;s.valid={{true,true}};
    M24 prior=M24::Zero();prior.topLeftCorner<18,18>()=P;
    prior.topRightCorner<18,6>()=gyro.cross;prior.bottomLeftCorner<6,18>()=gyro.cross.transpose();
    prior.bottomRightCorner<6,6>()=gyro.covariance;
    Eigen::Matrix<double,2,24> H;H.leftCols<18>()=observed.H;H.rightCols<6>()=-observed.G*W;
    const Eigen::Matrix2d S=H*prior*H.transpose()+s.covariance;
    const M24 expected=prior-prior*H.transpose()*S.inverse()*H*prior;
    V18 dx=V18::Zero();assert(update(P,gyro,observed,W,s,true,true,dx).applied);
    assert((P-expected.topLeftCorner<18,18>()).norm()<1e-12);
    assert((gyro.cross-expected.topRightCorner<18,6>()).norm()<1e-12);
    assert((gyro.covariance-expected.bottomRightCorner<6,6>()).norm()<1e-12);
    s.velocity[0]=1e5;s.velocity[1]=observed.prediction[1];
    const auto partial=update(P,gyro,observed,W,s,true,true,dx);
    assert(partial.applied && !partial.used[0] && partial.used[1]);
  }
  {
    M18 P=M18::Identity()*0.1;
    GyroContext gyro;gyro.select(1.0,1.1,0.01);
    Eigen::Matrix<double,3,6> W=Eigen::Matrix<double,3,6>::Zero();W.rightCols<3>().setIdentity();
    const auto observed=model(Eigen::Matrix3d::Identity(),{0,0,0},{0,0,0},identity);
    Sample s;s.covariance=Eigen::Matrix2d::Identity()*0.04;s.valid={{true,true}};
    s.velocity << std::sqrt(7.0*0.14),std::sqrt(7.0*0.15);
    const M18 before=P;V18 dx=V18::Zero();
    const auto rejected=update(P,gyro,observed,W,s,true,true,dx);
    assert(!rejected.applied && rejected.joint_nis>11.83 && (P-before).norm()==0.0);
    s.valid[0]=false;s.velocity[0]=std::numeric_limits<double>::quiet_NaN();
    s.velocity[1]=0.01;
    const auto fallback=update(P,gyro,observed,W,s,true,true,dx);
    assert(fallback.applied && !fallback.used[0] && fallback.used[1]);
  }
  Buffer buffer; Sample s; s.time=1.0;
  assert(buffer.push(s)); assert(!buffer.push(s));
  s.time=0.9; assert(buffer.push(s));
  assert(buffer.samples.front().time==0.9); // Reorder future observations before consumption.
  for(int i=0;i<500;++i) { s.time=1.01+0.01*i; assert(buffer.push(s)); }
  assert(buffer.samples.size()<=200 && buffer.samples.back().time-buffer.samples.front().time<=2.0);
  buffer.clear(); s.time=0.5; assert(buffer.push(s));
  std::cout << "wheel model, signed channels, correlated gyro, NIS, PSD and buffer checks passed\n";
}
