#include "lio/ESKF.h"
#include <cassert>
#include <iostream>

using namespace LI2Sup;
using namespace BASIC;

int main() {
  g_gravity_norm=9.8;
  g_imu_base=SE3(SO3(),V3(0.1,-0.03,0.2));
  for(int mask=1;mask<4;++mask) for(double velocity:{-0.1,0.0,0.1}) {
    ESKF filter;
    ESKF::Options options;
    filter.SetInitialConditions(options,V3::Zero(),V3::Zero(),1.0,V3(0,0,-9.8));
    filter.SetX(SysState(1.0)); filter.SetLastObsTime(1.0); filter.SetObsTime(1.1);
    filter.EnableWheelFusion(true); filter.init_=true;
    filter.SetCov(ESKF::COV::Identity()*0.1);
    IMUData left,right; left.secs=1.0;right.secs=1.1;
    left.acc=right.acc=V3(0,0,9.8);
    left.gyr=right.gyr=V3(0,0,0.05);
    assert(!filter.PredictWheelInterval(left,right,1.05));
    filter.SetWheelInitialized(true);
    assert(filter.PredictWheelInterval(left,right,1.05));
    wheel_fusion::Sample wheel;
    wheel.time=1.05;wheel.velocity << velocity,-0.04;
    wheel.covariance=Eigen::Matrix2d::Identity()*0.04;wheel.valid={{true,true}};
    const auto update=filter.UpdateWheel(wheel,mask&1,mask&2,9.0,11.83);
    assert(update.applied);
    assert(std::abs(filter.GetTime()-1.05)<1e-12);
    assert(filter.PredictWheelInterval(left,right,1.1));
    assert(!filter.PredictWheelInterval(left,right,1.05));
    assert(!filter.UpdateWheel(wheel,true,true,9.0,11.83).applied);
    // A LiDAR update must also condition the retained gyro noise, otherwise the
    // next wheel observation reuses an inconsistent joint covariance.
    filter.UpdateObserve([](const ESKF::KFState&, M6& information, V6& residual) {
      information=M6::Identity()*10.0; residual.setZero();
      residual[0]=0.001;residual[4]=-0.001;
    });
    const auto P=filter.GetCov().cast<double>().eval();
    assert(P.allFinite() && (P-P.transpose()).norm()<1e-6);
    assert(P.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff()>-1e-7);
    Eigen::Matrix<double,21,21> joint=Eigen::Matrix<double,21,21>::Zero();
    joint.topLeftCorner<18,18>()=P;
    joint.topRightCorner<18,3>()=filter.GetStateGyroCross();
    joint.bottomLeftCorner<3,18>()=filter.GetStateGyroCross().transpose();
    joint.bottomRightCorner<3,3>()=filter.GetGyroNoiseCovariance();
    assert(joint.allFinite());
    assert(joint.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff()>-1e-7);
    if(mask&1 && velocity<0) assert(filter.GetSysState().v.x()<0.0);
    filter.SetInitialConditions(options,V3::Zero(),V3::Zero());
    filter.SetX(SysState(2.0));
    assert(filter.GetTime()==2.0);
  }
  // Synthetic constant-speed turn with a biased accelerometer. Both runs use
  // identical IMU integration; only one receives independent wheel observations.
  g_imu_base=SE3(SO3(),V3::Zero());
  double errors[2]{};
  for(int fused=0;fused<2;++fused) {
    ESKF filter; ESKF::Options options;
    filter.SetInitialConditions(options,V3::Zero(),V3::Zero(),1.0,V3(0,0,-9.8));
    filter.SetX(SysState(1.0,SO3(),V3::Zero(),V3(0.1,0,0)));
    filter.EnableWheelFusion(true);filter.SetWheelInitialized(true);
    filter.init_=false; // Localization has no legacy IMU-rate debug initialization flag.
    filter.SetLastObsTime(1.0);filter.SetObsTime(5.0);
    for(int step=1;step<=400;++step) {
      IMUData left,right;left.secs=1.0+(step-1)*0.01;right.secs=1.0+step*0.01;
      left.acc=right.acc=V3(0.02,0.005,9.8);
      left.gyr=right.gyr=V3(0,0,0.05);
      assert(filter.PredictWheelInterval(left,right,right.secs));
      if(fused && step%10==0) {
        wheel_fusion::Sample sample;sample.time=right.secs; sample.velocity << 0.1,0.05;
        sample.valid={{true,true}};sample.covariance=Eigen::Matrix2d::Identity()*1e-4;
        assert(filter.UpdateWheel(sample,true,true,9.0,11.83).applied);
      }
    }
    const V3 truth(2.0*std::sin(0.2),2.0*(1.0-std::cos(0.2)),0);
    errors[fused]=(filter.GetSysState().p-truth).norm();
  }
  assert(errors[1]<errors[0]);
  std::cout << "synthetic turn position errors: pure=" << errors[0] << " fused=" << errors[1] << '\n';
  std::cout << "ESKF timestamp ordering, reverse velocity, channel switches and PSD checks passed\n";
}
