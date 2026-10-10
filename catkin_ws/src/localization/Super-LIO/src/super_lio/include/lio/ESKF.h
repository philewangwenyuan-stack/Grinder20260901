#ifndef ESKF_HPP_
#define ESKF_HPP_

#include <nav_msgs/Odometry.h>
#include <array>
#include <vector>

#include "basic/alias.h"
#include "basic/Manifold.h"
#include "common/ds.h"
#include "params.h"


namespace LI2Sup{

class ESKF {
public:
  using Ptr   = std::shared_ptr<ESKF>;
                                                  //         0 3 6 9 12 15
  using STATE = Eigen::Matrix<BASIC::scalar, 18, 1>;     //Flatten: R p v bg ba g. 
  using STATE_DOF = Eigen::Matrix<BASIC::scalar, 17, 1>; //Flatten: R p v bg ba g_2.
  using NOISE = Eigen::Matrix<BASIC::scalar, 12, 12>;
  using COV   = Eigen::Matrix<BASIC::scalar, 18, 18>;
  using F_X   = Eigen::Matrix<BASIC::scalar, 18, 18>;
  using F_W   = Eigen::Matrix<BASIC::scalar, 18, 12>;


  struct Options {
    Options(){}
    int num_iterations_ = 3;
    double quit_eps_ = 1e-6;

    double gyro_var_ = 1e-5;
    double acce_var_ = 1e-2;
    double bias_gyro_var_ = 1e-6;
    double bias_acce_var_ = 1e-4;
  };


  struct KFState{
    bool need_converge = true;
    BASIC::SE3  pose;
  };

  struct IterationDiagnostic {
    double update_inf = 0.0;
    int max_component = -1;
    double max_component_value = 0.0;
    bool update_finite = true;
    // R, p, v, gyro bias, accelerometer bias, gravity; infinity norms.
    std::array<double, 6> block_update_inf{};
  };

  struct UpdateDiagnostic {
    int max_iterations = 0;
    double quit_eps = 0.0;
    bool updates_finite = true;
    std::vector<IterationDiagnostic> iterations;
  };


  ESKF(Options option = Options()) : options_(option) { BuildNoise(option); }

  ESKF(Options options, const BASIC::V3& init_bg, const BASIC::V3& init_ba, const BASIC::V3& gravity = BASIC::V3(0, 0, -9.8))
      : options_(options) {
    BuildNoise(options);
    bg_ = init_bg;
    ba_ = init_ba;
    g_ = gravity;
  }

  void SetInitialConditions(Options options, const BASIC::V3& init_bg, const BASIC::V3& init_ba, const float imu_scale = 1.0,
                            const BASIC::V3& gravity = BASIC::V3(0, 0, -9.8));

  bool Predict(const IMUData& imu);

  using ObsFunc = std::function<void(const KFState& kf_state, BASIC::M6& HT_Vinv_H, BASIC::V6& HT_Vinv_r)>;
  bool UpdateObserve(ObsFunc obs);
  bool DidLastUpdateConverge() const { return last_update_converged_; }
  const UpdateDiagnostic& GetLastUpdateDiagnostic() const { return last_update_diagnostic_; }

  double GetTime() const { return current_time_; }

  SysState GetSysState() const { return SysState(current_time_, R_, p_, v_, bg_, ba_); }

  NavState GetNavState() const { return NavState(current_time_, R_, p_, v_); }

  DynamicState GetDynamicState() const { return DynamicState(current_time_, R_.R_, p_, v_, body_omega_, global_acc_); }

  KFState GetKFState() const { return KFState{need_converge_, GetSE3()}; }

  Pose_t   GetPoseT() const { return Pose_t(current_time_, R_, p_); }

  COV GetCov() const { return P_; }

  BASIC::SE3 GetSE3() const { return BASIC::SE3(R_, p_); }

  void SetObsTime(const double obs_time) { current_obs_time_ = obs_time; }
  void SetLastObsTime(const double obs_time) { last_obs_time_ = obs_time; }

  void SetX(const SysState& x);

  void SetCov(const COV& cov){ P_ = cov; }

  BASIC::V3 GetGravity() const { return g_; }

  bool init_ = false;
  bool Predict(const IMUData& imu, nav_msgs::Odometry& odom, nav_msgs::Odometry& odom1);

private:
  void BuildNoise(const Options& options);
  void Update();
  
  bool  need_converge_  = true;
  bool  last_update_converged_ = false;
  UpdateDiagnostic last_update_diagnostic_;
  float imu_scale_ = 1.0;
  IMUData last_imu_;
  double current_time_ = 0.0;
  double last_imu_time_ = -1.0;
  double last_obs_time_ = 0.0;
  double current_obs_time_ = 0.0;

  // nominal state
  BASIC::SO3 R_;
  BASIC::V3 p_ = BASIC::V3::Zero();
  BASIC::V3 v_ = BASIC::V3::Zero();
  BASIC::V3 bg_ = BASIC::V3::Zero();
  BASIC::V3 ba_ = BASIC::V3::Zero();
  BASIC::V3 g_{0, 0, - (BASIC::scalar)g_gravity_norm};
  BASIC::V3 global_acc_  = BASIC::V3::Zero();
  BASIC::V3 body_omega_  = BASIC::V3::Zero();

  // error state
  STATE dx_ = STATE::Zero();

  // covariance
  COV P_ = COV::Identity();

  // noise
  NOISE Q_ = NOISE::Zero();

  Options options_;

  double  forward_time_ = -1;
  IMUData forward_last_imu_;
  BASIC::SO3 fw_R_;
  BASIC::V3 fw_p_ = BASIC::V3::Zero();
  BASIC::V3 fw_v_ = BASIC::V3::Zero();
};


}


#endif
