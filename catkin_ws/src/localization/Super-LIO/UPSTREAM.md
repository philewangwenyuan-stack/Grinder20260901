# Super-LIO source provenance

- Upstream: https://github.com/Liansheng-Wang/Super-LIO.git
- Upstream checkout: `239984eb8cff0ba5efdf999e9b90d02b2c7d5c5e`
- License: GPL-3.0, preserved in `LICENSE`; retain third-party notices in the source.
- Imported into the Grinder root repository on 2026-09-28 as ordinary files.

This snapshot includes the existing Grinder/MID-360S integration changes from
the local checkout, including relocation/map services, ROS diagnostics,
configuration, launch files and RViz settings. It is not an unmodified upstream
checkout. Existing tracked files and newly added source/documentation files were
preserved during import; build trees, caches and local analysis outputs are not
distributed as source.
The import also declares the existing `basic` dependency in the Super-LIO
package/export metadata so Catkin dependency selection includes that package.

There is no submodule initialization step. Use Git from the Grinder root for
future changes and compare upstream updates against the checkout above.
The former nested Git history remains in an ignored `.git.nested-backup-*`
directory on the migration workstation; it is not required to build a fresh clone.

## Optional internal wheel fusion

The Grinder extension consumes valid-only `geometry_msgs/TwistWithCovarianceStamped`
on `/chassis/wheel_twist`. Existing `/odom_wheel`
remains RPP's velocity feedback; its unavailable pose is never fused. There is
no additional TF broadcaster or external EKF. Mapping and localization share
the same chronological IMU/wheel propagation implementation.

Managed parameters in `grinder_scheduler/config/super_lio.yaml`:

```yaml
super_lio_wheel_fusion_mapping_enabled: false
super_lio_wheel_fusion_localization_enabled: false
super_lio_wheel_fuse_linear_velocity: true
super_lio_wheel_fuse_angular_velocity: false
super_lio_wheel_twist_topic: /chassis/wheel_twist
super_lio_wheel_max_age_sec: 0.25
super_lio_wheel_single_nis_gate: 9.0
super_lio_wheel_joint_nis_gate: 11.83
```

Both mode switches default off. Enable a mode only after calibration and replay;
parameter changes take effect on the next managed mode restart. Direct launch
arguments are `wheel_fusion_enabled`, `wheel_fuse_linear_velocity`,
`wheel_fuse_angular_velocity`, `wheel_twist_topic`, `wheel_max_age_sec`,
`wheel_single_nis_gate` and `wheel_joint_nis_gate`, with matching defaults.
They are supported by `Livox_mid360.launch`, `relocation.launch` and
`mid360_mapping.launch`. Standard `legacy_tf: false` is required. Selecting no
observation while enabling a mode is a configuration error.
Fusion readiness is explicit in both applications and independent of the legacy
IMU-rate debug initialization flag. Reinitialization clears this readiness until
the mapping/localization initialization sequence has completed.

Signed base-frame forward speed supports reverse motion; yaw rate is positive
for left rotation and is not inverted while reversing. The model includes the
existing measured IMU/base rotation and lever arm. No lateral-zero or height
constraint is added. A separate wheel update acts on the 18D ESKF state. Original
gyro samples bracketing the current time are retained as nuisance noise variables
with their conditional means, covariance and state cross-covariance. Consequently,
IMU interval splitting and wheel yaw/lever-arm observations do not treat shared
gyro noise as independent evidence. LiDAR updates also condition this retained
noise. In the enabled path, the LiDAR covariance uses its existing inverse-
information posterior `Qk` for all 18 state dimensions before the attitude reset;
this preserves a consistent joint state/noise covariance. The disabled path
retains its historical covariance calculation. Innovation gating rejects scalar outliers (default NIS 9) and inconsistent
joint updates (default NIS 11.83). An invalid channel does not disable a valid
selected channel. Wheel observations are never re-used by LiDAR iterations.

The first subsequent IMU sample provides interpolation at scan/wheel boundaries;
there is no waiting for wheel data and no state-history rollback. Wheel samples
older than the state, duplicated, stale (>0.25 s), non-finite, in the wrong frame
or with invalid covariance are rejected. Future observations can arrive out of
order and are consumed chronologically. The buffer is limited to
2 s/200 samples and is cleared on initialization/relocalization, state reset,
and time rollback. Missing/rejected wheel data leaves IMU/LiDAR processing active;
it does not relax registration/readiness checks or override task safety.

The chassis publisher uses the midpoint of the successful wheel-read transaction
minus `wheel_feedback_delay_sec`, initially zero. This is an estimated host time,
not a servo sampling timestamp: successful reads may still return an old controller
cache. Default v/yaw variances are each 0.04, with zero cross covariance; calibrate
them and the delay using textured-area runs, including reverse, turns and disc-on
operation. Read/conversion failures do not publish a fusion message; real stopped
feedback does. Source timestamps/sequence counters remain a future firmware improvement.

Inspect `/super_lio/wheel_fusion_diagnostics` (also on `/diagnostics`) for mode,
channel switches, last applied observation age, accepted/rejected/late/expired
counts, residuals, variances and NIS. The angular residual is also available with
yaw fusion disabled, for time alignment/slip investigation. Disable the relevant
mode switch and restart that mode to return to the original propagation path.

Tests `super_lio_wheel_fusion` and `super_lio_wheel_eskf` cover model Jacobians,
signed motion, correlated gyro noise, covariance PSD, channel selection, time
ordering and a synthetic biased-IMU trajectory. Synthetic error reduction is not
a field accuracy claim. With a local test master, run the additional ROS interface
regression using `rosrun super_lio test_wheel_ros`; it exercises message rejection,
reverse motion, single consumption, future buffering and reset without hardware.
Record `/livox/lidar /livox/imu /chassis/wheel_twist
/lio/odom /super_lio/registration_diagnostics /super_lio/wheel_fusion_diagnostics
/tf /tf_static` and compare the same bag with fusion off, linear only and both
channels; use bag clock and measured reference poses for accuracy assessment.
Without a field bag/reference trajectory the feature stays disabled by default.

## Per-frame localization iteration diagnostics

See also the [base pose and mounting calibration](#base-pose-and-mounting-calibration)
section below for the navigation coordinate contract.

`/super_lio/registration_diagnostics` retains its existing registration fields
and adds `kf_*` keys to the `super_lio_registration` status. The array header
timestamp is the same frame timestamp as localization odometry. This is
instrumentation only: iteration limits, convergence criteria, state updates and
localization readiness checks are unchanged.

- `kf_iterations`, `kf_max_iterations`, `kf_quit_eps`: actual iteration count,
  configured limit and convergence threshold.
- `kf_convergence_reason`: `threshold_reached`, `iteration_limit`,
  `non_finite_update`, or `no_iterations`. `kf_iteration_limit_reached` means
  the limit was exhausted without convergence; `kf_updates_finite` covers all
  iterations in this frame.
- `kf_iteration_N_update_inf`, `kf_iteration_N_max_component`,
  `kf_iteration_N_max_component_value`, `kf_iteration_N_update_finite`:
  each iteration's maximum absolute update, component name, signed component
  value and finite-value check. N starts at 1. For a non-finite update, the
  maximum is reported as `inf` and the first non-finite component is identified.
- `kf_final_update_inf`, `kf_final_max_component`, and
  `kf_final_{rotation,position,velocity,gyro_bias,accel_bias,gravity}_update_inf`:
  final iteration summary, captured before the filter clears its update vector.
  These are infinity norms (maximum absolute component), not Euclidean norms;
  the six groups have different physical units and must be interpreted separately.

Unconverged frames also emit a warning summary throttled to once per second.
The diagnostic topic retains every frame, including frames omitted by the log
throttle. Numeric values retain double precision to distinguish updates close
to the configured threshold.

After rebuilding Super-LIO in Linux/ROS Noetic and restarting the localization
process through the normal lifecycle, inspect or record the topic:

```bash
rostopic echo /super_lio/registration_diagnostics
rosbag record -O /tmp/super_lio_iteration_trace.bag \
  /initialpose /super_lio/registration_diagnostics /lio/odom /tf /tf_static /rosout
```

Compare unconverged frames with successful neighboring frames. A decreasing
update that remains above the threshold at the iteration limit suggests an
iteration-budget investigation; oscillating or non-finite updates require
examining registration inputs and numerical behavior rather than loosening the
readiness check.

## Base pose and mounting calibration

In standard mode (`lio/ros/legacy_tf: false`), `/lio/odom`, `/lio/path`, position
outputs and `odom -> base_link` describe the chassis motion center. The ESKF,
deskewing, matching and world/map point clouds still use the original IMU state.
With `T_A_B` mapping coordinates in B into A, the output conversion is:

```text
T_odom_base = T_odom_imu * T_imu_laser * inverse(T_base_laser)
```

`lio/extrinsic/lidar_imu` supplies `T_imu_laser`, including the MID-360 internal
sensor offset. The six `base_to_laser_{x,y,z,roll,pitch,yaw}` launch arguments
provide the measured `T_base_laser`. The launch sets both the fixed
`base_link -> base_laser_link` TF and `/lio/extrinsic/base_to_laser_*` from the
same arguments. Translation uses metres; launch and LIO mounting angles use
radians. Zero mounting means base coincides with the **laser**, not the IMU;
only identity mounting AND identity internal extrinsics give raw IMU output.

Managed mapping and localization obtain these arguments from the mode manager.
The scheduler's existing `base_laser_{x,y,z}` and `base_laser_{roll,pitch,yaw}_deg`
settings remain the calibration source; the scheduler converts degrees to
radians. Changing calibration takes effect on the next normal mode restart;
do not add a second static TF publisher or change only the LIO parameters.
Direct roslaunch users must pass the same measured arguments for both modes.

Standard Odometry twist is expressed in `base_link`. It includes the lever-arm
velocity `omega cross r`, using the prediction-interval gyro mean corrected by
the latest gyro bias. Pose covariance uses world position/world small-angle
rotation errors; twist covariance includes attitude, velocity, bias cross terms
and gyro measurement variance (the existing `imu_ng` variance convention).
Legacy mode retains historical navigation odometry/TF and logs a warning that
mounting compensation is disabled. The existing IMU-rate `/lio/robo/odom`
debug stream remains a historical interface; navigation uses `/lio/odom`.

`/initialpose` and `lio/relocation/init_pose` represent the **base** position and
heading in standard mode. The input Z must be the base height in the map frame,
which need not equal height above the floor for an existing map. Gravity from
the stationary IMU supplies roll/pitch; mounting rotation is accounted for
when aligning the base heading. The complete guess is converted to IMU pose
before NDT/ICP. No sensor height is hardcoded. YAML initial angles remain in
degrees; `/initialpose` carries a quaternion. Initialize after the chassis has
settled following a guide-wheel change.

Mapping gravity alignment and the existing internal mapping-origin logic are
unchanged. Do not rewrite old PCD/occupancy maps to compensate mounting. Check
old-map alignment by replay/relocalization after rebuilding. Both loop backends
derive keyframe coordinates from the same raw base odometry used to place them;
the lightweight backend uses its stored planar pose for both directions so
world height and tilt are preserved. Neither applies mounting again.

Offline regression (ROS-independent C++ math and Windows-compatible launch tests):

```bash
g++ -std=c++17 -O0 -I/usr/include/eigen3 \
  -Icatkin_ws/src/localization/Super-LIO/src/super_lio/include \
  catkin_ws/src/localization/Super-LIO/src/super_lio/test/test_base_kinematics.cpp \
  -o /tmp/test_base_kinematics
/tmp/test_base_kinematics
PYTHONPATH=catkin_ws/src/grinder_scheduler/src python3 -m unittest discover \
  -s catkin_ws/src/grinder_scheduler/test -p 'test_super_lio*.py'
```

The C++ target is also registered with CTest when `CATKIN_ENABLE_TESTING=ON`.
After a Linux/ROS rebuild, replay a bag with the measured calibration and check
the TF closure against IMU/laser world points, map/cloud alignment at 0 and 30
degrees, base-center position during rotation, initial-pose reference, and
loop-cloud reprojection. Reuse prior maps before considering any remapping.
