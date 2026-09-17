# Progress

- 2026-09-17: Inspected local Super-LIO and confirmed absence of loop closure.
- 2026-09-17: Inspected LICO-mid360 main branch and identified its KNN/Scan Context -> VGICP -> GTSAM iSAM2 pipeline and dependency risks.
- 2026-09-17: Confirmed local ESKF exposes the full covariance needed for `/lio/odom`.
- 2026-09-17: Selected an optional standalone, bounded-memory planar loop backend using system PCL and an Eigen sparse pose graph for the first implementation.
- 2026-09-17: Added ESKF covariance block mapping to raw `/lio/odom`; angular twist uncertainty is explicitly marked high because angular velocity is not an ESKF state.
- 2026-09-17: Added configurable standard-frame output to Super-LIO (`world_frame`, `base_frame`, `legacy_tf`) and selected `odom -> base_laser_link` for MID-360 mapping while retaining legacy defaults for relocation.
- 2026-09-17: Added `super_lio_loop`: bounded keyframe queue/storage, spatial KNN-style candidate search, historical submap assembly, PCL ICP verification, robust sparse SE(2) graph optimization, graph covariance, corrected odometry/cloud/path, `map -> odom` TF, revision and marker outputs.
- 2026-09-17: First isolated Noetic build reached C++ compilation and found one typed ROS parameter default mismatch; corrected it for the next build.
- 2026-09-17: `super_lio_loop` now compiles and links successfully with ROS Noetic, Eigen 3.3.7, and PCL 1.10; no GTSAM/Ceres/PCL 1.13 dependency is required.
- 2026-09-17: Added loop-disabled fallback in the combined mapping launch: identity `map -> odom` plus raw registered cloud.
- 2026-09-17: User selected the industrial route. Re-scoped the lightweight Eigen/ICP node as fallback and started a GTSAM 4.2 + Scan Context + FastVGICP/PCL-GICP primary backend with explicit dependency and map-reprojection phases.
- 2026-09-17: Added `super_lio_graph_backend_node`: Pose3 iSAM2 graph, descriptor/spatial retrieval, two-hit confirmation, FastVGICP/PCL-GICP registration, multi-gate rejection, Hessian-derived loop covariance, marginal pose covariance, diagnostics, and optimized-keyframe service.
- 2026-09-17: Added pinned CPU-only installer for GTSAM 4.2.2 and fast_gicp commit `0e7ec144...`, preserving ROS Noetic PCL 1.10.
- 2026-09-17: Made industrial backend the launch default and made `PROFILE=mapping` fail configuration when GTSAM is absent; lightweight remains an explicit fallback.
- 2026-09-17: Added bounded, double-buffered 2D map reprojection from optimized keyframes after each loop revision.
- 2026-09-17: Verified fallback + mapper compile with stock Noetic/PCL 1.10; built temporary GTSAM 4.2.2 and fast_gicp, then compiled and linked the full industrial path successfully.
- 2026-09-17: Runtime smoke test confirmed `/super_lio_loop`, `/super_lio_loop/get_keyframe`, corrected topics, revision and status endpoints; launch/script syntax checks passed. Hardware/bag acceptance remains for the RK3588/MID-360S.
