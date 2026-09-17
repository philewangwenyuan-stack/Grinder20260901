# Super-LIO industrial loop closure and covariance plan

## Goal

Add an industrial primary loop-closure backend (descriptor retrieval, VGICP verification, GTSAM iSAM2 and covariance) while retaining the bounded-memory lightweight backend as a fallback and keeping the MID-360S mapping/localization workflow compatible.

## Phases

- [x] Phase 1 — Inspect LICO-mid360 and local Super-LIO interfaces.
- [x] Phase 2 — Publish ESKF pose/twist covariance in raw `/lio/odom`.
- [x] Phase 3 — Add a standalone bounded-memory loop backend with keyframes, spatial candidate search, ICP verification, SE(2) pose-graph optimization, corrected odometry/path/cloud, correction covariance, and diagnostics.
- [x] Phase 4 — Integrate standard `map -> odom -> base_laser_link` frames into the mapping launch while preserving legacy relocation behavior.
- [x] Phase 5 — Implement the industrial backend: Scan Context retrieval, spatial fallback, FastVGICP/PCL-GICP verification gates, registration covariance, robust GTSAM iSAM2 Pose3 graph and diagnostics.
- [x] Phase 6 — Select industrial/fallback backend from launch and add deterministic dependency setup/documentation for RK3588.
- [x] Phase 7 — Integrate loop revisions with the incremental 2D mapper, including bounded historical reprojection instead of leaving stale cells.
- [x] Phase 8 — Compile/test the available paths in ROS Noetic WSL and provide target-board verification/acceptance steps.

## Decisions

- Reuse LICO-mid360's architecture, not its whole implementation: keyframes, spatial/descriptor candidate search, ICP validation, pose graph.
- Keep the existing Eigen sparse SE(2) + PCL ICP implementation only as the dependency-free fallback.
- The production backend uses GTSAM 4.2 iSAM2 and Pose3 with tight z/roll/pitch odometry constraints. FastVGICP is preferred when installed; system PCL GICP is the explicit degraded registration path.
- Implement Scan Context-compatible retrieval locally so descriptor search does not force PCL 1.13 or Ceres into the ROS Noetic workspace.
- Keep loop closure optional and isolated in its own ROS package.
- Raw Super-LIO covariance comes from the existing 18-state ESKF (`R,p,v,bg,ba,g`).
- Do not claim historical 2D grid correction until keyframe reprojection is implemented and tested.

## Errors Encountered

| Error | Resolution |
| --- | --- |
| Initial shallow clone timed out with `.git` but no checkout | Fetched `origin/main` and checked out `FETCH_HEAD` in the temporary reference directory. |
| PowerShell `Get-ChildItem -Filter` was passed an array | Ignore; use explicit paths or enumerate then filter in subsequent checks. |
| First loop-package compile failed because ROS `param` deduced `float` vs `double` for voxel size | Changed the default literal to `0.25F`; retry compilation. |
| Full Super-LIO build in WSL stopped in existing `basic/logs.cpp` because `libgoogle-glog-dev` is absent | Loop package compiled independently; target board already required/installed glog. Re-run the integrated build on the RK3588. |
| First temporary GTSAM build compile used a `CMAKE_PREFIX_PATH` containing only the temporary prefix, so ROS message paths were hidden | Re-ran with the explicit combined prefix `/tmp/...:/opt/ros/noetic`; industrial compilation succeeded. |
| First runtime-smoke shell command let PowerShell consume bash PID variables | Terminated the exact test processes, then used separate managed exec sessions and confirmed clean shutdown. |
