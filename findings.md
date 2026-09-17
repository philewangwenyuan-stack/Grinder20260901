# Findings

## LICO-mid360 reference

- Reference: https://github.com/piluohong/LICO-mid360, inspected commit `c64a23d5239946bf6e3ab51bebb5bb62df9ee71e`.
- README describes ground segmentation and closure optimization based on KNN and voxel registration.
- Loop node is `src/laserPosegraphOpt.cpp`.
- It selects keyframes by translation/rotation thresholds, stores downsampled keyframe clouds, detects candidates with KNN or Scan Context, builds a nearby historical submap, verifies with FastVGICP, and adds robust loop factors to GTSAM iSAM2.
- Defaults include 1 m keyframe spacing, 30 degree angular spacing, 30 recent-frame exclusion in Scan Context, 15 neighboring keyframes in the ICP target, and fitness rejection around 0.5.
- It hardcodes 6 VGICP threads and 8 transform threads in places.
- Dependencies include GTSAM 4.0.3, Ceres 2.0, and PCL >= 1.13; the repository itself warns about conflicts with ROS Noetic's PCL 1.10.
- License is GPL-3.0; local Super-LIO is also GPLv3, but the plan avoids copying substantial source and instead implements the architecture independently.

## Local Super-LIO

- There is currently no loop closure, pose graph, Scan Context, or GTSAM/g2o backend.
- ESKF already exposes `GetCov()` returning an 18x18 covariance ordered `R,p,v,bg,ba,g`.
- `/lio/odom` currently leaves covariance empty and publishes the estimated pose as `map -> odom`, followed by identity `odom -> base_laser_link`.
- `/lio/map_cloud` contains the current scan transformed by the raw LIO pose, despite its legacy name.
- The lightweight 2D mapper consumes current registered clouds and maintains fixed-size log odds, but it cannot yet move historical cells after a non-rigid pose-graph correction.

## Compatibility decision

- First loop backend will be planar SE(2), suitable for the grinder's indoor floor operation, while retaining z/roll/pitch from raw LIO output.
- It will publish corrected output separately and expose covariance/diagnostics. Standard TF integration will be gated by parameters so relocation behavior remains intact.

## Industrial backend dependency facts

- Official `fast_gicp` builds a `fast_gicp` shared library and exports it through catkin; `FastVGICP` supports PCL 1.10 and exposes thread count and voxel resolution controls. This avoids LICO-mid360's PCL >= 1.13 requirement.
- GTSAM 4.2 exports a CMake package and provides the mature iSAM2/factor-graph path required for incremental optimization and marginal covariance.
- Dependencies will remain optional at configure time so the existing lightweight backend can still build, but the industrial launch mode must fail clearly rather than silently pretending that the fallback is production quality.

## Implemented industrial path

- The production node uses a bounded Pose3 keyframe graph and GTSAM 4.2 iSAM2, with strong roll/pitch/z odometry priors suitable for the mostly planar grinder while retaining full 3D registration.
- Loop candidates combine Scan Context-style descriptor retrieval and optimized-pose proximity. A candidate needs two consistent observations before insertion.
- FastVGICP is selected when present; PCL GICP is an explicit degraded path. Fitness, overlap, correction magnitude, consistency and Hessian condition gates must all pass.
- Loop constraint covariance comes from the registration normal matrix and residual variance, with numerical floors; corrected odometry covariance comes from the latest iSAM2 marginal.
- The 2D mapper now receives loop revisions, pulls optimized local keyframes through a back-pressured ROS service, rebuilds into staging arrays, and atomically swaps the finished map. This avoids both unbounded scan retention inside the mapper and stale historical cells.
