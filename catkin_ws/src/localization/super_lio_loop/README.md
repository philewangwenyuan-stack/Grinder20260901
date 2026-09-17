# Super-LIO loop backend

This package deliberately contains two backends:

- `super_lio_graph_backend_node` is the production path: Scan Context-style retrieval, spatial fallback, FastVGICP (or explicit PCL-GICP degraded mode), registration observability gates, registration-derived covariance, robust GTSAM 4.2 iSAM2 Pose3 optimization, and marginal covariance.
- `super_lio_loop_node` is a small Eigen/ICP fallback for dependency recovery and bench diagnosis. It is not the production default.

## Dependencies on Ubuntu 20.04 / RK3588

```bash
cd /home/neardi/work/Grinder/catkin_ws
JOBS=2 bash ./install_super_lio_loop_deps.sh
PROFILE=mapping ./build_grinder_platform.sh
source devel/setup.bash
```

The installer pins GTSAM 4.2.2 and a tested fast_gicp commit, disables CUDA/tests/examples, and keeps ROS Noetic's PCL 1.10. It does not use the PCL >= 1.13 dependency from LICO-mid360.

## Launch

Production:

```bash
roslaunch cloud_to_occupancy_grid mid360_mapping.launch loop_backend:=industrial
```

Explicit fallback:

```bash
roslaunch cloud_to_occupancy_grid mid360_mapping.launch loop_backend:=lightweight
```

If the industrial executable was not built because GTSAM is missing, the production launch fails immediately (`required=true`). It never silently substitutes the lightweight backend.

Diagnostics:

```bash
rostopic echo /super_lio_loop/status
rostopic echo /super_lio_loop/revision
rostopic echo /diagnostics
rostopic hz /lio/loop_odom
```

Accept a loop only when descriptor distance, registration fitness, overlap and Hessian condition all pass the configured limits. Tune these from recorded bags before changing them on a live machine.
