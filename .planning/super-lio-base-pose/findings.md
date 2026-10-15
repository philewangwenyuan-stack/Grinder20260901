# Findings

- ESKF uses right-multiplicative rotation errors, global position/velocity, gyro bias in IMU coordinates.
- Standard pub_odom currently labels raw IMU pose as base_link; legacy has separate historical behavior.
- Existing calibration arguments use radians and lidar_imu is T_I_L.
- Managed localization passes calibration; managed mapping currently does not.
- Initialpose callback hardcodes IMU z=0.2. Input must instead represent base pose in map coordinates.
- Loop backends derive local clouds by inverse raw odometry, so changing both odometry semantics and local cloud conversion remains consistent.
- Windows PATH currently has Python but no g++/clang++/cmake.
- WSL Ubuntu 20.04 has g++, Eigen and ROS Noetic. Independent C++ tests pass.
- Lightweight loop stored a planar pose but inverted a full 3D pose for local clouds; changed inverse to the same planar frame, preserving world height/tilt in stored clouds.
- Scheduler already converts persisted base_laser_* degrees to manager launch radians. Reuse that source unchanged.
- /lio/robo/odom is an existing separate IMU-rate debug/legacy stream; this task changes the navigation /lio/odom stream.
