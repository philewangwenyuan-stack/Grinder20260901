cmake_minimum_required(VERSION 3.0.2)
project(grinder_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  grinder_chassis_driver
  nav_msgs
  rospy
  sensor_msgs
  slamware_ros_sdk
  std_msgs
  std_srvs
  tf2_ros
  xacro
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    gazebo_ros
    geometry_msgs
    grinder_chassis_driver
    nav_msgs
    rospy
    sensor_msgs
    slamware_ros_sdk
    std_msgs
    std_srvs
    tf2_ros
    xacro
)

catkin_install_python(PROGRAMS
  scripts/chassis_sim_node.py
  scripts/slamware_sim_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  meshes
  urdf
  worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

