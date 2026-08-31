cmake_minimum_required(VERSION 3.0.2)
project(grinder_scheduler)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  grinder_chassis_driver
  message_generation
  nav_msgs
  rospy
  sensor_msgs
  slamware_ros_sdk
  std_msgs
  std_srvs
)

catkin_python_setup()

add_message_files(
  FILES
  SchedulerStatus.msg
  MapPreviewMetadata.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS diagnostic_msgs geometry_msgs grinder_chassis_driver message_runtime nav_msgs rospy sensor_msgs slamware_ros_sdk std_msgs std_srvs
)

catkin_install_python(PROGRAMS
  scripts/scheduler_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
