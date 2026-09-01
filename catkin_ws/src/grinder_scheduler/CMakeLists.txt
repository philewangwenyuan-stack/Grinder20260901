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

# Build the shared MST27 planner kernel next to mst27.py so the dynamically
# loaded Python planner can use the native implementation on the target host.
set(MST27_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/path_planner/mst27")
if(EXISTS "${MST27_SOURCE_DIR}/CMakeLists.txt")
  add_subdirectory("${MST27_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/mst27_cpp")
endif()

catkin_install_python(PROGRAMS
  scripts/scheduler_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
