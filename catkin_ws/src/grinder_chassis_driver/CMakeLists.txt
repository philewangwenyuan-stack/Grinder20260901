cmake_minimum_required(VERSION 3.0.2)
project(grinder_chassis_driver)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  message_generation
  nav_msgs
  rospy
  std_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  WheelSpeedCommand.msg
  WheelSpeedState.msg
  ChassisStatus.msg
)

add_service_files(
  FILES
  EnableChassis.srv
  ClearFault.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS diagnostic_msgs message_runtime nav_msgs rospy std_msgs
)

catkin_install_python(PROGRAMS
  scripts/chassis_driver_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
