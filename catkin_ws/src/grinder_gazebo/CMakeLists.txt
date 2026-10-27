cmake_minimum_required(VERSION 3.0.2)
project(grinder_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  grinder_chassis_driver
  nav_msgs
  livox_ros_driver2
  roscpp
  rospy
  sensor_msgs
  slamware_ros_sdk
  std_msgs
  std_srvs
  tf2_ros
  xacro
)
find_package(gazebo REQUIRED)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    gazebo_ros
    geometry_msgs
    grinder_chassis_driver
    nav_msgs
    livox_ros_driver2
    roscpp
    rospy
    sensor_msgs
    slamware_ros_sdk
    std_msgs
    std_srvs
    tf2_ros
    xacro
)

add_library(grinder_mid360_sensor SHARED src/mid360_sensor_plugin.cpp)
target_compile_features(grinder_mid360_sensor PRIVATE cxx_std_14)
target_include_directories(grinder_mid360_sensor PRIVATE ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(grinder_mid360_sensor ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(grinder_mid360_sensor ${catkin_EXPORTED_TARGETS})
install(TARGETS grinder_mid360_sensor LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

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

