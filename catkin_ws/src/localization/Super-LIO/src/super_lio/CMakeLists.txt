cmake_minimum_required(VERSION 3.16)
project(super_lio)


#DEBUG CONFIGURATION
# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -pthread -fsanitize=address")

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -pthread -fexceptions -flto=auto")
add_definitions(-DROOT=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Find packages
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  roscpp
  rospy
  basic
  pcl_ros
  std_msgs
  geometry_msgs
  livox_ros_driver2
  sensor_msgs
  visualization_msgs
  nav_msgs
  message_generation
)
find_package(PythonLibs REQUIRED)

add_message_files(
  FILES
  CloudPose.msg
  CloudPose2.msg
  DoubleCloudPose.msg
)

add_service_files(
  FILES
  GetMap.srv
  SetMap.srv
  SetInitState.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

# Include directories for libraries
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  include
  3rdparty
)

# Package setup for catkin
catkin_package(
  CATKIN_DEPENDS basic diagnostic_msgs geometry_msgs livox_ros_driver2 nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
)

# LIO library source files
file(GLOB_RECURSE LIO_LIB_SRC "src/lio/*.cpp")

file(GLOB_RECURSE LIO_LIB_HEADERS "include/lio/*.hpp" "include/lio/*.h")

# library build
add_library(lio ${LIO_LIB_SRC} ${LIO_LIB_HEADERS})
add_dependencies(lio
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Target specific includes
target_include_directories(lio PUBLIC
  include
  3rdparty
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Linking libraries
target_link_libraries(lio
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PYTHON_LIBRARIES}
  tbb 
  gflags 
  glog
)


add_executable(super_lio_node src/apps/super_lio_node.cpp)
add_dependencies(super_lio_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(super_lio_node
  lio
  ${catkin_LIBRARIES}
)


add_executable(relocation_node src/apps/relocation_node.cpp src/apps/relocation.cpp)
add_dependencies(relocation_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(relocation_node
  lio
  ${catkin_LIBRARIES}
)


catkin_install_python(PROGRAMS
  scripts/monitor_hud.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
