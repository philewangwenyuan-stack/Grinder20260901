cmake_minimum_required(VERSION 3.0.2)
project(cloud_to_occupancy_grid)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  super_lio_loop
  tf2
  tf2_ros
)
find_package(Boost REQUIRED COMPONENTS filesystem system)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs std_srvs super_lio_loop tf2 tf2_ros
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(cloud_to_occupancy_grid_node src/cloud_to_occupancy_grid_node.cpp)
add_dependencies(cloud_to_occupancy_grid_node ${catkin_EXPORTED_TARGETS})
target_compile_features(cloud_to_occupancy_grid_node PRIVATE cxx_std_14)
target_link_libraries(cloud_to_occupancy_grid_node
  ${catkin_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
)

install(TARGETS cloud_to_occupancy_grid_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
