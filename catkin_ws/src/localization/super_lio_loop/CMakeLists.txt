cmake_minimum_required(VERSION 3.0.2)
project(super_lio_loop)

option(SUPER_LIO_REQUIRE_INDUSTRIAL_BACKEND
  "Fail configuration when GTSAM is unavailable" OFF)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  diagnostic_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
  message_generation
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters registration)
find_package(GTSAM 4.2 QUIET)
find_package(fast_gicp QUIET)

add_service_files(FILES GetKeyframe.srv)
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS
  diagnostic_msgs geometry_msgs nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_msgs
  tf2 tf2_ros visualization_msgs message_runtime
)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(super_lio_loop_node src/super_lio_loop_node.cpp)
add_dependencies(super_lio_loop_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_features(super_lio_loop_node PRIVATE cxx_std_14)
target_link_libraries(super_lio_loop_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

if(GTSAM_FOUND)
  add_executable(super_lio_graph_backend_node src/super_lio_graph_backend_node.cpp)
  add_dependencies(super_lio_graph_backend_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_compile_features(super_lio_graph_backend_node PRIVATE cxx_std_14)
  target_include_directories(super_lio_graph_backend_node PRIVATE ${GTSAM_INCLUDE_DIR})
  target_link_libraries(super_lio_graph_backend_node
    ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)
  if(fast_gicp_FOUND)
    target_compile_definitions(super_lio_graph_backend_node PRIVATE SUPER_LIO_HAS_FAST_GICP=1)
    target_include_directories(super_lio_graph_backend_node PRIVATE ${fast_gicp_INCLUDE_DIRS})
    target_link_libraries(super_lio_graph_backend_node ${fast_gicp_LIBRARIES})
    message(STATUS "super_lio_loop: industrial backend uses FastVGICP")
  else()
    message(WARNING "super_lio_loop: fast_gicp not found; industrial backend will use slower PCL GICP")
  endif()
else()
  if(SUPER_LIO_REQUIRE_INDUSTRIAL_BACKEND)
    message(FATAL_ERROR
      "GTSAM 4.2 is required for the industrial loop backend. Run catkin_ws/install_super_lio_loop_deps.sh")
  endif()
  message(WARNING "super_lio_loop: GTSAM 4.2 not found; only the lightweight fallback will be built")
endif()

set(SUPER_LIO_LOOP_TARGETS super_lio_loop_node)
if(TARGET super_lio_graph_backend_node)
  list(APPEND SUPER_LIO_LOOP_TARGETS super_lio_graph_backend_node)
endif()
install(TARGETS ${SUPER_LIO_LOOP_TARGETS}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
