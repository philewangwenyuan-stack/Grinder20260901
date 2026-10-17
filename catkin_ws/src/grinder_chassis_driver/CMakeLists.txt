cmake_minimum_required(VERSION 3.0.2)
project(grinder_chassis_driver)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  diagnostic_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  std_msgs
  std_srvs
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
  INCLUDE_DIRS include
  LIBRARIES grinder_chassis_core
  CATKIN_DEPENDS diagnostic_msgs geometry_msgs message_runtime nav_msgs roscpp rospy std_msgs std_srvs
)

add_library(grinder_chassis_core
  src/core.cpp
  src/emergency_record.cpp
  src/modbus_transport.cpp
  src/chassis_controller.cpp
)
target_include_directories(grinder_chassis_core PUBLIC include ${JSONCPP_INCLUDE_DIRS})
target_compile_options(grinder_chassis_core PRIVATE -Wall -Wextra -Wpedantic ${JSONCPP_CFLAGS_OTHER})
target_link_libraries(grinder_chassis_core PUBLIC ${JSONCPP_LIBRARIES} Threads::Threads)

add_executable(chassis_driver_node src/chassis_driver_node.cpp)
target_include_directories(chassis_driver_node PRIVATE include ${catkin_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS})
target_compile_options(chassis_driver_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(chassis_driver_node grinder_chassis_core ${catkin_LIBRARIES})
add_dependencies(chassis_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(grinder_chassis_core_test
    test/test_core.cpp
    test/test_emergency_record.cpp
    test/test_controller.cpp
    test/test_modbus_transport.cpp
  )
  if(TARGET grinder_chassis_core_test)
    target_link_libraries(grinder_chassis_core_test grinder_chassis_core gtest_main util)
  endif()
  find_package(rostest REQUIRED)
  catkin_install_python(PROGRAMS test/ros_driver_fixture.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/chassis_driver.test DEPENDENCIES chassis_driver_node)
endif()

install(TARGETS grinder_chassis_core chassis_driver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
