cmake_minimum_required(VERSION 3.16)
project(mst27 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_mst27_cpp cpp/mst27_cpp.cpp)
target_compile_features(_mst27_cpp PRIVATE cxx_std_17)

# Keep the extension next to mst27.py.  The Python planner prepends its own
# directory to sys.path, so it can import this module without ROS changes.
set_target_properties(_mst27_cpp PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)

foreach(_CONFIG Debug Release RelWithDebInfo MinSizeRel)
  string(TOUPPER "${_CONFIG}" _CONFIG_UPPER)
  set_target_properties(_mst27_cpp PROPERTIES
    "LIBRARY_OUTPUT_DIRECTORY_${_CONFIG_UPPER}" "${CMAKE_CURRENT_SOURCE_DIR}"
    "RUNTIME_OUTPUT_DIRECTORY_${_CONFIG_UPPER}" "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endforeach()

