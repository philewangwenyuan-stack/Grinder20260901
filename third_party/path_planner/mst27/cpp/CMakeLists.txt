# Built by grinder_scheduler's catkin_make invocation.
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_mst27_cpp mst27_cpp.cpp)
target_compile_features(_mst27_cpp PRIVATE cxx_std_17)

# Keep the extension next to mst27.py so the dynamically loaded planner can
# import it without a PYTHONPATH change.
set(_mst27_output_dir "${CMAKE_CURRENT_LIST_DIR}/..")
set_target_properties(_mst27_cpp PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${_mst27_output_dir}"
  RUNTIME_OUTPUT_DIRECTORY "${_mst27_output_dir}"
)

foreach(_CONFIG Debug Release RelWithDebInfo MinSizeRel)
  string(TOUPPER "${_CONFIG}" _CONFIG_UPPER)
  set_target_properties(_mst27_cpp PROPERTIES
    "LIBRARY_OUTPUT_DIRECTORY_${_CONFIG_UPPER}" "${_mst27_output_dir}"
    "RUNTIME_OUTPUT_DIRECTORY_${_CONFIG_UPPER}" "${_mst27_output_dir}"
  )
endforeach()

