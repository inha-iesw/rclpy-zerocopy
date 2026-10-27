# The lock is the source registry; target-specific seams stay in repair/CMakeLists.txt.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_LIST_DIR}/verify.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/lineage/upstream-tests.lock.json"
  "${CMAKE_CURRENT_SOURCE_DIR}/../upstream.lock.json")
execute_process(
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/verify.py"
    "${CMAKE_CURRENT_SOURCE_DIR}"
  RESULT_VARIABLE _lineage_status OUTPUT_VARIABLE _lineage_registry
  ERROR_VARIABLE _lineage_error)
if(NOT _lineage_status EQUAL 0)
  message(FATAL_ERROR "Upstream test lineage validation failed: ${_lineage_error}")
endif()
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/upstream-lineage.cmake" "${_lineage_registry}")
include("${CMAKE_CURRENT_BINARY_DIR}/upstream-lineage.cmake")

foreach(_target IN LISTS G3_UPSTREAM_TESTS)
  ament_add_gtest(${_target} "${G3_UPSTREAM_SOURCE_${_target}}" TIMEOUT 30)
  target_compile_features(${_target} PRIVATE cxx_std_17)
  target_link_libraries(${_target}
    ros2_cuda_ipc_core::ros2_cuda_ipc_core CUDA::cudart)
endforeach()
foreach(_target IN LISTS G3_UPSTREAM_HELPERS)
  add_executable(${_target} "${G3_UPSTREAM_SOURCE_${_target}}")
  target_compile_features(${_target} PRIVATE cxx_std_17)
  target_link_libraries(${_target} ros2_cuda_ipc_core::ros2_cuda_ipc_core)
endforeach()
