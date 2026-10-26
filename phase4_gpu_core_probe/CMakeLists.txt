cmake_minimum_required(VERSION 3.16)
project(phase4_gpu_core_probe)

find_package(ament_cmake REQUIRED)
option(G3_REPAIRED_CORE "Build the repaired-core safety tests instead of rejection diagnostics" OFF)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ros2_cuda_ipc_core REQUIRED)
  find_package(CUDAToolkit REQUIRED)

  if(G3_REPAIRED_CORE)
    add_subdirectory(repair)
  else()
    # These tests reproduce defects in the pinned candidate. Passing them is
    # evidence for rejection, not a GPU adoption or safety gate PASS.
    ament_add_gtest(test_producer_reuse test/producer_reuse.cpp TIMEOUT 30)
    target_compile_features(test_producer_reuse PRIVATE cxx_std_17)
    target_link_libraries(test_producer_reuse
      ros2_cuda_ipc_core::ros2_cuda_ipc_core CUDA::cudart)

    ament_add_gtest(test_metadata_lifecycle test/metadata_lifecycle.cpp TIMEOUT 10)
    target_compile_features(test_metadata_lifecycle PRIVATE cxx_std_17)
    target_link_libraries(test_metadata_lifecycle
      ros2_cuda_ipc_core::ros2_cuda_ipc_core)

    ament_add_pytest_test(test_upstream_fanout test/test_upstream_fanout.py TIMEOUT 60)
  endif()
endif()

ament_package()
