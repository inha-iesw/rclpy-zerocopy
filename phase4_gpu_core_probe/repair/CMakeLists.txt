ament_add_gtest(test_repaired_reservations test/test_reservations.cpp TIMEOUT 20)
target_compile_features(test_repaired_reservations PRIVATE cxx_std_17)
target_link_libraries(test_repaired_reservations
  ros2_cuda_ipc_core::ros2_cuda_ipc_core)

ament_add_gtest(test_repaired_producer test/test_producer.cpp TIMEOUT 40)
target_compile_features(test_repaired_producer PRIVATE cxx_std_17)
target_link_libraries(test_repaired_producer
  ros2_cuda_ipc_core::ros2_cuda_ipc_core CUDA::cudart)

ament_add_gtest(test_repaired_quarantine test/test_quarantine.cpp TIMEOUT 20)
target_compile_features(test_repaired_quarantine PRIVATE cxx_std_17)
target_link_libraries(test_repaired_quarantine
  ros2_cuda_ipc_core::ros2_cuda_ipc_core CUDA::cudart ${CMAKE_DL_LIBS})
target_link_options(test_repaired_quarantine PRIVATE "-Wl,--export-dynamic-symbol=cuEventRecord")

ament_add_gtest(test_repaired_contexts test/test_contexts.cpp TIMEOUT 20)
target_compile_features(test_repaired_contexts PRIVATE cxx_std_17)
target_link_libraries(test_repaired_contexts
  ros2_cuda_ipc_core::ros2_cuda_ipc_core CUDA::cudart CUDA::cuda_driver)

ament_add_gtest(test_repaired_primary_release test/test_primary_release.cpp TIMEOUT 20)
target_compile_features(test_repaired_primary_release PRIVATE cxx_std_17)
target_link_libraries(test_repaired_primary_release
  ros2_cuda_ipc_core::ros2_cuda_ipc_core CUDA::cudart ${CMAKE_DL_LIBS})
target_link_options(test_repaired_primary_release PRIVATE
  "-Wl,--export-dynamic-symbol=cuDevicePrimaryCtxRelease_v2")

ament_add_gtest(test_pool_retirement test/lifetime/test_pool_retirement.cpp TIMEOUT 30)
target_compile_features(test_pool_retirement PRIVATE cxx_std_17)
target_link_libraries(test_pool_retirement
  ros2_cuda_ipc_core::ros2_cuda_ipc_core CUDA::cudart)

ament_add_gtest(test_legacy_release test/lifetime/test_legacy_release.cpp TIMEOUT 30)
target_compile_features(test_legacy_release PRIVATE cxx_std_17)
target_link_libraries(test_legacy_release
  ros2_cuda_ipc_core::ros2_cuda_ipc_core CUDA::cudart CUDA::cuda_driver)

# Reuse compatible upstream oracles against this repaired core. API/protocol
# migrations of manager/mapper/ref tests remain separate from these sources.
set(_upstream_tests
  "${CMAKE_CURRENT_SOURCE_DIR}/../../build/g3/repaired/ros2_cuda_ipc/ros2_cuda_ipc_core/test")
foreach(_name cuda_driver_context gpu_buffer_pool publisher_messages
    buffer_metadata_cache ipc_handle_cache read_handle)
  ament_add_gtest(test_upstream_${_name} "${_upstream_tests}/test_${_name}.cpp" TIMEOUT 30)
  target_compile_features(test_upstream_${_name} PRIVATE cxx_std_17)
  target_link_libraries(test_upstream_${_name}
    ros2_cuda_ipc_core::ros2_cuda_ipc_core CUDA::cudart)
endforeach()

find_package(rclpy_agnocast_gpu_msgs REQUIRED)
find_package(agnocastlib REQUIRED)
find_package(rclcpp REQUIRED)

ament_add_gtest(test_gpu_descriptor test/test_descriptor.cpp TIMEOUT 10)
target_compile_features(test_gpu_descriptor PRIVATE cxx_std_17)
target_link_libraries(test_gpu_descriptor ${rclpy_agnocast_gpu_msgs_TARGETS})

enable_language(CUDA)
add_executable(g3_transport_peer transport/peer.cpp transport/checksum.cu)
target_compile_features(g3_transport_peer PRIVATE cxx_std_17)
set_target_properties(g3_transport_peer PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED ON)
target_link_libraries(g3_transport_peer
  ros2_cuda_ipc_core::ros2_cuda_ipc_core CUDA::cudart
  ${rclpy_agnocast_gpu_msgs_TARGETS})
ament_target_dependencies(g3_transport_peer agnocastlib rclcpp)

ament_add_pytest_test(test_repaired_transport test/test_transport.py TIMEOUT 120
  ENV G3_TRANSPORT_PEER=$<TARGET_FILE:g3_transport_peer>)
set_tests_properties(test_repaired_transport PROPERTIES LABELS "requires_kernel_module;cuda")
