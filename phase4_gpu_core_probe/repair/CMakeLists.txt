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

ament_add_gtest_executable(test_metadata_collision test/lifetime/test_metadata_collision.cpp
  test/cleanup/isolated_main.cpp SKIP_LINKING_MAIN_LIBRARIES)
target_compile_features(test_metadata_collision PRIVATE cxx_std_17)
target_link_libraries(test_metadata_collision
  ros2_cuda_ipc_core::ros2_cuda_ipc_core CUDA::cudart)
foreach(_case First Second)
  set(_test "metadata_collision_${_case}")
  set(_xml "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${_test}.gtest.xml")
  ament_add_test(${_test}
    COMMAND $<TARGET_FILE:test_metadata_collision>
      "--gtest_filter=Collision/RepairedMetadataCollision.ExistingNameSurvivesFailedInitialisation/${_case}"
      "--gtest_output=xml:${_xml}"
    RESULT_FILE "${_xml}"
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_cmake_gtest/${_test}.txt" TIMEOUT 20)
endforeach()

ament_add_gtest(test_mapper_shutdown test/lifetime/test_mapper_shutdown.cpp TIMEOUT 20)
target_compile_features(test_mapper_shutdown PRIVATE cxx_std_17)
target_include_directories(test_mapper_shutdown PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/../../build/g3/repaired/ros2_cuda_ipc/ros2_cuda_ipc_core/src")
target_link_libraries(test_mapper_shutdown ros2_cuda_ipc_core::ros2_cuda_ipc_core)

# Validate ancestry and register original/adapted sources from one tracked inventory.
set(_upstream_tests
  "${CMAKE_CURRENT_SOURCE_DIR}/../../build/g3/repaired/ros2_cuda_ipc/ros2_cuda_ipc_core/test")
include(lineage/register.cmake)
target_link_libraries(test_upstream_buffer_mapper ${CMAKE_DL_LIBS})
target_link_options(test_upstream_buffer_mapper PRIVATE "-Wl,--export-dynamic-symbol=connect")
target_link_options(test_upstream_buffer_ref PRIVATE
  "-Wl,--wrap=_ZNSt6chrono3_V212steady_clock3nowEv")
target_link_options(test_upstream_gpu_buffer_manager PRIVATE
  "-Wl,--wrap=_ZNSt6chrono3_V212steady_clock3nowEv")

target_include_directories(g3_vmm_fd_import_helper PRIVATE "${_upstream_tests}")
target_include_directories(test_upstream_vmm_fd_memory_driver PRIVATE "${_upstream_tests}")
target_compile_definitions(test_upstream_vmm_fd_memory_driver PRIVATE
  VMM_FD_IMPORT_HELPER_PATH="$<TARGET_FILE:g3_vmm_fd_import_helper>")
add_dependencies(test_upstream_vmm_fd_memory_driver g3_vmm_fd_import_helper)

find_package(rclpy_agnocast_gpu_msgs REQUIRED)
find_package(agnocastlib REQUIRED)
find_package(rclcpp REQUIRED)

ament_add_gtest(test_gpu_descriptor test/test_descriptor.cpp TIMEOUT 10)
target_compile_features(test_gpu_descriptor PRIVATE cxx_std_17)
target_link_libraries(test_gpu_descriptor ${rclpy_agnocast_gpu_msgs_TARGETS})

enable_language(CUDA)
add_executable(g3_transport_peer transport/peer.cpp transport/checksum.cu)
add_executable(g3_cleanup_peer transport/peer.cpp transport/checksum.cu
  test/cleanup/import_failure.cpp test/transport/fd_reply_failure.cpp)
foreach(_peer g3_transport_peer g3_cleanup_peer)
  target_compile_features(${_peer} PRIVATE cxx_std_17)
  set_target_properties(${_peer} PROPERTIES CUDA_STANDARD 17 CUDA_STANDARD_REQUIRED ON)
  target_link_libraries(${_peer}
    ros2_cuda_ipc_core::ros2_cuda_ipc_core CUDA::cudart
    ${rclpy_agnocast_gpu_msgs_TARGETS})
  ament_target_dependencies(${_peer} agnocastlib rclcpp)
endforeach()

target_compile_definitions(g3_cleanup_peer PRIVATE G3_CLEANUP_INJECTION)
target_link_libraries(g3_cleanup_peer ${CMAKE_DL_LIBS})
foreach(_symbol cuIpcOpenEventHandle cuMemUnmap cuMemAddressFree cuMemRelease
    cuEventDestroy_v2 cuCtxPopCurrent_v2 cuMemImportFromShareableHandle cuDevicePrimaryCtxRetain
    cuCtxPushCurrent_v2 cuDevicePrimaryCtxRelease_v2 cuMemGetAllocationGranularity
    cuMemAddressReserve cuMemMap cuMemSetAccess cuEventCreate cuStreamWaitEvent
    connect recvmsg sendmsg close)
  target_link_options(g3_cleanup_peer PRIVATE "-Wl,--export-dynamic-symbol=${_symbol}")
endforeach()

ament_add_pytest_test(test_repaired_transport test/test_transport.py TIMEOUT 120
  ENV G3_TRANSPORT_PEER=$<TARGET_FILE:g3_transport_peer>
    G3_CLEANUP_PEER=$<TARGET_FILE:g3_cleanup_peer>)
set_tests_properties(test_repaired_transport PROPERTIES LABELS "requires_kernel_module;cuda")

# Sticky cleanup failures must each run in a fresh process.
ament_add_pytest_test(test_isolated_runner test/cleanup/test_isolated_runner.py TIMEOUT 30
  ENV G3_PRODUCER_CLEANUP=$<TARGET_FILE:test_producer_cleanup>
    G3_SUBSCRIBER_CLEANUP=$<TARGET_FILE:test_subscriber_cleanup>)

ament_add_gtest_executable(test_producer_cleanup
  test/cleanup/producer_cleanup.cpp test/cleanup/producer_failure.cpp
  test/cleanup/isolated_main.cpp SKIP_LINKING_MAIN_LIBRARIES)
target_compile_features(test_producer_cleanup PRIVATE cxx_std_17)
target_link_libraries(test_producer_cleanup
  ros2_cuda_ipc_core::ros2_cuda_ipc_core CUDA::cudart ${CMAKE_DL_LIBS})
foreach(_symbol cuEventDestroy_v2 cuMemUnmap cuMemAddressFree cuMemRelease
    cuEventQuery cuCtxPushCurrent_v2 cuCtxPopCurrent_v2 cuMemSetAccess
    cuMemCreate cuDevicePrimaryCtxRetain shm_open bind opendir)
  target_link_options(test_producer_cleanup PRIVATE "-Wl,--export-dynamic-symbol=${_symbol}")
endforeach()
foreach(_case EventDestroy MemUnmap MemAddressFree MemRelease EventQuery ContextPush
    RollbackMemUnmap RollbackMemAddressFree RollbackMemRelease
    FinalContextPop FinalContextPopMismatch)
  set(_test "producer_cleanup_${_case}")
  set(_xml "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${_test}.gtest.xml")
  ament_add_test(${_test}
    COMMAND $<TARGET_FILE:test_producer_cleanup>
      "--gtest_filter=Cleanup/RepairedProducerCleanup.FailurePoisonsEpoch/${_case}"
      "--gtest_output=xml:${_xml}"
    RESULT_FILE "${_xml}"
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_cmake_gtest/${_test}.txt" TIMEOUT 20)
endforeach()

ament_add_gtest_executable(test_producer_initialisation
  test/cleanup/producer_initialisation.cpp test/cleanup/producer_failure.cpp
  test/cleanup/isolated_main.cpp SKIP_LINKING_MAIN_LIBRARIES)
target_compile_features(test_producer_initialisation PRIVATE cxx_std_17)
target_link_libraries(test_producer_initialisation
  ros2_cuda_ipc_core::ros2_cuda_ipc_core CUDA::cudart ${CMAKE_DL_LIBS})
foreach(_symbol cuMemAddressReserve cuMemAddressFree cuMemCreate cuMemRelease
    cuMemMap cuMemUnmap cuMemSetAccess cuMemExportToShareableHandle
    cuEventCreate cuEventDestroy_v2 cuIpcGetEventHandle
    cuCtxPushCurrent_v2 cuCtxPopCurrent_v2 cuDevicePrimaryCtxRetain cuDevicePrimaryCtxRelease_v2
    shm_open socket bind listen)
  target_link_options(test_producer_initialisation PRIVATE "-Wl,--export-dynamic-symbol=${_symbol}")
endforeach()
foreach(_case MemAddressReserve MemCreate MemMap MemSetAccess MemExport EventCreate IpcGetEventHandle
    Socket Bind Listen EventContextPush PrimaryRetain AllocationContextPush)
  set(_test "producer_initialisation_${_case}")
  set(_xml "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${_test}.gtest.xml")
  if(_case MATCHES "^(PrimaryRetain|AllocationContextPush)$")
    set(_filter "Context/ProducerContextInitialisation.EarlyFailureReturnsEpochAndAllowsRetry/${_case}")
  else()
    set(_filter "Allocation/ProducerInitialisation.SecondBlockFailureRollsBackAndAllowsRetry/${_case}")
  endif()
  ament_add_test(${_test}
    COMMAND $<TARGET_FILE:test_producer_initialisation>
      "--gtest_filter=${_filter}"
      "--gtest_output=xml:${_xml}"
    RESULT_FILE "${_xml}"
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_cmake_gtest/${_test}.txt" TIMEOUT 20)
endforeach()


ament_add_gtest_executable(test_subscriber_cleanup
  test/cleanup/subscriber_cleanup.cpp test/cleanup/subscriber_failure.cpp
  test/cleanup/isolated_main.cpp SKIP_LINKING_MAIN_LIBRARIES)
target_compile_features(test_subscriber_cleanup PRIVATE cxx_std_17)
target_include_directories(test_subscriber_cleanup PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/../../build/g3/repaired/ros2_cuda_ipc/ros2_cuda_ipc_core/src")
target_link_libraries(test_subscriber_cleanup
  ros2_cuda_ipc_core::ros2_cuda_ipc_core CUDA::cudart ${CMAKE_DL_LIBS})
foreach(_symbol cuEventCreate cuEventRecord cuEventQuery cuEventDestroy_v2 cuStreamWaitEvent
    cuCtxPushCurrent_v2 cuCtxPopCurrent_v2)
  target_link_options(test_subscriber_cleanup PRIVATE "-Wl,--export-dynamic-symbol=${_symbol}")
endforeach()
foreach(_case Record Query CompletionDestroy BindWaitAndDestroy
    ReleaserSuccess WorkerSuccess ReleaserFailure WorkerFailure Create Wait)
  set(_test "subscriber_cleanup_${_case}")
  set(_xml "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${_test}.gtest.xml")
  if(_case MATCHES "^(Releaser|Worker)")
    set(_filter "PopOrigins/SubscriberContextPop.DrainWaitsForContextRestoration/${_case}")
  elseif(_case MATCHES "^(Create|Wait)$")
    set(_filter "Recoverable/SubscriberBindFailure.KeepsPublicationForRetryWhenCleanupSucceeds/${_case}")
  else()
    set(_filter "Failures/SubscriberCleanupFailure.RetainsReadAndRejectsReplacementMappers/${_case}")
  endif()
  ament_add_test(${_test}
    COMMAND $<TARGET_FILE:test_subscriber_cleanup>
      "--gtest_filter=${_filter}" "--gtest_output=xml:${_xml}"
    RESULT_FILE "${_xml}"
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_cmake_gtest/${_test}.txt" TIMEOUT 20)
endforeach()

ament_add_pytest_test(test_upstream_lineage lineage/test_verify.py TIMEOUT 60)

ament_add_gtest(test_fd_receive test/transport/test_fd_receive.cpp TIMEOUT 30)
target_compile_features(test_fd_receive PRIVATE cxx_std_17)
target_link_libraries(test_fd_receive
  ros2_cuda_ipc_core::ros2_cuda_ipc_core CUDA::cudart ${CMAKE_DL_LIBS})
foreach(_symbol recvmsg cuDevicePrimaryCtxRetain cuMemImportFromShareableHandle
    cuCtxPushCurrent_v2 cuCtxPopCurrent_v2 cuDevicePrimaryCtxRelease_v2)
  target_link_options(test_fd_receive PRIVATE "-Wl,--export-dynamic-symbol=${_symbol}")
endforeach()
