cmake_minimum_required(VERSION 3.14)
project(rclpy_agnocast)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(agnocastlib REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 CONFIG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy_agnocast_msgs REQUIRED)
find_package(Threads REQUIRED)

include(cmake/adapter_identity.cmake)
_rclpy_agnocast_headers_digest(types_headers_digest
  "${CMAKE_CURRENT_SOURCE_DIR}/include/rclpy_agnocast/types")
_rclpy_agnocast_selected_headers_digest(agnocast_headers_digest)

add_library(rclpy_agnocast_types SHARED src/types/registry.cpp src/types/operations.cpp)
target_compile_features(rclpy_agnocast_types PUBLIC cxx_std_17)
target_include_directories(rclpy_agnocast_types PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(rclpy_agnocast_types PUBLIC
  RCLPY_AGNOCAST_ROS_ABI="${rclcpp_VERSION}:$ENV{ROS_DISTRO}"
  RCLPY_AGNOCAST_TYPES_HEADERS_SHA256="${types_headers_digest}"
  RCLPY_AGNOCAST_RUNTIME_AGNOCAST_HEADERS_SHA256="${agnocast_headers_digest}"
)
target_link_libraries(rclpy_agnocast_types PRIVATE Threads::Threads ${CMAKE_DL_LIBS})
install(TARGETS rclpy_agnocast_types EXPORT rclpy_agnocast_types_targets
  LIBRARY DESTINATION lib
)
ament_export_targets(rclpy_agnocast_types_targets HAS_LIBRARY_TARGET)

add_library(rclpy_agnocast_core STATIC
  src/loan.cpp
  src/publisher_worker.cpp
  src/subscriber/lease.cpp
  src/subscriber/queue.cpp
  src/subscriber/provenance.cpp
  src/types/root.cpp
  src/types/lease_root.cpp
  src/tensor/export.cpp
)
target_compile_features(rclpy_agnocast_core PUBLIC cxx_std_17)
target_include_directories(rclpy_agnocast_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(rclpy_agnocast_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
target_link_libraries(rclpy_agnocast_core PUBLIC Threads::Threads rclpy_agnocast_types)

add_library(rclpy_agnocast_native STATIC
  src/buffer_owner.cpp
  src/runtime.cpp
  src/subscriber/buffer_owner.cpp
  src/subscriber/executor.cpp
  src/subscriber/native.cpp
  src/types/native_publisher.cpp
)
target_compile_features(rclpy_agnocast_native PUBLIC cxx_std_17)
target_include_directories(rclpy_agnocast_native PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(rclpy_agnocast_native PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
target_link_libraries(rclpy_agnocast_native PUBLIC
  agnocastlib::agnocast
  Python3::Module
  rclcpp::rclcpp
  rclpy_agnocast_core
  rclpy_agnocast_msgs::rclpy_agnocast_msgs__rosidl_generator_cpp
)

pybind11_add_module(_native MODULE
  src/bindings.cpp
  src/bind_publisher.cpp
  src/subscriber/bindings.cpp
  src/types/bindings.cpp
  src/types/bind_root.cpp
  src/types/bind_lease_root.cpp
  src/tensor/bindings.cpp
)
target_compile_features(_native PRIVATE cxx_std_17)
target_link_libraries(_native PRIVATE rclpy_agnocast_native rclpy_agnocast_types)
target_compile_definitions(_native PRIVATE
  $<$<BOOL:${BUILD_TESTING}>:BUILD_TESTING>
)

ament_get_python_install_dir(PYTHON_INSTALL_DIR)
ament_python_install_package(${PROJECT_NAME})
install(TARGETS _native
  LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_type_registry test/types/test_registry.cpp)
  target_link_libraries(test_type_registry rclpy_agnocast_types)
  ament_add_gtest(test_field_ops test/types/test_field_ops.cpp)
  target_link_libraries(test_field_ops rclpy_agnocast_types)
  ament_add_gtest(test_type_root test/types/test_root.cpp)
  target_include_directories(test_type_root PRIVATE src)
  target_link_libraries(test_type_root rclpy_agnocast_core)
  ament_add_gtest(test_tensor_export test/tensor/test_export.cpp)
  target_include_directories(test_tensor_export PRIVATE src)
  target_link_libraries(test_tensor_export rclpy_agnocast_core)
  add_library(test_type_provider MODULE test/types/provider_library.cpp)
  target_link_libraries(test_type_provider PRIVATE rclpy_agnocast_types)
  set_target_properties(test_type_provider PROPERTIES
    CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN YES)
  ament_add_gtest(test_provider_lifetime test/types/test_provider_lifetime.cpp)
  target_link_libraries(test_provider_lifetime rclpy_agnocast_types ${CMAKE_DL_LIBS})
  target_compile_definitions(test_provider_lifetime PRIVATE
    TEST_PROVIDER_LIBRARY="$<TARGET_FILE:test_type_provider>")
  add_dependencies(test_provider_lifetime test_type_provider)
  ament_add_pytest_test(test_type_discovery test/types/test_discovery.py TIMEOUT 60)
  ament_add_pytest_test(test_type_native_abi test/types/test_native_abi.py TIMEOUT 60)
  ament_add_pytest_test(test_type_bindings test/types/test_bindings.py TIMEOUT 60)
  ament_add_pytest_test(test_type_lease_bindings test/types/test_lease_bindings.py TIMEOUT 60)
  ament_add_pytest_test(test_cpu_tensor test/tensor/test_cpu_tensor.py TIMEOUT 120)
  ament_add_pytest_test(test_cpu_tensor_readonly test/tensor/test_readonly.py TIMEOUT 120)
  ament_add_pytest_test(test_cpu_tensor_system test/tensor/test_system.py
    ENV "RCLPY_AGNOCAST_SUBSCRIBER=$<TARGET_FILE:rclpy_agnocast_shared_buffer_subscriber>"
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test"
    TIMEOUT 300)
  set_tests_properties(test_cpu_tensor_system PROPERTIES
    LABELS "pytest;system;requires_kernel_module;requires_heaphook")
  ament_add_pytest_test(test_compat_node test/compat/test_node.py TIMEOUT 60)
  ament_add_pytest_test(test_typed_subscriber_api test/subscriber/test_typed_api.py TIMEOUT 60)
  ament_add_pytest_test(test_typed_subscriber_dispatch test/subscriber/test_typed_dispatch.py
    TIMEOUT 120)

  target_compile_definitions(rclpy_agnocast_core PUBLIC BUILD_TESTING)
  target_sources(rclpy_agnocast_core PRIVATE src/publisher/observation.cpp)
  target_link_libraries(rclpy_agnocast_core PUBLIC ${CMAKE_DL_LIBS})
  target_sources(_native PRIVATE src/publisher/bind_observation.cpp)
  target_sources(_native PRIVATE src/subscriber/bind_testing.cpp)
  target_sources(_native PRIVATE src/types/bind_testing.cpp)
  target_sources(_native PRIVATE src/tensor/bind_testing.cpp)

  ament_add_pytest_test(test_subscriber_lease test/subscriber/test_lease.py TIMEOUT 60)
  ament_add_pytest_test(test_subscriber_dispatch test/subscriber/test_dispatch.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test"
    TIMEOUT 60
  )
  ament_add_pytest_test(test_subscriber_api test/subscriber/test_api.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test"
    TIMEOUT 60
  )
  ament_add_pytest_test(test_subscriber_shutdown test/subscriber/test_shutdown.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test"
    TIMEOUT 120
  )
  ament_add_pytest_test(test_subscriber_fairness test/subscriber/test_fairness.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test"
    TIMEOUT 60
  )
  ament_add_gtest(test_subscriber_queue test/subscriber/test_queue.cpp TIMEOUT 60)
  target_include_directories(test_subscriber_queue PRIVATE src)
  target_link_libraries(test_subscriber_queue rclpy_agnocast_core)
  ament_add_pytest_test(test_subscriber_system test/subscriber/test_system.py
    ENV "RCLPY_AGNOCAST_SUBSCRIBER=$<TARGET_FILE:rclpy_agnocast_shared_buffer_subscriber>"
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test"
    TIMEOUT 180
  )
  set_tests_properties(test_subscriber_system PROPERTIES
    LABELS "pytest;system;requires_kernel_module;requires_heaphook")
  ament_add_pytest_test(test_subscriber_guard_system test/subscriber/test_guard_system.py
    ENV "RCLPY_AGNOCAST_SUBSCRIBER=$<TARGET_FILE:rclpy_agnocast_shared_buffer_subscriber>"
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test"
    TIMEOUT 180
  )
  set_tests_properties(test_subscriber_guard_system PROPERTIES
    LABELS "pytest;system;requires_kernel_module;requires_heaphook")
  ament_add_pytest_test(test_cpu_checkpoint test/checkpoint
    ENV "RCLPY_AGNOCAST_SUBSCRIBER=$<TARGET_FILE:rclpy_agnocast_shared_buffer_subscriber>"
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test"
    TIMEOUT 240
  )
  set_tests_properties(test_cpu_checkpoint PROPERTIES
    LABELS "pytest;system;requires_kernel_module;requires_heaphook")

  ament_add_gtest(test_publisher_observation test/publisher/test_observation.cpp)
  target_include_directories(test_publisher_observation PRIVATE src)
  target_link_libraries(test_publisher_observation rclpy_agnocast_core)

  add_executable(rclpy_agnocast_shared_buffer_subscriber
    test/support/shared_buffer_subscriber.cpp
  )
  target_compile_features(rclpy_agnocast_shared_buffer_subscriber PRIVATE cxx_std_17)
  target_link_libraries(rclpy_agnocast_shared_buffer_subscriber PRIVATE
    agnocastlib::agnocast
    rclcpp::rclcpp
    rclpy_agnocast_msgs::rclpy_agnocast_msgs__rosidl_generator_cpp
    Threads::Threads
  )

  ament_add_gtest(test_loan_state
    test/test_loan_state.cpp
  )
  if(TARGET test_loan_state)
    target_link_libraries(test_loan_state rclpy_agnocast_core)
  endif()

  ament_add_gtest(test_publisher_worker
    test/test_publisher_worker.cpp
  )
  if(TARGET test_publisher_worker)
    target_link_libraries(test_publisher_worker rclpy_agnocast_core)
  endif()

  ament_add_pytest_test(test_publisher_buffer
    test/test_publisher_buffer.py
    TIMEOUT 60
  )
  ament_add_pytest_test(test_python_lifecycle
    test/test_python_lifecycle.py
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test"
    TIMEOUT 60
  )

  ament_add_pytest_test(test_publisher_system
    test/test_publisher_system.py
    ENV
      "RCLPY_AGNOCAST_SUBSCRIBER=$<TARGET_FILE:rclpy_agnocast_shared_buffer_subscriber>"
    TIMEOUT 180
  )
  if(TEST test_publisher_system)
    set_tests_properties(test_publisher_system PROPERTIES
      LABELS "pytest;system;requires_kernel_module;requires_heaphook"
    )
  endif()

  ament_add_pytest_test(test_runtime_coexistence
    test/test_runtime_coexistence.py
    TIMEOUT 180
  )
  if(TEST test_runtime_coexistence)
    set_tests_properties(test_runtime_coexistence PROPERTIES
      LABELS "pytest;system;requires_kernel_module;requires_heaphook"
    )
  endif()
endif()

install(DIRECTORY include/rclpy_agnocast/types/
  DESTINATION include/rclpy_agnocast/types
)

ament_package(CONFIG_EXTRAS "cmake/adapter_identity.cmake")
