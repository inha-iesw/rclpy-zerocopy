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

add_library(rclpy_agnocast_types SHARED src/types/registry.cpp)
target_compile_features(rclpy_agnocast_types PUBLIC cxx_std_17)
target_include_directories(rclpy_agnocast_types PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(rclpy_agnocast_types PUBLIC
  RCLPY_AGNOCAST_ROS_ABI="${rclcpp_VERSION}:$ENV{ROS_DISTRO}"
)
target_link_libraries(rclpy_agnocast_types PRIVATE Threads::Threads)
install(TARGETS rclpy_agnocast_types EXPORT rclpy_agnocast_types_targets
  LIBRARY DESTINATION lib
)
ament_export_targets(rclpy_agnocast_types_targets HAS_LIBRARY_TARGET)

add_library(rclpy_agnocast_core STATIC
  src/loan.cpp
  src/publisher_worker.cpp
  src/subscriber/lease.cpp
  src/subscriber/queue.cpp
)
target_compile_features(rclpy_agnocast_core PUBLIC cxx_std_17)
target_include_directories(rclpy_agnocast_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(rclpy_agnocast_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
target_link_libraries(rclpy_agnocast_core PUBLIC Threads::Threads)

add_library(rclpy_agnocast_native STATIC
  src/buffer_owner.cpp
  src/runtime.cpp
  src/subscriber/buffer_owner.cpp
  src/subscriber/executor.cpp
  src/subscriber/native.cpp
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
  ament_add_pytest_test(test_type_native_abi test/types/test_native_abi.py TIMEOUT 60)

  target_compile_definitions(rclpy_agnocast_core PUBLIC BUILD_TESTING)
  target_sources(rclpy_agnocast_core PRIVATE src/publisher/observation.cpp)
  target_link_libraries(rclpy_agnocast_core PUBLIC ${CMAKE_DL_LIBS})
  target_sources(_native PRIVATE src/publisher/bind_observation.cpp)
  target_sources(_native PRIVATE src/subscriber/bind_testing.cpp)

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

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
