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

add_library(rclpy_agnocast_core STATIC
  src/loan.cpp
  src/publisher_worker.cpp
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
)
target_compile_features(_native PRIVATE cxx_std_17)
target_link_libraries(_native PRIVATE rclpy_agnocast_native)
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
