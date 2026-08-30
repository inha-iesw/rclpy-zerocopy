cmake_minimum_required(VERSION 3.14)
project(phase2_agnocast_probe)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(agnocastlib REQUIRED)
find_package(agnocast_sample_interfaces REQUIRED)

add_executable(phase2_probe
  src/probe_common.cpp
  src/probe_main.cpp
  src/publisher_mode.cpp
  src/subscriber_mode.cpp
  src/exhaust_mode.cpp
)
target_compile_features(phase2_probe PRIVATE cxx_std_17)
target_include_directories(phase2_probe PRIVATE
  include
  ${agnocastlib_INCLUDE_DIRS}
)
ament_target_dependencies(phase2_probe
  rclcpp
  agnocastlib
  agnocast_sample_interfaces
)

install(TARGETS phase2_probe
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/run_characterization.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES scripts/probe_runtime.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_probe_runtime test/test_probe_runtime.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
endif()

ament_package()
