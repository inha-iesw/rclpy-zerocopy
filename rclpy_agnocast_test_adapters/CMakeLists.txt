cmake_minimum_required(VERSION 3.14)
project(rclpy_agnocast_test_adapters)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy_agnocast_codegen REQUIRED)

ament_python_install_package(${PROJECT_NAME})
rclpy_agnocast_generate_adapters(_types TYPES
  rclpy_agnocast_test_msgs/msg/Scalars
  rclpy_agnocast_test_msgs/msg/Containers
  rclpy_agnocast_test_msgs/msg/Nested
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_installed_provider test/test_installed_provider.py TIMEOUT 60)
endif()
ament_package()
