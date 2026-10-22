cmake_minimum_required(VERSION 3.14)
project(rclpy_agnocast_codegen)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY cmake resource DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rclpy_agnocast_codegen
  DESTINATION share/${PROJECT_NAME}/generator FILES_MATCHING PATTERN "*.py")
install(FILES resource/generate.py DESTINATION share/${PROJECT_NAME}/generator
  RENAME __main__.py)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_generator test TIMEOUT 180)
endif()

ament_package(CONFIG_EXTRAS "cmake/rclpy_agnocast_codegen-extras.cmake")
