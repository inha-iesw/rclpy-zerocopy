cmake_minimum_required(VERSION 3.14)
project(rclpy_agnocast_msgs)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SharedBuffer.msg"
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_shared_buffer_type
    test/test_shared_buffer_type.cpp
  )
  if(TARGET test_shared_buffer_type)
    target_compile_features(test_shared_buffer_type PRIVATE cxx_std_17)

    rosidl_get_typesupport_target(cpp_typesupport_target
      ${PROJECT_NAME} rosidl_typesupport_cpp
    )
    target_link_libraries(test_shared_buffer_type ${cpp_typesupport_target})
  endif()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
