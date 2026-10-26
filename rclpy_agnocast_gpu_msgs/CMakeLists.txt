cmake_minimum_required(VERSION 3.14)
project(rclpy_agnocast_gpu_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ros2_cuda_ipc_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GpuTensor.msg"
  DEPENDENCIES ros2_cuda_ipc_msgs
)

ament_export_dependencies(rosidl_default_runtime ros2_cuda_ipc_msgs)
ament_package()
