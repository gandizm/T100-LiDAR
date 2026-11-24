cmake_minimum_required(VERSION 3.8)
# Message definitions only (CustomMsg / CustomPoint, unchanged from Livox-SDK/livox_ros_driver2, MIT).
# The package keeps Livox's name because FAST-LIO includes livox_ros_driver2/msg/custom_msg.hpp.
project(livox_ros_driver2)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
