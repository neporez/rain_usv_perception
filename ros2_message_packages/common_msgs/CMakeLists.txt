cmake_minimum_required(VERSION 3.8)
project(common_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Int32Stamped.msg"
  "msg/Float64Stamped.msg"
  "msg/StringStamped.msg"
  "msg/Int32ArrayStamped.msg"
  "msg/Float64ArrayStamped.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
