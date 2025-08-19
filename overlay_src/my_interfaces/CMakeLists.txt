cmake_minimum_required(VERSION 3.5)
project(my_interfaces)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChaseMode.msg"
)
ament_package()
