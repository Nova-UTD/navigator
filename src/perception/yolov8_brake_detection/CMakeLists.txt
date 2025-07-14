cmake_minimum_required(VERSION 3.5)
project(yolov8_brake_detection)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Specify message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BrakeStatus.msg"
)

# Python scripts
install(
  PROGRAMS
    yolov8_node.py
    yolov8_predict_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install other package files
ament_package()
