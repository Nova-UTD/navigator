# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(voltron_msgs)

# Generate messages
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Gear.msg"
  "msg/CanFrame.msg"
  "srv/SafetyEvent.srv"
  "srv/SafetyCommand.srv"
  DEPENDENCIES
    "builtin_interfaces"
    "geometry_msgs"
    "sensor_msgs"
    "shape_msgs"
    "std_msgs"
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
