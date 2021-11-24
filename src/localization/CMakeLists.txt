# Package:   nova_gps
# Filename:  CMakeLists.txt
# Author:    Avery Bainbridge
# Email:     axb200157@utdallas.edu
# Copyright: 2021, Nova
# License:   MIT License

cmake_minimum_required(VERSION 3.5)
project(imu_transformer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
#find_package(rclcpp REQUIRED)
#find_package(std_msgs REQUIRED)
ament_auto_find_build_dependencies()
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)

# Build message headers
ament_export_dependencies(rosidl_default_runtime)

# Build a library from our source files

add_executable(imu_transformer_exe src/imu_transformer_exe.cpp)
ament_target_dependencies(imu_transformer_exe rclcpp)

install(TARGETS imu_transformer_exe DESTINATION lib/${PROJECT_NAME})

# Tests
if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  ament_lint_auto_find_test_dependencies()
#  
#  ament_add_gtest(tests
#    test/test_can_frame.cpp
#    test/test_can_bus.cpp
#    test/test_can_interface_node.cpp)
#  ament_target_dependencies(tests rclcpp voltron_test_utils)
#  target_link_libraries(tests can_lib gtest_main)
endif()

ament_auto_package()
