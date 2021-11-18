# Package:   nova_gps
# Filename:  CMakeLists.txt
# Author:    Avery Bainbridge
# Email:     axb200157@utdallas.edu
# Copyright: 2021, Nova
# License:   MIT License

cmake_minimum_required(VERSION 3.5)
project(nova_gps)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Build message headers
#rosidl_generate_interfaces(${PROJECT_NAME} "msg/CanFrame.msg")
ament_export_dependencies(rosidl_default_runtime)

# Build a library from our source files
ament_auto_add_library(i2c_lib STATIC
  src/I2CInterface.cpp)
target_include_directories(i2c_lib PUBLIC include)
#rosidl_target_interfaces(i2c_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(interface src/interface.cpp src/ConcreteGPSInterface.cpp src/GPSInterfaceNode.cpp)
target_include_directories(interface PUBLIC include)
ament_target_dependencies(interface rclcpp)
target_link_libraries(interface i2c_lib)

install(TARGETS interface DESTINATION lib/${PROJECT_NAME})

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
