# Package:   can_interface
# Filename:  CMakeLists.txt
# Author:    Joshua Williams
# Email:     joshmackwilliams@protonmail.com
# Copyright: 2021, Voltron UTD
# License:   MIT License

cmake_minimum_required(VERSION 3.5)
project(can_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_export_dependencies(rosidl_default_runtime)

# Build a library from our source files
ament_auto_add_library(can_lib STATIC
  src/ConcreteCanBus.cpp
  src/CanFrame.cpp
  src/CanInterfaceNode.cpp
  src/CanBusDecorator.cpp)
target_include_directories(can_lib PUBLIC include)
#rosidl_target_interfaces(can_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(interface src/interface.cpp)
ament_target_dependencies(interface rclcpp)
target_link_libraries(interface can_lib)

install(TARGETS interface DESTINATION lib/${PROJECT_NAME})

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(tests
    test/test_can_frame.cpp
    test/test_can_bus.cpp
    test/test_can_interface_node.cpp)
  ament_target_dependencies(tests rclcpp voltron_test_utils)
  target_link_libraries(tests can_lib gtest_main)
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
