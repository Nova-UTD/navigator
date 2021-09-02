# Package:   epas_translator
# Filename:  CMakeLists.txt
# Author:    Joshua Williams
# Email:     joshmackwilliams@protonmail.com
# Copyright: 2021, Voltron UTD
# License:   MIT License

cmake_minimum_required(VERSION 3.5)
project(epas_translator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Build a library from our source files. This is just a convenience
# mechanism to prevent having to explicitly list all files for every
# executable.
ament_auto_add_library(epas_steering_lib STATIC
  src/ReporterNode.cpp
  src/ControllerNode.cpp)
target_include_directories(epas_steering_lib PUBLIC include)

add_executable(reporter src/reporter.cpp)
target_link_libraries(reporter epas_steering_lib)

add_executable(controller src/controller.cpp)
target_link_libraries(controller epas_steering_lib)

install(TARGETS reporter controller DESTINATION lib/${PROJECT_NAME})

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(tests
    test/test_reporter.cpp
    test/test_controller.cpp)
  ament_target_dependencies(tests rclcpp voltron_can voltron_test_utils std_msgs)
  target_link_libraries(tests epas_steering_lib gtest_main)
endif()

ament_auto_package()
