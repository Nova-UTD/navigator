# Package:   pid_controller
# Filename:  CMakeLists.txt
# Author:    Joshua Williams
# Email:     joshmackwilliams@protonmail.com
# Copyright: 2021, Voltron UTD
# License:   MIT License

cmake_minimum_required(VERSION 3.5)
project(pid_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(pid_lib STATIC
  src/PidController.cpp
  src/PidControllerNode.cpp)
target_include_directories(pid_lib PUBLIC include)

add_executable(pid_controller src/pid_controller.cpp)
ament_target_dependencies(pid_controller rclcpp)
target_link_libraries(pid_controller pid_lib)

install(TARGETS pid_controller DESTINATION lib/${PROJECT_NAME})

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
