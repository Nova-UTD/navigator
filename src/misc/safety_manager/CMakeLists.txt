# Package:   safety_manager
# Filename:  CMakeLists.txt
# Author:    Joshua Williams
# Email:     joshmackwilliams@protonmail.com
# Copyright: 2021, Nova UTD
# License:   MIT License

cmake_minimum_required(VERSION 3.5)
project(safety_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(safety_manager_lib STATIC
  src/SafetyManagerNode.cpp)
target_include_directories(safety_manager_lib PUBLIC include)

add_executable(safety_manager src/safety_manager.cpp)
target_link_libraries(safety_manager safety_manager_lib)

install(TARGETS safety_manager DESTINATION lib/${PROJECT_NAME})

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(tests test/test_safety_manager.cpp)
  ament_target_dependencies(tests rclcpp voltron_msgs voltron_test_utils)
  target_link_libraries(tests gtest_main safety_manager_lib)
endif()

ament_auto_package()
