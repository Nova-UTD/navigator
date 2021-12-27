# Package:   bridge_manager
# Filename:  CMakeLists.txt
# Author:    Raghav Pillai
# Email:     raghavpillai101@gmail.com
# Copyright: 2021, Nova UTD
# License:   MIT License

cmake_minimum_required(VERSION 3.5)
project(bridge_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Build library from source files
ament_auto_add_library(bridge_manager_lib OBJECT
  src/BridgeManagerNode.cpp)
target_include_directories(bridge_manager_lib PUBLIC include)

add_executable(bridge_manager src/bridge_manager.cpp)
target_link_libraries(bridge_manager bridge_manager_lib)

install(TARGETS bridge_manager DESTINATION lib/${PROJECT_NAME})

# Tests (to add later)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(tests test/test_bridge_manager.cpp)
  ament_target_dependencies(tests rclcpp voltron_msgs voltron_test_utils)
  target_link_libraries(tests gtest_main bridge_manager_lib)
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
