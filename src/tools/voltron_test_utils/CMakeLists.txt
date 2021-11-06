# Package:   voltron_test_utils
# Filename:  CMakeLists.txt
# Author:    Joshua Williams
# Email:     joshmackwilliams@protonmail.com
# Copyright: 2021, Voltron UTD
# License:   MIT License

cmake_minimum_required(VERSION 3.5)
project(voltron_test_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(tests
    test/test_test_publisher_subscriber.cpp
    test/test_test_client_server.cpp)
  ament_target_dependencies(tests rclcpp std_msgs std_srvs)
  target_include_directories(tests PRIVATE include)
  target_link_libraries(tests gtest_main)
endif()

ament_auto_package()
