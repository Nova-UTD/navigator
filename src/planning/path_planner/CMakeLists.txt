#
# Package:   path_planner
# Filename:  CMakeLists.txt
# Author:    Jim Moore
# Email:     jim3moore@gmail.com
# Copyright: 2021, Nova UTD
# License:   MIT License
#
cmake_minimum_required(VERSION 3.5)

project(path_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Build a library from our source files. This is just a convenience
# mechanism to prevent having to explicitly list all files for every
# executable.
ament_auto_add_library(path_planner_lib STATIC
  include/path_planner/lane_points.hpp
  include/path_planner/path_point.hpp
  include/path_planner/segmented_path.hpp
  include/path_planner/ideal_point.hpp
  include/path_planner/car_pose.hpp
  src/path_planner.cpp
  src/map_utils.cpp
  src/segmented_path.cpp
)
target_include_directories(path_planner_lib PUBLIC include)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)   # General test dependency
  ament_lint_auto_find_test_dependencies() # Find any test dependencies declared in package.xml

  ament_add_gtest(tests
    test/test_path_planner.cpp
    test/test_segmented_path.cpp)

  ament_target_dependencies(tests
    autoware_auto_common autoware_auto_geometry
    autoware_auto_msgs
    had_map_utils)

  target_link_libraries(tests path_planner_lib gtest_main)
endif()

# ament package generation and installing
ament_auto_package()
