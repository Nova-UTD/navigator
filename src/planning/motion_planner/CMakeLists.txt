#
# Package:   MotionPlanner
# Filename:  CMakeLists.txt
# Author:    Jim Moore
# Email:     jim3moore@gmail.com
# Copyright: 2021, Nova UTD
# License:   MIT License
#
cmake_minimum_required(VERSION 3.5)

project(motion_planner)

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
ament_auto_add_library(motion_planner_lib SHARED
  include/motion_planner/LanePoints.hpp
  include/motion_planner/PathPoint.hpp
  include/motion_planner/SegmentedPath.hpp
  include/motion_planner/IdealPoint.hpp
  include/motion_planner/CarPose.hpp
  include/motion_planner/MotionPlannerNode.hpp
  include/motion_planner/Collision.hpp
  src/MotionPlanner.cpp
  src/map_utils.cpp
  src/SegmentedPath.cpp
  src/MotionPlannerNode.cpp
)
target_include_directories(motion_planner_lib PUBLIC include)
#commenting out to disable unused variable errors for the moment
#autoware_set_compile_options(motion_planner_lib)

rclcpp_components_register_node(motion_planner_lib
  PLUGIN "navigator::MotionPlanner::MotionPlannerNode"
  EXECUTABLE motion_planner_exe
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)   # General test dependency
  ament_lint_auto_find_test_dependencies() # Find any test dependencies declared in package.xml

  ament_add_gtest(tests
    test/test_MotionPlanner.cpp
    test/test_SegmentedPath.cpp)

  ament_target_dependencies(tests
    autoware_auto_common autoware_auto_geometry
    autoware_auto_msgs
    had_map_utils)

  target_link_libraries(tests motion_planner_lib gtest_main)
endif()

# ament package generation and installing
ament_auto_package(
  INSTALL_TO_SHARE
)
