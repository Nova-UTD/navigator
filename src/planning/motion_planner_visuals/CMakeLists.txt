#
# Package:   MotionPlanner
# Filename:  CMakeLists.txt
# Author:    Jim Moore
# Email:     jim3moore@gmail.com
# Copyright: 2021, Nova UTD
# License:   MIT License
#
cmake_minimum_required(VERSION 3.5)

project(motion_planner_visuals)

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
ament_auto_add_library(motion_planner_visuals_lib STATIC
  include/motion_planner_visuals/MotionPlannerVisualsNode.hpp
  src/MotionPlannerVisualsNode.cpp
)
target_include_directories(motion_planner_visuals_lib PUBLIC include)

# ament package generation and installing
ament_auto_package()
