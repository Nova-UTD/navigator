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

# Build a library from our source files
set(MOTION_PLANNER_VISUALS_LIB motion_planner_visuals)
ament_auto_add_library(${MOTION_PLANNER_VISUALS_LIB} SHARED
  include/motion_planner_visuals/MotionPlannerVisualsNode.hpp
  src/MotionPlannerVisualsNode.cpp
)
#autoware_set_compile_options(${MOTION_PLANNER_VISUALS_LIB})

rclcpp_components_register_node(${MOTION_PLANNER_VISUALS_LIB}
  PLUGIN "navigator::MotionPlannerVisuals::MotionPlannerVisualsNode"
  EXECUTABLE motion_planner_visuals_exe
)

target_include_directories(${MOTION_PLANNER_VISUALS_LIB} PUBLIC include)

# ament package generation and installing
ament_auto_package(
  INSTALL_TO_SHARE
)
