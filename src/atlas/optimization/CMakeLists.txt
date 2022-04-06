# Filename:  CMakeLists.txt
# Author:    Joshua Williams
# Email:     joshmackwilliams@protonmail.com
# Copyright: 2021, Nova UTD
# License:   MIT License

# No package name is specified above since this is our standard
# CMakeLists.txt file and will be the same across multiple
# projects. To use it, just add nova_auto_package as a
# buildtool_depend in package.xml and copy this file into the root of
# your package.

cmake_minimum_required(VERSION 3.5)
get_filename_component(directory_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${directory_name})

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include/optimization
)

add_library(optimization_node src/OptimizationNode.cpp)
ament_target_dependencies(optimization_node 
  GTSAM
  Eigen3
  rclcpp
  voltron_msgs
)

add_executable(optimizer exe/optimizer src/OptimizationNode.cpp)
ament_target_dependencies(optimizer voltron_msgs rclcpp nav_msgs GTSAM)
target_link_libraries(optimizer gtsam)

install(
  TARGETS optimizer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()