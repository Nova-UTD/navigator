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
project(floam)

find_package(ament_cmake_auto REQUIRED)
find_package(Ceres REQUIRED)
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}  
)
link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${CERES_LIBRARY_DIRS}
)
ament_auto_find_build_dependencies()

add_executable(laser_processing_node src/LaserProcessingNode.cpp src/lidar.cpp src/laserProcessingClass.cpp)

add_executable(odom_estimation_node 
src/OdomEstimationNode.cpp
src/lidar.cpp
src/lidarOptimization.cpp
src/odomEstimationClass.cpp
)

target_include_directories(laser_processing_node PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
PRIVATE
${CERES_INCLUDE_DIRS}
)

target_include_directories(odom_estimation_node
PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
PRIVATE
${CERES_INCLUDE_DIRS}
)

ament_target_dependencies(
  laser_processing_node
  Ceres
  geometry_msgs
  nav_msgs
  PCL
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2_eigen
  tf2_ros
  tf2_sensor_msgs
  tf2
)

ament_target_dependencies(
  odom_estimation_node
  Ceres
  geometry_msgs
  nav_msgs
  PCL
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2_eigen
  tf2_ros
  tf2_sensor_msgs
  tf2
)

target_include_directories(laser_processing_node SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR} ${CERES_LIBRARY_DIRS})
target_include_directories(odom_estimation_node SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR} ${CERES_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
target_link_libraries (laser_processing_node
${PCL_LIBRARIES}
${CERES_LIBRARIES}
)
target_link_libraries (odom_estimation_node
${PCL_LIBRARIES}
${CERES_LIBRARIES}
)

ament_package()