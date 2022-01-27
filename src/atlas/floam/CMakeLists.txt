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
ament_auto_find_build_dependencies()

add_executable(laser_processing_node src/LaserProcessingNode.cpp)


# target_sources(laser_processing_node PUBLIC 
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/lidar)

  target_include_directories(laser_processing_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
    laser_processing_node
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



target_include_directories(laser_processing_node SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries (laser_processing_node ${PCL_LIBRARIES})

# add_executable(laser_processing_node exe/laser_processing_node.cpp)
# target_include_directories(laser_processing_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

#   ament_target_dependencies(
#     laser_processing_node
#     geometry_msgs
#     nav_msgs
#     sensor_msgs
#     rclcpp
#     std_msgs
#     tf2
#     # eigen_conversions
#     )


ament_package()