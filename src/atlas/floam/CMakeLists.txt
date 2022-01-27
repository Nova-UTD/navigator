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

find_package(nova_auto_package REQUIRED)

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${CERES_LIBRARY_DIRS}
)

# Add Eigen manually... WSH
# target_include_directories(laser_processing_node PUBLIC
#   ${EIGEN3_INCLUDE_DIR}
# )

nova_auto_package()
