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
# copied from path_publisher package for rrt by Hansika Sundaresan

cmake_minimum_required(VERSION 3.5)
get_filename_component(directory_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${directory_name})

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(nova_auto_package REQUIRED)

# Default to C++17 
if(NOT CMAKE_CXX_STANDARD) 
    set(CMAKE_CXX_STANDARD 17) 
endif() 

nova_auto_package()
