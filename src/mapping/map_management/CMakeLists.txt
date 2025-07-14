# Filename:  CMakeLists.txt
# Author:    Will Heitman
# Email:     project.nova@utdallas.edu
# Copyright: 2023, Nova UTD
# License:   MIT License

# No package name is specified above since this is our standard
# CMakeLists.txt file and will be the same across multiple
# projects. To use it, just add nova_auto_package as a
# buildtool_depend in package.xml and copy this file into the root of
# your package.

cmake_minimum_required(VERSION 3.5)
get_filename_component(directory_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${directory_name})

add_library(route_manager STATIC src/RouteManager.cpp)
target_include_directories(route_manager PUBLIC include/)
find_package(nova_auto_package REQUIRED)

# Default to C++17 
if(NOT CMAKE_CXX_STANDARD) 
    set(CMAKE_CXX_STANDARD 17) 
endif() 

nova_auto_package()
