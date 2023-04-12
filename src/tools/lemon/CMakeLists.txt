# Package:   libboost
# Filename:  CMakeLists.txt
# Author:    Joshua Williams
# Email:     joshmackwilliams@protonmail.com
# Copyright: 2021, Nova UTD
# License:   MIT License

cmake_minimum_required(VERSION 3.10)
get_filename_component(directory_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${directory_name})

find_package(ament_cmake_auto REQUIRED)

FIND_PACKAGE( LEMON REQUIRED )
ament_export_include_directories(${LEMON_INCLUDE_DIRS})

ament_auto_package()
