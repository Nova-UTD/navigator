# Package:   navigator_msgs
# Filename:  CMakeLists.txt
# Author:    Joshua Williams
# Email:     joshmackwilliams@protonmail.com
# Copyright: 2021, Nova UTD
# License:   MIT License

cmake_minimum_required(VERSION 3.5)
get_filename_component(directory_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${directory_name})

find_package(ament_cmake_auto REQUIRED) # The greatest package in history
ament_auto_find_build_dependencies() # Find dependencies listed in package.xml

# Find message files
file(GLOB_RECURSE msg_filenames RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg") # Find all message files
file(GLOB_RECURSE srv_filenames RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv") # Find all srv files
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${msg_filenames}
  ${srv_filenames}
  DEPENDENCIES
  ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
)

ament_auto_package()
