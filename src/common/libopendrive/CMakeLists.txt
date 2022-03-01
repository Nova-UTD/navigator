# Package:   libopendrive
# Filename:  CMakeLists.txt
# Author:    Joshua Williams
# Email:     joshmackwilliams@protonmail.com
# Copyright: 2021, Nova UTD
# License:   MIT License

cmake_minimum_required(VERSION 3.10)
get_filename_component(directory_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${directory_name})

find_package(ament_cmake_auto REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libOpenDRIVE)

set(${PROJECT_NAME}_LIBRARIES OpenDrive)

ament_export_include_directories("include/libOpenDRIVE")
ament_export_include_directories("include/libOpenDRIVE/Thirdparty")
install(DIRECTORY libOpenDRIVE/ DESTINATION include/libOpenDRIVE)

ament_auto_package()
