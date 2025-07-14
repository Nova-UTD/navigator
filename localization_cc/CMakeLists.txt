# Filename:  CMakeLists.txt
# Author:    Kevin Ge
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

# Set debug 
set(CMAKE_BUILD_TYPE Release)

add_library(viz STATIC src/Visualizer.cpp)
target_include_directories(viz PUBLIC include/)

add_library(global_map STATIC src/GlobalMap.cpp)
target_include_directories(global_map PUBLIC include/)

add_library(utils STATIC src/Utils.cpp)
target_include_directories(utils PUBLIC include/)

find_package(nova_auto_package REQUIRED)

# Default to C++17 
if(NOT CMAKE_CXX_STANDARD) 
    set(CMAKE_CXX_STANDARD 17) 
endif() 

nova_auto_package()

set(Open3D_DIR "/usr/local/lib/cmake/Open3D")
find_package(Open3D REQUIRED)
target_link_libraries(${PROJECT_NAME}_lib Open3D::Open3D)

find_package(absl REQUIRED)
target_link_libraries(
    ${PROJECT_NAME}_lib 
    absl::strings absl::status absl::statusor absl::flat_hash_map
)

find_package(OusterSDK REQUIRED)
include_directories(${OusterSDK_INCLUDE_DIRS} /usr/local/include/optional-lite)
link_directories(${OusterSDK_LIBRARY_DIRS} /usr/lib/x86_64-linux-gnu)
add_definitions(${OusterSDK_DEFINITIONS})
target_link_libraries(${PROJECT_NAME}_lib OusterSDK::ouster_client OusterSDK::ouster_viz)

find_package(PCL 1.14.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(${PROJECT_NAME}_lib ${PCL_LIBRARIES})

find_package(pcl_conversions REQUIRED)
include_directories(${pcl_conversions_INCLUDE_DIRS})
link_directories(${pcl_conversions_LIBRARY_DIRS})
add_definitions(${pcl_conversions_DEFINITIONS})
target_link_libraries(${PROJECT_NAME}_lib ${pcl_conversions_LIBRARIES})

find_package(rclcpp REQUIRED)
include_directories(${rclcpp_INCLUDE_DIRS})
link_libraries(${rclcpp_LIBRARY_DIRS})
add_definitions(${rclcpp_DEFINITIONS})
target_link_libraries(global_map ${rclcpp_LIBRARIES})
target_link_libraries(utils ${rclcpp_LIBRARIES})
