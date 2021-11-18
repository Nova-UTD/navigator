#
 # Package:   path_planner
 # Filename:  CMakeLists.txt
 # Author:    Jim Moore
 # Email:     jim3moore@gmail.com
 # Copyright: 2021, Nova UTD
 # License:   MIT License
 #
cmake_minimum_required(VERSION 3.5)

project(path_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(PATH_PLANNER_LIB_SRC
  src/path_planner.cpp
  src/parameterized_spline.cpp
  src/map_utils.cpp
)

set(PATH_PLANNER_LIB_HEADERS
  include/path_planner/path_planner.hpp
  include/path_planner/spline.hpp
  include/path_planner/parameterized_spline.hpp
  include/path_planner/lane_points.hpp
  include/path_planner/map_utils.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${PATH_PLANNER_LIB_SRC}
  ${PATH_PLANNER_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)   # General test dependency
  ament_lint_auto_find_test_dependencies() # Find any test dependencies declared in package.xml

  ament_add_gtest(tests
    test/test_path_planner.cpp
    test/test_parameterized_spline.cpp)

  ament_target_dependencies(tests
    autoware_auto_common autoware_auto_geometry
    autoware_auto_msgs
    had_map_utils)

  target_link_libraries(tests ${PROJECT_NAME} gtest_main)
  add_dependencies(tests ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package()
