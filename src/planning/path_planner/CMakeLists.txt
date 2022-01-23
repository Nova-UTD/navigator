cmake_minimum_required(VERSION 3.5)
project(path_planner)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# build
set(PATH_PLANNER_LIB path_planner)
ament_auto_add_library(${PATH_PLANNER_LIB} SHARED
  include/path_planner/path_planner.hpp
  include/path_planner/path_planner_node.hpp
  src/path_planner.cpp
  src/path_planner_node.cpp
)
autoware_set_compile_options(${PATH_PLANNER_LIB})

set(PATH_PLANNER_NODE_EXE ${PATH_PLANNER_LIB}_exe)
rclcpp_components_register_node(${PATH_PLANNER_LIB}
  PLUGIN "navigator::path_planner::PathPlannerNode"
  EXECUTABLE ${PATH_PLANNER_NODE_EXE}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
)
