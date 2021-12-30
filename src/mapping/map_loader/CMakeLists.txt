cmake_minimum_required(VERSION 3.5)
project(map_loader)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(polypartition STATIC
  src/polypartition.cpp)
target_include_directories(polypartition PUBLIC include)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(map_grid_loader_node src/map_grid_loader_node.cpp)
add_executable(lanelet_loader_node src/lanelet_loader_node.cpp)
target_include_directories(map_grid_loader_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
                          lanelet_loader_node
                          geometry_msgs
                          rclcpp 
                          lanelet2_core
                          lanelet2_io
                          lanelet2_projection
                          std_msgs
                          visualization_msgs
                          )
target_link_libraries(lanelet_loader_node polypartition)
install(TARGETS map_grid_loader_node lanelet_loader_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
