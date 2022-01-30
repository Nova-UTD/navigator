cmake_minimum_required(VERSION 3.5)
project(obstacle_drawer)

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
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(voltron_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(obstacle_classes REQUIRED)

add_library(obstacle_drawer_lib STATIC src/obstacle_drawer_node.cpp)
target_include_directories(obstacle_drawer_lib PUBLIC include)
ament_target_dependencies(obstacle_drawer_lib 
  rclcpp 
  voltron_msgs 
  visualization_msgs 
  geometry_msgs
  obstacle_classes
)

add_executable(obstacle_drawer_exe src/obstacle_drawer.cpp)
target_link_libraries(obstacle_drawer_exe obstacle_drawer_lib)

install(TARGETS
  obstacle_drawer_exe
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(tests test/obstacle_drawer_test.cpp)
  ament_target_dependencies(tests voltron_test_utils)
  target_link_libraries(tests gtest_main obstacle_drawer_lib)

endif()

ament_package()
