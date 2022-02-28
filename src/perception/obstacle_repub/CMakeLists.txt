cmake_minimum_required(VERSION 3.5)
project(obstacle_repub)

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
find_package(lgsvl_msgs REQUIRED)
find_package(zed_interfaces REQUIRED)
find_package(voltron_msgs REQUIRED)
find_package(obstacle_classes REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(obstacle_repub_lib STATIC src/obstacle_repub_node.cpp)
target_include_directories(obstacle_repub_lib PUBLIC include)
ament_target_dependencies(obstacle_repub_lib 
  rclcpp 
  lgsvl_msgs 
  zed_interfaces 
  voltron_msgs 
  obstacle_classes
  tf2 
  tf2_geometry_msgs
)

add_executable(obstacle_repub_exe src/obstacle_repub.cpp)
target_link_libraries(obstacle_repub_exe obstacle_repub_lib)

install(TARGETS
  obstacle_repub_exe
  DESTINATION lib/${PROJECT_NAME}
)
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(tests test/obstacle_repub_test.cpp)
  ament_target_dependencies(tests voltron_test_utils)
  target_link_libraries(tests gtest_main obstacle_repub_lib)
  
endif()

ament_package()
