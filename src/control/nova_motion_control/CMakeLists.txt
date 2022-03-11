cmake_minimum_required(VERSION 3.5)
project(nova_motion_control)

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
ament_auto_find_build_dependencies()


ament_auto_add_library(controls_lib STATIC
  src/PurePursuit.cpp
  src/PidController.cpp
  src/MotionControlNode.cpp)
target_include_directories(controls_lib PUBLIC include)


add_executable(MotionControl src/MotionControl.cpp)
ament_target_dependencies(MotionControl rclcpp)
target_link_libraries(MotionControl controls_lib)


install(TARGETS MotionControl DESTINATION lib/${PROJECT_NAME})


install(TARGETS MotionControl
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)