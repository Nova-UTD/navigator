cmake_minimum_required(VERSION 3.5)
project(nova_pure_pursuit)

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


ament_auto_add_library(pp_lib STATIC
  src/PurePursuit.cpp
  src/PurePursuitNode.cpp)
target_include_directories(pp_lib PUBLIC include)


add_executable(LateralController src/LateralController.cpp)
ament_target_dependencies(LateralController rclcpp)
target_link_libraries(LateralController pp_lib)


install(TARGETS LateralController DESTINATION lib/${PROJECT_NAME})


install(TARGETS LateralController
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch)