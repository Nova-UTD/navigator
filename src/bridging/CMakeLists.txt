# Raghav Pillai
cmake_minimum_required(VERSION 3.5)

project(bridging)

## dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()
# Disable warnings due to external dependencies (Eigen)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

# build library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/bridging_creation.cpp
)
autoware_set_compile_options(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(HUNGARIAN_ASSIGN_GTEST hungarian_assigner_gtest)
  ament_add_gtest(${HUNGARIAN_ASSIGN_GTEST} test/src/test_hungarian_assigner.cpp)
  autoware_set_compile_options(${HUNGARIAN_ASSIGN_GTEST})
  target_include_directories(${HUNGARIAN_ASSIGN_GTEST} PRIVATE "test/include" "include")
  target_link_libraries(${HUNGARIAN_ASSIGN_GTEST} ${PROJECT_NAME})
endif()

# Ament Exporting
ament_auto_package()