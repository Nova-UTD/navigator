# Package:   nova_auto_package
# Filename:  cmake/nova_auto_package.cmake
# Author:    Joshua Williams
# Email:     joshmackwilliams@protonmail.com
# Copyright: 2021, Nova UTD
# License:   MIT License

macro(nova_auto_package)

# Use C++ 20, unless manually set otherwise
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# Add options to enable extra warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED) # The greatest package in history
ament_auto_find_build_dependencies() # Find dependencies listed in package.xml

# Build a shared library
file(GLOB_RECURSE source_filenames "src/*.cpp") # Find all source files in src/
if(source_filenames) # If we have any source files
  ament_auto_add_library(${PROJECT_NAME}_lib SHARED DIRECTORY src) # Then make a library
endif()

# Build all executables
file(GLOB_RECURSE executable_filenames "exe/*.cpp") # Get all source files in exe/
foreach(filename ${executable_filenames}) # Iterate over them
  get_filename_component(executable_name ${filename} NAME_WE) # Pull the name with no extension
  ament_auto_add_executable(${executable_name} ${filename}) # Add an executable by that name
endforeach()

# Build all tests
file(GLOB test_filenames "test/*.cpp") # Find test source files
if(BUILD_TESTING AND test_filenames) # If we have any tests to build
  ament_auto_find_test_dependencies() # Find the test dependencies
  ament_auto_add_gtest(tests ${test_filenames}) # And add our tests
endif()

ament_auto_package()

endmacro()
