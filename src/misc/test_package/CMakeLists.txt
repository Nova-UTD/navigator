# Package:   test_package
# Filename:  CMakeLists.txt
# Author:    Joshua Williams
# Email:     joshmackwilliams@protonmail.com
# Copyright: 2021, Nova UTD
# License:   MIT License

get_filename_component(directory_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)

cmake_minimum_required(VERSION 3.5)
project(${directory_name})

# Since we don't have the latest version of ament_cmake, copy some of
# the new (and very useful) code into this project
macro(ament_auto_add_gtest target)
  cmake_parse_arguments(_ARGN
    "SKIP_LINKING_MAIN_LIBRARIES;SKIP_TEST"
    "RUNNER;TIMEOUT;WORKING_DIRECTORY"
    "APPEND_ENV;APPEND_LIBRARY_DIRS;ENV"
    ${ARGN})
  if(NOT _ARGN_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR
      "ament_auto_add_gtest() must be invoked with at least one source file")
  endif()

  # add executable
  set(_argn_executable ${_ARGN_UNPARSED_ARGUMENTS})
  if(_ARG_SKIP_LINKING_MAIN_LIBRARIES)
    list(APPEND _argn_executable "SKIP_LINKING_MAIN_LIBRARIES")
  endif()
  ament_add_gtest_executable("${target}" ${_argn_executable})

  # add include directory of this package if it exists
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
    target_include_directories("${target}" PUBLIC
      "${CMAKE_CURRENT_SOURCE_DIR}/include")
  endif()

  # link against other libraries of this package
  if(NOT ${PROJECT_NAME}_LIBRARIES STREQUAL "")
    target_link_libraries("${target}" ${${PROJECT_NAME}_LIBRARIES})
  endif()

  # add exported information from found dependencies
  ament_target_dependencies(${target}
    ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
    ${${PROJECT_NAME}_FOUND_TEST_DEPENDS}
  )

  # add test
  set(_argn_test "")
  if(_ARG_RUNNER)
    list(APPEND _argn_test "RUNNER" "${_ARG_RUNNER}")
  endif()
  if(_ARG_TIMEOUT)
    list(APPEND _argn_test "TIMEOUT" "${_ARG_TIMEOUT}")
  endif()
  if(_ARG_WORKING_DIRECTORY)
    list(APPEND _argn_test "WORKING_DIRECTORY" "${_ARG_WORKING_DIRECTORY}")
  endif()
  if(_ARG_SKIP_TEST)
    list(APPEND _argn_test "SKIP_TEST")
  endif()
  if(_ARG_ENV)
    list(APPEND _argn_test "ENV" ${_ARG_ENV})
  endif()
  if(_ARG_APPEND_ENV)
    list(APPEND _argn_test "APPEND_ENV" ${_ARG_APPEND_ENV})
  endif()
  if(_ARG_APPEND_LIBRARY_DIRS)
    list(APPEND _argn_test "APPEND_LIBRARY_DIRS" ${_ARG_APPEND_LIBRARY_DIRS})
  endif()
  ament_add_gtest_test("${target}" ${_argn_test})
endmacro()

# Additional code copied from ament_cmake
macro(ament_auto_find_test_dependencies)
  set(_ARGN "${ARGN}")
  if(_ARGN)
    message(FATAL_ERROR "ament_auto_find_test_dependencies() called with "
      "unused arguments: ${_ARGN}")
  endif()

  if(NOT _AMENT_PACKAGE_NAME)
    ament_package_xml()
  endif()

  # try to find_package() all test dependencies
  foreach(_dep ${${PROJECT_NAME}_TEST_DEPENDS})
    find_package(${_dep} QUIET)
    if(${_dep}_FOUND)
      list(APPEND ${PROJECT_NAME}_FOUND_TEST_DEPENDS ${_dep})
    endif()
  endforeach()
endmacro()

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

# Build a static library
file(GLOB_RECURSE source_filenames "src/*.cpp") # Find all source files in src/
if(source_filenames) # If we have any source files
  ament_auto_add_library(${PROJECT_NAME}_lib STATIC DIRECTORY src) # Then make a library
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

if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY "launch" DESTINATION "share/${PROJECT_NAME}")
endif()

ament_auto_package()
