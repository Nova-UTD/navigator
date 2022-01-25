# Package:   nova_auto_package
# Filename:  nova_auto_package_extras.cmake
# Author:    Joshua Williams
# Email:     joshmackwilliams@protonmail.com
# Copyright: 2021, Nova UTD
# License:   MIT License

find_package(ament_cmake_gtest REQUIRED)

include(${nova_auto_package_DIR}/ament_auto_find_test_dependencies.cmake)
include(${nova_auto_package_DIR}/ament_auto_add_gtest.cmake)
include(${nova_auto_package_DIR}/nova_auto_package.cmake)
