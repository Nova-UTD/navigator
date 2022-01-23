# Package:   nova_auto_package
# Filename:  CMakeLists.txt
# Author:    Joshua Williams
# Email:     joshmackwilliams@protonmail.com
# Copyright: 2021, Nova UTD
# License:   MIT License

cmake_minimum_required(VERSION 3.5)
project(nova_auto_package)
find_package(ament_cmake_auto REQUIRED)
ament_package(CONFIG_EXTRAS "nova_auto_package_extras.cmake")
install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})
