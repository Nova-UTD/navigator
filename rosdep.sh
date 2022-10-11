#!/bin/sh



apt-get update

apt-get install -y ros-humble-diagnostic-updater
ros-humble-velodyne-msgs \
ros-humble-velodyne-driver \
ros-humble-velodyne-laserscan \
ros-humble-velodyne-pointcloud \
ros-humble-autoware-auto-msgs \
ros-humble-lanelet2-core \
ros-humble-lanelet2-io \
ros-humble-lanelet2-projection \
ros-humble-lanelet2 \
ros-humble-tf-transformations \
ros-humble-ros-testing \
ros-humble-lgsvl-msgs \
ros-humble-vision-opencv \
ament-cmake-nose \
ros-humble-pcl-conversions \
software-properties-common \
ament-cmake-nose \
ros-humble-cv-bridge \
ros-humble-pcl-conversions

apt-get install -y ros-humble-robot-localization

pip3 install pymap3d==2.9.1