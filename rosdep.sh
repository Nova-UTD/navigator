#!/bin/sh



apt-get update

apt-get install -y ros-humble-diagnostic-updater
apt-get install -y ros-humble-velodyne-msgs
apt-get install -y ros-humble-velodyne-driver
apt-get install -y ros-humble-velodyne-laserscan
apt-get install -y ros-humble-velodyne-pointcloud
apt-get install -y ros-humble-autoware-auto-msgs
apt-get install -y ros-humble-lanelet2-core
apt-get install -y ros-humble-lanelet2-io
apt-get install -y ros-humble-lanelet2-projection
apt-get install -y ros-humble-lanelet2
apt-get install -y ros-humble-tf-transformations
apt-get install -y ros-humble-ros-testing
apt-get install -y ros-humble-lgsvl-msgs
apt-get install -y ros-humble-vision-opencv \
ament-cmake-nose \
ros-humble-pcl-conversions \
libgtsam-dev \
libgtsam-unstable-dev \
software-properties-common

apt-get install -y ros-humble-robot-localization

pip3 install pymap3d==2.9.1