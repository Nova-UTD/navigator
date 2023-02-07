#!/bin/sh

apt-get update

apt-get install -y ros-foxy-diagnostic-updater \
libfprint-2-2:amd64 \
libpam-fprintd:amd64 \
fprintd

apt install -y ros-foxy-velodyne-msgs \
ros-foxy-velodyne-driver \
ros-foxy-velodyne-laserscan \
ros-foxy-tf-transformations \
ros-foxy-ros-testing \
ros-foxy-vision-opencv \
ros-foxy-pcl-conversions \
software-properties-common \
ros-foxy-ament-cmake-nose \
ros-foxy-cv-bridge \
ros-foxy-pcl-conversions \
ros-foxy-tf-transformations \
ros-foxy-ament-cmake-nose \
python3-nose \
ros-foxy-lgsvl-msgs \
ros-foxy-pcl-conversions \
ros-foxy-ros-testing \
python3-matplotlib \
python3-scipy \
ros-foxy-rviz2 \
ros-foxy-robot-localization \
ros-foxy-derived-object-msgs \
byobu \
ros-foxy-velodyne-pointcloud \
nvidia-cuda-toolkit