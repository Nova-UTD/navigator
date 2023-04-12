#!/bin/sh

apt-get update

apt-get install -y ros-humble-diagnostic-updater \
libfprint-2-2:amd64 \
libpam-fprintd:amd64 \
fprintd

apt install -y ros-humble-velodyne-msgs \
ros-humble-velodyne-driver \
ros-humble-velodyne-laserscan \
ros-humble-joy-linux \
ros-humble-pcl-ros \
minicom \
ros-humble-joy-linux \
ros-humble-rosbridge-server \
ros-humble-image-transport \
ros-humble-async-web-server-cpp \
ros-humble-tf-transformations \
ros-humble-rmw-fastrtps-cpp \
ros-humble-ros-testing \
ros-humble-vision-opencv \
ros-humble-pcl-conversions \
software-properties-common \
ros-humble-ament-cmake-nose \
ros-humble-cv-bridge \
ros-humble-pcl-conversions \
ros-humble-tf-transformations \
ros-humble-ament-cmake-nose \
python3-nose \
ros-humble-lgsvl-msgs \
ros-humble-pcl-conversions \
ros-humble-ros-testing \
python3-matplotlib \
python3-scipy \
ros-humble-rviz2 \
ros-humble-robot-localization \
byobu \
ros-humble-velodyne-pointcloud \
nvidia-cuda-toolkit \
alsa-base \
alsa-utils \
libsndfile1-dev \
espeak \
ffmpeg \
libespeak1