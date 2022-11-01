#!/bin/sh

apt-get update

apt-get install -y ros-foxy-diagnostic-updater \
ros-foxy-velodyne-msgs \
ros-foxy-velodyne-driver \
ros-foxy-velodyne-laserscan \
ros-foxy-velodyne-pointcloud \
ros-foxy-autoware-auto-msgs \
ros-foxy-lanelet2-core \
ros-foxy-lanelet2-io \
ros-foxy-lanelet2-projection \
ros-foxy-lanelet2 \
ros-foxy-tf-transformations \
ros-foxy-ros-testing \
ros-foxy-lgsvl-msgs \
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
ros-foxy-derived-object-msgs

apt-get install byobu \

pip3 install pymap3d==2.9.1 \
dictor \
requests \
opencv-python==4.2.0.32 \
pygame \
tabulate \
pexpect \
transforms3d \
pep8 \
autopep8 \
cmake_format==0.6.11 \
pylint \
simple-pid \
py-trees==0.8.3 \
networkx==2.2 \
Shapely==1.7.1 \
psutil \
xmlschema==1.0.18 \
ephem \
matplotlib \
six \
simple-watchdog-timer \
numpy==1.18.4 \
distro

# Conflicting dependencies are noted here
Shapely==1.6.4.post2