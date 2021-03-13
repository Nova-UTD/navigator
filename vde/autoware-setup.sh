#!/bin/sh
cd /home/docker
sudo apt update
rosdep update
sudo chown -R docker: .
git clone https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto.git
cd AutowareAuto
vcs import < autoware.auto.$ROS_DISTRO.repos
rosdep install -y -i --from-paths src
git checkout tags/1.0.0 -b release-1.0.0