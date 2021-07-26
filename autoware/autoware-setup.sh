#!/bin/bash
export TERM=xterm
echo "$(tput -T xterm setaf 3)$(tput -T xterm smso)[VDE]$(tput -T xterm rmso)$(tput -T xterm sgr0) Preparing for Autoware build"
cd /opt/
sudo apt update
rosdep update
sudo chown -R docker: .
echo -e "$(tput -T xterm setaf 3)$(tput -T xterm smso)[VDE]$(tput -T xterm rmso)$(tput -T xterm sgr0) Cloning Autoware"
git clone https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto.git
cd AutowareAuto
vcs import < autoware.auto.$ROS_DISTRO.repos
rosdep install -y -i --from-paths src
git checkout tags/1.0.0 -b release-1.0.0
