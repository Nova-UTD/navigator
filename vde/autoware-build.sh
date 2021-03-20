#!/bin/bash
echo -e "$(tput -T xterm setaf 3)$(tput -T xterm smso)[VDE]$(tput -T xterm rmso)$(tput -T xterm sgr0) Sourcing ROS"
source "/opt/ros/$ROS_DISTRO/setup.bash"
echo -e "$(tput -T xterm setaf 3)$(tput -T xterm smso)[VDE]$(tput -T xterm rmso)$(tput -T xterm sgr0) Building Autoware"
cd /opt/AutowareAuto && colcon build