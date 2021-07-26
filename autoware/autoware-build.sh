#!/bin/bash
echo -e "$(tput -T xterm setaf 3)$(tput -T xterm smso)[VDE]$(tput -T xterm rmso)$(tput -T xterm sgr0) Sourcing ROS"
source "/opt/ros/$ROS_DISTRO/setup.bash"
echo -e "$(tput -T xterm setaf 3)$(tput -T xterm smso)[VDE]$(tput -T xterm rmso)$(tput -T xterm sgr0) Building Autoware"
echo "export COLCON_DEFAULTS_FILE=/path/to/AutowareAuto/tools/ade_image/colcon-defaults.yaml" >> /home/docker/.bashrc
export COLCON_DEFAULTS_FILE=/path/to/AutowareAuto/tools/ade_image/colcon-defaults.yaml
# cd /opt/AutowareAuto && colcon build --packages-skip tvm_vendor --continue-on-error
cd /opt/
apt install wget
wget https://autoware-auto.s3.us-east-2.amazonaws.com/releases/1.0.0/autoware_auto_1.0.0_foxy_arm64.tar.gz
tar -xzf autoware_auto_1.0.0_foxy_arm64.tar.gz
rm -r AutowareAuto
mv opt/AutowareAuto/ AutowareAuto