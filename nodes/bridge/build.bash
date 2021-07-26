#!/bin/bash

# Source ROS and Autoware.Auto (is this necessary?)
# source /opt/AutowareAuto/setup.bash

# Source our custom dependencies
ls /opt/dep_ws
source /opt/dep_ws/install/setup.bash

# Build our package
cd /opt/ws/
colcon build