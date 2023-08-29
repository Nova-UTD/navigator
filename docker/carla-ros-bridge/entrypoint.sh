#!/bin/bash

echo "🦊 Sourcing ROS2 Foxy..."
source /opt/ros/foxy/setup.bash

echo "🚦 Setting up CARLA..."
export CARLA_ROOT="/carla"
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg":${PYTHONPATH}

echo "🌉 Sourcing CARLA-ROS Bridge..."
export ROS_BRIDGE_ROOT="/carla-ros-bridge"
source "${ROS_BRIDGE_ROOT}/install/setup.bash"

alias launch_bridge="ros2 launch /launch/carla_bridge.launch.py"

echo "👍 Finished environment setup."

echo ""
echo "====================================================================="
echo "🚀 To launch the CARLA-ROS2 bridge use: 👀 launch_bridge 👀"
echo " which is an alias for: ros2 launch /launch/carla_bridge.launch.py"
echo "====================================================================="
echo ""

if [ -z ${@+x} ]; then
    exec bash
else 
    exec bash -c "$@"
fi