#!/bin/bash

echo -e "\e[1;33m=====================================================================\e[0m"
echo -e "\e[1;33m▀█▄   ▀█▀                   ██                    ▄                   \e[0m"
echo -e "\e[1;33m █▀█   █   ▄▄▄▄   ▄▄▄▄ ▄▄▄ ▄▄▄    ▄▄▄ ▄  ▄▄▄▄   ▄██▄    ▄▄▄   ▄▄▄ ▄▄  \e[0m"
echo -e "\e[1;33m █ ▀█▄ █  ▀▀ ▄██   ▀█▄  █   ██   ██ ██  ▀▀ ▄██   ██   ▄█  ▀█▄  ██▀ ▀▀ \e[0m"
echo -e "\e[1;33m █   ███  ▄█▀ ██    ▀█▄█    ██    █▀▀   ▄█▀ ██   ██   ██   ██  ██     \e[0m"
echo -e "\e[1;33m▄█▄   ▀█  ▀█▄▄▀█▀    ▀█    ▄██▄  ▀████▄ ▀█▄▄▀█▀  ▀█▄▀  ▀█▄▄█▀ ▄██▄    \e[0m"
echo -e "\e[1;33m                                ▄█▄▄▄▄▀                               \e[0m"
echo -e "\e[1;33m=====================================================================\e[0m"    
echo "Developed by Nova, a student-run autonomous driving group at UT Dallas"
echo "Find out more at https://nova-utd.github.io/navigator"   
echo "🐢 Sourcing ROS2 Humble..."
source /opt/ros/humble/setup.bash

#echo "🔗 Configuring the ROS DDS..."
# FASTRTPS_DEFAULT_PROFILES_FILE=/navigator/data/fastrtps.xml
# RMW_FASTRTPS_USE_QOS_FROM_XML=1
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# source /opt/cyclone_ws/install/setup.bash

echo "🧭 Sourcing Navigator..."
source /navigator/install/setup.bash

# echo "🔌 Setting up CARLA API..."
# export CARLA_ROOT="/workspace/simulator"
# export SCENARIO_RUNNER_ROOT="/workspace/scenario_runner"
# export LEADERBOARD_ROOT="/workspace/leaderboard"
# export ROS_BRIDGE_ROOT="/workspace/ros-bridge"
# export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":"${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg":${PYTHONPATH}
# source /workspace/install/setup.bash

echo "👍 Finished environment setup"

if [ -z ${@+x} ]; then
    exec bash
else 
    exec bash -c "$@"
fi