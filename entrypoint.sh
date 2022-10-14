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
echo "Sourcing ROS2 Humble..."
source /opt/ros/humble/setup.bash

echo "Setting up CARLA API..."
export PYTHONPATH="/navigator/data/carla-0.9.13-py3.7-linux-x86_64.egg":${PYTHONPATH}

/bin/bash
