#!/bin/sh

sudo add-apt-repository -y ppa:deadsnakes/ppa
apt-get update
apt-get install -y ros-foxy-robot-localization \
python3.7-dev \
python3.7-venv

pip3 install pymap3d==2.9.1