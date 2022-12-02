#!/bin/sh

# To build: 'docker build -t navigator .'

# The below command is to run a docker container. The container must already be built. 
# It passes the ROS_DOMAIN_ID from host. To generate this "randomly" based on your username, 
# see https://gist.github.com/wheitman/ceaec50cd4cb79a496f43e6c0e20a8b2
docker run \
    -it \
    --rm \
    -v $PWD:/navigator \
    -v /home/share/carla:/workspace \
    --net=host \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -e="DISPLAY" \
    -e ROS_DOMAIN_ID \
    navigator
