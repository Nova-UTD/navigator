#!/bin/sh

# To build: 'docker build -t navigator .'

# The below command is to run a docker container. The container must already be built. 
docker run \
    -it \
    --rm \
    -v $PWD:/navigator \
    -v /home/share/carla:/workspace \
    --net=host \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -e="DISPLAY" \
    --name navigator \
    navigator
