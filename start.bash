#!/bin/bash


if [[ $* == *-h* ]]
then
    echo "Optional arguments:"
    echo "-h: This help page"
    echo "-b: Start the LGSVL bridge to run with VDE"
    echo "-i: Include an interactive shell for debugging"
    exit 0
fi

if [[ $* == *-b* ]]
then
    echo "Starting lgsvl_bridge"
    lgsvl_bridge
fi

if [[ $* == *-i* ]]
then
    echo "Starting Docker Compose in background..."
    docker-compose up -d
    echo "Starting interactive shell"
    docker exec -it -u docker vde_pillar_1 /bin/bash
else
    echo "Starting Docker Compose..."
    docker-compose up
fi

