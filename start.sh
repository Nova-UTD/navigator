#!/bin/sh
docker run -it -v /home/wheitman/navigator:/navigator -v /home/share/carla:/workspace --net=host -v $HOME/.Xauthority:/root/.Xauthority -e="DISPLAY" navigator