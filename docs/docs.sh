#!/bin/bash

port=$((8000 + $ROS_DOMAIN_ID))

docker run --rm --volume="$PWD:/srv/jekyll:Z" -e JEKYLL_UID=$UID -e JEKYLL_GID=$UID -p $port:$port jekyll/jekyll jekyll serve --port $port
