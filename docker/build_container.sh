docker run \
    -it \
    --rm \
    -v $PWD:/navigator \
    --net=host \
    -e="DISPLAY" \
    novautd/navigator:latest \
    "apt update && apt install -y ros-foxy-rmw-cyclonedds-cpp && colcon build --cmake-clean-cache"