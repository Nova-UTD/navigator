# BUILD COMMANDS
colcon build --symlink-install
Flags:
    --symlink-install
    --cmake-clean-cache
    --packages-select <package_name>

# SOURCE COMMAND
. install/setup.bash

# LAUNCHING COMMANDS
navigator launch {vehicle, carla, perception}
    Ex. navigator launch vehicle

# ROS COMMANDS
rviz2
rqt
rqt_graph
ros2 topic list/echo
etc.

## BUILD SEQUENCE
colcon build --symlink-install
. install/setup.bash

## CLEAN BUILD SEQUENCE
rm -rf build/ install/
colcon build --symlink-install --cmake-clean-cache
. install/setup.bash

### TESTING SEQUENCE
launch carla
launch carla_bridge (carla_interface repo)
launch navigator