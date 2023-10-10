# multi-stage for caching
FROM ros:foxy

# This variable tells future scripts that user input isn't available during the Docker build.
ENV DEBIAN_FRONTEND noninteractive

# Steps followed from here: https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/
# Clone the carla-ros-bridge repo
RUN mkdir -p /carla-ros-bridge
WORKDIR /carla-ros-bridge
RUN git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git src/ros-bridge

# Sourcing ROS (need to run this using bash instead of sh)
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash"

# apt-get & apt installs
# Also need to upgrade the apt packages, otherwise there are incompatibilities running ros2/rviz2
RUN apt-get update && apt update && apt upgrade -y && \
    # pip is needed for rosdep install step below
    apt-get install -y python3-pip && \
    apt install -y \
        # required ROS packages for rosdep install step below
        ros-foxy-rviz2 \
        ros-foxy-cv-bridge \
        ros-foxy-pcl-conversions \
        ros-foxy-pcl-ros \
        ros-foxy-ackermann-msgs \
        ros-foxy-rqt-gui-py \
        ros-foxy-python-qt-binding \
        ros-foxy-derived-object-msgs \
    # cleanup to make image smaller
    && apt clean && rm -rf /var/lib/apt/lists/*

# Now install ROS dependencies
RUN rosdep update && rosdep install --from-paths src --ignore-src -r

# Build the CARLA ROS2 bridge
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build"

# RUN useradd -ms /bin/bash docker
RUN usermod -a -G dialout root
RUN usermod -a -G tty root
# USER docker

ENV ROS_VERSION 2

COPY ./entrypoint.sh /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]