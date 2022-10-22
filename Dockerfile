ARG OVERLAY_WS=/opt/ws

# multi-stage for caching
FROM ros:foxy

# clone overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    apt install -y git cmake python3-pip && \
    pip3 install -U colcon-common-extensions vcstool

# This variable tells future scripts that user input isn't available during the Docker build.
ENV DEBIAN_FRONTEND noninteractive

WORKDIR $OVERLAY_WS

COPY ./docker/install-dependencies.sh /tmp/install-dependencies.sh
RUN apt update && apt install -y software-properties-common
RUN /tmp/install-dependencies.sh && rm -rf /var/lib/apt/lists/*

ENV ROS_VERSION 2

# Clone the official CARLA ROS bridge for the leaderboard, including a specific commit (for consistency)
RUN git clone --recurse-submodules -b leaderboard-2.0 --single-branch https://github.com/carla-simulator/ros-bridge /sim-bridge


WORKDIR /navigator
COPY ./docker/entrypoint.sh /opt/entrypoint.sh
ENTRYPOINT [ "/opt/entrypoint.sh" ]