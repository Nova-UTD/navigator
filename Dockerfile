ARG CARLA_WS=/home/share/carla

# multi-stage for caching
FROM ros:foxy

# clone overlay source
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    apt install -y git cmake python3-pip && \
    pip3 install -U colcon-common-extensions vcstool

# This variable tells future scripts that user input isn't available during the Docker build.
ENV DEBIAN_FRONTEND noninteractive

COPY ./docker/install-dependencies.sh /tmp/install-dependencies.sh
RUN apt update && apt install -y software-properties-common
RUN /tmp/install-dependencies.sh && rm -rf /var/lib/apt/lists/*

# RUN pip3 install -r scena

ENV ROS_VERSION 2

WORKDIR /navigator
COPY ./docker/entrypoint.sh /opt/entrypoint.sh
ENTRYPOINT [ "/opt/entrypoint.sh" ]