ARG OVERLAY_WS=/opt/ws

# multi-stage for caching
FROM ros:eloquent

# clone overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY rosdep.sh /tmp/rosdep.sh
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    apt install -y git cmake python3-pip && \
    pip3 install -U colcon-common-extensions vcstool

ENV DEBIAN_FRONTEND noninteractive
RUN /tmp/rosdep.sh && rm -rf /var/lib/apt/lists/*
WORKDIR $OVERLAY_WS

# Install using rosdep. NOTE: Rosdep's -r will continue despite errors.
RUN apt update && \
    apt-get install -y software-properties-common && \
    add-apt-repository ppa:borglab/gtsam-release-4.0 && \
    apt install -y libgtsam-dev libgtsam-unstable-dev

RUN rosdep update --include-eol-distros

COPY src/ src/

RUN rosdep install -y -r --from-paths src --ignore-src

ENV ROS_VERSION 2

RUN . /opt/ros/eloquent/setup.sh && colcon build

COPY param/ param/
COPY data/ data/
COPY main.launch.py main.launch.py

ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

CMD ["ros2", "launch", "main.launch.py"]