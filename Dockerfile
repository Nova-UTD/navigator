ARG OVERLAY_WS=/opt/ws

# multi-stage for caching
FROM ros:foxy

# clone overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY rosdep.sh /tmp/rosdep.sh
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    apt install -y git cmake python3-pip && \
    pip3 install -U colcon-common-extensions vcstool

ENV DEBIAN_FRONTEND noninteractive

WORKDIR $OVERLAY_WS

COPY src/ src/

# Install using rosdep. NOTE: Rosdep's -r will continue despite errors.
RUN apt update && \
    apt-get install -y software-properties-common && \
    add-apt-repository ppa:borglab/gtsam-release-4.0 && \
    apt install -y libgtsam-dev libgtsam-unstable-dev && \
    rosdep update && rosdep install -y -r --from-paths src --ignore-src

RUN /tmp/rosdep.sh && rm -rf /var/lib/apt/lists/*

# FILES HERE should be moved to "rosdep.sh" periodically.
RUN apt update && \
    apt install -y ros-foxy-velodyne \
    ros-foxy-angles \
    libpcap-dev

ENV ROS_VERSION 2

RUN . /opt/ros/foxy/setup.sh && colcon build


COPY param/ param/
COPY params.yaml params.yaml
COPY data/ data/
COPY carla.launch.py carla.launch.py
COPY carla-0.9.13-cp37-cp37m-manylinux_2_27_x86_64.whl carla-0.9.13-cp37-cp37m-manylinux_2_27_x86_64.whl

WORKDIR $OVERLAY_WS/venv

ENV OVERLAY_WS $OVERLAY_WS
RUN curl https://bootstrap.pypa.io/get-pip.py | python3.8
RUN exec bash
RUN pip3 install carla==0.9.12
RUN /usr/bin/python3.7 -m venv venv

RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh