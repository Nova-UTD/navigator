ARG OVERLAY_WS=/opt/ws

#FROM nvidia/cuda:10.1-devel-ubuntu18.04

# Eloquent setup
#RUN echo 'Etc/UTC' > /etc/timezone &&     ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime &&     apt-get update &&     apt-get install -q -y --no-install-recommends tzdata &&     rm -rf /var/lib/apt/lists/*
#RUN apt-get update && apt-get install -q -y --no-install-recommends     dirmngr     gnupg2     && rm -rf /var/lib/apt/lists/*
#RUN echo "deb http://snapshots.ros.org/eloquent/final/ubuntu bionic main" > /etc/apt/sources.list.d/ros2-snapshots.list
#RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA
#ENV LANG=C.UTF-8
#ENV LC_ALL=C.UTF-8
#ENV ROS_DISTRO=eloquent
#RUN apt-get update && apt-get install -y --no-install-recommends     ros-eloquent-desktop
#ENTRYPOINT ["/ros_entrypoint.sh"]
#CMD ["bash"]
#RUN apt-get update && apt-get install --no-install-recommends -y     build-essential     git     python3-colcon-common-extensions     python3-colcon-mixin     python3-rosdep     python3-vcstool     && rm -rf /var/lib/apt/lists/*
#RUN rosdep init &&   rosdep update --rosdistro $ROS_DISTRO
#RUN colcon mixin add default       https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml &&     colcon mixin update &&     colcon metadata add default       https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml &&     colcon metadata update
#RUN apt-get update && apt-get install -y --no-install-recommends     ros-eloquent-ros-base=0.8.5-1*     && rm -rf /var/lib/apt/lists/*
# End Eloquent setup

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

RUN . /opt/ros/eloquent/setup.sh && colcon build --symlink-install --packages-skip fast_gicp

COPY param/ param/
COPY data/ data/
COPY main.launch.py main.launch.py

ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

CMD ["ros2", "launch", "main.launch.py"]