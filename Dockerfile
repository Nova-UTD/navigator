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

RUN /tmp/rosdep.sh && rm -rf /var/lib/apt/lists/*

# RUN echo "\
# repositories: \n\
#   ros2/demos: \n\
#     type: git \n\
#     url: https://github.com/ros2/demos.git \n\
#     version: ${ROS_DISTRO} \n\
# " > ../overlay.repos
# RUN vcs import ./ < ../overlay.repos

# copy manifests for caching
# WORKDIR /opt
# RUN mkdir -p /tmp/opt && \
#     find ./ -name "package.xml" | \
#       xargs cp --parents -t /tmp/opt && \
#     find ./ -name "COLCON_IGNORE" | \
#       xargs cp --parents -t /tmp/opt || true
WORKDIR $OVERLAY_WS

COPY src/ src/

# Install using rosdep. NOTE: Rosdep's -r will continue despite errors.
RUN apt update && \
    apt-get install -y software-properties-common && \
    add-apt-repository ppa:borglab/gtsam-release-4.0 && \
    apt install -y libgtsam-dev libgtsam-unstable-dev && \
    rosdep update && rosdep install -y -r --from-paths src --ignore-src
RUN apt update && apt install ros-foxy-lgsvl-bridge
# install overlay dependencies


# COPY --from=cacher $OVERLAY_WS/src ./src


# build overlay source
# COPY --from=cacher $OVERLAY_WS/src ./src
# ARG OVERLAY_MIXINS="release"
# RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
#     colcon build \
#       --packages-select \
#         demo_nodes_cpp \
#         demo_nodes_py \
#       --mixin $OVERLAY_MIXINS
ENV ROS_VERSION 2

RUN . /opt/ros/foxy/setup.sh && colcon build

# source entrypoint setup
# ENV OVERLAY_WS $OVERLAY_WS
# RUN sed --in-place --expression \
#       '$isource "$OVERLAY_WS/install/setup.bash"' \
#       /ros_entrypoint.sh

# run launch file

COPY param/ param/
COPY data/ data/
COPY main.launch.py main.launch.py

ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

CMD ["ros2", "launch", "main.launch.py"]