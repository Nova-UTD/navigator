# Base ROS 2 Humble image with desktop + turtlesim
FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-lc"]

# System deps (+ CycloneDDS RMW)
RUN apt-get update && apt-get install -y \
    python3-pip python3-tk python3-colcon-common-extensions \
    python3-ament-package \
    ros-humble-turtlesim x11-apps \
    ros-humble-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*

# Pin setuptools compatible with Humble, plus wheel
RUN python3 -m pip install --no-cache-dir -U pip && \
    pip3 install --no-cache-dir 'setuptools==65.5.1' 'setuptools_scm<7' wheel

# Overlay workspace (your packages)
WORKDIR /workspaces/overlay
COPY overlay_src/ ./src/

# Build overlay
RUN source /opt/ros/humble/setup.bash && \
    colcon build --merge-install

# Entrypoint
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# DDS + X11 defaults
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=23
ENV ROS_LOCALHOST_ONLY=0
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_SOFTWARE=1

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
