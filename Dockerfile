# multi-stage for caching
FROM ros:humble

# This variable tells future scripts that user input isn't available during the Docker build.
ENV DEBIAN_FRONTEND noninteractive

# To fix GPG key error when running apt-get update
RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/3bf863cc.pub
RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64/7fa2af80.pub

RUN apt-get update && apt-get install -y python3-pip git ninja-build libglib2.0-0 libsm6 libxrender-dev libxext6 libgl1-mesa-glx \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install torch
RUN apt update && apt install -y nvidia-cuda-toolkit
RUN pip3 install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu101
# RUN pip3 install -U openmim && mim install mmcv-full && git clone https://github.com/open-mmlab/mmsegmentation.git /mmsegmentation
# WORKDIR /mmsegmentation
# RUN pip install -v -e . && mim download mmsegmentation --config pspnet_r50-d8_512x1024_40k_cityscapes --dest .

# # Install MMCV
RUN pip3 install -U openmim && mim install mmcv-full

# clone overlay source
# RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
#     apt-get update && \
#     apt install -y git cmake python3-pip && \
#     pip3 install -U colcon-common-extensions vcstool

COPY ./docker/install-dependencies.sh /opt/docker_ws/
RUN apt update && apt install -y software-properties-common && /opt/docker_ws/install-dependencies.sh
COPY ./docker/install-pip-dependencies.sh /opt/docker_ws/
RUN /opt/docker_ws/install-pip-dependencies.sh && rm -rf /var/lib/apt/lists/*

COPY ./docker ./opt/docker_ws

# # Copy and build our modified version of CycloneDDS, important ROS middleware
# COPY ./src/tools/cyclonedds /opt/cyclone_ws/cyclonedds
# COPY ./src/tools/rmw_cyclonedds /opt/cyclone_ws/rmw_cyclonedds
# WORKDIR /opt/cyclone_ws
# RUN . /opt/ros/foxy/setup.sh && colcon build --cmake-clean-cache

# RUN apt update && echo "net.core.rmem_max=8388608\nnet.core.rmem_default=8388608\n" | tee /etc/sysctl.d/60-cyclonedds.conf

#################
# CLEAN UP ZONE #
#################
# Code here should either be moved to install-dependencies.sh or removed
# before each major release.
#RUN apt update && apt install -y ros-foxy-octomap octovis ros-foxy-pcl-ros ros-foxy-tf2-eigen

# https://stackoverflow.com/questions/66669735/ubuntu-20-04-cant-find-pcl-because-of-incorrect-include-directory-after-install
RUN mkdir /lib/x86_64-linux-gnu/cmake/pcl/include && ln -s /usr/include/pcl-1.10/pcl /lib/x86_64-linux-gnu/cmake/pcl/include/pcl

# Lemon
WORKDIR /workspace
RUN apt update && apt install -y wget && wget http://lemon.cs.elte.hu/pub/sources/lemon-1.3.1.tar.gz && tar xvzf lemon-1.3.1.tar.gz && cd lemon-1.3.1 && mkdir build && cd build && cmake .. && make && make install
RUN apt update && apt install -y ros-humble-ament-cmake*
#################
#  END CLEANUP  #
#################

ENV ROS_VERSION 2

WORKDIR /navigator
COPY ./docker/entrypoint.sh /opt/entrypoint.sh

# RUN useradd -ms /bin/bash docker
RUN usermod -a -G dialout root
RUN usermod -a -G tty root
# USER docker

ENTRYPOINT [ "/opt/entrypoint.sh" ]