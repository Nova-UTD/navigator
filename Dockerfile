# multi-stage for caching
FROM ros:humble

# This variable tells future scripts that user input isn't available during the Docker build.
ENV DEBIAN_FRONTEND noninteractive

LABEL maintainer="Nova"

# Get wget first so we can get the NVIDIA keys
#RUN apt-get install -y wget

# To fix GPG key error when running apt-get update
# https://developer.nvidia.com/blog/updating-the-cuda-linux-gpg-repository-key/
###RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub
###RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu2204/x86_64/7fa2af80.pub
#RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.0-1_all.deb && dpkg -i cuda-keyring_1.0-1_all.deb

# Install torch
#RUN apt install -y nvidia-cuda-toolkit
#RUN pip3 install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu101

### REMOVE?? ###
### RUN pip3 install -U openmim && mim install mmcv-full && git clone https://github.com/open-mmlab/mmsegmentation.git /mmsegmentation
### WORKDIR /mmsegmentation
### RUN pip install -v -e . && mim download mmsegmentation --config pspnet_r50-d8_512x1024_40k_cityscapes --dest .

# Install MMCV  https://github.com/open-mmlab/mmcv (requires PyTorch)
#RUN pip3 install -U openmim && mim install mmcv-full

# apt-get & apt installs
# Also need to upgrade the apt packages, otherwise there are incompatibilities running ros2/rviz2
RUN apt-get update && apt update && apt upgrade -y && \
    # =================================
    ## apt-get installs ===============
    apt-get install -y \
    # important linux tool
    wget \  
    # install python libraries
    python3-pip \
    # be able to use git inside container
    git \
    #
    fprintd \
    #
    libfprint-2-2:amd64 \
    #
    libgl1-mesa-glx \
    #
    libglib2.0-0 \
    #
    libpam-fprintd:amd64 \
    #
    libsm6 \
    #
    libxrender-dev \
    #
    libxext6 \
    #
    ninja-build \
    #
    ros-humble-diagnostic-updater \
    # =================================
    # apt installs ====================
    && apt install -y \
    #
    alsa-base \
    #
    alsa-utils \
    #
    byobu \
    #
    espeak \
    #
    ffmpeg \
    #
    libespeak1 \
    #
    libsndfile1-dev \
    #
    minicom \
    #
    python3-matplotlib \
    #
    python3-nose \
    #
    python3-scipy \
    #
    ros-humble-ament-cmake* \
    # needed for ros2_numpy package (maybe others)
    ros-humble-ament-cmake-nose \
    #
    ros-humble-async-web-server-cpp \
    #
    ros-humble-cv-bridge \
    #
    ros-humble-image-transport \
    #
    ros-humble-joy-linux \
    #
    ros-humble-lgsvl-msgs \
    # Nav2 stack - used for path planning
    ros-humble-navigation2 \
    #
    ros-humble-pcl-conversions \
    #
    ros-humble-pcl-ros \
    #
    ros-humble-rmw-fastrtps-cpp \
    #
    ros-humble-robot-localization \
    #
    ros-humble-rosbridge-server \
    #
    ros-humble-ros-testing \
    # rqt provides oscilliscope-style plotting of topics, etc
    ros-humble-rqt* \
    #
    ros-humble-rviz2 \
    #
    ros-humble-tf-transformations \
    # ROS Support for Velodyne LiDAR
    ros-humble-velodyne-driver \
    ros-humble-velodyne-laserscan \
    ros-humble-velodyne-msgs \
    ros-humble-velodyne-pointcloud \
    #
    ros-humble-vision-opencv \
    #
    software-properties-common \
    #
    libopen3d-dev\
    # cleanup to make image smaller
    && apt clean && rm -rf /var/lib/apt/lists/*

# pip3 installs
RUN pip3 install \
    #
    autopep8==2.3.1 \
    #
    cmake_format==0.6.11 \
    #
    dictor==0.1.12 \
    #
    ephem==4.1.5 \
    # Used for the road signs classifier API call to the model.
    inference-sdk==0.26.0 \
    #
    matplotlib==3.6.0 \
    #  (possibly needs to be version 2.2)
    networkx==3.3 \
    # Scientific Computing - used widely
    numpy==1.23.5 \
    #
    open3d==0.19.0 \
    #
    opencv-python==4.10.0.84 \
    #
    pathfinding==1.0.11 \
    #
    pep8==1.7.1 \
    #
    pexpect==4.9.0 \
    #
    psutil==5.9.0 \
    #
    pygame==2.6.0 \
    #
    pylint==3.2.7 \
    #
    pymap3d==2.9.1 \
    #
    pynmea2==1.19.0 \
    #
    pyproj==3.6.1 \
    #
    pyserial==3.5 \
    #
    python-can==4.4.2 \
    #
    py-trees==0.8.3 \
    #
    requests==2.28.2 \
    #
    scikit-image==0.24.0 \
    # Scientific Computing - used widely
    scipy==1.14.1 \
    #
    shapely==2.0.2 \
    #
    simple-pid==2.0.1 \
    #
    simple-watchdog-timer==0.1.1 \
    #
    six==1.16.0 \
    #
    tabulate==0.9.0 \
    #
    transforms3d==0.4.2 \
    #
    xmlschema==1.0.18 \
    # distro \
    torch==2.1.2 \
    torchvision==0.16.2 \
    # openmim \
    # mmcv-full==1.6.0 \
    # packaging \
    # prettytable \
    # python-dateutil>=2.1 \
    # kiwisolver>=1.0.1 \
    # pyparsing>=2.0.1 \
    # cycler>=0.10 \
    # wcwidth \
    # six>=1.5 \
    # mmcls \
    easydict==1.13 \
    g2o-python==0.0.12 \
    rosbags==0.10.4 

# Install Black for Python code formatting.
RUN pip3 install black==24.10.0

# install mmdetection3d for 3d object detection
RUN pip3 install -U openmim
RUN mim install mmengine
RUN mim install 'mmcv>=2.0.0rc4'
RUN mim install 'mmdet>=3.0.0'
RUN mim install "mmdet3d>=1.1.0"

WORKDIR /

# https://stackoverflow.com/questions/66669735/ubuntu-20-04-cant-find-pcl-because-of-incorrect-include-directory-after-install
RUN mkdir /lib/x86_64-linux-gnu/cmake/pcl/include && ln -s /usr/include/pcl-1.10/pcl /lib/x86_64-linux-gnu/cmake/pcl/include/pcl

# Lemon
# Library for Efficient Modeling and Optimization in Networks. 
# It is a C++ template library providing efficient implementations of common data structures and algorithms 
# with focus on combinatorial optimization tasks connected mainly with graphs and networks.
RUN git clone https://github.com/The-OpenROAD-Project/lemon-graph.git && cd lemon-graph \
    # This specific commit is for Lemon-1.3.1. See https://github.com/The-OpenROAD-Project/lemon-graph/commits/master.
    && git checkout 62ac753 \
    && mkdir build && cd build && cmake .. && make && make install

# RUN useradd -ms /bin/bash docker
RUN usermod -a -G dialout root
RUN usermod -a -G tty root
# USER docker

ENV ROS_VERSION 2
WORKDIR /navigator
RUN rm -rf kiss-icp
RUN git clone https://github.com/pulipakaa24/kiss-icp.git && cd kiss-icp && make editable
RUN pip3 install map-closures==2.0.2 kiss-slam==0.0.2

COPY ./docker/entrypoint.sh /opt/entrypoint.sh

ENTRYPOINT [ "/opt/entrypoint.sh" ]
