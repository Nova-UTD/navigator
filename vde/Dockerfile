ARG ROS_DISTRO=foxy

# NOTE: When changing the ROS version, don't forget to update
    # ROS_DISTRO and CODENAME (Ubuntu release name)
    # ROS's official Dockerfiles are found at:
    # https://github.com/osrf/docker_images

FROM ros:$ROS_DISTRO
ARG CODENAME=focal

# Disable non-free repositories
RUN if [ "$(uname -m)" = "x86_64" ]; then \
        echo "\
deb http://archive.ubuntu.com/ubuntu/ ${CODENAME} main universe\n\
deb http://archive.ubuntu.com/ubuntu/ ${CODENAME}-backports main universe\n\
deb http://archive.ubuntu.com/ubuntu/ ${CODENAME}-updates main universe\n\
deb http://security.ubuntu.com/ubuntu/ ${CODENAME}-security main universe\n\
" > /etc/apt/sources.list; \
    else \
        echo "\
deb http://ports.ubuntu.com/ubuntu-ports/ ${CODENAME} main universe\n\
deb http://ports.ubuntu.com/ubuntu-ports/ ${CODENAME}-backports main universe\n\
deb http://ports.ubuntu.com/ubuntu-ports/ ${CODENAME}-updates main universe\n\
deb http://ports.ubuntu.com/ubuntu-ports/ ${CODENAME}-security main universe\n\
" > /etc/apt/sources.list; \
    fi

# set the locale
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        gettext-base \
        locales \
        keyboard-configuration && \
    rm -rf /var/lib/apt/lists/*
RUN locale-gen en_US.UTF-8; dpkg-reconfigure -f noninteractive locales
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8
ENV LC_ALL en_US.UTF-8



# download node.js?
RUN echo deb https://deb.nodesource.com/node_10.x ${CODENAME} main | tee /etc/apt/sources.list.d/nodesource.list
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key 1655A0AB68576280

# COPY ros-deps /tmp/
RUN apt-get update && \
    apt-get install -y \
      python3-vcstool \
      ros-$ROS_DISTRO-cyclonedds \
      ros-$ROS_DISTRO-rmw-cyclonedds-cpp 
    # /bin/bash /tmp/ros-deps && \
    # rm -rf /var/lib/apt/lists/* /tmp/ros-deps

# install from ./apt-packages file using apt
COPY apt-packages /tmp/
RUN apt-get update && \
    apt-get install -y \
        $(cat /tmp/apt-packages | cut -d# -f1 | envsubst) \
    && rm -rf /var/lib/apt/lists/* /tmp/apt-packages


RUN echo 'ALL ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo 'Defaults env_keep += "DEBUG ROS_DISTRO"' >> /etc/sudoers

# install from ./pip3-packages using pip
# Currently this is just pip-tools and pylint
COPY pip3-packages /tmp/
RUN pip3 install -U \
        $(cut -d# -f1 </tmp/pip3-packages) \
    && rm -rf /root/.cache /tmp/pip-* /tmp/pip3-packages

# Clone and install udpreplay, which reproduces network activity
    # from a pcap file. Not sure what this means exactly.
RUN git clone https://github.com/rigtorp/udpreplay && mkdir -p udpreplay/build \
      && cd udpreplay/build && cmake .. && make && make install \
      && cd - && rm -rf udpreplay/

# This adds an indicator in the terminal for git status
    # when you're currently in a git repository.
COPY bashrc-git-prompt /
RUN cat /bashrc-git-prompt >> /etc/skel/.bashrc && \
    rm /bashrc-git-prompt

# Copy config file for GDB, a popular debugger for C++ and
    # other languages. The config currently disables GDB's
    # address stability feature, which basically gives the
    # same memory address to pointers across debugging
    # sessions. With the current config, GDB will use the
    # kernel's randomly assigned memory addresses for
    # program variables. Not sure why this is here, but
    # there has to be a good reason for it.
COPY gdbinit /etc/gdb/


# ===================== CLEAN UP ZONE ===================== #
# Commands in the cleanup zone will be cleaned up before every release
# and put into the correct place.
RUN apt-get update \
  && apt-get install -y \
    unzip \
    ca-certificates \
    libx11-6 \
    libxau6 \
    libxcb1 \
    libxdmcp6 \
    libxext6 \
    libvulkan1 \
    libgl1 \
    libgtk2.0-0 \
    vulkan-utils \
  && apt-get clean

ADD "https://gitlab.com/nvidia/container-images/vulkan/raw/master/nvidia_icd.json" /etc/vulkan/icd.d/nvidia_icd.json
RUN chmod 644 /etc/vulkan/icd.d/nvidia_icd.json

# ===================== END OF CLEAN UP ZONE ===================== #

# Do full package upgrade as last step
# to avoid disrupting layer caching
RUN apt-get update && \
    apt-get -y dist-upgrade && \
    rm -rf /var/lib/apt/lists/*

# It looks like env.sh sources ROS, then sources
    # other workspaces under /opt that have an
    # "env.sh" file. Not sure when this is run.
COPY env.sh /etc/profile.d/ade_env.sh

# Custom aliases and settings for git
COPY gitconfig /etc/gitconfig


COPY entrypoint /ade_entrypoint
COPY autoware-setup.sh /tmp/
COPY autoware-build.sh /tmp/
COPY colcon-defaults.yaml /usr/local/etc/colcon-defaults.yaml
RUN chmod +x /ade_entrypoint && \
    chmod +x /tmp/autoware-setup.sh && \
    chmod +x /tmp/autoware-build.sh && \
    echo "export COLCON_DEFAULTS_FILE=/usr/local/etc/colcon-defaults.yaml" >> \
    /etc/skel/.bashrc

# Add a welcome to our new bash
RUN echo "echo \"This is the\"" >> \
    /etc/skel/.bashrc
RUN echo "figlet \"Voltron\"" >> \
    /etc/skel/.bashrc
RUN echo "echo Development Environment" >> \
    /etc/skel/.bashrc

# Add our "docker" user, give it a home directory and shell,
    # give it a password, add "docker" to the sudo group,
    # and give access to its home directory
RUN useradd --create-home --shell /bin/bash docker && \
    echo "docker:docker" | chpasswd && \
    adduser docker sudo && \
    chown -R docker: /home/docker

USER docker
WORKDIR /home/docker

RUN /tmp/autoware-setup.sh

USER root

# An ENTRYPOINT is an executable run when the container
    # starts.
RUN /tmp/autoware-build.sh
ENTRYPOINT ["/ade_entrypoint"]
CMD ["/bin/sh", "-c", "trap 'exit 147' TERM; tail -f /dev/null & wait ${!}"]
