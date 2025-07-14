FROM ros:humble

# This variable tells future scripts that user input isn't available during the Docker build.
ENV DEBIAN_FRONTEND noninteractive

LABEL maintainer="Nova"

# ========== BEGIN Install System Utilities ==========

RUN apt-get update -y && \
  apt-get install -y \
  wget \
  ninja-build

# ========== END Install System Utilities ==========

# ========== Begin PCL 1.14.1 Installation ==========

WORKDIR /

RUN apt-get update -y && \
  apt-get install -y \
  libboost-all-dev=1.74.0.3ubuntu7 \
  libeigen3-dev=3.4.0-2ubuntu2 \
  libflann-dev=1.9.1+dfsg-11

# Build VTK 9.3.1 from source
# Install dependencies
RUN apt-get update -y && \
  apt-get install -y \ 
  mesa-common-dev \
  mesa-utils \
  freeglut3-dev

RUN wget https://www.vtk.org/files/release/9.3/VTK-9.3.1.tar.gz
RUN tar -xf VTK-9.3.1.tar.gz
WORKDIR /VTK-9.3.1
RUN mkdir build && cd build && \
  cmake -GNinja -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DCMAKE_INSTALL_PREFIX="/usr/local" .. && \
  cmake --build . --target install -- -j$(nproc)

WORKDIR /

# Install optional OpenMP for PCL 
RUN apt-get update -y && \
  apt-get install -y \
  libomp-dev

RUN wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.14.1.tar.gz
RUN tar -xf pcl-1.14.1.tar.gz
WORKDIR /pcl-pcl-1.14.1

RUN cmake -GNinja -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DCMAKE_INSTALL_PREFIX="/usr/local" . && \
  cmake --build . --target install -- -j$(nproc)

# Clean up
RUN rm -dfr /pcl-pcl-1.14.1 && rm /pcl-1.14.1.tar.gz

# ========== End PCL 1.14.1 Installation ==========

# ========== Begin Open3D v0.18 Installation  ==========
WORKDIR /

# Open3D requires libc++-dev to be installed.
RUN apt-get install -y libc++-dev
# Open3D requires Eigen3 to be reachable via <Eigen> instead of <eigen3/Eigen>.
RUN ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

# Install Open3D v0.18 pre-compiled C++ API.
RUN wget https://github.com/isl-org/Open3D/releases/download/v0.18.0/open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz
# Extract the tarball to /tmp
RUN tar -xf open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz -C /tmp

RUN mv -v /tmp/open3d-devel-linux-x86_64-cxx11-abi-0.18.0/lib/cmake/* /usr/local/lib/cmake
RUN mv -v /tmp/open3d-devel-linux-x86_64-cxx11-abi-0.18.0/lib/*.so /usr/local/lib/
RUN mv -v /tmp/open3d-devel-linux-x86_64-cxx11-abi-0.18.0/include/open3d /usr/local/include/
RUN mv -v /tmp/open3d-devel-linux-x86_64-cxx11-abi-0.18.0/share /usr/local/share/open3d

# Cleanup
RUN rm -dfr /tmp/open3d-devel-linux-x86_64-cxx11-abi-0.18.0 && rm /open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz

# ========== End Open3D v0.18 Installation ==========

# ========== Begin OusterSDK Installation ==========

WORKDIR /

# Install dependencies as specified here: https://static.ouster.dev/sdk-docs/cpp/building.html.
RUN apt-get update -y && \
  apt-get install -y \
  libjsoncpp-dev \ 
  libcurl4-openssl-dev \
  libtins-dev \
  libpcap-dev \
  libglfw3-dev \
  libglew-dev \
  libspdlog-dev \
  libpng-dev \
  libflatbuffers-dev

# OusterSDK references JsonCPP as jsoncpp_lib, but the library is installed as jsoncpp.
RUN ln -s /usr/lib/x86_64-linux-gnu/libjsoncpp.so /usr/lib/x86_64-linux-gnu/libjsoncpp_lib.so

RUN wget https://github.com/ouster-lidar/ouster-sdk/archive/refs/tags/release-0.13.1.tar.gz
RUN tar -xf release-0.13.1.tar.gz
WORKDIR /ouster-sdk-release-0.13.1

RUN cmake -GNinja -DCMAKE_INSTALL_PREFIX="/usr/local" \
  -DBUILD_PCAP=ON -DBUILD_OSF=OFF -DBUILD_VIZ=ON -DBUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF . && \
  cmake --build . --target install -- -j$(nproc)

# Fix OusterSDKConfig.cmake to avoid including jsoncpp twice.
# See: https://github.com/open-source-parsers/jsoncpp/pull/1435.
RUN sed -i '/find_dependency(jsoncpp)/c\
  if (TARGET jsoncpp_static AND NOT TARGET JsonCpp::JsonCpp)\n\
  \tadd_library(JsonCpp::JsonCpp INTERFACE IMPORTED)\n\
  \tset_target_properties(JsonCpp::JsonCpp PROPERTIES INTERFACE_LINK_LIBRARIES "jsoncpp_static")\n\
  elseif (TARGET jsoncpp_lib AND NOT TARGET JsonCpp::JsonCpp)\n\
  \tadd_library(JsonCpp::JsonCpp INTERFACE IMPORTED)\n\
  \tset_target_properties(JsonCpp::JsonCpp PROPERTIES INTERFACE_LINK_LIBRARIES "jsoncpp_lib")\n\
  endif ()' \
  /usr/local/lib/cmake/OusterSDK/OusterSDKConfig.cmake

# Clean up
RUN rm -dfr /ouster-sdk-release-0.13.1 && rm /release-0.13.1.tar.gz

# ========== End OusterSDK Installation ==========

# ========== Begin g2o 20230223_git Installation  ==========
WORKDIR /

RUN wget https://github.com/RainerKuemmerle/g2o/archive/refs/tags/20230223_git.tar.gz
RUN tar -xf 20230223_git.tar.gz

WORKDIR /g2o-20230223_git
RUN mkdir build && cd build && \
  cmake -GNinja -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DCMAKE_INSTALL_PREFIX="/usr/local" .. && \
  cmake --build . --target install -- -j$(nproc)

# Clean up
RUN rm -dfr /g2o-20230223_git && rm /20230223_git.tar.gz
# ========== End g2o 20230223_git Installation  ==========

# Install Abseil, an extension of the C++ standard library for easier programming.
RUN apt-get update -y && \
  apt-get install -y libabsl-dev

WORKDIR /navigator
COPY ./docker/entrypoint.sh /opt/entrypoint.sh

ENTRYPOINT [ "/opt/entrypoint.sh" ]
