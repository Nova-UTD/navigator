FROM vde_base:latest
WORKDIR /opt/ws
ARG IMAGE_NAME=bridge
COPY . /opt/ws/bridge/
RUN ls /opt/ws/bridge

COPY build.bash /tmp/build.bash
RUN sudo chmod +x /tmp/build.bash && \
    /tmp/build.bash