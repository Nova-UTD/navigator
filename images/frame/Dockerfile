FROM vde_pillar:latest

# Copy entire package to /opt/package
COPY . /opt/package/

# Build our package
WORKDIR /opt/package/
RUN . /opt/ros/foxy/setup.sh && \
    . /opt/AutowareAuto/setup.sh && \
    colcon build && \
    . install/setup.sh

CMD . install/setup.sh && ros2 launch ${package_name} container.launch.py