# Here we take our autoware Docker image, then add some of our custom libraries.
FROM vde_base:latest



# ===== ROS Packages =====
# Create a workspace to put all of our dependencies into
RUN ["mkdir","-p","/opt/dep_ws/src"]
WORKDIR /opt/dep_ws/src
RUN ["git", "clone", "https://github.com/Voltron-UTD/voltron_test_utils.git"]
COPY build_ws.bash /tmp/build_ws.bash
RUN chmod +x /tmp/build_ws.bash && /tmp/build_ws.bash
