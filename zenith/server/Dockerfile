FROM navigator:latest

# Set working directory to /navigator/zenith/server.
RUN mkdir -p /navigator/zenith/server
WORKDIR /navigator/zenith/server

# Install starship cli prompt.
# Starship is used to prettify the terminal.
ENV TERM=xterm-256color
RUN curl -sS https://starship.rs/install.sh | sh -s -- -y

# Setup starship configuration
RUN mkdir -p /root/.config
RUN echo 'command_timeout = 1000\nformat = """$directory$git_branch[❯](bold red)[❯](bold yellow)[❯](bold green) """' \
    > /root/.config/starship.toml
RUN echo 'eval "$(starship init bash)"' >> /root/.bashrc

# Install Python dependencies.
COPY ./requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

# Copy all files.
COPY . .

# Source ROS2 and run server on port 8000.
# Must be run with the following:
#   - src is mounted at /navigator/src.
#   - install is mounted at /navigator/install.
#   - build is mounted at /navigator/build.
#   - log is mounted at /navigator/log.
ENTRYPOINT ["/bin/bash", "-c", ". /navigator/install/setup.bash && NAVIGATOR_SRC=/navigator/src uvicorn api:app --host 127.0.0.1 --port 8000 --log-config=log_conf.yaml"]

