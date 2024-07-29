---
layout: default
---

{: .fs-6 }
Navigator is a simple, end-to-end, and open-source autonomous driving framework.

[System overview](/navigator/system-overview){: .btn .btn-primary .fs-5 .mb-4 .mb-md-0 .mr-2 } [View it on GitHub](https://github.com/nova-utd/navigator){: .btn .fs-5 .mb-4 .mb-md-0 }

## Why Navigator?
Despite major advances in autonomous driving research, there has yet to exist a single framework that is both simple and extensible, all while being public and transparent.

Navigator is our answer to this delimma. It's built on standard technologies, is kept as simple as possible, and its modular design makes adding new features straightforward.

## System requirements
- [**Docker**](https://docs.docker.com/desktop/)
- [**CARLA 0.9.13**](https://carla.readthedocs.io/en/latest/start_quickstart/) for simulated testing (optional)
- For best results, a dedicated GPU with at least 6 GB of memory, ideally 8 GB
- x86_64 CPU
- About 20 GB of disk space

## Installation

> Note: Lines with "$" are on host, while lines with "#" are within the container.

1. [Install Docker](https://www.docker.com/get-started/). 
> Nova Members: Docker is already installed on the Quad.

2. Choose a number between 0-100 and use this for your user-wide ROS_DOMAIN_ID environment variable. Add the following line to your `.bashrc` file 
```
export ROS_DOMAIN_ID=57
```
3. Clone our repository (checkout the `dev` branch if you plan to do development, or `main` if you want the most recently release)
```
$ git clone -b dev git@github.com:Nova-UTD/navigator.git
$ cd navigator
```
> Any contributor: If simply running the stack is the goal, then checking out `dev` or `main` can make sense. If you aim to contribute, you should create your own fork of the `dev` branch and check that repository out instead. More details about contributing [can be found here](contributing/contributing-overview.md).

4. Build our Docker container. The sequence of commands used to build the container is given in the `Dockerfile` file and the build and run parameters are specified in the `docker-compose.yml` file.
```
$ docker compose build navigator
```
> Nova Members: A Docker image for Navigator already exists on the Quad, so there is no need to rebuild it (unless you've changed it!).
> Other Users: Some customization of the `docker-compose.yml` file may be needed to link it up with your installation of Carla. You should be able to build the image without doing that first.

5. Run the Docker image. Here we are specifically launching the container so it is configured to run with the CARLA simulator:
```
$ docker compose run navigator_carla
```

6.  Now that you're in the container, build the navigator ROS workspace:
```
# colcon build --symlink-install
``` 
That's it!
> Note that if you've build the workspace outside the container or want to start with a fresh build, you can delete the `build`, `install`, and `log` directories and call `colcon build` with the following extra flag:
```
# rm -r -f build
# rm -r -f install
# colcon build --symlink-install --cmake-clean-cache
```

## CARLA Simulator
1. Make sure you've installed Navigator using the steps above.
2. On the host, start CARLA: `./CarlaUE4.sh` See the [CARLA docs](https://carla.readthedocs.io/en/latest/start_quickstart/#running-carla) for more info. Note that CARLA runs natively on the host, while Navigator and the CARLA-ROS2 bridge run in (separate) containers. Currently it is standard for Carla to output the following, which looks a bit like an error, but is fine:
```
chmod: changing permissions of '/home/share/carla/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping': Operation not permitted
4.26.2-0+++UE4+Release-4.26 522 0
Disabling core dumps.
```
> Nova Members: On the Quad, CARLA is installed at `/home/share/carla/`, so you can add an alias to the `.bashrc` file with the line `alias runcarla="/home/share/carla/CarlaUE4.sh"`. The preferred way to interact with the simulator is to run it offline and also on a unique port. You can add the following lines to your `.bashrc` file to make this the default (notice that this uses your ROS_DOMAIN_ID to make the port unique):
```
export CARLA_PORT=$(($ROS_DOMAIN_ID+2000))
alias runcarla="/home/share/carla/CarlaUE4.sh -carla-rpc-port=$CARLA_PORT -RenderOffScreen"
```

### CARLA-ROS2 Bridge
The current CARLA-ROS2 bridge, that allows ROS to communicate with the CARLA simulator is compatible with ROS2 Foxy, not the ROS2 Humble we use for Navigator.  We use a second Docker container to run the CARLA-ROS2 bridge.

3. The docker files and ROS packages that act as the interface between Navigator and CARLA are located in the `carla_interface` repository. Clone this repository using a new terminal window:
```
$ git clone -b dev git@github.com:Nova-UTD/carla_interface.git
$ cd carla_interface
```

4. Similar to Navigator, the `Dockerfile` and `docker-compose.yml` files in the repository let you build the image:
```
$ docker compose build carla_bridge
```
> Nova Members: The Quad should already have an image for this Docker container built.

5. Run the Docker container:
```
$ docker compose run carla_bridge
```
The `entrypoint.sh` script (in the `/docker` directory) sources ROS2 (Foxy) and sets environment variables needed for the bridge.

### Launching a Demo
Launching a demo requires three terminal windows. Recall `$` are commands run on the host and `#` are commands run in a container.
1. In the first terminal window, launch Carla using `runcarla` (assumes you set up the alias described above).
2. In the second terminal window, run the CARLA-ROS2 bridge container (should be in the `carla_interface` respository root directory) and then launch the carla interface nodes:
```
$ docker compose run carla_interface
# ros2 launch launch.carla_interface.py
```
3. In the third terminal window, run the Navigator container (should be in the `navigator` respository root directory) and then launch the Navigator stack:
```
$ docker compose run navigator_carla
# ros2 launch launches/launch.carla.py
```
> Note: There are several different launch configurations in the `launches` directory and several of these are nested. The node definitions for the Navigator launch script are located in the file `launches/launch_node_definitions.py`.

## Real-world use
To run in the real-world, you'll need to provide your own LiDAR stream, map data, and throttle/brake/steering interface. We provide our own interface, for use with our specific hardware, as an example.
