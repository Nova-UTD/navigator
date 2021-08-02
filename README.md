```
██╗   ██╗██████╗ ███████╗
██║   ██║██╔══██╗██╔════╝
██║   ██║██║  ██║█████╗  
╚██╗ ██╔╝██║  ██║██╔══╝  
 ╚████╔╝ ██████╔╝███████╗
  ╚═══╝  ╚═════╝ ╚══════╝                      
```
The Voltron Development Environment, our Docker-based runtime environment. It includes our full stack. We borrow lots of code from [Autoware.Auto](autoware.auto), so thanks to them!

# Installation
## Requirements
- Your computer will need a *64-bit processor*. If you don't have one for some reason, you'll need to [build from scratch](https://github.com/Voltron-UTD/vde/blob/main/README.md#building-vde-yourself)
- Make sure you have *Git, Docker and Docker Compose* installed.
    - For Docker, download instructions are [here](https://docs.docker.com/get-docker/)
    - For Docker Compose, see [their Docs](https://docs.docker.com/compose/install/)
    - For Git, see [Getting Started Installing Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git)
- These instructions will assume you're doing this on *Linux*. If you're on Windows or Mac, you'll need to modify the inital setup instructions a little, but the process is the same in a broad sense.
    - [Windows instructions are available here.](https://github.com/Voltron-UTD/vde/wiki/VDE-for-Windows)

## Installing from prebuilt images
1. Clone this repo onto your local machine using `git clone https://github.com/Voltron-UTD/vde.git`.
2. Move into the new repo with `cd vde`
3. Give proper executable permissions to our start script with `sudo chmod +x start.sh`.
4. Enter using `./start.sh`. Feel free to examine the contents of this script to see what it does. It's short!

If everything goes well, you should now be inside the VDE container. Congratulations, you're ready to develop! 🎉🎉🎉

## Building VDE yourself
If you don't have a 64-bit computer or you want to modify VDE to suit your needs, you'll need to build VDE on your own.

1. Clone this repo onto your local machine using `git clone https://github.com/Voltron-UTD/vde.git`.
2. Move into the new repo with `cd vde`
3. Give proper executable permissions to our start script with `sudo chmod +x start.sh`.
4. Open `docker-compose.yml` in a text editor and comment out the `image:` line under "base", then uncomment the `build:` line. This tells Compose to look for the VDE image locally instead of pulling the prebuilt image.
5. Build the container using `docker-compose build`. This will take some time, around 15-30 minutes.
6. Enter using `./start.sh`. Feel free to examine the contents of this script to see what it does. It's short!

If everything goes well, you should now be inside the VDE container. Congratulations, you're ready to develop! 🎉🎉🎉

## Rebuilding Autoware.Auto (if you ever need to)
_**Note:** Autoware.Auto is now built automatically when you run `colcon build`. These instructions will simply rebuild it._
1. Are you inside the container? Your terminal will have "docker@..." if you're inside. If not, run `./start.sh`
2. You'll need to give yourself permissions for the home folder uisng `sudo chown -R docker: .` (This is quirky and will be fixed)
3. Clone the official Autoware.Auto Git repo using `cd /opt/ && git clone https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto.git`
4. Move into the Autoware repo using `cd /opt/AutowareAuto`
5. Source your ROS installation using `source /opt/ros/foxy/setup.bash` The build **will fail** if you forget this.
6. Finally run `colcon build`. This will take a while to finish.
At this point, your environment setup is complete. More information about building Autoware.Auto can be found [here](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/building.html)

# Target stack
Here are the components we wish to include. A check means that it's been successfully integrated. The categories are purely for human clarity.

## Perception
- [x] Lidar nodes (Front and Rear)
- [x] Lidar filters (Front and Rear)
- [x] Lidar downsampler
- [x] Lidar fusion

## Mapping
- [x] Lanelet provider (includes visualizer)
- [x] 3D map publisher

## Localization
- [x] NDT localizer

## Planning
- [ ] Path planner (includes lane and parking)
- [x] Route planner

## Control
- [ ] Steering angle calculator

## Interface
- [ ] EPAS Interface (includes "reporter" and "controller")
- [ ] ROS2 Web Bridge
- ~~[ ] Vehicle Bridge~~
- [ ] Web interface server

## Miscellaneous
- [x] URDF Publisher
- [x] Odom-Baselink Publisher (hack, should be removed)

# File structure
- `docker-compose.yml` is the most important piece of VDE. It outlines exactly which containers are executed, and the config of each. It also defines shared volumes, network policies, and other important information.
- `images` contains our Docker images. Each subdirectory includes a Dockerfile, and usually nothing else. The exception is the base image, which includes other files for the build process.
- `nodes` contains the executables (nodes), each one is run in a separate Docker container (using the `frame` image). For clarity, nodes are organized into:
    - `control`
    - `interface`
    - `localization`
    - `mapping`
    - `misc`
    - `perception`
    - `planning`
    
    Our C++ code should accordingly be split into matching namespaces, so our Route Planner is referenced via `voltron::planning::route_planner`.

- `roslibs` are our custom libraries that are added to the build environment via the `pillar` container. As of 7/29/21, this is just Josh Williams's `voltron_test_utils`.
- `data` is where our maps and URDF are stored. It can also include things like the robot's 3D model and a config file for Rviz. This folder stores files that aren't needed at build and that don't store node params.
- `param` stores node params. The files are divided in the same categories as `nodes`. Keeping params in the same directory makes them easier to access than by storing each param under its matching node. Additionally, some nodes may need access to the same param file.

## container.launch.py
VDE is set up so that each node's container (based off our "frame" image) automatically launches a `container.launch.py` file. This file is stored under the node package's `launch` directory. Using a launch file allows us to launch each node in a flexible, readable way.

Other files are included, but they're self-explanatory (e.g. README.md).

# `/vehicle` topics
We should consolidate key data into the same namespace. This frees us from relying on algorithm-specific topic names (e.g. `/localizatin/ndt_pose`). The `/vehicle` topics should include:
- `./state/pose` (PoseStamped)
- `./state/velocity` (TwistStamped, includes lin and ang)
- `./state/acceleration` (AccelStamped, includes lin and ang)
- `./state/blinkers` (Custom msg, enum)
- `./state/headlights` (Custom msg, enum)
- `./state/gear` (Custom msg, enum)
- `./state/steering`
    - A custom steering message with a float from -1.0 to 1.0, where *-1.0 is full left and 1.0 is full right*. This can be scaled to steering wheel angle or any tire angle with a vehicle-specific constant that we can store in [voltron.urdf](./data/voltron.urdf).

We should also include an *action server* for vehicle commands. ROS Actions are synchronous requests, where the requesting node can receives status updates and confirmation when an action is complete. This is useful for physical processes like turning the steering wheel, turning on headlights, and so on.


# FAQ
**Q:** Are files generated by VDE kept?

**A:** No, by design. Docker is nice because every time you run `./start.sh`, you get exactly the same environment. If files were kept between starts, this repeatability is no longer guaranteed. The only files that are kept are the ones added during the Docker build. ***Any files developed in the container should be backed up! You should assume that files on the vehicle will regularly be cleared!***

**Q:** What should be run outside of VDE?

**A:** At the moment,
- The LGSVL bridge, since this is run only outisde of the vehicle, and therefore depends on the host environment.
- The web browser to display our interface. GUI apps don't run well in Docker (it wasn't designed for it).
