```

██╗   ██╗██████╗ ███████╗
██║   ██║██╔══██╗██╔════╝
██║   ██║██║  ██║█████╗  
╚██╗ ██╔╝██║  ██║██╔══╝  
 ╚████╔╝ ██████╔╝███████╗
  ╚═══╝  ╚═════╝ ╚══════╝
                                     
```
 *✨ now with [Byobu](https://www.byobu.org/)! ✨* 
 
The Voltron Development Environment, our Docker-based development setup. It's really a modified version of Autoware.Auto's [ADE](https://ade-cli.readthedocs.io/).

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

## Updating
To pull the latest images, simply run `docker-compose pull` before starting with `start.sh`. The up-to-date images will be downloaded from our GitHub Container Registry.

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

# Notes
- By default, the home directory inside the container is linked to a folder called "vdehome" in the host user's home folder. You'll see this in docker-compose.yml on the line `${HOME}/vdehome:/home/docker/`. This means that any file you place in the home directory will be saved.
- All work done outside of the home directory is erased when you stop the container.
- ***Any files developed in the container should be backed up! You should assume that files on the vehicle will regularly be cleared!***
