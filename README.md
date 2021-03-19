```

██╗   ██╗██████╗ ███████╗
██║   ██║██╔══██╗██╔════╝
██║   ██║██║  ██║█████╗  
╚██╗ ██╔╝██║  ██║██╔══╝  
 ╚████╔╝ ██████╔╝███████╗
  ╚═══╝  ╚═════╝ ╚══════╝
                         
```
The Voltron Development Environment, our Docker-based development setup. It's really a modified version of Autoware.Auto's [ADE](https://ade-cli.readthedocs.io/).

# Installation
## Requirements
- Make sure you have Git, Docker and Docker Compose installed.
    - For Docker, download instructions are [here](https://docs.docker.com/get-docker/)
    - For Docker Compose, see [their Docs](https://docs.docker.com/compose/install/)
    - For Git, see [Getting Started Installing Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git)
- These instructions will assume you're doing this on Linux. If you're on Windows or Mac, you'll need to modify the inital setup instructions a little, but the process is the same in a broad sense.

## Initial setup
1. Clone this repo onto your local machine using `git clone ...`.
2. Move into the new repo with `cd vde`
3. Give proper executable permissions to our scripts with:
```
sudo chmod +x vde/entrypoint
sudo chmod +x vde/autoware-setup.sh
sudo chmod +x start.sh
```
3. Build the container using `docker-compose build`. This will take some time, around 15-30 minutes.
4. Enter using `./start.sh`. Feel free to examine the contents of this script to see what it does. It's short!

If everything goes well, you should now be inside the VDE container. Congratulations, you're ready to develop! 🎉🎉🎉

## Building Autoware.Auto
_**Note:** Autoware.Auto is now built automatically when you run `colcon build`. These instructions will simply rebuild it._
1. Are you inside the container? Your terminal will have "docker@..." if you're inside. If not, run `./start.sh`
2. You'll need to give yourself permissions for the home folder uisng `sudo chown -R docker: .` (This is quirky and will be fixed)
3. Clone the official Autoware.Auto Git repo using `cd ~ && git clone https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto.git`
4. Move into the Autoware repo using `cd ~/AutowareAuto`
5. Source your ROS installation using `source /opt/ros/foxy/setup.bash` The build **will fail** if you forget this.
6. Finally run `colcon build`. This will take a while to finish.
At this point, your environment setup is complete. More information about building Autoware.Auto can be found [here](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/building.html)

# Notes
- By default, the home directory inside the container is linked to a folder called "vdehome" in the host user's home folder. You'll see this in docker-compose.yml on the line `${HOME}/vdehome:/home/docker/`. This means that any file you place in the home directory will be saved.
- All work done outside of the home directory is erased when you stop the container.
- ***Any files developed in the container should be backed up! You should assume that files on the vehicle will regularly be cleared!***
