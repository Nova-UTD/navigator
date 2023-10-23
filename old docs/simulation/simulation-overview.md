---
layout: default
title: Simulation
nav_order: 5
---

# Simulation overview
{: .no_toc }

*Maintained by Connor Scally & Daniel Vayman*

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

Before demonstrating our codebase on the vehicle, in the real world, we must first test our stack in a virtual one. The following documentation outlines essential CARLA usage and syntax, to allow for simulating our stack in a virtual enviroment. 

Nova utilizes CARLA for virtualization. For further information on CARLA, and to learn more about advanced usage, please see the following links: 

- https://carla.org/
- https://carla.readthedocs.io/en/latest/

## Simulation Enviroment

 * Prerequisites:

    1. CARLA Simulator: Please follow the instructions in the above links to install CARLA on your chosen operating system
    2. Navigator: Please see our GitHub page for the latest releases of Navigator
    3. [RVIZ](http://wiki.ros.org/rviz) (Or an equivalent ROS visualizer)
    4. [ROS2](https://docs.ros.org/en/foxy/index.html)
    5. Dependencies for the above: Self-explanatory, Navigator comes with most of what you need, CARLA may not, do not forget to check!



## Launching the simulator & running Navigator:

* Launching **CARLA**:

    1. Your first step should be to navigate to your CARLA directory and launch CARLA with the CARLAUE4.sh script with the -RenderOffScreen flag. If you are on a unix system, the command will look like this:

    ```
    $ /home/share/carla/CarlaUE4.sh -RenderOffscreen
    ```

 - The "RenderOffscreen" flag hides the rendering window, which saves some resources. See [here](https://carla.readthedocs.io/en/latest/start_quickstart/#command-line-options) for more details


* Launching **RVIZ** (*within our Docker container*):

    1. Open a new terminal window.
    
    2. Navigate to the root directory of Navigator

    3. Enable Docker to launch GUI programs
    
        `$ xhost +`
    
    3. Run our docker container start script. (If first time running container, refer to [step 3](/navigator/index)) 
       
        `$ ./start.sh `
    
    4. Source the setup script via a command like: `. install/setup.bash`

    5. Run: `rviz2`
    6. Select File followed by Open Config Select default.rviz from the share folder. It is recommended that you have your own copy of this as well             for your own configuration.
    
* Launching **Navigator**:

    1. Open a new terminal window.

    2. Navigate to the root directory of Navigator.

    3. Run the docker container `./start.sh`

    4. Run `source /install/setup.bash` (if you have bash sourcing ROS automatically (see below), that works too)

    5. Run `ros2 launch carla_interface carla-lite.launch.py`

    6. Check RVIZ and terminal output. The sim_bridge will publish sensor data just as if you were driving on campus, and it will similary accept commands      from our [standard topics](https://github.com/Nova-UTD/navigator/wiki/Topic-and-transform-structure). As of writing, our custom bridge publishes:

    - GNSS (GPS)
    - IMU
    - Front and rear Lidar (not fully functional)
    - Front  RGB camera
    - Front depth camera
    - CARLA ground truths for
    - Car's odometry (position, orientation, speed)
    - CARLA virtual bird's-eye camera (/carla/birds_eye_rgb)

    *The most up-to-date information on our bridge's capabilities* can be found [at the top of the script itself](https://github.com/Nova-UTD/navigator         /blob/dev/src/interface/sim_bridge/sim_bridge/sim_bridge_node.py#L3).


## Using the Simulator:

- You can control our ego vehicle with `ros2 run manual_control manual_control_node`
   - At the moment, this only supports keyboard control through NoMachine or similar, ***not SSH***.
   - If you get a "pynput" error, try running `pip3 install pynput`.
- You can change a number of simulation settings by editing our script's contants ([here](https://github.com/Nova-UTD/navigator/blob/fd05a57a46f286961956ea35986c0107a786acdf/src/interface/sim_bridge/sim_bridge/sim_bridge_node.py#L27)).
   - Don't forget to rebuild the package or use `colcon build --symlink-install` (recommended).
   - ROS param support in the works.

## Troubleshooting:

- If you get a "pynput" error, try running `pip3 install pynput`.
- If you get a CARLA segmentation fault, it's likely you just need to restart CARLA. This will be fixed... eventually. This should only happen after starting the bridge 10 times or so, and should not happen while the bridge is running.
- If CARLA gives you a SIGFAULT error attach the -carla-rpc-port=N where N = your favorite (Not in use) port number.

## Sourcing Foxy Automatically in Bash:

1. Open your terminal
2. Write the command --> `gedit ~/.bashrc` (or nano, whatever really)
3. Go under the last line line and write --> ```source /opt/ros/foxy/setup.bash```
4. Save and exit
5. Now with every new shell you open, it will source automatically

