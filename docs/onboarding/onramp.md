Navigator relies on ROS2 - the Robot Operating System. Before you can contribute to Navigator, you'll need to become familiar with developing for ROS. Our stack currently uses ROS2 Humble. Below is a concrete task that will require you to learn many of the fundamental elements of ROS as well as Docker, since we use Docker quite heavily as well. It will likely also require a fair bit of reading, learning, and exploration - all things Navigator is going to require of you too.  Good luck!

[Here is a video](turtlesim_demo.mp4) that shows what the final product should look like.

# The Task

For this task, you will be using a well documented example/tutorial ROS2 package called `turtlesim`. It is part of the built-in ROS2 install. You will be supplementing this example with several additional ROS2 nodes. The scope of your task is outlined by the following desired functionality:

1. ### Launch a docker container
    based on an `ubuntu:jammy` image on top of which you have preinstalled a version of ROS2 Humble, *compiled from source code* (not a package install 🙀 - this is to make you can bend Docker to your will).

2. ### Use one launch file to:
    - Run the built-in turtlesim node that brings up the GUI/canvas with the first turtle, 
    - Spawn a second turtle using the built in spawn service,
    - Run a custom ROS node to navigate the first turtle using your mouse pointer.
3. ### In a second container (can be the same docker image), use a second launch file to: 
    - Run a custom ROS node to make the second turtle chase the first turtle. The second turtle can stop following once it is sufficiently close to the first turtle, but should begin again if the first turtle moves away. 
    - Run a custom node that publishes a custom message that signals whether the second turtle should chase or run away from the first turtle. The node should flip/change this mode every 10 seconds.  Again the second turtle can stop running away if it is far enough away from the first turtle.  

## Note this requires...
1. Graphics to be displayed from within a docker container.
2. Your ROS2 messages to communicate across docker containers.
3. Writing a custom ROS package(s) with three nodes and one custom message type (for the run away mode).

# SSH Information

We encourage you to code from the Quad (our beefy server in the upstairs lab). This will give you experience working in the Ubuntu/Linux OS environment. Also, it's already equipped with Docker. How can I connect remotely? Use SSH!

- We use [NoMachine](https://www.nomachine.com/) for remote desktop.  
- If off-campus, be sure to connect via the [UTD GlobalProtect VPN](https://atlas.utdallas.edu/TDClient/30/Portal/Requests/ServiceDet?ID=167) to enable SSH.

[Here's a video tutorial](ssh_tutorial.mp4) that demonstrates GlobalProtect VPN, SSH, NoMachine, and VS Code SSH.

Also, feel free to set up an SSH key for easier log-in access.

## For any clarification questions regarding the onramp task, be sure to reach out to Justin or Daniel.
