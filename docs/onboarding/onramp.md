Navigator relies on ROS2 - the Robot Operating System. Before you can contribute to Navigator, you'll need to become familiar with developing for ROS. Our stack currently uses ROS2 Humble. Below is a concrete task that will require you to learn many of the fundamental elements of ROS. It will likely also require a fair bit of reading, learning, and exploration - all things Navigator is going to require of you too.  Good luck!

[Here is a video](turtlesim_demo.mp4) that shows what the final product should look like.

# The Task

1. Launch a docker container based on an `ubuntu:jammy` image on top of which you have preinstalled a version of ROS2 Humble, *compiled from source code* (not a package install 🙀).
2. Use a launch file to:
    - Run the turtlesim node that brings up the GUI/canvas, 
    - Spawn a second turtle using the built in spawn service,
    - Run a custom node to navigate the first turtle using your mouse pointer.
3. In a second container (can be the same docker image), use another launch file to: 
    - Run a custom node to make the second turtle chase the first turtle. The second turtle can stop following once it is sufficiently close to the first turtle, but should begin again if the first turtle moves away. 
    - Run a custom node that publishes a custom message that signals whether the second turtle should chase or run away from the first turtle. The node should flip/change this mode every 10 seconds.  Again the second turtle can stop running away if it is far enough away from the first turtle.  

Note this requires...
1. Graphics to be displayed from within a docker container.
2. Your ROS2 messages to communicate across docker containers.
3. Writing a custom ROS package(s) with three nodes and one custom message type (for the run away mode).
