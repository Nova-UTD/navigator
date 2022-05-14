# Navigator

Navigator is our autonomous driving stack. This is what Nova is all about - the code that makes our car run. 

If you're curious about the project and how you can get involved, check out [our website](https://nova-utd.github.io/). 

## Progress

Navigator is getting better all the time. Here's what our system is currently able to do: 
- Construct maps of new envirmonments
- Plan a route, given a map of the lanes
- Localize the car on the map in real time
- Generate steering angles and pedal positions given our position and route
- Physically turn the wheel to those angles in real time
- Yield appropriately to cars and pedestrians (mostly)
- Run the same software both in the simulator, on pre-recorded data, and in the real world

We are not currently able to: 
- Follow complex traffic maneuvers, such as traffic lights, unprotected left turns, and construction
- Update our map in real-time (so-called "healing")
- Accurately localize in sparse environments, such as in massive parking lots
- Provide a clean user interface for inexperienced passengers and other road users
- Set custom pickup and dropoff points (ride hailing)

## Building and Running Navigator
Navigator is currently built on top of ROS2 Foxy. You can install Foxy by following [these steps](https://docs.ros.org/en/foxy/Installation.html). You can then build our repository as a normal ROS2 workspace.

Note that we use a number of dependencies, both as `apt` packages, `pip3` packages, and even as external CMake projects included in this repo (yes, we need to break these into submodules).

## Using our code
We designed Navigator to be used and extended by the community. We encourage you to adapt Navigator for your own project.

## Thanks
Our code relies heavily on a number of other open-source projects, including ROS2. This project was forked from Autoware, so thanks to them! We also use Eigen, PCL, Numpy, Pytorch, [ndt_omp](https://github.com/koide3/ndt_omp), the [ROS robot_localization](https://docs.ros.org/en/melodic/api/robot_localization/html/index.html) package, [ros2_numpy](https://github.com/Box-Robotics/ros2_numpy), and [GTSAM](https://gtsam.org/docs/), among many others.
