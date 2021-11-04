# Navigator

Navigator is our autonomous driving stack. This is what Nova is all about - the code that makes our car run. 

If you're here for technical information, read on! Also, maybe check out our wiki for more in-depth information. 

If you're just curious about the project, and how you can get involved, check out [our website](https://nova-utd.github.io/). 

## Progress

Navigator is not finished. In fact, Navigator is not even close to finished. Here's what we are currently able to do: 
- Construct maps of new envirmonments
- Plan a route, given a map of the lanes
- Steer along that route

We are not currently able to: 
- Control throttle and brake
- Avoid obstacles
- Follow traffic laws
- Basically anything not listed above

Our software is continually in development, so we'll be updating these lists as new feautres arrive. 

## Building Navigator

Building Navigator is a bit of a complex process - what did you expect for a self-driving vehicle project? However, with a bit of troubleshooting, hopefully the guide below is enough to help you get started. 

#### Dependencies

- Our stack is built on [ROS](https://github.com/ros), so you will need to install that before building. Specifically, we are using ROS version 2, distribution Foxy. 
- TODO: We're still working on ducumenting all of our dependencies, so there may be more here in the future. 

#### Compilation

Once you've installed the dependencies, clone this repository. The `main` branch contains our most recent, stable, tested release. It's set up as a colcon workspace, so once you've cloned it, you just need to run: 

```
. /opt/ros/foxy/setup.bash # Source ROS, wherever you've installed it
cd navigator # Move to the root directory of our repository
colcon build # Build everything
```

This will probably take a while depending on your computer. You may get some stderr output from some of the packages. That's fine, as long as it's nothing too threatening and everything actually builds - it happens on our working builds as well, and we'll get around to fixing those warnings at some point. If all of the packages build, you are ready to launch. 

#### Troubleshooting

TODO - common build issues

## Running Navigator

Since we don't want to test every unstable change in the real world first, we currently launch our stack in one of two environments - the physical vehicle and the simulator. 

#### Launching - Vehicle

TODO

#### Launching - Simulator

## Other Information

#### Autoware.Auto

Huge thanks to [Autoware.Auto](autoware.auto). Our code is heavily based on theirs, and straight up includes large portions of it. This wouldn't have been possible (or at least, would have been much, much harder) without them!

#### License

Our code is licensed under the MIT License, which can be found in the LICENSE file. Alternatively, read an online version [here](https://mit-license.org/). Not that Autoware's code, including the sections of their code that are included in our project, are under Autoware's license, not ours. 
