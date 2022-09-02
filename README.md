<img src="https://github.com/Nova-UTD/Nova-UTD.github.io/raw/master/assets/res/navigator_logo.png" width="600" />

Navigator is our autonomous driving stack. This is what Nova is all about - the code that makes our car run. 

If you're curious about the project and how you can get involved, check out [our website](https://nova-utd.github.io/). 

## Progress

Navigator is not finished. In fact, Navigator is not even close to finished. Here's what we are currently able to do: 
- Construct maps of new envirmonments
- Plan a route, given a map of the lanes
- Localize the car on the map in real time
- Generate steering angles given our position and route
- Physically turn the wheel to those angles in real time

We are not currently able to: 
- Intelligently control throttle and brake
- Avoid obstacles by changing lanes or speed
- Yield appropriately to cars and pedestrians
- Follow traffic laws of any sort
- Report real-time data in a meaningful way

Our software is continually in development, so we'll be updating these lists as new feautres arrive. 

## Building and Running Navigator

Building and properly running Navigator is a fairly complex process. If you're interested in reproducing our results in simulation (or even in the real world!), please check out our wiki for more detailed documentation of our project. 

## Using our code

If you're interested in reusing our code, you're welcome and encouraged to do so! Our project is implemented as a collection of ROS nodes, which should be easy to reuse in other projects. The function and use of each node will be documented on our project wiki. 

Our code is licensed under the MIT License, which can be found in the LICENSE file. Alternatively, read an online version [here](https://mit-license.org/). Note that Autoware's code, including the sections of their code that are included in our project, are under Autoware's license, not ours. 

## Autoware.Auto

Huge thanks to [Autoware.Auto](autoware.auto). Our code is heavily based on theirs, and straight up includes large portions of it. This wouldn't have been possible (or at least, would have been much, much harder) without them!
