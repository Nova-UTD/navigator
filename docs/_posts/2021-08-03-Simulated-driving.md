---
layout: post
date:   2021-08-03 09:00:18 -0500
categories: simulation
author: "Will Heitman"
---
<iframe width="560" height="315" src="https://www.youtube.com/embed/FSZigdNs9aY" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
This is just a small post to show our software driving a car in simulation. The process is the same as in real-world use: Provide a location estimate (done manually until we get a GPS sensor) then a goal location. Once the goal location it given, the car immediately steers and accelerates on its own.

As part of our tests of the [updated VDE](http://localhost:8080/vde/2021/08/03/Reorganizing-our-stack.html), we ran the finished stack in the open-source [SVL Simulator](https://www.svlsimulator.com/).

This is basically a simulated version of [Demo 1](http://localhost:8080/d1-overview), except that we added a primitive, "bang-bang" controller for throttle and brake. In other words, the entire drive was done hands-free.

There's still a whole lot to add! But having the ability to test code in simulation before moving to a real vehicle saves time, and it's obviously safer too.

A couple notes:
- Our map data is misaligned, so the car *thinks* that it's driving perfectly in the lane (look at the blue lane on the right). But in the simulated environment, it's actually going into the bike lane on the right. Better map alignment would fix this.
- The car is driving at a fixed target speed of 6 meters per second, or about 13 mph. It can't go any faster without breaking our localizer.
- As mentioned, the throttle and brake are controlled using [bang-bang control](https://en.wikipedia.org/wiki/Bang%E2%80%93bang_control), so the car is either pressing the throttle or brake at a fixed strength.
