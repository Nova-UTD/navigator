---
layout: default
title: Controls
nav_order: 5
---
# Controls overview
{: .no_toc }

*Maintained by Egan Johnson*

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

The Controls system takes our vehicle's current state and our target trajectory as inputs. Based on the difference between our current state and this desired trajectory, the system calculates the ideal:
- Steering angle
- Brake position
- Throttle position

To make our system platform-agnostic, the Controller's outputs are between (-1.0, 1.0) for steering and (0.0, 1.0) for throttle and brake positions. The Interface system is responsible for scaling these values to a specific platform.

## Sources of error
The vehicle does not move perfectly. Physical error is introduced. There will always be errors in our model-- that is, our set of assumptions about the vehicle's physical properties and behavior. 

Most importantly, there's error in our current state, one of the inputs to our controller. In order for our controller to follow the desired trajectory, it must have an accurate understanding of where the car currently is, how fast it's going, and which way it's facing.

## Current controllers
Our "unified controller" generates the desired steering angle and pedal positions within the same node. However, steering and pedal positions are calculated using two different controllers.

As of Sept. 21, 2022, our steering controller uses Pure Pursuit, while our throttle and brake are calculated using a simple PID controller. This combination works suitably at low speeds, though something more sophisticated like MPC would be necissary for highway driving.