---
layout: default
title: Interface
nav_order: 6
---
# Interface overview
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

The **Interface** subsystem represents the only code that is vehicle-specific by necessity. It is the link between software and hardware. Since your hardware is probably different than ours, then you'll likely have to modify our interface code to suit your needs.

The exception is our CARLA bridge, a simple script that spawns an ego vehicle and connects its sensors and actuators to ROS.

In this overview, we'll go over our hardware platform, Hail Bopp.

## About Hail Bopp
<div class="sketchfab-embed-wrapper"> <iframe title="HaleBopp" frameborder="0" allowfullscreen mozallowfullscreen="true" webkitallowfullscreen="true" allow="autoplay; fullscreen; xr-spatial-tracking" xr-spatial-tracking execution-while-out-of-viewport execution-while-not-rendered web-share src="https://sketchfab.com/models/892f57e431944b548047e9d09411755d/embed?autostart=1&annotations_visible=1&annotation_cycle=10&ui_hint=0&dnt=1" style="width: 100%; height: 400px;"> </iframe></div>

Hail Bopp includes an Electronic Power Assisted Steering (EPAS) system, which is a motor on our steering column that allows us to apply steering force programatically.

Our vehicle also has front and rear Velodyne Puck (VLP-16) LiDAR sensors. These generate rich, precise 3D scans of our surroundings using 16 channels of spinning lasers.

Our platform uses ZED stereo cameras to gather color images and their corresponding depth information. Stereo cams can give us limited 3D information, but they are best used in conjuction with more accurate LiDAR data.

We currently use an NVIDIA Jetson AGX Xavier for our onboard computing. This is likely to change soon, as we have exceeded the power capabilities of this device.

Finally, an Adafruit Metro Grand Central serves as our real-time microcontroller. This device handles time-critical services, such as releasing our steering wheel if our high-level software fails.

## Electrical overview
<a href="/static/electrical.html"><img src="/assets/res/2021-08-03-schematic.png" alt="Voltron Secondary System schematic"></a>