---
layout: default
title: Sensing
nav_order: 2
---

# Sensing overview
{: .no_toc }

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---

The Sensing system is responsible for processing raw sensor data into a usable form for the Perception system.

For example, raw LiDAR data from our front and rear sensors is merged into a single reference frame, downsampled, and cropped to remove points along the vehicle itself (points of our vehicle's doors, for example).

More info to come! But our filters aren't reinvinting the wheel.