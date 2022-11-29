---
layout: default
title: Troubleshooting
nav_order: 5
---
# Troubleshooting
{: .no_toc }
 
*Maintained by Nova members*
 
## Table of contents
{: .no_toc .text-delta }
 
1. TOC
{:toc}
 
---
 
### "The RMW implementation has been specified as..."
<small>As of 11 Nov '22</small>
```
CMake Error at /opt/ros/foxy/share/rmw_implementation/cmake/rmw_implementation-extras.cmake:54 (message):
    The RMW implementation has been specified as 'rmw_cyclonedds_cpp' via
    environment variable 'RMW_IMPLEMENTATION', but it is not available at this time.

    Currently available middlewares:

    'rmw_fastrtps_cpp'
```
#### Solution
Navigator uses a slightly modified ROS Middleware (RMW) implementation called CycloneDDS. In order for ROS and its nodes to use it, our custom version must be built and sourced, just like any other ROS workspace.

Docker includes the CycloneDDS workspace under `/opt/cyclone_ws`, and this workspace should be sourced automatically as part of the Docker `entrypoint.sh`. If you receive the above error, CycloneDDS was either not properly built or sourced.

You can verify that the CycloneDDS is loaded properly using the command `# ros2 doctor --report`, which should show:
```
...
processor        : x86_64

   RMW MIDDLEWARE
middleware name    : rmw_cyclonedds_cpp
...
```
