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

### "New publisher discovered on this topic, offering incompatible QoS"

```
[WARN] [1669075701.737584987] [leaderboard_node]: New publisher discovered on this topic, offering incompatible QoS. No messages will be received from it. Last incompatible policy: DURABILITY_QOS_POLICY
```

A warning similar to the above example will appear when a publisher and subscriber attempt to exchange messages with incompatible quality of service QoS policies.

#### Solution
Edit the QoS policy of either the relevant publisher or subscriber so that the two can "speak" to each other.

For more information on QoS in ROS2, including compatibility between policies, [see here](https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html).

### "RuntimeError: trying to create rpc server for traffic manager..."
```
...
  File "/workspace/leaderboard/leaderboard/leaderboard_evaluator.py", line 83, in __init__
    self.traffic_manager = self.client.get_trafficmanager(args.traffic_manager_port)
RuntimeError: trying to create rpc server for traffic manager; but the system failed to create because of bind error.
```
This is caused when a CARLA client attempts to connect to CARLA on a port that is already being used. This is either because:
1. Another user is currently using the simulator
    - In which case you should change your port by using the `--traffic_manager_port` or similar, depending on which script you're running
2. An earlier CARLA client has died before it freed its port, and so there is some orphaned Python process on the system that is using up the RPC port. One brute force method is to run `sudo pkill -9 python`, but this is a very ugly solution.
 
### When running the leaderboard evaluator: "No module named 'agent'"
<small>As of 12 Nov '22</small>
```
# ./leaderboard.bash
Starting the CARLA evaluation script.
This may take some time. Sit tight!
Traceback (most recent call last):
...
ModuleNotFoundError: No module named 'agent'
```
#### Solution: Ensure that there isn't a typo in the `--agent` flag to `leaderboard_evaluator.py`. As of writing, this is in `leaderboard.bash`.

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
