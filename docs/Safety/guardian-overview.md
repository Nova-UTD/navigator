Last maintained by Chitsein Htun on 10/24/2023


# **Node Overview**
The guardian node is a safety node created to monitor all node systems running in the Navigator, publish several statuses based on the diagnostics, and plays sounds to alert the user based on the severity of a safety event violation. It includes two executables, guardian_node and sound_node.


# **Node Executables**
### **guardian_node**
##### **Overview**
guardian_node handles the monitoring and publishing of statuses.

##### **Publishers**
**status_array_pub**
message type: [diagnostic_msgs.msg.DiagnosticArray](https://docs.ros2.org/galactic/api/diagnostic_msgs/msg/DiagnosticArray.html)
topic: '/status'
Publishes an array of diagnostic information on each running node on the watchlist, with a global status that reflects the overall state at the end of the array.

**current_mode_pub**
message type: navigator_msgs.msg.Mode
topic: '/guardian/mode'
Publishes what mode the navigator stack needs to be in based on whether a safety event violation has triggered a disable of the auto or manual mode.

##### **Subscribers**
**mode_request_sub**
message type: navigator_msgs.msg.Mode
topic: '/requested_mode'
Receives information about what mode the user has request for the the navigator stack - disabled, manual control, or autonomous control.

**status_sub**
message type: [diagnostic_msgs.msg.DiagnosticStatus](https://docs.ros2.org/galactic/api/diagnostic_msgs/msg/DiagnosticStatus.html)
topic: 'node_statuses'
Receives diagnostics information about a node currently operating during the navigator stack execution.

**path_sub**
message type: nav_msgs.msg.Path
topic: '/planning/path'
Receives a path from the planning node.

**clock_sub**
message type: [rosgraph_msgs.msg.Clock](https://docs.ros.org/en/melodic/api/rosgraph_msgs/html/msg/Clock.html)
topic: '/clock'
Receives information about the internal ROS clock.


### **sound_node**
sound_node subscribes to the status updates and plays noises to alert the user matching the severity of safety event violations

##### **Publishers**
None.

##### **Subscribers**
**current_mode_sub**
message type: navigator_msgs.msg.Mode
topic: '/guardian/mode'
Receives information about the mode that the navigator stack is in.

**is_waiting_sub**
message type: [std_msgs.msg.Bool](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html)
topic: '/planning/is_waiting'
Receives information about whether the planning subsystem is waiting to move into the junction or not.
