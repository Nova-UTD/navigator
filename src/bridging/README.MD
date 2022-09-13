Bridging
=======

# Domain Description

The `bridging` sub-directory contains nodes and libraries related to communication between the webview and navigator
Packages under this directory enable navigator to transmit messages between the stack and the webview. 
Modules and libraries that report and transmit or report other data, errors or system statistics
may also be packages under this directory. 

Bridging works by transmitting messages along the "/incoming_bridge_messages" topic. The BridgeManagerNode will send
any messages along this topic to the local server handler. Topic messages must be sent as 'std_msgs::msg::String', the
standard ROS string type. Any errors will be reported in the standard console output. 