node_dict: dict = { # Node dictionary for converting node executable to package name.
                # Format is "<executable/process-name>: <package/node-name>"
    "rcl": "ros_client_library",
    "launch": "launch",

    "liaison_node": "carla_interface",
    "bridge": "carla_ros_bridge",
    "controller": "carla_controller",
    "carla_spawn_objects": "carla_objects",
    
    "carla_estimation_node": "state_estimation",
    "robot_state_publisher": "robot_state_publisher",
    "vt_viz_exe": "vt_viz",
    "lidar_processing_node": "sensor_processing",
    "map_management_node": "map_management_node",
}