from launch.text_colors import text_colors as colors

processes = {} # Holder for processes

nodes: dict = { # Node dictionary for converting node executable to package name
    "launch": "launch",
    "unified_controller_node": "unified_controller_node",
    "ukf_node": "localization_map_odom",
    "visualizer": "odr_visualizer_node",
    "static_transform_publisher": "static_transform_publisher",
    "robot_state_publisher": "robot_state_publisher",
    "vt_viz_exe": "vt_viz_node",
    "ZoneFusionLaunch": "zone_fusion",
    "ObstacleZonerLaunch": "obstacle_zoner",
    "BehaviorPlannerLaunch": "behavior_planner",
    "publisher": "path_publisher_node",
    "motion_planner": "motion_planner_node"
}

def get_node_from_process(process: str) -> str:
    """
    Takes an executable process name from console and matches with node list [ros2 node list] 
    @param process[str]     String process to parse
    """
    try: # Try catch to return process if exists, or show error if not
        return nodes[process]
    except:
        print(f"{colors.FAIL}{colors.BOLD}<{process}> NOT FOUND, PACKAGE NOT DEFINED{colors.ENDC}")
        return process