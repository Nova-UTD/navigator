"""This module contains the launch description for the vehicle."""

from os import path
import sys

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart
import launch_ros.actions

sys.path.append(path.abspath("/navigator/"))
from launches.launch_node_definitions import *
from launches.utils import (
    err_fatal,
    try_get_launch_description_from_include,
    IncludeError,
)


def generate_launch_description():
    """Vehicle launch file description."""

    # Include perception launch file and extract launch entities for reuse.
    try:
        perception_launch_description = try_get_launch_description_from_include(
            "launches/launch.perception.py"
        )
    except IncludeError as e:
        err_fatal(e)

    # Include nav2 launch file and extract launch entities for reuse.
    # try:
    #     nav2_launch_description = try_get_launch_description_from_include(
    #         "launches/launch.nav2.py"
    #     )
    # except IncludeError as e:
    #     err_fatal(e)

    perception_launch_entities = perception_launch_description.describe_sub_entities()
    # nav2_launch_entities = nav2_launch_description.describe_sub_entities()

    return LaunchDescription(
        [
            # Set the global config file as a parameter for all nodes.
            launch_ros.actions.SetParameter(name="global_config", value="/navigator/param/global_parameters.yaml"),
            
            map_manager_carla,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=map_manager_carla,
                    on_start=[
                        LogInfo(
                            msg="Map Manager Started... launching the rest of Navigator..."
                        ),
                        # CONTROL
                        # carla_controller,
                        # INTERFACE
                        # leaderboard_liaison,
                        # web_bridge,
                        ##joy_linux,
                        ##joy_translation,
                        # LOCALIZATION
                        # gnss_averager,
                        # mcl,
                        # MAPPING
                        # MISC
                        # costmap_recorder,
                        # rqt,
                        # camera_streamer,
                        # PERCEPTION
                        *perception_launch_entities,
                        # PLANNING
                        # routing_monitor,
                        routing_hardcoded,  # Use this to load a manual route saved as a csv. Comment out routing_monitor
                        grid_route_costmap,
                        grid_summation,
                        intersection_manager,
                        # junction_manager,
                        path_planner,
                        # *nav2_launch_entities,
                        # path_planner_nav2,
                        pure_pursuit_controller,
                        # SAFETY
                        ##airbags,
                        ##guardian,
                        rviz,
                    ],
                )
            ),
        ]
    )
