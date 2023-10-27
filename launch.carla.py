from os import name, path, environ
import sys

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, LogInfo, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory

sys.path.append(path.abspath('/navigator/'))
from launch_node_definitions import *

NAVIGATOR_DIR = "/navigator/"

def generate_launch_description():

    ## lines indicate these were originally uncommented

    return LaunchDescription([
        map_manager_carla,
        RegisterEventHandler(
            OnProcessStart(
                target_action=map_manager_carla,
                on_start=[
                    LogInfo(msg='Map Manager Started... launching the rest of Navigator...'),
                    # CONTROL
                    #carla_controller,

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
                    # recorder,
                    # rqt,
                    # camera_streamer,

                    # PERCEPTION
                    # image_segmentation,
                    # semantic_projection,
                    #carla_lidar_processor,
                    ground_seg,
                    static_grid,
                    # prednet_inference,
                    # driveable_area,

                    # PLANNING
                    routing_monitor,
                    grid_summation,
                    junction_manager,
                    rtp,

                    # SAFETY
                    ##airbags,
                    ##guardian,
                    rviz,
                ]
            )
        )
    ])