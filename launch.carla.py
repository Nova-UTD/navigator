from os import name, path, environ
import sys

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory

sys.path.append(path.abspath('/navigator/'))
from launch_node_definitions import *

NAVIGATOR_DIR = "/navigator/"

def generate_launch_description():

    ## lines indicate these were originally uncommented

    return LaunchDescription([
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
        carla_rviz,
        # rqt,
        # camera_streamer,

        # PERCEPTION
        # image_segmentation,
        # semantic_projection,
        carla_lidar_processor,
        ground_seg,
        static_grid,
        # prednet_inference,
        # driveable_area,

        # PLANNING
        grid_summation,
        junction_manager,
        rtp,

        # SAFETY
        ##airbags,
        ##guardian,

        # STATE ESTIMATION
        ## map_manager,  <<----- seems to have hard coded stuff for campus testing
        gnss_processor,
    ])