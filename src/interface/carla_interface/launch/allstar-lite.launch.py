'''
Spawn all nodes needed to connect to and drive in the CARLA
simulator WITHOUT Leaderboard evaluation. This speeds up load times
and therefore facilitates quick iteration.

WSH.
'''

from os import name, path, environ

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory


def generate_launch_description():

    carla_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'carla_spawn_objects'), '/carla_spawn_objects.launch.py']),
        launch_arguments={
            'objects_definition_file': '/navigator/data/carla_objects.json'}.items(),
    )

    carla_bridge_official = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'carla_ros_bridge'), '/carla_ros_bridge.launch.py']),
        launch_arguments={
            'host': 'localhost',
            'port': '2000',
            'synchronous_mode': 'True',
            'town': 'Town01',
            'register_all_sensors': 'False',
            'ego_vehicle_role_name': 'hero'
        }.items(),
    )

    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('carla_interface'), '/allstar.launch.py'])
    )

    return LaunchDescription([
        carla_spawner,
        carla_bridge_official,
        main_launch
    ])
