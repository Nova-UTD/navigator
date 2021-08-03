from os import path, environ

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

    with_obstacles_param = DeclareLaunchArgument(
        'with_obstacles',
        default_value='False',
        description='Enable obstacle detection'
    )
    
    path_planner = Node(
        package='vt_path_planner',
        name='behavior_planner_node',
        namespace='planning',
        executable='behavior_planner_node_exe',
        parameters=[
            ('/opt/param/planning/path_planner.param.yaml'),
            {'enable_object_collision_estimator': LaunchConfiguration('with_obstacles')}
        ],
        output='screen',
        remappings=[
            ('HAD_Map_Service', '/had_maps/HAD_Map_Service'),
            ('vehicle_state', '/vehicle/vehicle_kinematic_state'),
            ('route', 'global_path'),
            ('gear_report', '/vehicle/gear'),
            ('vehicle_state_command', '/vehicle/state_command')
        ]
    )
    lane_planner = Node(
        package='lane_planner_nodes',
        name='lane_planner_node',
        namespace='planning',
        executable='lane_planner_node_exe',
        parameters=[('/opt/param/planning/lane_planner.param.yaml')],
        remappings=[('HAD_Map_Service', '/had_maps/HAD_Map_Service')]
    )
    parking_planner = Node(
        package='parking_planner_nodes',
        name='parking_planner_node',
        namespace='planning',
        executable='parking_planner_node_exe',
        parameters=[('/opt/param/planning/parking_planner.param.yaml')],
        remappings=[('HAD_Map_Service', '/had_maps/HAD_Map_Service')]
    )

    return LaunchDescription([
        with_obstacles_param,
        path_planner,
        lane_planner,
        parking_planner,
    ])