import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from os import path

import subprocess

# bag_dir =


def generate_launch_description():
    launch_path = path.realpath(__file__)
    launch_dir = path.dirname(launch_path)
    param_dir = path.join(launch_dir, "param")
    parameter_file = parameters = [
        (path.join(param_dir, "mapping", "lio_sam.param.yaml"))]
    launch_path = os.path.realpath(__file__)
    launch_dir = os.path.dirname(launch_path)

    bag_player = Node(
        executable='ros bag play',
        parameters=[],
        arguments="/home/main/voltron/assets/bags/18_12_21/rosbag2_2021_12_18-17_59_24"
    )

    # run a bag
    # bag_process = subprocess.run(
    #     "ros2 bag play /mnt/sda1/bags/april16/rosbag2_2022_04_16-22_02_11".split())

    # print("urdf_file_name : {}".format(xacro_path))

    return LaunchDescription([
        # Node(
        #     package='tf2_ros', # Fuse the map and odom frames
        #     executable='static_transform_publisher',
        #     arguments='0.0 0.0 0.0 0.0 0.0 0.7372773 0.6755902 map odom'.split(' '),
        #     parameters=[parameter_file],
        #     output='screen'
        #     ),
        Node(
            package='tf2_ros',  # Fuse the map and odom frames
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 1.0 map odom'.split(' '),
            parameters=[parameter_file],
            output='screen'
        ),
        # Node(
        #     package='robot_state_publisher', # Publish robot URDF
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{
        #         'robot_description': Command(['xacro', ' ', xacro_path])
        #     }]
        # ),
        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            name='lio_sam_imuPreintegration',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[os.path.join(launch_dir, "data", "hail_bopp.urdf")]
        )
    ])
