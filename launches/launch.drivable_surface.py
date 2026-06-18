import launch
from launch import LaunchDescription
from launch.actions import LogInfo, DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='image_segmentation',
            executable='image_seg_node',
            name='image_seg_node',
            output='screen',
        ),

        Node(
            package='depth_processing',
            executable='depth_processing_node',
            name='depth_processing_node',
            output='screen',
        ),

        Node(
            package='occupancy_grid',
            executable='occupancy_grid',
            name='occupancy_grid_node',
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2'],
            name='rviz2',
            output='screen'
        )
    ])