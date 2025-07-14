from os import path

from launch import LaunchDescription
from launch_ros.actions import Node


launch_path = path.realpath(__file__)
launch_dir = path.join(path.dirname(launch_path), '..')
print(launch_dir)

def generate_launch_description():

    launch_path = path.realpath(__file__)
    launch_dir = path.dirname(launch_path)
    param_dir = path.join(launch_dir, "param")
    data_dir = path.join(launch_dir, "data")

    complex_yolo_model = Node(
        package = 'lidar_detection',
        executable = 'lidar_detection_node',
        name = 'lidar_detection_node',
        parameters=[
            {'model': 'complex_yolo'},
            {'device': 'cuda:0'},
            {'conf_thresh': 0.9}
        ],
    )

    mmdetection3d_model = Node(
        package = 'lidar_detection',
        executable = 'lidar_detection_node',
        name = 'lidar_detection_node',
        parameters=[
            {'model': 'mmdetection3d'},
            {'device': 'cuda:0'},
            {'conf_thresh': 0.4}
        ],
    )

    object_viz_deteced_node = Node(
        package='visualizer',
        executable='object_visualizer_node',
        name='object_visualizer_node',
        parameters=[
            {'topic': '/detected/objects3d'}
        ]
    )

    multi_object_tracker_3d_node = Node(
        package = 'multi_object_tracker_3d',
        executable = 'multi_object_tracker_3d_node',
        name = 'multi_object_tracker_3d_node',
        parameters=[],
    )

    object_viz_tracked_node = Node(
        package='visualizer',
        executable='object_visualizer_node',
        name='object_visualizer_node',
        parameters=[
            {'topic': '/tracked/objects3d'}
        ]
    )

    return LaunchDescription([
        complex_yolo_model,
        # mmdetection3d_model,
        object_viz_deteced_node,
        multi_object_tracker_3d_node,
        object_viz_tracked_node,
    ])
