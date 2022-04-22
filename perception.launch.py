from os import path

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    launch_path = path.realpath(__file__)
    launch_dir = path.dirname(launch_path)
    param_dir = path.join(launch_dir,"param")

    # BRIDGING
    carla_sim_bridge = Node(
        package='sim_bridge',
        executable='sim_bridge_node',
    )

    # MISC

    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[path.join(launch_dir, "data", "voltron.urdf")]
    )

    # PERCEPTION

    obstacle_detector_2d = Node(
        package='darknet_inference',
        executable='darknet_inference_node',
        name='object_detector_2d_node',
        parameters=[{'publish_labeled_image': False}],
        remappings=[
            ('/color_image', '/sensors/zed/left_rgb'),
            ('/obstacle_array_2d', '/obstacle_array_2d'),
        ]    
    )

    pcd_gen = Node(
        package='point_cloud_gen',
        executable='pcd_gen_node',
        remappings=[
            ('/depth_image', '/sensors/zed/depth_img'),
            ('/depth_point_cloud', '/depth_point_cloud'),
        ] 
    )

    bbox_gen = Node(
        package='bbox_generator',
        executable='bbox_gen_node',
        name='bbox_gen_node',
        remappings=[
            ('/depth_image', '/sensors/zed/depth_img'),
            ('/obstacle_array_2d', '/obstacle_array_2d'),
            ('/lidar_fused', '/lidar_fused'),
            ('/obstacle_array_3d', '/obstacle_array_3d'),
        ]    
    )

    obstacle_drawer = Node(
        package='obstacle_drawer',
        name='obstacle_drawer_node',
        executable='obstacle_drawer_exe',
        remappings=[
            ('/visualizations', '/detections/visualizations'),
            ('/obstacle_array_3d', '/obstacle_array_3d'),
        ]  
    )

    lidar_fusion = Node(
        package='lidar_fusion',
        name='lidar_fusion_node',
        executable='lidar_fusion_node',
        remappings=[
            ('/lidar_front', '/lidar_front/velodyne_points'),
            ('/lidar_rear', '/lidar_rear/velodyne_points'),
            ('/lidar_fused', '/lidar_fused')
        ]      
    )


    return LaunchDescription([
        urdf_publisher,
        obstacle_detector_2d,
        lidar_fusion,
        obstacle_drawer,
        bbox_gen,
        #pcd_gen
    ])
