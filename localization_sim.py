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

    launch_path = path.realpath(__file__)
    launch_dir = path.dirname(launch_path)
    param_dir = path.join(launch_dir,"param")
    interface = "vcan0"
    map_name = "borregas_host"

    with_svl = DeclareLaunchArgument(
        'with_svl',
        default_value='False',
        description='Enable SVL bridge'
    )

    # CONTROL
    steering_controller = Node(
        package='vt_steering_controller',
        executable='controller_exe',
        name='controller_exe',
        namespace='control'
    )

    # INTERFACE
    vehicle_bridge = Node(
        package='vt_vehicle_bridge',
        executable='svl_bridge_exe',
    )

    svl_bridge = Node(
        executable='lgsvl_bridge',
    )

    epas_reporter = Node(
        package='epas_translator',
        executable='reporter',
        parameters=[(path.join(param_dir,"interface","epas_reporter.param.yaml"))],
        remappings=[
            ("incoming_can_frames", "incoming_can_frames"),
            ("real_steering_angle", "real_steering_angle")
        ]
    )

    epas_controller = Node(
        package='voltron_epas_steering',
        executable='controller',
        parameters=[(path.join(param_dir,"interface","epas_controller.param.yaml"))],
        remappings=[
            ("steering_power", "steering_power"),
            ("outgoing_can_frames", "outgoing_can_frames")
        ]
    )
    # steering_pid
    can = Node(
        package='voltron_can',
        executable='interface',
        parameters=[(path.join(param_dir,"interface","can_interface.param.yaml"))],
        remappings=[
        ],
        arguments=[interface]
    )

    gnss = Node(
        package='nova_gps',
        executable='interface',
        parameters=[],
        remappings=[],
        arguments=['/dev/i2c-8']
    )

    # LOCALIZATION
    ndt = Node(
        package='ndt_nodes',
        executable='p2d_ndt_localizer_exe',
        namespace='localization',
        name='p2d_ndt_localizer_node',
        parameters=[(path.join(param_dir,"localization","ndt_localizer.param.yaml"))],
        remappings=[
            ("points_in", "/lidars/points_fused_downsampled"),
            ("observation_republish", "/lidars/points_fused_viz"),
        ]
    )

    hubble = Node(
        package='hubble',
        executable='hubble-exe',
        parameters=[(path.join(param_dir,"localization","hubble.param.yaml"))]
    )

    # MAPPING
    lanelet_server = Node(
        package='lanelet2_map_provider',
        executable='lanelet2_map_provider_exe',
        namespace='had_maps',
        name='lanelet2_map_provider_node',
        parameters=[(path.join(launch_dir, "data", "maps", map_name, "lanelet_server.param.yaml"))]
    )

    lanelet_visualizer = Node(
        package='lanelet2_map_provider',
        executable='lanelet2_map_visualizer_exe',
        name='lanelet2_map_visualizer_node',
        namespace='had_maps'
    )

    pcd_publisher = Node(
        package='ndt_nodes',
        executable='ndt_map_publisher_exe',
        namespace='localization',
        parameters=[(path.join(launch_dir, "data", "maps", map_name, "pcd_publisher.param.yaml"))]
    )

    # MISC
    odom_bl_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0','0','0','0','0','0','odom','base_link'
        ]
    )

    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[path.join(launch_dir, "data", "voltron.urdf")]
    )
    
    visuals = Node(
        package='vt_viz',
        name='vt_viz_node',
        executable='vt_viz_exe',
    )

    # PERCEPTION
    lidar_front = Node(
        package='velodyne_nodes',
        namespace="lidar_front",
        executable='velodyne_cloud_node_exe',
        parameters=[(path.join(param_dir,"perception","lidar_front.param.yaml"))],
        remappings=[("topic", "points_raw")],
        arguments=["--model", "vlp16"])

    lidar_rear = Node(
        package='velodyne_nodes',
        namespace="lidar_rear",
        executable='velodyne_cloud_node_exe',
        parameters=[(path.join(param_dir,"perception","lidar_rear.param.yaml"))],
        remappings=[("topic", "points_raw")],
        arguments=["--model", "vlp16"])
    
    lidar_front_filter = Node(
        package="point_cloud_filter_transform_nodes",
        executable="point_cloud_filter_transform_node_exe",
        namespace = "lidar_front",
        parameters=[(path.join(param_dir,"perception","pc_filter_transform_front_sim.param.yaml"))],
        remappings = [("points_in", "/lidar_front/points_raw")]
    )

    lidar_rear_filter = Node(
        package="point_cloud_filter_transform_nodes",
        executable="point_cloud_filter_transform_node_exe",
        namespace = "lidar_rear",
        parameters=[(path.join(param_dir,"perception","pc_filter_transform_rear_sim.param.yaml"))],
        remappings = [("points_in", "/lidar_rear/points_raw")]
    )

    lidar_fusion = Node(
        package='point_cloud_fusion_nodes',
        executable='pointcloud_fusion_node_exe',
        namespace="lidars",
        parameters=[(path.join(param_dir,"perception","lidar_fusion.param.yaml"))],
        remappings=[
            ("output_topic", "points_fused"),
            ("input_topic1", "/lidar_front/points_filtered"), 
            ("input_topic2", "/lidar_rear/points_filtered")
        ]
    )

    lidar_downsampler = Node(
        package='voxel_grid_nodes',
        executable='voxel_grid_node_exe',
        namespace='lidars',
        parameters=[(path.join(param_dir,"perception","lidar_downsampler.param.yaml"))],
        remappings=[
            ("points_in", "points_fused"),
            ("points_downsampled", "points_fused_downsampled")
        ],
    )
    
    # PLANNING
    route_planner = Node(
        package='lanelet2_global_planner_nodes',
        name='lanelet2_global_planner_node',
        namespace='planning',
        executable='lanelet2_global_planner_node_exe',
        respawn=True,
        remappings=[('HAD_Map_Client', '/had_maps/HAD_Map_Service'),
                    ('ndt_pose', '/localization/ndt_pose'),
                    ('vehicle_kinematic_state', '/vehicle/vehicle_kinematic_state')]
    )
    
    path_planner = Node(
        package='behavior_planner_nodes',
        name='behavior_planner_node',
        namespace='planning',
        executable='behavior_planner_node_exe',
        parameters=[
            (path.join(param_dir,"planning","path_planner.param.yaml"))
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
        parameters=[(path.join(param_dir,"planning","lane_planner.param.yaml"))],
        remappings=[('HAD_Map_Service', '/had_maps/HAD_Map_Service')]
    )
    parking_planner = Node(
        package='parking_planner_nodes',
        name='parking_planner_node',
        namespace='planning',
        executable='parking_planner_node_exe',
        parameters=[(path.join(param_dir,"planning","parking_planner.param.yaml"))],
        remappings=[('HAD_Map_Service', '/had_maps/HAD_Map_Service')]
    )

    # LIO-SAM only needs three inputs: IMU, Lidar, and GPS
    return LaunchDescription([
        # CONTROL
        # steering_controller,

        # INTERFACE
        # can,
        # epas_controller,
        # epas_reporter,
        # gnss,
        svl_bridge, # <- Provides IMU, GPS, and raw Lidar
        vehicle_bridge,

        # LOCALIZATION
        # ndt,

        # MAPPING
        # lanelet_server,
        # lanelet_visualizer,
        # pcd_publisher,
        hubble,

        # MISC
        # odom_bl_publisher,
        urdf_publisher,
        visuals,

        # PERCEPTION <- Provided filtered Lidar
        lidar_front,
        lidar_rear,
        lidar_front_filter,
        lidar_rear_filter,
        lidar_fusion,
        lidar_downsampler,

        # PLANNING
        # route_planner,
        # path_planner,
        # lane_planner,
        # parking_planner,
    ])