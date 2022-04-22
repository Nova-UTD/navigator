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

    launch_path = path.realpath(__file__)
    launch_dir = path.dirname(launch_path)
    param_dir = path.join(launch_dir,"param")
    interface = "vcan0"
    map_name = "grandloop"

    # CONTROL
    unified_controller = Node(
        package='unified_controller',
        executable='unified_controller_node'
    )

    # INTERFACE
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

    speedometer_reporter = Node(
        package='can_translation',
        executable='float_reporter',
        parameters=[(path.join(param_dir,"interface","speedometer_reporter.param.yaml"))],
        remappings=[
            ("incoming_can_frames", "incoming_can_frames_can1"),
            ("result_topic", "vehicle_speedometer")
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
        arguments=['/dev/ttyACM0']
    )

    # carla = Node(
    #     package='sim_bridge',
    #     executable='sim_bridge_node'
    # )

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
    map_odom_ukf = Node(
        package='robot_localization',
        executable='ukf_node',
        name='localization_map_odom',
        parameters=[(path.join(param_dir,"atlas","map_odom.param.yaml"))],
        remappings=[
            ("/odom0", "/sensors/gnss/odom"),
            ("/imu0", "/imu_primary/data"),
            ("/twist0", "/can/speedometer_twist")
        ]
    )

    # odom_bl_ukf = Node(
    #     package='robot_localization',
    #     executable='ukf_node',
    #     name='localization_map_odom',
    #     parameters=[(path.join(param_dir,"atlas","odom_bl.param.yaml"))],
    #     remappings=[
    #         ("/imu0", "/imu_primary/data")
    #     ]
    # )

    # MAPPING
    lanelet_server = Node(
        package='lanelet2_map_provider',
        executable='lanelet2_map_provider_exe',
        namespace='had_maps',
        name='lanelet2_map_provider_node',
        parameters=[(path.join(launch_dir, "data", "maps", map_name, "lanelet_server.param.yaml"))]
    )

    odr_viz = Node(
        package='odr_visualizer',
        executable='visualizer',
        parameters=[
            (path.join(param_dir,"mapping","odr.param.yaml"))
        ],
        output='screen'
    )

    # lanelet_visualizer = Node(
    #     package='lanelet2_map_provider',
    #     executable='lanelet2_map_visualizer_exe',
    #     name='lanelet2_map_visualizer_node',
    #     namespace='had_maps'
    # )

    pcd_loader = Node(
        package='map_publishers',
        executable='pcd_loader',
        parameters=[(path.join(launch_dir, "data", "maps", map_name, "map.param.yaml"))]
    )

    # MISC
    odom_bl_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.0','0.0','0.0','0.0','0.0','0.0','1.0','odom','base_link'
        ]
    )

    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[path.join(launch_dir, "data", "hail_bopp.urdf")]
    )
    
    visuals = Node(
        package='vt_viz',
        name='vt_viz_node',
        executable='vt_viz_exe',
    )

    # PERCEPTION
    lidar_driver_front = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        namespace='lidar_front',
        parameters=[(path.join(launch_dir, "param", "perception","lidar_driver_front.param.yaml"))]
    )
    lidar_pointcloud_front = Node(
        package='velodyne_pointcloud',
        executable='velodyne_convert_node',
        namespace='lidar_front',
        parameters=[(path.join(launch_dir, "param", "perception","lidar_pointcloud_front.param.yaml"))]
    )
    lidar_driver_rear = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        namespace='lidar_rear',
        parameters=[(path.join(launch_dir, "param", "perception","lidar_driver_rear.param.yaml"))]
    )
    lidar_pointcloud_rear = Node(
        package='velodyne_pointcloud',
        executable='velodyne_convert_node',
        namespace='lidar_rear',
        parameters=[(path.join(launch_dir, "param", "perception","lidar_pointcloud_rear.param.yaml"))]
    )

    curb_detector = Node(
        package='curb_detection',
        executable='curb_detector'
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
    
    # path_planner = Node(
    #     package='behavior_planner_nodes',
    #     name='behavior_planner_node',
    #     namespace='planning',
    #     executable='behavior_planner_node_exe',
    #     parameters=[
    #         (path.join(param_dir,"planning","path_planner.param.yaml"))
    #     ],
    #     output='screen',
    #     remappings=[
    #         ('HAD_Map_Service', '/had_maps/HAD_Map_Service'),
    #         ('vehicle_state', '/vehicle/vehicle_kinematic_state'),
    #         ('route', 'global_path'),
    #         ('gear_report', '/vehicle/gear'),
    #         ('vehicle_state_command', '/vehicle/state_command')
    #     ]
    # )

    path_publisher = Node(
        package='path_publisher',
        executable='publisher',
        parameters=[
            (path.join(param_dir,"planning","path_publisher.param.yaml"))
        ],
        namespace='planning',
        output='screen',
        respawn=True
    )

    motion_planner = Node(
        package='motion_planner',
        name='motion_planner_node',
        namespace='planning',
        executable='motion_planner',
        output='screen',
        remappings=[
            ('vehicle_kinematic_state', '/vehicle/vehicle_kinematic_state')
        ],
        parameters=[
            (path.join(param_dir,"planning","motion_planner.param.yaml"))
        ],
    )
    behavior_planner = Node(
        package='nova_behavior_planner',
        name='behavior_planner',
        namespace='planning',
        executable='BehaviorPlannerLaunch',
        output='screen',
        parameters=[
            (path.join(param_dir,"planning","path_publisher.param.yaml"))
        ],
        remappings=[
            
        ]
    )
    obstacle_zoner = Node(
        package='obstacle_zoner',
        name='obstacle_zoner',
        namespace='planning',
        executable='ObstacleZonerLaunch',
        output='screen',
    )
    zone_fusion = Node(
        package='zone_fusion',
        name='zone_fusion',
        namespace='planning',
        executable='ZoneFusionLaunch',
        output='screen',
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

    obstacle_republisher = Node(
        package='obstacle_repub',
        name='obstacle_republisher_node',
        executable='obstacle_repub_exe',
        remappings=[
            ('svl_obstacle_array', '/ground_truth_3d/detections'),
            ('zed_obstacle_array', '/zed_2i/obj_det/objects'),
            ('nova_obstacle_array', '/obstacles/array')
        ]
    )

    obstacle_drawer = Node(
        package='obstacle_drawer',
        name='obstacle_drawer_node',
        executable='obstacle_drawer_exe',
        remappings=[
            ('obstacle_marker_array', '/obstacles/marker_array'),
            ('nova_obstacle_array', '/obstacles/array')
        ]
    )
    
    # VIZ
    lanelet_visualizer = Node(
        package='map_publishers',
        executable='lanelet_loader'
    )
    return LaunchDescription([
        # CONTROL
        unified_controller,

        # INTERFACE
        # can,
        # carla,
        # epas_controller,
        # epas_reporter,
        # gnss,
        # speedometer_reporter,

        # LOCALIZATION
        # ndt,
        map_odom_ukf,

        # MAPPING
        odr_viz,

        # MISC
        odom_bl_link,
        urdf_publisher,
        visuals,

        # PERCEPTION
        # lidar_driver_front,
        # lidar_pointcloud_front,
        # lidar_driver_rear,
        # lidar_pointcloud_rear,
        # curb_detector,

        # PLANNING
        # route_planner,
        # path_planner,
        # lane_planner,
        # parking_planner,
        zone_fusion,
        obstacle_zoner,
        behavior_planner,
        path_publisher,
        motion_planner
    ])
