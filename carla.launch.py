from os import name, path, environ

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python import get_package_share_directory


def generate_launch_description():

    leaderboard_liaison = Node(
        package='carla_interface',
        executable='liaison_node',
        parameters=[]
    )

    lidar_processor = Node(
        package='sensor_processing',
        executable='lidar_processing_node'
    )

    carla_controller = Node(
        package='carla_controller',
        executable='controller'
    )

    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[path.join("/navigator/data", "carla.urdf")]
    )

    airbags = Node(
        package='airbags',
        executable='airbag_node'
    )

    camera_streamer = Node(
        package='web_video_server',
        executable='web_video_server'
    )

    joy_linux = Node(
        package="joy_linux",
        executable="joy_linux_node"
    )

    joy_translation = Node(
        package="joy_translation",
        executable="joy_translation_node"
    )

    gnss_processor = Node(
        package='state_estimation',
        executable='gnss_processing_node'
    )

    gnss_averager = Node(
        package='state_estimation',
        executable='gnss_averaging_node'
    )

    mcl = Node(
        package='state_estimation',
        executable='mcl_node'
    )

    map_manager = Node(
        package='map_management',
        executable='map_management_node'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d' + '/navigator/data/mcl.rviz']
    )

    ground_seg = Node(
        package='occupancy_cpp',
        executable='ground_segmentation_exe'
    )

    image_segmentation = Node(
        package='segmentation',
        executable='image_segmentation_node'
    )

    semantic_projection = Node(
        package='segmentation',
        executable='image_projection_node'
    )

    static_grid = Node(
        package='occupancy_cpp',
        executable='static_grid_exe'
    )

    grid_summation = Node(
        package='costs',
        executable='grid_summation_node'
    )

    junction_manager = Node(
        package='costs',
        executable='junction_manager'
    )

    rqt = Node(
        package='rqt_gui',
        executable='rqt_gui',
        arguments=["--perspective-file=/navigator/data/rqt.perspective"]
    )

    route_reader = Node(
        package='carla_interface',
        executable='route_reader_node'
    )

    rtp = Node(
        package='rtp',
        executable='rtp_node'
    )

    prednet_inference = Node(
        package='prednet_inference',
        executable='prednet_inference_node'
    )
    
    driveable_area = Node(
        package='driveable_area',
        executable='driveable_area_node'
    )

    web_bridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket'
    )

    guardian = Node(
        package='guardian',
        executable='guardian_node',
        parameters=[
            {'simulated': True}
        ]
    )

    recorder = Node(
        package='recording',
        executable='recorder'
    )

    return LaunchDescription([
        # CONTROL
        carla_controller,

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
        urdf_publisher,
        # rviz,
        # rqt,
        # camera_streamer,

        # PERCEPTION
        # image_segmentation,
        # semantic_projection,
        lidar_processor,
        ground_seg,
        static_grid,
        # prednet_inference,
        # driveable_area,

        # PLANNING
        grid_summation,
        route_reader, # <<---- needs to be moved to carla bridge docker
        junction_manager,
        rtp,

        # SAFETY
        ##airbags,
        ##guardian,

        # STATE ESTIMATION
        map_manager,
        gnss_processor,
    ])
