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

NAVIGATOR_DIR = "/home/nova/navigator/"

def generate_launch_description():

    # leaderboard_liaison = Node(
    #     package='carla_interface',
    #     executable='liaison_node',
    #     parameters=[]
    # )

    lidar_processor = Node(
        package='sensor_processing',
        executable='dual_lidar_processing_node'
    )

    mcl = Node(
        package='state_estimation',
        executable='mcl_node'
    )

    camera_streamer = Node(
        package='web_video_server',
        executable='web_video_server'
    )

    # carla_spawner = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([get_package_share_directory(
    #         'carla_spawn_objects'), '/carla_spawn_objects.launch.py']),
    #     launch_arguments={
    #         'objects_definition_file': '/navigator/data/carla_objects.json'}.items(),
    # )

    # carla_controller = Node(
    #     package='carla_controller',
    #     executable='controller'
    # )

    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[path.join(NAVIGATOR_DIR, "data", "hail_bopp.urdf")]
    )

    guardian = Node(
        package='guardian',
        executable='guardian_node'
    )

    sounds = Node(
        package='guardian',
        executable='sound_node'
    )

    gnss = Node(
        package='gnss',
        executable='gnss_interface_node'
    )

    mcl = Node(
        package='state_estimation',
        executable='mcl_node'
    )

    map_manager = Node(
        package='map_management',
        executable='map_management_node',
        parameters=[
            {'from_file': True},
            {'data_path': '/home/nova/navigator/data'}
        ]
    )

    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + '/navigator/data/real_world.rviz'],
        respawn=True
    )

    ground_seg = Node(
        package='occupancy_cpp',
        executable='ground_segmentation_exe',
        parameters=[
            {'sensitivity': 0.13},
        ]
    )

    image_segmentation = Node(
        package='segmentation',
        executable='image_segmentation_node'
    )

    semantic_projection = Node(
        package='segmentation',
        executable='image_projection_node'
    )

    mcu_interface = Node(
        package='mcu_interface',
        executable='mcu_interface_node'
    )

    junction_manager = Node(
        package='costs',
        executable='junction_manager'
    )

    joy = Node(
        package='joy_linux',
        executable='joy_linux_node',
        parameters=[
        {"dev":"/dev/input/by-id/usb-©Microsoft_Corporation_Controller_061ABA4-joystick"}
        ]
    )

    joy_translator = Node(
        package='joy_translation',
        executable='joy_translation_node'
    )

    epas = Node(
        package='epas',
        executable='epas_node'
    )

    linear_actuator = Node(
        package='linear_actuator',
        executable='linear_actuator_node'
    )

    controller = Node(
        package="parade_controller",
        executable="parade_controller_node"
    )

    right_lidar_driver = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        parameters=[
            "/home/nova/navigator/param/perception/lidar_driver_right.param.yaml"],
        namespace='velo_right'
    )
    left_lidar_driver = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        parameters=[
            "/home/nova/navigator/param/perception/lidar_driver_left.param.yaml"],
        namespace='velo_left'
    )

    left_lidar_pointcloud = Node(
        package='velodyne_pointcloud',
        executable='velodyne_convert_node',
        parameters=[
            "/home/nova/navigator/param/perception/lidar_pointcloud_left.param.yaml"],
        namespace='velo_left'
    )

    right_lidar_pointcloud = Node(
        package='velodyne_pointcloud',
        executable='velodyne_convert_node',
        parameters=[
            "/home/nova/navigator/param/perception/lidar_pointcloud_right.param.yaml"],
        namespace='velo_right'
    )

    camera = Node(
        package='camera',
        executable='camera_node'
    )

    gps_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver'
    )

    web_bridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket'
    )

    clock = Node(
        package='clock',
        executable='clock_node'
    )

    static_grid = Node(
        package='occupancy_cpp',
        executable='static_grid_exe'
    )

    recorder = Node(
        package='recording',
        executable='recorder'
    )

    odom2tf = Node(
        package='recording',
        executable='odom2tf'
    )

    grid_summation = Node(
        package='costs',
        executable='grid_summation_node'
    )

    rtp = Node(
        package='rtp',
        executable='rtp_node'
    )

    return LaunchDescription([
        # CONTROL
        # controller,

        # INTERFACE
        # camera_streamer,
        joy,
        joy_translator,
        epas,
        mcu_interface,
        linear_actuator,
        # web_bridge,
        gnss,
        left_lidar_driver,
        left_lidar_pointcloud,
        right_lidar_driver,
        right_lidar_pointcloud,
        # camera,

        # MISC
        clock,
        
        # recorder,
        urdf_publisher,
        # rviz,

        # PERCEPTION
        ground_seg,
        lidar_processor,
        static_grid,
        

        # PLANNING
        map_manager,
        grid_summation,
        # rtp,
        odom2tf,
        junction_manager,


        # SAFETY
        guardian,
        sounds,
    ])
