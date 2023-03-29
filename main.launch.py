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

    # leaderboard_liaison = Node(
    #     package='carla_interface',
    #     executable='liaison_node',
    #     parameters=[]
    # )

    lidar_processor = Node(
        package='sensor_processing',
        executable='lidar_processing_node'
    )

    mcl = Node(
        package='state_estimation',
        executable='mcl_node'
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
        arguments=[path.join("/navigator/data", "hail_bopp.urdf")]
    )

    # carla_bridge_official = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([get_package_share_directory(
    #         'carla_ros_bridge'), '/carla_ros_bridge.launch.py']),
    #     launch_arguments={
    #         'host': 'localhost',
    #         'port': str(2000 + int(environ['ROS_DOMAIN_ID'])),
    #         'synchronous_mode': 'True',
    #         'town': 'Town02',
    #         'register_all_sensors': 'False',
    #         'ego_vehicle_role_name': 'hero',
    #         'timeout': '30'
    #     }.items(),
    # )

    gnss_processor = Node(
        package='state_estimation',
        executable='gnss_processing_node'
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
        namespace='',
        executable='rviz2',
        name='rviz2',
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

    mcu_interface = Node(
        package='mcu_interface',
        executable='mcu_interface_node'
    )

    joy = Node(
        package='joy_linux',
        executable='joy_linux_node'
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

    left_lidar_driver = Node(
        package = 'velodyne_driver',
        executable = 'velodyne_driver_node',
        parameters = ["/navigator/param/perception/lidar_driver_left.param.yaml"],
        namespace='velo_left'
    )

    right_lidar_driver = Node(
        package = 'velodyne_driver',
        executable = 'velodyne_driver_node',
        parameters = ["/navigator/param/perception/lidar_driver_right.param.yaml"],
        namespace='velo_right'

    )

    left_lidar_pointcloud = Node(
        package = 'velodyne_pointcloud',
        executable = 'velodyne_convert_node',
        parameters = ["/navigator/param/perception/lidar_pointcloud_left.param.yaml"],
        namespace='velo_left'
    )

    right_lidar_pointcloud = Node(
        package = 'velodyne_pointcloud',
        executable = 'velodyne_convert_node',
        parameters = ["/navigator/param/perception/lidar_pointcloud_right.param.yaml"],
        namespace='velo_right'
    )

    camera = Node(
        package = 'camera',
        executable = 'camera_node'
    )

    gps_node = Node(
        package = 'nmea_navsat_driver',
        executable = 'nmea_serial_driver'
    )

    

    return LaunchDescription([
        # CONTROL
        # controller,

        # INTERFACE
        #joy,
        #joy_translator,
        #epas,
        #mcu_interface,
        #linear_actuator,

        left_lidar_driver,
        left_lidar_pointcloud,
        right_lidar_driver,
        right_lidar_pointcloud,

        camera,

        gps_node,

        # MISC
        urdf_publisher,
        # rviz,
    ])
