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

    leaderboard_liaison = Node(
        package='carla_interface',
        executable='liaison_node',
        parameters=[]
    )

    # urdf_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     arguments=[path.join(launch_dir, "data", "carla.urdf")]
    # )

    lidar_processor = Node(
        package='sensor_processing',
        executable='lidar_processing_node'
    )

    mcl = Node(
        package='state_estimation',
        executable='mcl_node'
    )

    state_estimation = Node(
        package='state_estimation',
        executable='carla_estimation_node'
    )

    carla_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'carla_spawn_objects'), '/carla_spawn_objects.launch.py']),
        launch_arguments={
            'objects_definition_file': '/navigator/data/carla_objects_mcl.json'}.items(),
    )

    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[path.join("/navigator/data", "carla.urdf")]
    )

    carla_bridge_official = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'carla_ros_bridge'), '/carla_ros_bridge.launch.py']),
        launch_arguments={
            'host': 'localhost',
            'port': str(2000 + int(environ['ROS_DOMAIN_ID'])),
            'synchronous_mode': 'True',
            'town': 'Town02',
            'register_all_sensors': 'False',
            'ego_vehicle_role_name': 'hero',
            'timeout': '10'
        }.items(),
    )

    return LaunchDescription([

        # INTERFACE
        carla_bridge_official,
        carla_spawner,
        leaderboard_liaison,

        # LOCALIZATION
        # mcl

        # MAPPING

        # MISC
        urdf_publisher,

        # PERCEPTION
        lidar_processor,


        # STATE ESTIMATION
        state_estimation,
    ])
