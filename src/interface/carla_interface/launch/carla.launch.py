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

    lidar_processor = Node(
        package='sensor_processing',
        executable='lidar_processing_node'
    )

    mcl = Node(
        package='state_estimation',
        executable='mcl_node'
    )

    carla_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'carla_spawn_objects'), '/carla_spawn_objects.launch.py']),
        launch_arguments={
            'objects_definition_file': '/navigator/data/carla_objects.json'}.items(),
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
            'timeout': '30'
        }.items(),
    )

    state_estimation = Node(
        package='state_estimation',
        executable='gnss_processing_node'
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

    return LaunchDescription([
        # CONTROL
        carla_controller,

        # INTERFACE
        # carla_bridge_official,
        # carla_spawner,
        leaderboard_liaison,

        # LOCALIZATION
        # mcl

        # MAPPING

        # MISC
        urdf_publisher,
        rviz,

        # PERCEPTION
        lidar_processor,
        ground_seg,

        # STATE ESTIMATION
        # map_manager,
        state_estimation,
    ])
