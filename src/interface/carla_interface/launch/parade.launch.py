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

<<<<<<< HEAD
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

=======
>>>>>>> 6f24d93b (Replaced all references to lidar_front/rear to right/left. Parade launch file now contains necessary lidar packages)
    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[path.join("/navigator/data", "carla.urdf")]
    )

    left_lidar_driver = Node(
        package = 'velodyne_driver',
        executable = 'velodyne_driver_node',
        parameters = ["/navigator/param/perception/lidar_driver_left.param.yaml"]
    )

    right_lidar_driver = Node(
        package = 'velodyne_driver',
        executable = 'velodyne_driver_node',
        parameters = ["/navigator/param/perception/lidar_driver_right.param.yaml"]
    )

    left_lidar_pointcloud = Node(
        package = 'velodyne_pointcloud',
        executable = 'velodyne_transform_node',
        parameters = ["/navigator/param/perception/lidar_pointcloud_left.param.yaml"]
    )

    right_lidar_pointcloud = Node(
        package = 'velodyne_pointcloud',
        executable = 'velodyne_transform_node',
        parameters = ["/navigator/param/perception/lidar_pointcloud_right.param.yaml"]
    )

    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + '/navigator/data/mcl.rviz']
    )

<<<<<<< HEAD
    ground_seg = Node(
        package='occupancy_cpp',
        executable='ground_segmentation_exe'
=======
    # semantic_projection = Node(
    #     package='segmentation',
    #     executable='image_projection_node'
    # )

    throttle_node = Node(
        package = 'throttle_control',
        executable = 'throttle_node'
>>>>>>> 6f24d93b (Replaced all references to lidar_front/rear to right/left. Parade launch file now contains necessary lidar packages)
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
    

    return LaunchDescription([
<<<<<<< HEAD
        # INTERFACE
        joy,
        joy_translator,
        epas,
        mcu_interface,

        # MISC
        urdf_publisher,
        # rviz,
=======
        # CONTROL
        throttle_node,
        joy_interface_node,
        joy_linux,
        epas_node,

        # LIDAR
        left_lidar_driver,
        right_lidar_driver,
        left_lidar_pointcloud,
        right_lidar_pointcloud,

        # MISC
        urdf_publisher,
        rviz,


>>>>>>> 6f24d93b (Replaced all references to lidar_front/rear to right/left. Parade launch file now contains necessary lidar packages)
    ])
