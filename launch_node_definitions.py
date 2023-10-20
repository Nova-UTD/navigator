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

NAVIGATOR_DIR = "/navigator/"

airbags = Node(
    package='airbags',
    executable='airbag_node'
)

camera_streamer = Node(
    package='web_video_server',
    executable='web_video_server'
)

carla_guardian = Node(
    package='guardian',
    executable='guardian_node',
    parameters=[
        {'simulated': True}
    ]
)

clock = Node(
    package='clock',
    executable='clock_node'
)

controller_parade = Node(
    package="parade_controller",
    executable="parade_controller_node"
)

driveable_area = Node(
    package='driveable_area',
    executable='driveable_area_node'
)

grid_summation = Node(
    package='costs',
    executable='grid_summation_node'
)

ground_seg = Node(
    package='occupancy_cpp',
    executable='ground_segmentation_exe',
    parameters=[
        {'sensitivity': 0.13},
    ]
)

guardian = Node(
    package='guardian',
    executable='guardian_node'
)

gnss_averager = Node(
    package='state_estimation',
    executable='gnss_averaging_node'
)

image_segmentation = Node(
    package='segmentation',
    executable='image_segmentation_node'
)

joy_translator = Node(
    package='joy_translation',
    executable='joy_translation_node'
)

junction_manager = Node(
    package='costs',
    executable='junction_manager'
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
        {'data_path': '/navigator/data'}
    ]
)

map_manager_carla = Node(
    package='map_management',
    executable='map_management_node',
    parameters=[
        {'from_file': False}
    ]
)

odom2tf = Node(
    package='recording',
    executable='odom2tf'
)

prednet_inference = Node(
    package='prednet_inference',
    executable='prednet_inference_node'
)

recorder = Node(
    package='recording',
    executable='recorder'
)

routing_monitor = Node(
    package='helpers',
    executable='routing_monitor'
)

rqt = Node(
    package='rqt_gui',
    executable='rqt_gui',
    arguments=["--perspective-file=/navigator/data/rqt.perspective"]
)

rtp = Node(
    package='rtp',
    executable='rtp_node'
)

rviz = Node(
    package='rviz2',
    namespace='',
    executable='rviz2',
    name='rviz2',
    arguments=['-d' + '/navigator/data/real_world.rviz'],
    respawn=True
)

semantic_projection = Node(
    package='segmentation',
    executable='image_projection_node'
)

sounds = Node(
    package='guardian',
    executable='sound_node'
)

static_grid = Node(
    package='occupancy_cpp',
    executable='static_grid_exe'
)

web_bridge = Node(
    package='rosbridge_server',
    executable='rosbridge_websocket'
)
