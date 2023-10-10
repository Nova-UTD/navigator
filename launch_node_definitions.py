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

# MOVE TO VEHICLE_INTERFACE
camera = Node(
    package='camera',
    executable='camera_node'
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

# MOVE TO VEHICLE_INTERFACE
# I don't think we use this... maybe the old gps?
# but we should keep it for broader vehicle interface...
gps_node = Node(
    package='nmea_navsat_driver',
    executable='nmea_serial_driver'
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
    package='guardian'it,
    executable='guardian_node'
)

# MOVE TO VEHICLE_INTERFACE
gnss = Node(
    package='gnss',
    executable='gnss_interface_node'
)

# ??? I think leave in Navigator (at least for now)
gnss_processor = Node(
    package='state_estimation',
    executable='gnss_processing_node'
)

# ??? I think leave in Navigator (at least for now)
gnss_averager = Node(
    package='state_estimation',
    executable='gnss_averaging_node'
)
it
# MOVE TO VEHICLE_INTERFACE
hailbopp_epas = Node(
    package='epas',
    executable='epas_node'
)

# MOVE TO VEHICLE_INTERFACE
hailbopp_linear_actuator = Node(
    package='linear_actuator',
    executable='linear_actuator_node'
)

# MOVE TO VEHICLE_INTERFACE
hailbopp_mcu = Node(
    package='mcu_interface',
    executable='mcu_interface_node'
)

# MOVE TO VEHICLE_INTERFACE
hailbopp_urdf_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    arguments=[path.join(NAVIGATOR_DIR, "data", "hail_bopp.urdf")]
)

image_segmentation = Node(
    package='segmentation',
    executable='image_segmentation_node'
)

# MOVE TO VEHICLE_INTERFACE
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

junction_manager = Node(
    package='costs',
    executable='junction_manager'
)

# MOVE TO VEHICLE_INTERFACE
lidar_driver_right = Node(
    package='velodyne_driver',
    executable='velodyne_driver_node',
    parameters=[
        "/navigator/param/perception/lidar_driver_right.param.yaml"],
    namespace='velo_right'
)

# MOVE TO VEHICLE_INTERFACE
lidar_driver_left = Node(
    package='velodyne_driver',
    executable='velodyne_driver_node',
    parameters=[
        "/navigator/param/perception/lidar_driver_left.param.yaml"],
    namespace='velo_left'
)

# MOVE TO VEHICLE_INTERFACE
lidar_pointcloud_left = Node(
    package='velodyne_pointcloud',
    executable='velodyne_transform_node',
    parameters=[
        "/navigator/param/perception/lidar_pointcloud_left.param.yaml"],
    namespace='velo_left'
)

# MOVE TO VEHICLE_INTERFACE
lidar_pointcloud_right = Node(
    package='velodyne_pointcloud',
    executable='velodyne_transform_node',
    parameters=[
        "/navigator/param/perception/lidar_pointcloud_right.param.yaml"],
    namespace='velo_right'
)

# MOVE TO VEHICLE_INTERFACE
lidar_processor = Node(
    package='sensor_processing',
    executable='dual_lidar_processing_node'
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

odom2tf = Node(
    package='recording',
    executable='odom2tf'
)

prednet_inference = Node(
    package='prednet_inference',
    executable='prednet_inference_node'
)

# MOVE TO VEHICLE_INTERFACE
radar_processor = Node(
    package='sensor_processing',
    executable='delphi_esr_radar_processing_node'
)

recorder = Node(
    package='recording',
    executable='recorder'
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

# COPY TO VEHICLE_INTERFACE?  Keep a copy here for later customization?
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
