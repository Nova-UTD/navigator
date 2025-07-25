"""This module defines nodes used in launch files."""

from launch_ros.actions import Node

NAVIGATOR_DIR = "/navigator/"

pure_pursuit_controller = Node(
    package='pure_pursuit_controller',
    executable='pure_pursuit_controller'
)

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

costmap_recorder = Node(
    package="recording", 
    executable="costmap_recorder"
)

driveable_area = Node(
    package='driveable_area',
    executable='driveable_area_node'
)

grid_route_costmap = Node(
    package='costs',
    executable='route_costmap_node'
)

grid_summation = Node(
    package='costs',
    executable='grid_summation_node'
)

ground_seg = Node(
    package='ground_seg',
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
        {'data_path': '/navigator/data'},
        {'route_path': '/navigator/data/maps/ps4_loop_route.txt'}
    ]#,
    #prefix=['xterm -e gdb -ex run --args']
)

map_manager_carla = Node(
    package='map_management',
    executable='map_management_node',
    parameters=[
        {'from_file': False}
    ]#,
    #prefix=['xterm -e gdb -ex run --args']
)

odom2tf = Node(
    package='recording',
    executable='odom2tf'
)

path_planner = Node(
    package="path_planners", 
    executable="path_planner_node",
    parameters=[
        {'planner': 'dijkstra'}
    ]
) # options: dijkstra, ara-star, dynamic-programming, neural-network, trrt-star

path_planner_nav2 = Node(
    package='nav2_path_planner',
    executable='nav2_path_planner_node'
)

prednet_inference = Node(
    package='prednet_inference',
    executable='prednet_inference_node'
)

pedestrian_intent_to_enter_road = Node(
    package='pedestrian_intent_to_enter_road',
    executable='pedestrian_intent_to_enter_road'
)

recorder = Node(
    package='recording',
    executable='recorder'
)

routing_monitor = Node(
    package='helpers',
    executable='routing_monitor'
)

routing_hardcoded = Node(
    package='helpers',
    executable='routing_hardcoded',
    parameters=[{'filename': "/navigator/Route.csv"}]
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
    arguments=['-d' + '/navigator/data/navigator_default.rviz'],
    respawn=False,
    output={'both': 'log'}
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

traffic_light_detector = Node(
    package='traffic_light_detector',
    executable='traffic_light_node'
)
    
road_signs_classifier = Node(
   	package='road_signs_classifier',
   	executable='road_signs_classifier',
   	name='road_signs_classifier',
   	parameters=[],
)

image_segmentation = Node(
    package='image_segmentation',
    executable='image_seg_node'
)

depth_processing = Node(
    package='depth_processing',
    executable='depth_processing_node'
)

occupancy_grid_node = Node(
    package='occupancy_grid',
    executable='occupancy_grid'
)

intersection_manager = Node(
    package='intersection_manager',
    executable='intersection_manager'
)

lane_type_detector = Node(
   	package='lane_type_detector',
   	executable='lane_type_detector',
   	name='lane_type_detector',
)

pedestrian_skeleton = Node(
   	package='pedestrian_skeleton',
   	executable='pedestrian_skeleton',
   	name='pedestrian_skeleton',
   	parameters=[],
)

road_user_detector = Node(
    package='road_user_detection',
    executable='road_user_detection'
)