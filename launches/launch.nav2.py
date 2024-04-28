import os
import sys

from os import path

sys.path.append(path.abspath("/navigator/"))

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    navigator_dir = "/navigator"  # get_package_share_directory('navigator')

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    map_yaml_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = []  # [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Only it applys when `use_namespace` is True.
    # '<robot_namespace>' keyword shall be replaced by 'namespace' launch argument
    # in config file 'nav2_multirobot_params.yaml' as a default & example.
    # User defined config file should contain '<robot_namespace>' keyword for the replacements.
    params_file = ReplaceString(
        source_file=params_file,
        # replacements={'<nav2_namespace>': ('/', namespace)},
        replacements={"<nav2_namespace>": namespace},
        condition=IfCondition(use_namespace),
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"autostart": autostart}

    configured_params2 = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    default_namespace = "nav2"
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value=default_namespace, description="Top-level namespace"
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="True",
        description="Whether to apply a namespace to the navigation stack",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(
            navigator_dir,
            "src",
            "planning",
            "nav2_path_planner",
            "nav2_path_planner",
            "nav2_map.param.yaml",
        ),
        description="Full path to map yaml file to load",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            navigator_dir, "param", "planning", "nav2.param.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Automatically startup the nav2 stack",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    # Specify the actions
    load_nodes = GroupAction(
        [
            PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
            # see nav2_bringup localization launch file
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {"yaml_filename": map_yaml_file}],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            # see nav2_bringup navigation launch file
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params2],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params2],
                arguments=["--ros-args", "--log-level", log_level],
                remappings=remappings,
            ),
            # merged the lifecycle_managers from nav2_bringup localization and navigation launch files
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="nav2_lifecycle_manager",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"autostart": autostart},
                    {"node_names": ["map_server", "smoother_server", "planner_server"]},
                ],
            ),
            # These are the static transforms relating nav2_map -> nav2_odom -> nav2_base_link. These are on top of each other
            # because of the way we are using our costmap
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--yaw",
                    "0",
                    "--pitch",
                    "0",
                    "--roll",
                    "0",
                    "--frame-id",
                    "%s_map" % default_namespace,
                    "--child-frame-id",
                    "%s_odom" % default_namespace,
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--yaw",
                    "0",
                    "--pitch",
                    "0",
                    "--roll",
                    "0",
                    "--frame-id",
                    "%s_odom" % default_namespace,
                    "--child-frame-id",
                    "%s_base_link" % default_namespace,
                ],
            ),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the nav2 nodes we want
    ld.add_action(load_nodes)

    return ld
