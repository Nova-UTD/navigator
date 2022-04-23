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

    launch_path = path.realpath(__file__)
    launch_dir = path.dirname(launch_path)
    param_dir = path.join(launch_dir,"../param")

    serial = Node(
        package = 'serial',
        executable = 'serial',
        parameters = [
            (path.join(param_dir, "interface", "serial.param.yaml"))],
        remappings = [
            ("serial_incoming_lines", "serial_incoming_lines"),
            ("serial_outgoing_lines", "serial_outgoing_lines")])
    
    servo_throttle = Node(
        package = 'servo',
        executable = 'servo',
        parameters = [
            (path.join(param_dir, "interface", "servo_throttle.param.yaml"))],
        remappings = [
            ("servo_commands", "serial_outgoing_lines"),
            ("servo_positions", "throttle_position")])

    servo_brake = Node(
        package = 'servo',
        executable = 'servo',
        parameters = [
            (path.join(param_dir, "interface", "servo_brake.param.yaml"))],
        remappings = [
            ("servo_commands", "serial_outgoing_lines"),
            ('servo_positions', 'brake_position')])

    epas_can = Node(
        package = 'can_interface',
        executable = 'interface',
        remappings = [
            ('can_interface_incoming_can_frames', 'epas_incoming_can'),
            ('can_interface_outgoing_can_frames', 'epas_outgoing_can')],
        arguments = ['can0'])

    
    epas_reporter = Node(
        package = 'epas_translator',
        executable = 'reporter',
        parameters = [
            (path.join(param_dir,"interface","epas_reporter.param.yaml"))],
        remappings = [
            ("epas_translator_incoming_can_frames", "epas_incoming_can"),
            ("epas_translator_real_steering_angle", "real_steering_angle")])

    epas_controller = Node(
        package = 'epas_translator',
        executable = 'controller',
        remappings = [
            ("epas_translator_steering_power", "steering_power"),
            ("epas_translator_outgoing_can_frames", "epas_outgoing_can"),
            ("epas_translator_enable", "steering_enable")])

    steering_pid = Node(
        package = 'pid_controller',
        executable = 'pid_controller',
        parameters = [
            (path.join
             (param_dir,"interface","steering_pid_controller.param.yaml"))],
        remappings = [
            ("output", "steering_power"),
            ("target", "steering_target"),
            ("measurement", "real_steering_angle")])

            
    zed_interface = Node (
        package = 'zed_interface',
        executable = 'zed_interface_exe'
    )
    return LaunchDescription([
        serial,
        servo_throttle,
        servo_brake,
        epas_can,
        epas_reporter,
        epas_controller,
        steering_pid,
        zed_interface
    ])
