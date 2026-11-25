import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    default_config = os.path.join(
        get_package_share_directory('adnav_launch'),
        'config',
        'adnav_serial.yaml'
    )

    declare_config = DeclareLaunchArgument(
        'adnav_serial_config',
        default_value=default_config,
        description='Path to the adnav_node parameter YAML file.',
    )

    node=Node(
        name = 'adnav_node',
        package = 'adnav_driver',
        executable = 'adnav_driver',
        emulate_tty = True,
        output = 'screen',
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters = [LaunchConfiguration('adnav_serial_config')]
    )

    ld.add_action(declare_config)
    ld.add_action(node)
    return ld