import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    adnav_serial_config_cmd = DeclareLaunchArgument(
        "adnav_serial_config",
        default_value=os.path.join(
            get_package_share_directory("adnav_launch"), "config", "adnav_serial.yaml"
        ),
        description="Path to the adnav serial config file",
    )

    adnav_config = LaunchConfiguration("adnav_serial_config")

    node = Node(
        name="adnav_node",
        package="adnav_driver",
        executable="adnav_driver",
        emulate_tty=True,
        output="screen",
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[adnav_config],
    )

    ld.add_action(adnav_serial_config_cmd)
    ld.add_action(node)
    return ld
