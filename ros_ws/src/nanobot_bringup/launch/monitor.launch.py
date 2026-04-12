import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.actions import Node

JOYSTICK_PACKAGE_NAME = "nanobot_joystick"


def generate_launch_description():
    foxglove_launch_file = os.path.join(
        get_package_share_directory('foxglove_bridge'),
        'launch',
        'foxglove_bridge_launch.xml'
    )
    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(foxglove_launch_file),
        launch_arguments={
            'port': '8765',
            'address': '0.0.0.0',
            'capabilities': "['clientPublish', 'connectionGraph', 'assets']"
        }.items()
    )

    joystick_launch_file = os.path.join(
        get_package_share_directory(JOYSTICK_PACKAGE_NAME),
        "launch",
        "joystick.launch.py",
    )
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([joystick_launch_file])
    )

    return LaunchDescription([
        foxglove_bridge, 
        joystick
        ])
