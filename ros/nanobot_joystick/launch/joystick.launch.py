import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node

PACKAGE_NAME = "nanobot_joystick"


def generate_launch_description():
    joystick_config_path = os.path.join(
        os.path.join(get_package_share_directory(PACKAGE_NAME)), "config", "joystick.yaml"
    )
    
    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[joystick_config_path],
    )
    
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_node",
        parameters=[joystick_config_path],
        remappings=[("/cmd_vel", "/cmd_vel_joy")]
    )
    
    return LaunchDescription([
        joy_node,
        teleop_node,
    ])

