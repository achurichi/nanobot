import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node

PACKAGE_NAME = "nanobot_joystick"


def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory(PACKAGE_NAME))
    joystick_config_path = os.path.join(pkg_path, "config", "joystick.yaml")
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joystick_config_path],
    )
    
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joystick_config_path],
        remappings=[
            ('/cmd_vel', 'diff_controller/cmd_vel_unstamped')
        ]
    )
    
    return LaunchDescription([
        joy_node,
        teleop_node
    ])

