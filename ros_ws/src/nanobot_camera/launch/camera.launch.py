import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node

PACKAGE_NAME = "nanobot_camera"


def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory(PACKAGE_NAME))
    camera_config_path = os.path.join(pkg_path, "config", "camera.yaml")
    
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            parameters=[camera_config_path],
            remappings=[
                ("/image_raw", "/camera/image_raw"),
                ("/image_raw/compressed", "/camera/image_raw/compressed"),
            ],
        )
    ])

