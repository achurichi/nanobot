import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE_NAME = "nanobot_lidar"

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory(PACKAGE_NAME))
    lidar_config_path = os.path.join(pkg_path, "config", "lidar.yaml")
    
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[lidar_config_path],
        output='screen'
    )
    
    return LaunchDescription([
        lidar_node
    ])
