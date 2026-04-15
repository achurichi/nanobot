import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource


def generate_launch_description():
    rosbridge_websocket_path = os.path.join(
        get_package_share_directory("rosbridge_server"), "launch", "rosbridge_websocket_launch.xml"
    )
    websocket = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource([rosbridge_websocket_path]),
    )
    
    return LaunchDescription([
        websocket
    ])

