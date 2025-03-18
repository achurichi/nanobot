import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

PACKAGE_NAME = "nanobot_navigation"


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    map_arg = DeclareLaunchArgument(
        "map",
        default_value="/home/nanobot/ros_ws/src/nanobot_navigation/maps/room.yaml",
        description="Map file"
    )
    map = LaunchConfiguration("map")
    
    localization_path = os.path.join(
        get_package_share_directory(PACKAGE_NAME), "launch", "localization.launch.py"
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([localization_path]),
        launch_arguments={
            "map": map,
            "use_sim_time": use_sim_time
        }.items(),
    )
    
    navigation_path = os.path.join(
        get_package_share_directory(PACKAGE_NAME), "launch", "navigation.launch.py"
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([navigation_path]),
        launch_arguments={
            "map_subscribe_transient_local": "true",
            "use_sim_time": use_sim_time
        }.items(),
    )
    
    return LaunchDescription(
        [
            use_sim_time_arg,
            map_arg,
            localization,
            navigation
        ]
    )
