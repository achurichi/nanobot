import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

PACKAGE_NAME = "nanobot_navigation"


def generate_launch_description():
    # Declare arguments
    generate_map_arg = DeclareLaunchArgument(
        "generate_map",
        default_value="false",
        description="Generate a new map using RTAB-Map"
    )
    generate_map = LaunchConfiguration("generate_map")
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    map_arg = DeclareLaunchArgument(
        "map",
        description="Map file"
    )
    map_file = LaunchConfiguration("map")
    
    rtabmap_path = os.path.join(
        get_package_share_directory(PACKAGE_NAME), "launch", "rtabmap.launch.py"
    )
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rtabmap_path]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "generate_map": generate_map,
            "map": map_file
        }.items(),
    )
    
    navigation_path = os.path.join(
        get_package_share_directory(PACKAGE_NAME), "launch", "nav2_navigation.launch.py"
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([navigation_path]),
        condition=UnlessCondition(generate_map),
        launch_arguments={
            "map_subscribe_transient_local": "true",
            "use_sim_time": use_sim_time
        }.items(),
    )
    
    return LaunchDescription(
        [
            generate_map_arg, 
            use_sim_time_arg, 
            map_arg,
            rtabmap,
            navigation
        ]
    )
