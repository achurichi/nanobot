import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

PACKAGE_NAME = "nanobot_navigation"


def generate_launch_description():
    # Declare arguments
    generate_map_arg = DeclareLaunchArgument(
        "generate_map",
        default_value="false",
        description="Generate a new map using SLAM Toolbox"
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
        default_value="/home/nanobot/ros_ws/src/nanobot_navigation/maps/room.yaml",
        description="Map file"
    )
    map = LaunchConfiguration("map")
    
    slam_params_file_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value="/home/nanobot/ros_ws/src/nanobot_navigation/config/mapper_params_online_async.yaml",
        description="SLAM parameters file"
    )
    slam_params_file = LaunchConfiguration("slam_params_file")
    
    # Option 1: Launch SLAM Toolbox online async mapping if generate_map is true
    online_async_path = os.path.join(
        get_package_share_directory(PACKAGE_NAME), "launch", "slam_toolbox_online_async.launch.py"
    )
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([online_async_path]),
        condition=IfCondition(generate_map),
        launch_arguments={
            "slam_params_file": slam_params_file,
            "use_sim_time": use_sim_time
        }.items(),
    )
    
    # Option 2: Launch Nav2 navigation stack if generate_map is false
    localization_path = os.path.join(
        get_package_share_directory(PACKAGE_NAME), "launch", "nav2_localization.launch.py"
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([localization_path]),
        condition=UnlessCondition(generate_map),
        launch_arguments={
            "map": map,
            "use_sim_time": use_sim_time
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
            slam_params_file_arg,
            mapping,
            localization,
            navigation
        ]
    )
