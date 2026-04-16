import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = "nanobot_navigation"


def launch_setup(context, *args, **kwargs):
    # Get the path to your YAML file
    # Replace 'nanobot_navigation' with your actual package name
    config_path = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        'config',
        'rtabmap_params.yaml'
    )

    generate_map_str = context.perform_substitution(LaunchConfiguration('generate_map')).lower()
    is_mapping = (generate_map_str == 'true')

    node_arguments = ['-d'] if is_mapping else []

    # Dynamic overrides based on CLI arguments
    parameter_overrides = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'database_path': LaunchConfiguration('map'),
        'RGBD/LocalizationMode': 'false' if is_mapping else 'true',
        'Mem/IncrementalMemory': 'true' if is_mapping else 'false',
    }

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[config_path, parameter_overrides], # Load the yaml first, then overrides
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('odom', '/diff_controller/odom'),     
            ('grid_map', '/map')             
        ],
        arguments=node_arguments
    )

    return [rtabmap_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('generate_map', default_value='true'),
        DeclareLaunchArgument('map'),
        OpaqueFunction(function=launch_setup)
    ])