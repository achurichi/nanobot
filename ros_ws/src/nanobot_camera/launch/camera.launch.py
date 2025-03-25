import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

PACKAGE_NAME = "nanobot_camera"


def generate_launch_description():
    # Declare arguments
    gpu_camera_arg = DeclareLaunchArgument(
        "gpu_camera",
        default_value="true",
        description="Whether to use the camera with GPU acceleration",
    )
    gpu_camera = LaunchConfiguration("gpu_camera")

    # Option 1: Run RealSense node directly with no GPU acceleration
    pkg_path = os.path.join(get_package_share_directory(PACKAGE_NAME))
    camera_config_path = os.path.join(pkg_path, "config", "camera.yaml")
    camera = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="",
        name="camera",
        output="screen",
        parameters=[camera_config_path],
        condition=UnlessCondition(gpu_camera),
    )

    # Option 2: Run RealSense node inside a Docker container with GPU acceleration
    docker_camera = Node(
        package="nanobot_camera",
        executable="docker_camera",
        namespace="",
        name="docker_camera",
        output="screen",
        condition=IfCondition(gpu_camera),
    )

    return LaunchDescription(
        [
            gpu_camera_arg,
            camera,
            docker_camera,
        ]
    )
