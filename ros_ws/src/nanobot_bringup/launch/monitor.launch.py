import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

SIM_PACKAGE_NAME = "nanobot_simulation"
JOYSTICK_PACKAGE_NAME = "nanobot_joystick"


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    world_file = os.path.join(
        get_package_share_directory(SIM_PACKAGE_NAME), "config", "nanobot.rviz"
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", world_file],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    rqt_image_view = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        arguments=["/camera/color/image_raw/compressed"],
        output="screen",
    )

    joystick_launch_file = os.path.join(
        get_package_share_directory(JOYSTICK_PACKAGE_NAME),
        "launch",
        "joystick.launch.py",
    )
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([joystick_launch_file])
    )

    return LaunchDescription([use_sim_time_arg, rviz, rqt_image_view, joystick])
