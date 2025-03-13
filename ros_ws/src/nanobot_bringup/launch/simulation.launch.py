import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

PACKAGE_NAME = "nanobot_bringup"
SIM_PACKAGE_NAME = "nanobot_simulation"


def generate_launch_description():
    rsp_launch_file = os.path.join(
        get_package_share_directory(PACKAGE_NAME), "launch", "rsp.launch.py"
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rsp_launch_file]),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    world_file = os.path.join(
        get_package_share_directory(SIM_PACKAGE_NAME), "worlds", "boxes.sdf"
    )
    
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "-v", "4", world_file],
        output="screen",
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "nanobot",
            "-z", "0.05"  # Moves the robot 5 cm above the ground
        ],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    return LaunchDescription(
        [
            rsp,
            gazebo,
            spawn_entity,
            diff_drive_spawner,
            joint_broad_spawner,
        ]
    )
