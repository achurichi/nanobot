import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node

import xacro

CONTROLER_PACKAGE_NAME = "nanobot_diffdrive"
DESCRIPTION_PACKAGE_NAME = "description"


def generate_launch_description():
    # get URDF via xacro
    desc_pkg_path = os.path.join(get_package_share_directory(DESCRIPTION_PACKAGE_NAME))
    xacro_file = os.path.join(desc_pkg_path, "urdf", "robot.urdf.xacro")
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    # get controler
    controler_pkg_path = os.path.join(
        get_package_share_directory(CONTROLER_PACKAGE_NAME)
    )
    robot_controllers = os.path.join(
        controler_pkg_path, "config", "diffdrive_controllers.yaml"
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/diff_controller/cmd_vel", "/cmd_vel"),
        ],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    return LaunchDescription(
        [
            control_node,
            robot_state_pub_node,
            robot_controller_spawner,
            delay_joint_state_broadcaster_after_robot_controller_spawner,
        ]
    )
