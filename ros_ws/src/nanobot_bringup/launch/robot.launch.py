import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler,
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro

PACKAGE_NAME = "nanobot_bringup"
CONTROLER_PACKAGE_NAME = "nanobot_diffdrive"
DESCRIPTION_PACKAGE_NAME = "nanobot_description"
LIDAR_PACKAGE_NAME = "nanobot_lidar"
CAMERA_PACKAGE_NAME = "nanobot_camera"
IMU_PACKAGE_NAME = "nanobot_imu"
NAVIGATION_PACKAGE_NAME = "nanobot_navigation"
WEB_PACKAGE_NAME = "nanobot_web"


def generate_launch_description():
    # Declare arguments
    generate_map_arg = DeclareLaunchArgument(
        "generate_map",
        default_value="false",
        description="Generate a new map using SLAM Toolbox",
    )
    generate_map = LaunchConfiguration("generate_map")

    gpu_camera_arg = DeclareLaunchArgument(
        "gpu_camera",
        default_value="true",
        description="Whether to use the camera with GPU acceleration",
    )
    gpu_camera = LaunchConfiguration("gpu_camera")

    # get URDF via xacro
    xacro_file = os.path.join(
        os.path.join(get_package_share_directory(DESCRIPTION_PACKAGE_NAME)),
        "urdf",
        "robot.urdf.xacro",
    )
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

    # Lidar
    lidar_launch_path = os.path.join(
        get_package_share_directory(LIDAR_PACKAGE_NAME), "launch", "lidar.launch.py"
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([lidar_launch_path]),
    )

    # Camera
    camera_launch_path = os.path.join(
        get_package_share_directory(CAMERA_PACKAGE_NAME), "launch", "camera.launch.py"
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([camera_launch_path]),
        launch_arguments={
            "gpu_camera": gpu_camera,
        }.items(),
    )

    # IMU
    imu_launch_path = os.path.join(
        get_package_share_directory(IMU_PACKAGE_NAME), "launch", "imu.launch.py"
    )
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([imu_launch_path]),
    )

    # Navigation
    navigation_launch_path = os.path.join(
        get_package_share_directory(NAVIGATION_PACKAGE_NAME),
        "launch",
        "navigation.launch.py",
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([navigation_launch_path]),
        launch_arguments={
            "generate_map": generate_map,
            "map": "/home/nanobot/ros_ws/src/nanobot_navigation/maps/room.yaml",
            "use_sim_time": "false",
        }.items(),
    )

    # Twist Mux
    twist_mux_config_path = os.path.join(
        os.path.join(get_package_share_directory(PACKAGE_NAME)),
        "config",
        "twist_mux.yaml",
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_config_path, {"use_sim_time": False}],
        remappings=[("/cmd_vel_out", "/diff_controller/cmd_vel_unstamped")],
    )

    # Websocket connection
    web_launch_path = os.path.join(
        get_package_share_directory(WEB_PACKAGE_NAME), "launch", "web.launch.py"
    )
    web = IncludeLaunchDescription(PythonLaunchDescriptionSource([web_launch_path]))

    return LaunchDescription(
        [
            generate_map_arg,
            gpu_camera_arg,
            control_node,
            robot_state_pub_node,
            robot_controller_spawner,
            delay_joint_state_broadcaster_after_robot_controller_spawner,
            lidar,
            camera,
            imu,
            navigation,
            twist_mux,
            web,
        ]
    )
