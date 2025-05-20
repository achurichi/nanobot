import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

PACKAGE_NAME = "nanobot_bringup"
BRINGUP_PACKAGE_NAME = "nanobot_bringup"
SIM_PACKAGE_NAME = "nanobot_simulation"
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

    # Robot state publisher
    rsp_launch_file = os.path.join(
        get_package_share_directory(PACKAGE_NAME), "launch", "rsp.launch.py"
    )
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rsp_launch_file]),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # Gazebo
    gazebo_launch_file = os.path.join(
        get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
    )
    world_file = os.path.join(
        get_package_share_directory(SIM_PACKAGE_NAME), "worlds", "boxes.sdf"
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_file]),
        launch_arguments={
            "gz_args": ["-r -v4 ", world_file],
            "on_exit_shutdown": "true",
        }.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "nanobot", "-z", "0.1"],
        output="screen",
    )

    # Diff drive controller
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller"],
    )

    # Joint broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # ROS-Gazebo bridge
    bridge_params = os.path.join(
        get_package_share_directory(SIM_PACKAGE_NAME), "config", "gz_bridge.yaml"
    )
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
    )
    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "camera/color/image_raw",
            "camera/depth/image_rect_raw",
        ],
    )
    # Workaround for old gz-sensors version to work in rviz https://github.com/gazebosim/gz-sensors/issues/239
    depth_cam_link_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="depth_cam_link_tf",
        output="log",
        arguments=[
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "camera_link",
            "nanobot/base_link/depth_camera",
        ],
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
            "map": "/home/nanobot/ros_ws/src/nanobot_navigation/maps/boxes.yaml",
            "use_sim_time": "true",
        }.items(),
    )

    # Twist Mux
    twist_mux_config_path = os.path.join(
        os.path.join(get_package_share_directory(BRINGUP_PACKAGE_NAME)),
        "config",
        "twist_mux.yaml",
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_config_path, {"use_sim_time": True}],
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
            rsp,
            gazebo,
            spawn_entity,
            diff_drive_spawner,
            joint_broad_spawner,
            ros_gz_bridge,
            ros_gz_image_bridge,
            depth_cam_link_tf,
            navigation,
            twist_mux,
            web,
        ]
    )
