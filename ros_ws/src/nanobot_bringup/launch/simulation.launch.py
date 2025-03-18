import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

PACKAGE_NAME = "nanobot_bringup"
BRINGUP_PACKAGE_NAME = "nanobot_bringup"
SIM_PACKAGE_NAME = "nanobot_simulation"
NAVIGATION_PACKAGE_NAME = "nanobot_navigation"


def generate_launch_description():
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
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
    )
    world_file = os.path.join(
        get_package_share_directory(SIM_PACKAGE_NAME), "worlds", "boxes.sdf"
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_file]),
        launch_arguments={
            'gz_args': ['-r -v4 ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "nanobot",
            "-z", "0.1"
        ],
        output="screen",
    )

    # Diff drive controller
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
    
    # ROS-Gazebo bridge
    bridge_params = os.path.join(
        get_package_share_directory(SIM_PACKAGE_NAME),'config','gz_bridge.yaml'
    )
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )
    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["camera/realsense_camera/color/image_raw"]
    )
    
    # Navigation
    navigation_launch_path = os.path.join(
        get_package_share_directory(NAVIGATION_PACKAGE_NAME), "launch", "load_navigation.launch.py"
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([navigation_launch_path]),
        launch_arguments={
            "map": "/home/nanobot/ros_ws/src/nanobot_navigation/maps/boxes.yaml",
            "use_sim_time": "true",
        }.items(),
    )
    
    # Twist Mux
    twist_mux_config_path = os.path.join(
        os.path.join(get_package_share_directory(BRINGUP_PACKAGE_NAME)), "config", "twist_mux.yaml"
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_config_path, {"use_sim_time": True}],
        remappings=[("/cmd_vel_out", "/diff_controller/cmd_vel_unstamped")]
    )

    return LaunchDescription(
        [
            rsp,
            gazebo,
            spawn_entity,
            diff_drive_spawner,
            joint_broad_spawner,
            ros_gz_bridge,
            ros_gz_image_bridge,
            navigation,
            twist_mux,
        ]
    )
