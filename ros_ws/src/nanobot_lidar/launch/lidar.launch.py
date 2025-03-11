import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

PACKAGE_NAME = "nanobot_lidar"

CONTAINER_NAME = "lidar_container"
NODE_NAME = "lidar"


def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory(PACKAGE_NAME))
    lidar_config_path = os.path.join(pkg_path, "config", "lidar.yaml")
    
    # ROS 2 Component Container
    lidar_container = ComposableNodeContainer(
        name=CONTAINER_NAME,
        namespace="",
        package="rclcpp_components",
        executable="component_container_isolated",
        composable_node_descriptions=[],
        output="screen",
    )
    
    # Lidar Component
    lidar_component = ComposableNode(
        package="ldlidar_component",
        namespace="",
        plugin="ldlidar::LdLidarComponent",
        name=NODE_NAME,
        parameters=[lidar_config_path],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    
    # Load Lidar Component into Container
    load_composable_node = LoadComposableNodes(
        target_container=f"/{CONTAINER_NAME}",
        composable_node_descriptions=[lidar_component],
    )
    
    # Lifecycle Transitions
    wait_for_node = ExecuteProcess(
        name="wait_for_node",
        cmd=[f"until ros2 lifecycle get /{NODE_NAME} | grep -q 'unconfigured'; do sleep 1; done"],
        shell=True,
        output="screen"
    )
        
    configure_node = ExecuteProcess(
        cmd=[f"ros2 lifecycle set /{NODE_NAME} configure"],
        shell=True,
        output="screen"
    )

    activate_node = ExecuteProcess(
        cmd=[f"ros2 lifecycle set /{NODE_NAME} activate"],
        shell=True,
        output="screen"
    )
    
    delayed_configuration = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_for_node,
            on_exit=[configure_node],
        )
    )

    delayed_activation = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=configure_node,
            on_exit=[activate_node],
        )
    )

    return LaunchDescription(
        [
            lidar_container,
            load_composable_node,
            wait_for_node,
            delayed_configuration,
            delayed_activation
        ]
    )
