import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE_NAME = "nanobot_imu"


def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory(PACKAGE_NAME))

    imu_raw = Node(
        package="nanobot_imu",
        executable="imu",
        namespace="",
        name="nanobot_imu",
        parameters=[os.path.join(pkg_path, "config", "imu.yaml")],
        shell=True,
    )

    # madgwick_filter = Node(
    #     package="imu_filter_madgwick",
    #     executable="imu_filter_madgwick_node",
    #     name="madgwick_filter",
    #     parameters=[os.path.join(pkg_path, "config", "madgwick_filter.yaml")],
    # )

    # ekf_localization = Node(
    #     package="robot_localization",
    #     executable="ekf_node",
    #     name="ekf_filter_node",
    #     output="screen",
    #     parameters=[os.path.join(pkg_path, "config", "ekf.yaml")],
    # )

    return LaunchDescription(
        [
            imu_raw,
            # madgwick_filter,
            # ekf_localization,
        ]
    )
