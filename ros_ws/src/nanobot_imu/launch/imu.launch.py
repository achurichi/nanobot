from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    imu_raw_publisher = Node(
        package="nanobot_imu",
        executable="imu",
        namespace="",
        name="imu_raw_publisher",
        # Launch the node with root access (GPIO) in a shell
        prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
        shell=True,
    )
    
    imu_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter",
        parameters=[
            {"fixed_frame": "imu_base_link"},
        ],
    )
    
    return LaunchDescription([
       imu_raw_publisher,
       imu_filter,
    ])
