from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("anima_pallas_ros2"),
        "config",
        "pallas.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="anima_pallas_ros2",
                executable="anima_pallas_node",
                name="anima_pallas_node",
                parameters=[config_path],
                output="screen",
            )
        ]
    )
