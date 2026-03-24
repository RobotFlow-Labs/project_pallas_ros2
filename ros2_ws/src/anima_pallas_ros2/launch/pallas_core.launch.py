from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_name = LaunchConfiguration("config_name")
    config_path = PathJoinSubstitution(
        [FindPackageShare("anima_pallas_ros2"), "config", config_name]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_name",
                default_value="pallas_core.yaml",
                description="Config file from share/anima_pallas_ros2/config",
            ),
            Node(
                package="anima_pallas_ros2",
                executable="anima_pallas_core_node",
                name="anima_pallas_core_node",
                parameters=[config_path],
                output="screen",
            )
        ]
    )
