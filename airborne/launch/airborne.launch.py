from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=get_package_share_directory("airborne") + "/config/airborne.yaml",
            description="Airborne bridge ROS parameter YAML file",
        ),
        Node(
            package="airborne",
            executable="airborne_bridge",
            name="airborne_bridge",
            parameters=[LaunchConfiguration("params_file")],
            output="screen",
        ),
    ])
