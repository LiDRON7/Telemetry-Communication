from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory("airborne_telemetry"), "config", "airborne.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=config),
        Node(package="airborne_telemetry", executable="airborne_bridge",
             name="airborne_bridge", output="screen",
             parameters=[LaunchConfiguration("params_file")]),
    ])
