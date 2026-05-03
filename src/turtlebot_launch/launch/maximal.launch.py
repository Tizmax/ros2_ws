import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    turtlebot_launch_dir = get_package_share_directory("turtlebot_launch")
    wpa_cli_dir = get_package_share_directory("wpa_cli")

    minimal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot_launch_dir, "minimal.launch.py")
        )
    )
    rplidar_a1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot_launch_dir, "rplidar_a1.launch.py")
        )
    )
    kinect_min = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot_launch_dir, "kinect_min.launch.py")
        )
    )
    autodock = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot_launch_dir, "autodock.launch.py")
        )
    )
    wpa_cli = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wpa_cli_dir, "wpa_cli.launch.py")
        )
    )


    return LaunchDescription([
        minimal,
        rplidar_a1,
        # kinect_min,
        autodock,
        wpa_cli,
    ])
