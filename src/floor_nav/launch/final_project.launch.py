import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    turtlebot_launch_dir = get_package_share_directory("turtlebot_launch")
    occgrid_planner_dir = get_package_share_directory("occgrid_planner_base")

    slam_tb_sync = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot_launch_dir, "slam_tb_sync.launch.py")
        )
    )
    
    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(occgrid_planner_dir, 'planner.launch.py')
        )
    )

    collision_avoidance = Node(
            package='collision_avoidance_base', executable='collision_avoidance_base', name='collision_avoidance',
            parameters=[
                {'~/safety_diameter': 0.3},
                {'~/ignore_diameter': 1.0},
                {'~/max_velocity': 1.0},
                {'~/only_forward': False},
                ],
            remappings=[
                ('~/clouds', '/points'),
                ('~/scans', '/scan'),
                ('~/vel_input', '/vrep/safeCommand'),
                ('~/vel_output', '/velocity_smoother/input'),
                ],
            output='screen')

    return LaunchDescription([
        slam_tb_sync,
        planner,
        collision_avoidance,
    ])
