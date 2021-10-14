from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import ThisLaunchFile, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.launch_description_source import LaunchDescriptionSource
from pathlib import Path



def generate_launch_description():
    ld = LaunchDescription()
    self_package = "wall_follower"
    wall_following_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory(f"{self_package}"), "/wall_following.launch.py"
        ]),
    )
    find_wall_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory(f"{self_package}"), "/find_wall.launch.py"
        ]),
    )

    record_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory(f"{self_package}"), "/record_odometry_action_server.launch.py"
        ]),
    )

    ld.add_action(wall_following_launch)
    ld.add_action(find_wall_launch)
    ld.add_action(record_odometry_launch)
    return ld
