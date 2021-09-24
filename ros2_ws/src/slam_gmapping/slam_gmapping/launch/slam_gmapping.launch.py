import os
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


def generate_launch_description():

    # configuration
    ld = LaunchDescription()

    # configure parameter
    ## environment and package name
    package_node = {
        "self": ("slam_gmapping", "slam_gmapping"),
    }

    ## get parameter file path
    param_substitutions = Path(
        get_package_share_directory(package_node["self"][0]),
        'config',
        'gmapping.yaml'
    ) 

    # configure node
    slam_gmapping = Node(
        package='slam_gmapping', 
        executable='slam_gmapping', 
        parameters=[param_substitutions],
        # arguments=['--ros-args','--log-level','debug'],
        output='screen'),
    )

    # add node and launch in launch function
    ld.add_action(slam_gmapping)

    return ld

