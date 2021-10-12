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

from pathlib import Path

def generate_launch_description():
    ld = LaunchDescription()

    # get environment variable
    self_name_space = os.environ["VEHICLE_NAME"]
    self_package = "wall_follower"
    self_node_name = "wall_following"

    # get parameter file path
    pid_controller_config = Path(
        get_package_share_directory(self_package),
        "pid_controller.yaml"
    )

    # remapping topic 
    pid_controller_to_cmd = (f"/{self_name_space}/{self_node_name}/cmd_vel", "/cmd_vel")
    self_scan_to_scan = (f"/{self_name_space}/{self_node_name}/scan", "/scan")

    wall_following_node = Node(
        package=self_package,
        namespace=self_name_space,
        executable=self_node_name,
        output="screen",
        parameters=[pid_controller_config,],
        remappings=[pid_controller_to_cmd, self_scan_to_scan],
           
    )
    ld.add_action(wall_following_node)
    return ld