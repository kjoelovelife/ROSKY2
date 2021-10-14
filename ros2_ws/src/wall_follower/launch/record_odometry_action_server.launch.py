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
    self_node_name = "record_odometry_action_server"

    # get parameter file path
    example = Path(
        get_package_share_directory(self_package),
        "pid_controller.yaml"
    )
    
    # remapping topic 
    self_odom_to_odom_rf2o = (f"/{self_name_space}/{self_node_name}/odom", "/odom_rf2o")

    record_odometry_action_server_node = Node(
        package=self_package,
        executable="record_odometry_action_server",
        namespace=self_name_space,
        #parameters=[],
        remappings=[self_odom_to_odom_rf2o],
        output="log",
    )
    ld.add_action(record_odometry_action_server_node)
    return ld