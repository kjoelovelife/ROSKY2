import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import ThisLaunchFile, LaunchConfiguration, PythonExpression, TextSubstitution
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.launch_description_source import LaunchDescriptionSource
from launch.launch_context import LaunchContext
from pathlib import Path

PROJECT = "ROSKY2"
PACKAGE = "rosky2_slam"

def generate_launch_description():

    # configure function
    ld = LaunchDescription()

    # configure arguments
    ## environment and package name

    self_name_space = os.environ["VEHICLE_NAME"]
    self_package_path = f"{os.environ['HOME']}/{PROJECT}/ros2_ws/src/{PACKAGE}/map" 

    package_node = {
        "self": "rosky2_slam",
        "nav2_map_server": ("nav2_map_server", "map_saver_cli"),
    }

    now = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    default_name = f"map_{now}"
    ## launch argument

    ### Declare
    map_name = DeclareLaunchArgument(
        name="map_name",
        default_value=default_name,
        description="Which name do you want to use to save the map. Default is map_YYYY_mm_DD_HH_MM_SS"
    )
    
    _map = LaunchConfiguration(variable_name="map_name")

    # configure node
    nav2_map_server = Path(get_package_prefix("nav2_map_server"), "lib", "nav2_map_server", "map_saver_cli")
    nav2_map_server_node = Node(
        package="nav2_map_server",
        executable="map_saver_cli",
        arguments=[
            ["-f"], [f"{self_package_path}/{default_name}"],
        ],
        parameters=[
            {"save_map_timeout": 10000},
        ],
    )
    # add node and launch in launch function
    ld.add_action(map_name)
    ld.add_action(nav2_map_server_node)
    
    return ld
