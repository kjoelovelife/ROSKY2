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

    # remapping 
    ## topic and server
    self_cmd_to_cmd = (f"~/cmd_vel", "/cmd_vel")
    self_scan_to_scan = (f"~/scan", "/scan")
    self_find_wall_to_find_wall = (f"~/find_wall", f"/{self_name_space}/find_wall_server/find_wall")
    self_ominibotcar_driver_parameter_to_ominibotcar_driver_parameter = (f"~/ominibot_car_driver/get_parameters", f"/{self_name_space}/ominibot_car_driver/get_parameters")

    ## action (_action/cancal_goal, _action/get_result, _action/send_goal)
    self_record_odometry_to_record_odometry = (f'~/record_odometry', f'/{self_name_space}/record_odometry_server/record_odometry')


    wall_following_node = Node(
        package=self_package,
        namespace=self_name_space,
        executable=self_node_name,
        output="screen",
        parameters=[pid_controller_config,],
        remappings=[
            self_cmd_to_cmd, self_scan_to_scan, 
            self_find_wall_to_find_wall, 
            self_ominibotcar_driver_parameter_to_ominibotcar_driver_parameter,
            #self_record_odometry_to_record_odometry,
            ],
           
    )
    ld.add_action(wall_following_node)
    return ld