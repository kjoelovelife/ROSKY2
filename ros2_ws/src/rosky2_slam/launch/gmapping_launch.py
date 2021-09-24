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

    # configure function
    ld = LaunchDescription()

    # configure arguments
    ## environment and package name
    self_name_space = os.environ["VEHICLE_NAME"]
    package_node = {
        "self": "rosky2_slam",
        "slam_gmapping": ("slam_gmapping", "slam_gmapping"),
        "rviz2": ("rviz2", "rviz2"),
    }

    ## launch argument
    ### local
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    ### Declare
    open_rviz = DeclareLaunchArgument(
        'open_rviz',
        default_value='false',
        description='open rviz'
    )

    # get parameter file path
    rviz_config_dir = Path(
        get_package_share_directory(package_node["self"]),
        'rviz',
        'slam.rviz'
    )

    param_substitutions = Path(
        get_package_share_directory(package_node["self"]),
        "config",
        "gmapping.yaml"
    )

    # remapping topic
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
 
    # configure node
    slam_gmapping = Node(
        package=package_node["slam_gmapping"][0], 
        executable=package_node["slam_gmapping"][1], 
        parameters=[param_substitutions],
        output='screen',
    )

    rviz2 = Node(
        package=package_node["rviz2"][0],
        executable=package_node["rviz2"][1],
        name='rviz2',
        arguments=['-d', str(rviz_config_dir)],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration("open_rviz")),
        remappings=remappings,
    )

    # add node and launch in launch function
    ld.add_action(open_rviz)
    ld.add_action(slam_gmapping)
    ld.add_action(rviz2)

    return ld
