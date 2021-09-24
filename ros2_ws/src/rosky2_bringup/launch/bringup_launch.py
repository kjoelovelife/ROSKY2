#!/bin/usr/python3
# coding=UTF-8
#Copyright (c) 2021 Wei-Chih Lin(weichih.lin@protonmail.com)
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#

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
        "self": ["rosky2_bringup"],
        "ominibot_car": ["ominibot_car", "ominibot_car_driver"],
        "rf2o": ["rf2o_laser_odometry"],
        "lidar": ["ydlidar_ros2_driver", "ydlidar_ros2_driver_node"],
    }

    ## launch argument
    ### Declare
    rf2o_if_or_not = DeclareLaunchArgument(
        "rf2o",
        default_value="false",
        description="Use rf2o_laser_odometry.launch.py or not, defaulr is \"false\""
    )

    lidar_params_file = DeclareLaunchArgument(
        "lidar_params_file",
        default_value = str(Path(
            get_package_share_directory(package_node["self"][0]),
            "config",
            "ydlidar.yaml"
        )),
        description="Use ydlidar.yaml to get ydlidar parameters"
    )

    odom_topic = LaunchConfiguration("odom_topic", default="/odom_rf2o")

    # get parameter file path
    ominibot_car_config = Path(
        get_package_share_directory(package_node["self"][0]),
        "config",
        "ominibotcar.yaml"
    )

    # remapping topic 
    ominibot_car_driver_to_cmd = (f"/{self_name_space}/ominibot_car_driver/cmd_vel", "/cmd_vel")

    # configure node
    ominibot_car_driver_node = Node(
        package=package_node["ominibot_car"][0],
        namespace=self_name_space,
        executable=package_node["ominibot_car"][1],
        parameters=[
            ominibot_car_config,
        ],
        remappings=[
            ominibot_car_driver_to_cmd,
        ],
        output="screen",    
    )

    # configure launch file
    ## ydlidar will configure tf2 frame: odom -> base_footprint
    rf2o_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(package_node["rf2o"][0]), "/launch/rf2o_laser_odometry.launch.py"]),
        condition=IfCondition(LaunchConfiguration("rf2o")),
        launch_arguments={"odom_topic": odom_topic}.items()
    )

    ## ydlidar will configure tf2 frame: baselink -> laser_frame
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(package_node["lidar"][0]), "/launch/ydlidar_launch.py"]),
        launch_arguments={"params_file": LaunchConfiguration("lidar_params_file")}.items(),
    )

    # configure tf2 frame
    ## rosky2_nav need frame: base_footprint, we configure base_footprint -> base_link
    tf2_node = Node(package='tf2_ros',
                executable='static_transform_publisher',
                name='static_tf_base_footprint_to_base_link',
                arguments=['0', '0', '0.09', '0', '0', '0','base_footprint','base_link'],
                )


    # add node and launch in launch function
    ld.add_action(rf2o_if_or_not)
    ld.add_action(lidar_params_file)
    ld.add_action(ominibot_car_driver_node)
    ld.add_action(rf2o_launch)
    ld.add_action(ydlidar_launch)
    ld.add_action(tf2_node)

    return ld