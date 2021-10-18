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
