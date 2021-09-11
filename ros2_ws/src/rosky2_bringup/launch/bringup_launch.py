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
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # configure arguments
    self_package = 'rosky2_bringup'
    self_name_space = os.environ['VEHICLE_NAME']

    # configure function
    ld = LaunchDescription()

    # get parameter file
    ominibot_car_driver_config = Path(
        get_package_share_directory(self_package),
        'config',
        'ominibot_car_driver.yaml'
    )

    # configure remapping topic 
    ominibot_car_driver_to_cmd = ('/ominibot_car_driver/cmd_vel', '/cmd_vel')

    # configure node
    ominibot_car_driver_node = Node(
        package="ominibot_car",
        namespace=self_name_space,
        executable="ominibot_car_driver",
        parameters=[
            ominibot_car_driver_config,
        ],
        remappings=[
            ominibot_car_driver_to_cmd,
        ],
        output="screen",
        

    )



    # add node in launch function
    ld.add_action(ominibot_car_driver_node)
    return ld