#!/bin/usr/python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    remapping_cmd_vel = ("~/cmd_vel", "/cmd_vel") 
    action_server_example = Node(
        package="wall_follower",
        executable="action_server_example",
        remappings=[remapping_cmd_vel],
        parameters=[],
    )

    practice_yaml = os.path.expanduser("~/ROSKY2/ros2_ws/src/practice/config/practice.yaml") 
    action_client_example = Node(
        package="wall_follower",
        executable="action_client_example",
        remappings=[],
        parameters=[practice_yaml],
    )

    ld.add_action(action_server_example)
    ld.add_action(action_client_example)

    return ld