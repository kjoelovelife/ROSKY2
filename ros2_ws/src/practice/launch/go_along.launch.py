#!/bin/usr/python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    remapping_scan = ("~/scan", "/scan")
    distance = Node(
        package="practice",
        executable="distance",
        remappings=[remapping_scan],
    )

    practice_yaml = os.path.expanduser("~/ROSKY2/ros2_ws/src/practice/config/practice.yaml") 
    remapping_cmd_vel = ("~/cmd_vel", "/cmd_vel")
    remapping_distance_right = ("~/right", "/distance/right")
    go_along_action_server = Node(
        package="practice",
        executable="go_along",
        remappings=[remapping_cmd_vel, remapping_distance_right],
        parameters=[practice_yaml],
    )

    ld.add_action(distance)
    ld.add_action(go_along_action_server)

    return ld