from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    wall_following_node = Node(
        package="part_1",
        executable="wall_following",
        output="screen",
    )
    ld.add_action(wall_following_node)
    return ld