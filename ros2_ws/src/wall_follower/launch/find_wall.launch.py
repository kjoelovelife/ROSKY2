from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    find_wall_node = Node(
        package="part_1",
        executable="find_wall_service",
        output="log",
    )
    ld.add_action(find_wall_node)
    return ld