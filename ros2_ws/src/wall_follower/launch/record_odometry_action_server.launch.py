from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    record_odometry_action_server_node = Node(
        package="part_1",
        executable="record_odometry_action_server",
        output="log",
    )
    ld.add_action(record_odometry_action_server_node)
    return ld