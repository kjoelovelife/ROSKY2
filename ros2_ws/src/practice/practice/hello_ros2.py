#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyNode(Node):

    def __init__(self):
        super().__init__("hello_ros2")
        self.publiser_ = self.create_publisher(
            msg_type=String,
            topic="~/welcome_words",
            qos_profile=10,
        )

        self.timer = self.create_timer(
            timer_period_sec=1.0,
            callback=self.callback_timer,
        )

        self.words = String()

    def callback_timer(self):
        self.words.data = "Hello, ROS2! I'll make a interesting robot!!"
        self.publiser_.publish(self.words)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
