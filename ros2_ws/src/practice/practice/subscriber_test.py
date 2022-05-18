#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__("subscriber_test")
        self.subscriber_ = self.create_subscription(
            msg_type=String,
            topic="~/welcome_words",
            callback=self.callback_welcom_words,
            qos_profile=10,
        )

    def callback_welcom_words(self, msg):
        words = msg.data
        self.get_logger().info(f"I heard {words}!")


def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
