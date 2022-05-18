#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from test_interfaces.msg import Turtle
from geometry_msgs.msg import Twist

class TurtlePoseNode(Node):

    def __init__(self):
        super().__init__("turtle")

        self.publisher_ = self.create_publisher(
            msg_type=Twist,
            topic="~/cmd_vel",
            qos_profile=10,
            )

        self.subscriber_ = self.create_subscription(
            msg_type=Pose,
            topic="~/pose",
            callback=self.callback_pose,
            qos_profile=10,
            )

        self.status = Turtle()
        self.twist = Twist()
        self.timer = self.create_timer(timer_period_sec=1.0, callback=self.callback_timer)

    def callback_pose(self, msg):
        self.status = msg   

    def callback_timer(self):
        self.twist.linear.y = 1.0
        self.publisher_.publish(self.twist)
        information = f"""
        X: {self.status.x},
        Y: {self.status.y},
        theta: {self.status.theta},
        linear_velocity: {self.status.linear_velocity},
        angular_velocity: {self.status.angular_velocity}
        """
        self.get_logger().info(information)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePoseNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
