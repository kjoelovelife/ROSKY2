#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class Distance(Node):

    def __init__(self):
        super().__init__("distance")

        self.subscriber_ = self.create_subscription(
            msg_type=LaserScan,
            topic="~/scan",
            callback=self.callback_scan,
            qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.publisher_ = self.create_publisher(
            msg_type=Float32,
            topic="~/right",
            qos_profile=10,
        )

    def callback_scan(self, msg):
        distance = Float32()
        direction = len(msg.ranges) // 4 
        distance.data = msg.ranges[direction]
        self.publisher_.publish(distance)
        
def main(args=None):
    rclpy.init(args=args)
    node = Distance()
    rclpy.spin(node)
    node.shutdown()

if __name__ == "__main__":
    main()
