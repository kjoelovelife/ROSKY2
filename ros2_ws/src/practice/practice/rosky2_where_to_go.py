#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor

class Rosky2WhereToGo(Node):

    def __init__(self):
        super().__init__("rosky2_where_to_go")
        my_parameter_descriptor = ParameterDescriptor(description="How fast the ROSKY 2 GO!")
        self.declare_parameter(name="speed", value=0.05, descriptor=my_parameter_descriptor)
        self.get_logger().info("Start!")

        self.publisher_ = self.create_publisher(msg_type=Twist, topic="~/cmd_vel", qos_profile=10)

        self.twist = Twist()
        self.create_timer(timer_period_sec=3.0, callback=self.callback_timer)

    def callback_timer(self):
        speed = self.get_parameter('speed').get_parameter_value().double_value
        self.twist.linear.x = speed
        self.publisher_.publish(self.twist)

    def shutdown(self):
        self.twist.linear.x = 0.0
        self.publisher_.publish(self.twist)
        rclpy.shutdown()
    

def main(args=None):
    rclpy.init(args=args)
    node = Rosky2WhereToGo()
    rclpy.spin(node)
    node.shutdown()

if __name__ == "__main__":
    main()
