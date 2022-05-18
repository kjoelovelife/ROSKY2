#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServerNode(Node):

    def __init__(self):
        super().__init__("add_two_ints_server")
        self.server = self.create_service(
            srv_type=AddTwoInts, 
            srv_name="~/add_two_ints", 
            callback=self.callback_add_two_ints
        )

    def callback_add_two_ints(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f"{request.a} + {request.b} = {response.sum}")
        return response # need return response else will go wrong


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

# Don't forget to add node in setup.py

if __name__ == "__main__":
    main()