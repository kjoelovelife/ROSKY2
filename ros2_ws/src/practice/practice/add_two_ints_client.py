#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial # reference: https://www.geeksforgeeks.org/partial-functions-python/
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClientNode(Node):

    def __init__(self):
        super().__init__("add_two_ints_client")
        self.call_add_two_ints_server(6, 7)

    def call_add_two_ints_server(self, server_a, server_b):
        client = self.create_client(
            srv_type=AddTwoInts, 
            srv_name="~/add_two_ints"
        )
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Add Two Ints...")

        request = AddTwoInts.Request()
        request.a = server_a
        request.b = server_b
    
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_add_two_ints, a=server_a, b=server_b))

    def callback_call_add_two_ints(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(f"{a} + {b} = {response.sum}")
        except Exception as error:
            self.get_logger().error(f"Service call fialed {error}")

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClientNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()