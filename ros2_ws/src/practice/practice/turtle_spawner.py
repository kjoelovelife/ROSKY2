#!/usr/bin/env python3
import rclpy, random, math
from rclpy.node import Node
from functools import partial
from turtlesim.srv import Spawn

class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.count = 0
        self.create_timer(timer_period_sec=1.0, callback=self.callback_timer)

    def callback_timer(self):

        turtle_name  = f"turtle_{self.count}"
        turtle_x = random.uniform(0.0, 11.0)
        turtle_y = random.uniform(0.0, 11.0)
        turtle_theta = random.uniform(0.0, 2 * math.pi)
        self.call_spawn_server(
                       turtle_name, 
                       turtle_x, 
                       turtle_y, 
                       turtle_theta
        )
        self.count += 1

    def call_spawn_server(self, name, x, y, theta):
        client = self.create_client(
            srv_type=Spawn, 
            srv_name="~/spawn"
        )
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")
        request = Spawn.Request()
        request.name = name
        request.x = x
        request.y = y
        request.theta = theta
        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_spawn, name=name, x=x, y=y, theta=theta)
        )

    def callback_call_spawn(self, future, name, x, y, theta):
        try:
            response = future.result()
            if response.name != "":
                self.get_logger().info(f"Turtle {response.name} is now alive.")
        except Exception as error:
            self.get_logger().error(f"Service call failed {error}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
