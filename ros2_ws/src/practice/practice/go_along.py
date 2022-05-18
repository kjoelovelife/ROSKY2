#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from rosky2_interfaces.action import DesireSpeed

class GoAlong(Node):
    def __init__(self):
        super().__init__("go_along")
        self.distance = 0.0
        self.subscriber_ = self.create_subscription(
            msg_type=Float32,
            topic="~/right",
            callback=self.callback_right,
            qos_profile=10,
        )

        self.publisher_ = self.create_publisher(
            msg_type=Twist,
            topic="~/cmd_vel",
            qos_profile=10,
        )

        self.twist = Twist()
        self.action_server = ActionServer(
            node=self,
            action_type=DesireSpeed,
            action_name="~/action_server",
            execute_callback=self.callback_action_server,
        )

        self.feedback_publish = False
        self.timer_ = self.create_timer(timer_period_sec=0.5, callback=self.callback_timer)

    def callback_right(self, msg):
        self.distance = msg.data

    def callback_action_server(self, goal_handle):
        self.get_logger().info("Executing goal!")
        self.goal_handle = goal_handle
        self.feedback_msg = DesireSpeed.Feedback()
        speed = self.goal_handle.request.speed
        default_distance, tolerance = 0.25, 0.1
        interval, start_time = 0, self.get_clock().now().to_msg().sec
        self.feedback_publish = True
        while (interval < 10):
            if self.distance < (default_distance - tolerance):
                self.twist.angular.z = math.radians(10)
            elif self.distance > (default_distance + tolerance):
                self.twist.angular.z = math.radians(-10)
            else:
                self.twist.angular.z = 0.0
            self.twist.linear.x = speed
            self.feedback_msg.feedback = f"Now the distance is {self.distance}"
            self.publisher_.publish(self.twist)
            interval = self.get_clock().now().to_msg().sec - start_time

        self.feedback_publish = False
        goal_handle.succeed()
        result = DesireSpeed.Result()
        result.completed = "Now I will stop moving!"
        return result

    def callback_timer(self):
        if self.feedback_publish == True:
            self.goal_handle.publish_feedback(self.feedback_msg)

    def on_shutdown(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher_.publish(self.twist)
        

def main(args=None):
    rclpy.init(args=args)
    node = GoAlong()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.on_shutdown()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
