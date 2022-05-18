#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class TurtleMoveNode(Node):

    def __init__(self):
        super().__init__("turtle_move")

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

        self.target_x = 8.0
        self.target_y = 4.0

        self.pose = Pose()
        self.timer = self.create_timer(timer_period_sec=0.01, callback=self.callback_timer)

    def callback_pose(self, msg):
        self.pose = msg   

    def callback_timer(self):
        if self.pose == None:
            return

        distance_x = self.target_x - self.pose.x
        distance_y = self.target_y - self.pose.y
        distance = math.sqrt((distance_x **2) + (distance_y ** 2))

        msg = Twist()

        kp = 2
        if distance > 0.5:
            msg.linear.x = kp * distance
            goal_theta = math.atan2(distance_y, distance_x)
            diff = goal_theta - self.pose.theta

            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi

            msg.angular.z = 3 * kp * diff
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleMoveNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
