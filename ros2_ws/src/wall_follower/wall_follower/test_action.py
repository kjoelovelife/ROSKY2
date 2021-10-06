# import the Empty module from std_servs service interface
import sys, time
path = "/opt/ros/noetic/lib/python3/dist-packages" 
if path in sys.path:
    sys.path.remove(path)

import rclpy
# import the ROS2 python dependencies
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import Executor, MultiThreadedExecutor
# import the Twist module from geometry_msgs dependencies
# import the LaserScan module from sensor_msgs dependencies
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from custom_interface.action import OdomRecord
from custom_interface.srv import FindWall
from nav_msgs.msg import Odometry


class Service(Node):

    def __init__(self):
        # Here we have the class constructor

        # call the class constructor to initialize the node as service_moving
        super().__init__('test_action')
        self.cbg = ReentrantCallbackGroup()
        self.distance = OdomRecord.Feedback()
        # Action
        self.action_server = ActionServer(
            self, 
            OdomRecord, 
           'record_odometry', 
            self.callback_record_odometry,
            callback_group=self.cbg
            )
        self.get_logger().info("Start!")
        
    def callback_record_odometry(self, goal_handle):
        self.get_logger().info("Executing goal...")
        self.action_server_status = True
        current_time = self.action_server_seconds

        while self.action_server_status:
            if current_time == self.action_server_seconds:
                pass
            else:
                self.get_logger().info(f"time: {current_time}, {self.distance.current_total}")
                goal_handle.publish_feedback(self.distance)
                current_time = self.action_server_seconds
                
        goal_handle.succeed()
        test = OdomRecord.Result()
        test.list_of_odoms = self.record_odometry.list_of_odoms
        self.get_logger().info(f" test.list_of_odoms: {test.list_of_odoms}")
        return test


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    moving_service = Service()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(moving_service)
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()