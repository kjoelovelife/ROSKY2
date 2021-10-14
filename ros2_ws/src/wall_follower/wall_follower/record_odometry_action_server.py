import sys, time, threading, math
path = "/opt/ros/noetic/lib/python3/dist-packages" 
if path in sys.path:
    sys.path.remove(path)

from multiprocessing import cpu_count

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
from rosky2_interfaces.action import OdomRecord
from rosky2_interfaces.srv import FindWall
from nav_msgs.msg import Odometry

class RecordOdometryServer(Node):

    def __init__(self):
        super().__init__("record_odometry_action_server")

        # parameter
        ## parameter from ROS2 parameter server
        self.ros2_parameter = {}


        ## local parameter
        self.parameter = {
            "cbg": ReentrantCallbackGroup(),
        }

        # define variables
        self.position = [0.0, 0.0, 0.0]
        self.record_odometry = OdomRecord.Result()
        self.action_server_status = False
        self.action_server_seconds = 0.0
        self.distance = OdomRecord.Feedback()

        # Action
        self.action_server = ActionServer(
            self, 
            action_type=OdomRecord, 
            action_name="~/record_odometry", 
            execute_callback=self.callback_record_odometry,
            callback_group=self.parameter["cbg"]
        )
 
        # Subscriber
        self.subscriber = self.create_subscription(
            msg_type=Odometry, 
            topic="~/odom", 
            callback=self.callback_odom, 
            qos_profile=10,
            callback_group=self.parameter["cbg"]
        )

        # Server
        self.server = self.create_service(
            srv_type=FindWall, 
            srv_name="~/stop_record", 
            callback=self.callback_stop_record_service,
            callback_group=self.parameter["cbg"]
        )

        # timer
        self.timer = self.create_timer(timer_period_sec=1.0, callback=self.callback_timer, callback_group=self.parameter["cbg"])

        # log
        self.get_logger().info("Start!")

    def callback_record_odometry(self, goal_handle):
        self.get_logger().info("Executing goal...")
        self.action_server_status = True
        current_time = self.action_server_seconds
        
        self.get_logger().info(f"Start recording odometry!")
        while self.action_server_status:
            if current_time == self.action_server_seconds:
                pass
            else:
                #self.get_logger().info(f"time: {current_time}, {self.distance}")
                goal_handle.publish_feedback(self.distance)
                current_time = self.action_server_seconds
                
        goal_handle.succeed()
        return self.record_odometry
     

    def calculate_distance(self):
        if self.action_server_seconds <= 0.0:
            pass
        else:
            seconds = int(self.action_server_seconds)
            previous_x = self.record_odometry.list_of_odoms[seconds - 1].x
            previous_y = self.record_odometry.list_of_odoms[seconds - 1].y
            current_x = self.record_odometry.list_of_odoms[seconds].x
            current_y = self.record_odometry.list_of_odoms[seconds].y
            #self.get_logger().info(f"[{previous_x}, {previous_y}], [{current_x}, {current_y}]")
            distance = math.pow((math.pow(current_x - previous_x, 2) + math.pow(current_y - previous_y, 2)), 0.5)
            distance = distance if distance > 0.0001 else 0.0
            self.distance.current_total += distance
            #self.get_logger().info(f"{self.distance}")

    def callback_odom(self, msg):
        self.position = msg.pose.pose.position
            
    def callback_timer(self):
        if self.action_server_status:
            self.record_odometry.list_of_odoms.append(self.position)
            #self.get_logger().info(f"Timer! self.record_odometry: {self.record_odometry}")
            self.calculate_distance()
            self.action_server_seconds += 1.0

    def callback_stop_record_service(self, request, response):
        self.action_server_status = False
        response.wallfound = True
        return response
            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)

    # declare the node constructor
    record_odometer_server = RecordOdometryServer()
        
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    #rclpy.get_default_context().on_shutdown(wall_following.on_shutdown)
    executor = MultiThreadedExecutor(num_threads=cpu_count())
    executor.add_node(record_odometer_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    # Explicity destroy the node

    # shutdown the ROS communication
    executor.shutdown()
    record_odometer_server.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()