import sys, time, threading
path = "/opt/ros/noetic/lib/python3/dist-packages" 
if path in sys.path:
    sys.path.remove(path)

from multiprocessing import cpu_count

import rclpy
# import the ROS2 python dependencies
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import Executor, MultiThreadedExecutor
# import the Twist module from geometry_msgs dependencies
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs dependencies
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rosky2_interfaces.srv import FindWall
from functools import partial

class FindWallService(Node):

    def __init__(self):
        super().__init__("find_wall_server")

        # parameter
        ## from ros2 parameter server
        self.ros2_parameter = {}

        ## local parameter 
        self.parameter = {
            "cbg": ReentrantCallbackGroup(),
        }

        # create subscription
        self.subscriber = self.create_subscription(
            LaserScan, 
            '~/scan', 
            self.callback_scan, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=self.parameter["cbg"]
            )
        self.publisher_ = self.create_publisher(
            Twist, '~/cmd_vel', 
            10, 
            callback_group=self.parameter["cbg"]
            )

        # create service-server 
        self.server = self.create_service(
            FindWall, 
            '~/find_wall', 
            self.callback_find_wall_service,
            callback_group=self.parameter["cbg"]
            )

        # define the variable
        self.cmd = Twist()
        self.laser_right = 0.0
        self.laser_front = 0.0
        self.laser_back = 0.0
        self.laser_left = 0.0
        self.get_logger().info("Start!")

    def callback_scan(self, msg):
        angle_increment = len(msg.ranges) // 4
        right = angle_increment
        front = right + angle_increment
        left = front + angle_increment
        self.laser = msg.ranges
        self.laser_right = self.laser_right if msg.ranges[right] == 0 else msg.ranges[right]
        self.laser_front = self.laser_front if msg.ranges[front] == 0 else msg.ranges[front]
        self.laser_left = self.laser_left if msg.ranges[left] == 0 else msg.ranges[left]

        # remove 0 in self.laser
        self.laser = [distance for distance in self.laser if distance != 0]


    def callback_find_wall_service(self, request, response):
        """Will do these:
        1. Identify which laser ray is the shortest. Assume that is the one pointing to a wall.
        2. Rotate the robot until the front of the robot is facing the wall. This can be done by rotating the robot until ray 0 is the smaller one.
        3. Move the robot forward until ray 0 is shorter than 0.3m.
        4. Now, rotate the robot again until ray number 270 of the laser range is pointing to the wall.
        5. At this point, assume that the robot is situated to start moving along the wall.
        6. Return the service message with a True.
        """ 
        while self.laser == False:
            self.get_logger().info(f"Wait for topic \"/scan\"")

        response.wallfound = False

        # find wall
        self.get_logger().info(f"Start findingwall!")
        while not self.laser_front == min(self.laser):
            self.get_logger().info(f"front: {self.laser_front}, min: {min(self.laser)}")
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.05
            self.publisher_.publish(self.cmd)
        
        self.get_logger().info(f"I find the wall! Stop!")
        # stop
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0  
        self.publisher_.publish(self.cmd)
        
        self.get_logger().info(f"Start nearing the wall!")
        # near the wall
        while self.laser_front > 0.3:
            #self.get_logger().info(f"status: near the wall, min: {self.laser.index(min(self.laser))}, distance: {min(self.laser)}")
            self.cmd.linear.x = 0.025
            self.publisher_.publish(self.cmd)

        self.get_logger().info(f"I have already near the wall! Stop!")
        # stop
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)

        self.get_logger().info(f"Start rotating!")
        # rotate
        while not self.laser.index(min(self.laser)) in range(self.right - 10, self.right + 10):
            #self.get_logger().info(f"status: rotate, min: {self.laser.index(min(self.laser))}, distance: {min(self.laser)}")
            self.cmd.angular.z = 0.3
            self.publisher_.publish(self.cmd)

        self.get_logger().info(f"I finish the mission! Stop!")
        # stop
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0  
        self.publisher_.publish(self.cmd)
        
        # response
        response.wallfound = True

        # create client
        self.call_stop_record_server()

        return response

    def call_stop_record_server(self):
        client = self.create_client(FindWall, "stop_record")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server \"stop_record\"")
        request = FindWall.Request()
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_stop_record))

    def callback_call_stop_record(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Call server \"stop_record\"")
            while not response:
                pass
        except Exception as error:
            self.get_logger().error(f"Service call fieled {error}")

        
    
    def publish_cmd(self):
        self.publisher_.publish(self.cmd)

    def on_shutdown(self):
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)


            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)

    # declare the node constructor
    find_wall = FindWallService()
        
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    #rclpy.get_default_context().on_shutdown(wall_following.on_shutdown)
    executor = MultiThreadedExecutor(num_threads=cpu_count())
    executor.add_node(find_wall)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    # Explicity destroy the node
    find_wall.on_shutdown()
    # shutdown the ROS communication
    executor.shutdown()
    find_wall.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()