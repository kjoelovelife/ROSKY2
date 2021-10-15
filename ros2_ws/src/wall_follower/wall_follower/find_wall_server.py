import sys, time, threading, math, re
path = "/opt/ros/noetic/lib/python3/dist-packages" 
if path in sys.path:
    sys.path.remove(path)

from multiprocessing import cpu_count

import rclpy

# import the ROS2 python dependencies
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import Executor, MultiThreadedExecutor
from rcl_interfaces.srv import ListParameters, GetParameters

# import the Twist module from geometry_msgs dependencies
from geometry_msgs.msg import Twist

# import the LaserScan module from sensor_msgs dependencies
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rosky2_interfaces.srv import FindWall
from functools import partial

class FindWallServiceServer(Node):

    def __init__(self):
        super().__init__("find_wall_server")

        # parameter
        ## from ros2 parameter server
        self.ros2_parameter = {
        }

        ## local parameter
        self.parameter = {
            "motor_axis_length": 125.0 * math.pow(10, -3),
            "motor_axis_width": 150.0 * math.pow(10, -3),
            "distance_standard": 2.5,
            "cbg": ReentrantCallbackGroup(),
        }

        # create client
        self.service_client_ominibotcar_driver = self.create_client(GetParameters, "~/ominibot_car_driver/get_parameters")
        self.service_client_stop_record = self.create_client(FindWall, "~/stop_record")

        # call client
        self.call_server_get_parameters(client=self.service_client_ominibotcar_driver)

        # create subscription
        self.subscriber = self.create_subscription(
            msg_type=LaserScan, 
            topic="~/scan", 
            callback=self.callback_scan, 
            qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=self.parameter["cbg"]
        )

        # create publisher
        self.publisher_ = self.create_publisher(
            msg_type=Twist, 
            topic="~/cmd_vel", 
            qos_profile=10, 
            callback_group=self.parameter["cbg"]
        )

        # create service-server 
        self.server = self.create_service(
            srv_type=FindWall, 
            srv_name="~/find_wall", 
            callback=self.callback_find_wall_service,
            callback_group=self.parameter["cbg"]
        )

        # define variables
        self.cmd = Twist()
        self.laser_right = 0.0
        self.laser_front = 0.0
        self.laser_back = 0.0
        self.laser_left = 0.0
        self.get_logger().info("Start!")

    def call_server_get_parameters(self, client):
        """call service to get ros2 parameter
        Args:
          client: created client \"~/ominibot_car_driver/get_parameters\" in this node 
        """
        client_name = "ominibot_car_driver/get_parameters"
        for name in self.get_client_names_and_types_by_node(node_name=self.get_name(), node_namespace=self.get_namespace()):
            if re.search(client_name, name[0]):
                client_name = name[0] 
        while not client.wait_for_service(1.0):
            self.get_logger().warn(f"Waitting for Server {client_name} ...")
        
        self.get_logger().warn(f"Call Service {client_name}")
        request = GetParameters.Request()
        request.names = ["motor_axis_length", "motor_axis_width"]
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_ominibot_car_driver_get_parameters))
    
    def callback_call_ominibot_car_driver_get_parameters(self, future):
        """future of ros2 service-client
        Service name: /[namespace]/ominibot_car_driver/get_parameters
        """
        try:
            response = future.result().values
            self.parameter["motor_axis_length"] = response[0].double_value * math.pow(10, -3)
            self.parameter["motor_axis_width"] = response[1].double_value * math.pow(10, -3) 
        except Exception as error:
            self.get_logger().info(f"Service call fialed {error}")

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
        self.laser = [distance for distance in self.laser if distance != 0.0]


    def callback_find_wall_service(self, request, response):
        """Will do these:
        1. Identify which laser ray is the shortest. Assume that is the one pointing to a wall.
        2. Rotate the robot until the front of the robot is facing the wall. This can be done by rotating the robot until ray-front is the smaller one.
        3. Move the robot forward until ray-front is shorter than designated distance
        4. Now, rotate the robot again until ray-right of the laser range is pointing to the wall.
        5. At this point, assume that the robot is situated to start moving along the wall.
        6. Return the service message with a True.
        """
        front_distance_limit = self.parameter["motor_axis_length"] * (self.parameter["distance_standard"] - 0.5)
        while self.laser == False:
            self.get_logger().info(f"Wait for topic \"/scan\"")

        response.wallfound = False

        self.get_logger().info(f"Start findingwall!")
        while not self.laser_front == min(self.laser):
            #self.get_logger().info(f"front: {self.laser_front}, min: {min(self.laser)}")
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.05
            self.publisher_.publish(self.cmd)
        
        self.get_logger().info(f"I find the wall! Stop!")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0  
        self.publisher_.publish(self.cmd)
        
        self.get_logger().info(f"Start nearing the wall!")
        while self.laser_front > front_distance_limit:
            self.cmd.linear.x = 0.025
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)

        self.get_logger().info(f"I have already near the wall! Stop!")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)

        self.get_logger().info(f"Start rotating!")
        while not self.laser_right == min(self.laser):
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.05
            self.publisher_.publish(self.cmd)

        self.get_logger().info(f"I finish the mission! Stop!")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0  
        self.publisher_.publish(self.cmd)
        
        # response
        response.wallfound = True

        # create client
        self.call_stop_record_server(client=self.service_client_stop_record)

        return response

    def call_stop_record_server(self, client):
        """create ros2 service-client 
        Args:
          client: created client \"~/stop_record\" in ths node  
        """
        client_name = "stop_record"
        for name in self.get_client_names_and_types_by_node(node_name=self.get_name(), node_namespace=self.get_namespace()):
            if re.search(client_name, name[0]):
                client_name = name[0]
        while not client.wait_for_service(1.0):
            self.get_logger().warn(f"Waiting for server \"{client_name}\"")

        self.get_logger().warn(f"Call Service {client_name}")
        request = FindWall.Request()
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_stop_record))

    def callback_call_stop_record(self, future):
        try:
            response = future.result()
            while not response:
                pass
        except Exception as error:
            self.get_logger().error(f"Service call fieled {error}")


    def on_shutdown(self):
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)


            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)

    # declare the node constructor
    find_wall = FindWallServiceServer()
        
    # pause the program execution, waits for a request to kill the node (ctrl+c)
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

if __name__ == "__main__":
    main()