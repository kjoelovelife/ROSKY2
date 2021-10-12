import os, sys, time
path = "/opt/ros/noetic/lib/python3/dist-packages" 
if path in sys.path:
    sys.path.remove(path)

from multiprocessing import cpu_count

# import the ROS2 python dependencies
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import ReliabilityPolicy, QoSProfile
from functools import partial
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import Executor, MultiThreadedExecutor
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

## import msg, srv, action
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rosky2_interfaces.action import OdomRecord 
from rosky2_interfaces.srv import FindWall


class WallFollowing(Node):

    def __init__(self):
        # Here we have the class constructor
        # call the class constructor
        super().__init__("wall_following")

        # parameter
        ## from ros2 parameter server
        self.ros2_parameter = {
            "linear_speed_limit": 0.05,
            "angular_speed_limit": 0.5,
            "pid_standard": 2.0,
            "pid_thres_hold_min": 0.05,
            "pid_thres_hold_max": 0.07,
            "pid_kp": -3.0,
            "pid_kd": -5.0,
        }
        self.get_ros2_parameter()

        ## local parameter
        self.cu_error = 0
        self.last_error = 0
        self.follow_wall = False
        self.cbg = ReentrantCallbackGroup()

        # create client
        ## action client
        #self.call_record_odometry_action_server()

        ## server client
        #self.call_find_wall_server()

        ## test
        self.follow_wall = True
        self.get_logger().warn("Test mode!")

        # create the publisher object
        self.publisher_ = self.create_publisher(
            Twist, 
            "~/cmd_vel", 
            10,
            callback_group = self.cbg
            )
            
        # create the subscriber object
        self.subscriber = self.create_subscription(
            LaserScan, 
            "~/scan", 
            self.callback_scan, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group = self.cbg
            )
        # prevent unused variable warning
        self.subscriber

        # define the timer period
        self.timer_period = 0.5
        self.now =  time.time()

        # define the variable to save the received info
        self.laser_right = 0.0
        self.laser_front = 0.0
        self.laser_back = 0.0
        self.laser_left = 0.0

        # create a Twist message
        self.cmd = Twist()

        # creat_timer
        self.timer = self.create_timer(self.timer_period, self.callback_timer, callback_group=self.cbg)

    def get_ros2_parameter(self):
        """from ros2 parameter server to get parameter
        """
        for key in self.ros2_parameter.keys():
            self.declare_parameter(key, self.ros2_parameter[key])
            self.ros2_parameter[key] = self.get_parameter(key).value
            self.get_logger().info(f"Publish ros2 parameter, {key}: {self.ros2_parameter[key]}")
        #self.ros2_parameter["motor_axis_length"] = self.get_parameter(f"/{self.get_namespace}/ominibot_car_driver motor_axis_length").value
        #self.ros2_parameter["motor_axis_width"] =self.get_parameter(f"/{self.get_namespace}/ominibot_car_driver motor_axis_width").value


    def callback_timer(self):
        """update ros2_parameter every timer_period(default: 0.5)
        """
        for key in self.ros2_parameter.keys():
            temp = self.get_parameter(key).value
            if temp == self.ros2_parameter[key]:
                pass
            else:
                self.get_logger().info(f"ros2 parameter {key} change! From {self.ros2_parameter[key]} to {temp}")
                self.ros2_parameter[key] = temp
                    
    def call_find_wall_server(self):
        """create ros2 service-client 
        Service name: /find_wall 
        """
        client = self.create_client(FindWall, "/find_wall")
        self.follow_wall = False
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waitting for Server /find_wall...")

        self.get_logger().warn("Call Service /find_wall")
        request = FindWall.Request()
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_find_wall))
    
    def callback_call_find_wall(self, future):
        """future of ros2 service-client
        Service name: /find_wall
        """
        self.get_logger().warn(f"future: {future}")
        try:
            response = future.result()
            if response.wallfound == True:
                self.follow_wall = True
            self.get_logger().info(f"I am ready! Can start to follow the wall")
        except Exception as error:
            self.get_logger().info(f"Service call fialed {error}")

    def call_record_odometry_action_server(self):
        """create ros2 action-client
        Action-Server name: /record_odometry 
        """
        client = ActionClient(self, OdomRecord, "/record_odometry")
        while not client.wait_for_server(1.0):
            self.get_logger().warn("Waitting for Action Server /record_odometry...")

        self.get_logger().warn("Call Action Server /record_odometry")
        goal_msg = OdomRecord.Goal()
        send_goal_future = client.send_goal_async(goal_msg, feedback_callback=self.callback_feedback_record_odometry_action_server)
        send_goal_future.add_done_callback(partial(self.callback_goal_response_record_odometry_action_server))

    def callback_goal_response_record_odometry_action_server(self, future):
        """future of action-client
        Action-Server name: /record_odometry
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected...")
            return
        self.get_logger().info("Goal accepted!")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(partial(self.callback_get_result_record_odometry_action_server))

    def callback_get_result_record_odometry_action_server(self, future):
        """result of action-client
        Action-Server name: /record_odometry
        """
        self.record_odom_lists = future.result().result.list_of_odoms
        self.get_logger().info(f"All odometry: {self.record_odom_lists}")
    

    def callback_feedback_record_odometry_action_server(self, feedback_msg):
        """feedback of action-client
        Action-Server name: /record_odometry
        """
        current_total = feedback_msg.feedback.current_total
        self.get_logger().info(f"I went {current_total} m")


    def callback_scan(self, msg):
        """get message from ros2 topic
        Topic name: /scan
        """
        self.last_time = self.now
        self.now = time.time()
        # Save the laser scan info
        angle_increment = len(msg.ranges) // 4
        right = angle_increment
        front = right + angle_increment
        left = front + angle_increment
        self.laser_start = msg.ranges[0] 
        self.laser_right = msg.ranges[right] 
        self.laser_front = msg.ranges[front]
        self.laser_left = msg.ranges[left]
        # print the data
        if self.follow_wall == True:
            self.get_logger().info(f"ranges: {len(msg.ranges)}, front: {self.laser_front}, right: {self.laser_right}")
            self.motion()

    def motion(self):
        """use PD controller to send speed to ominibot_car_driver
        """
        linear_x  = self.ros2_parameter["linear_speed_limit"]
        if self.laser_front > self.ros2_parameter["pid_standard"]:
            self.cu_error = self.laser_right - self.ros2_parameter["pid_standard"]
            if abs(self.cu_error) > self.ros2_parameter["pid_thres_hold_min"] and abs(self.cu_error) < self.ros2_parameter["pid_thres_hold_max"]:
                angular_z = (self.cu_error * self.ros2_parameter["pid_kp"]) + ((self.cu_error - self.last_error) / (self.timer_period) * self.ros2_parameter["pid_kd"])
            elif abs(self.cu_error) > self.ros2_parameter["pid_thres_hold_max"]:
                angular_z = -self.ros2_parameter["angular_speed_limit"] if self.cu_error > 0 else self.ros2_parameter["angular_speed_limit"]
            else:
                angular_z = 0.0
            angular_z = min(max(angular_z, -self.ros2_parameter["angular_speed_limit"]), self.ros2_parameter["angular_speed_limit"])
        else:
            linear_x = 0.0
            angular_z = self.ros2_parameter["angular_speed_limit"] 

        self.cmd.linear.x = linear_x
        self.cmd.angular.z = angular_z
        self.last_error = self.cu_error
        self.publisher_.publish(self.cmd)

    def on_shutdown(self):
        """stop car before shutdown
        """
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        self.get_logger().info("Shutdown!")
            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    wall_following = WallFollowing()
        
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    #rclpy.get_default_context().on_shutdown(wall_following.on_shutdown)
    executor = MultiThreadedExecutor(num_threads=cpu_count())
    executor.add_node(wall_following)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    # Explicity destroy the node
   
    # shutdown the ROS communication
    wall_following.on_shutdown()
    executor.shutdown()
    wall_following.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()