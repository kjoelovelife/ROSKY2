import os, sys, time, math, re
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
from rcl_interfaces.srv import ListParameters, GetParameters

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
            "pid_integrals_limit": 3.0,
            "pid_kp": -3.0,
            "pid_ki": -0.1,
            "pid_kd": -0.1,
        }
        self.get_ros2_parameter()

        ## local parameter
        self.parameter = {
            "motor_axis_length": 125.0,
            "motor_axis_width": 150.0,
            "cbg": ReentrantCallbackGroup(),
        }


        # define variables
        self.cu_error = 0.0
        self.last_integrals_error = 0.0
        self.last_derivatives_error = 0.0

        self.last_time = self.get_clock().now().to_msg()
        self.laser_right = 0.0
        self.laser_front = 0.0
        self.laser_back = 0.0
        self.laser_left = 0.0
        self.follow_wall = False

        # create a Twist message
        self.cmd = Twist()

        # create client
        ## service client
        self.service_client_find_wall = self.create_client(FindWall, "~/find_wall")
        self.service_client_ominibotcar_driver = self.create_client(GetParameters, "~/ominibot_car_driver/get_parameters")
        

        ## action client
        self.action_client_record_odometry = ActionClient(self, OdomRecord, f"{self.get_namespace()}/record_odometry_action_server/record_odometry")
          
        # call server client
        self.call_server_get_parameters(client=self.service_client_ominibotcar_driver)
        self.call_record_odometry_action_server(client=self.action_client_record_odometry)
        self.call_find_wall_server(client=self.service_client_find_wall)
 
        ## test
        self.follow_wall = False
        if self.follow_wall == True:
            self.get_logger().warn("Test mode!")

        # create the publisher object
        self.publisher_ = self.create_publisher(
            msg_type=Twist, 
            topic="~/cmd_vel",
            qos_profile=10,
            callback_group = self.parameter["cbg"]
        )

        # create the subscriber object
        self.subscriber = self.create_subscription(
            msg_type=LaserScan, 
            topic="~/scan", 
            callback=self.callback_scan, 
            qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group = self.parameter["cbg"]
        )
        # prevent unused variable warning
        self.subscriber

        # define the timer period
        self.timer_period = 0.5

        # creat_timer
        self.timer = self.create_timer(
            timer_period_sec=self.timer_period, 
            callback=self.callback_timer, 
            callback_group=self.parameter["cbg"]
        )

    def get_ros2_parameter(self):
        """from ros2 parameter server to get parameter
        """
        for key in self.ros2_parameter.keys():
            self.declare_parameter(key, self.ros2_parameter[key])
            self.ros2_parameter[key] = self.get_parameter(key).value
            self.get_logger().info(f"Publish ros2 parameter, {key}: {self.ros2_parameter[key]}")

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
        future.add_done_callback(partial(self.callback_call_server_get_parameters))
    
    def callback_call_server_get_parameters(self, future):
        """future of ros2 service-client
        server: default is /[namespace]/ominibot_car_driver/get_parameters
        """
        try:
            response = future.result().values
            self.parameter["motor_axis_length"] = response[0].double_value * math.pow(10, -3)
            self.parameter["motor_axis_width"] = response[1].double_value * math.pow(10, -3) 
        except Exception as error:
            self.get_logger().info(f"Service call fialed {error}")

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
                    
    def call_find_wall_server(self, client):
        """call ros2 server
        Args:
          client: created client \"~/find_wall\" in this node 
        """
        client_name = "find_wall"
        for name in self.get_client_names_and_types_by_node(node_name=self.get_name(), node_namespace=self.get_namespace()):
            if re.search(client_name, name[0]):
                client_name = name[0]
        self.follow_wall = False
        while not client.wait_for_service(1.0):
            self.get_logger().warn(f"Waitting for Server {client_name} ...")

        self.get_logger().warn(f"Call Service {client_name}")
        request = FindWall.Request()
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_find_wall))
    
    def callback_call_find_wall(self, future):
        """future of ros2 service-client
        server: /[namespace]/find_wall  
        """
        self.get_logger().warn(f"future: {future}")
        try:
            response = future.result()
            if response.wallfound == True:
                self.follow_wall = True
            self.get_logger().info(f"I am ready! Can start to follow the wall")
        except Exception as error:
            self.get_logger().info(f"Service call fialed {error}")

    def call_record_odometry_action_server(self, client):
        """create ros2 action-client
        Args:
          client: created action client \"~/record_odometry\" in this node 
        """
        client_name = "record_odometry/_action/send_goal"
        for name in self.get_client_names_and_types_by_node(node_name=self.get_name(), node_namespace=self.get_namespace()):
            if re.search(client_name, name[0]):
                client_name = name[0]
        while not client.wait_for_server(1.0):
            self.get_logger().warn(f"Waitting for Action Server {client_name} ...")

        self.get_logger().warn(f"Call Action Server {client_name}")
        goal_msg = OdomRecord.Goal()
        send_goal_future = client.send_goal_async(goal_msg, feedback_callback=self.callback_feedback_record_odometry_action_server)
        send_goal_future.add_done_callback(partial(self.callback_goal_response_record_odometry_action_server))

    def callback_goal_response_record_odometry_action_server(self, future):
        """future of action-client
        Args:
          server: default is \"~/record_odometry\" 
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
        Args:
          server: default is \"~/record_odometry\" 
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
        # Save the laser scan info
        angle_increment = len(msg.ranges) // 4
        right = angle_increment
        front = right + angle_increment
        left = front + angle_increment
        self.laser_start = msg.ranges[0] 
        self.laser_right = self.laser_right if msg.ranges[right] == 0 else msg.ranges[right]
        self.laser_front = self.laser_front if msg.ranges[front] == 0 else msg.ranges[front]
        self.laser_left = self.laser_left if msg.ranges[left] == 0 else msg.ranges[left]
        # print the data
        if self.follow_wall == True:
            #self.get_logger().info(f"ranges: {len(msg.ranges)}, left: {self.laser_left}, front: {self.laser_front}, right: {self.laser_right}")
            self.motion(method="distance", _time=self.get_clock().now().to_msg())

    def motion(self, _time, method="distance", debug=False):
        """use PID controller to send speed to ominibot_car_driver
        """
        discrete_time = (float(_time.sec) + float(_time.nanosec + math.pow(10, -9))) - (float(self.last_time.sec) + float(self.last_time.nanosec * math.pow(10, -9)))
        front_distance_limit = (self.ros2_parameter["pid_standard"] - 0.5) * self.parameter["motor_axis_length"]
        right_distance_limit = (self.ros2_parameter["pid_standard"] - 0.5) * self.parameter["motor_axis_width"]
        linear_x = 0.05
        integrals_error = 0.0
        derivatives_error = 0.0
        if method == "angle":
            if self.laser_front > front_distance_limit:
                if self.laser_right < right_distance_limit:
                    tangent = self.laser_front / self.laser_right
                    degree = 90.0 - math.degrees(math.atan(tangent))
                    self.get_logger().info(f"time: {_time}, Angle: {degree}")
                    if degree > 25.0:
                        pass
            linear_x  = self.ros2_parameter["linear_speed_limit"]
            angular_z = 0.0
        elif method == "distance":    
            self.cu_error = self.laser_right - right_distance_limit                  
            if self.laser_front > front_distance_limit:
                if abs(self.cu_error) > self.ros2_parameter["pid_thres_hold_min"]:       
                    integrals_error = max(min(self.last_integrals_error + (self.cu_error * discrete_time), self.ros2_parameter["pid_integrals_limit"]), -1 * self.ros2_parameter["pid_integrals_limit"])
                    derivatives_error = (self.last_derivatives_error - self.cu_error) / discrete_time
                    angular_z = (self.ros2_parameter["pid_kp"] * self.cu_error) + (self.ros2_parameter["pid_ki"] * integrals_error) + (self.ros2_parameter["pid_kd"] * derivatives_error)
                else:
                    angular_z = 0.0
            else:
                linear_x = 0.0
                angular_z = self.ros2_parameter["angular_speed_limit"]
        else:
            linear_x = 0.0
            angular_z = 0.0 

        self.cmd.linear.x = linear_x
        self.cmd.angular.z = angular_z
        self.last_integrals_error = integrals_error
        self.last_derivatives_error = derivatives_error
        self.last_time = _time
        self.publisher_.publish(self.cmd)
        if debug == True:
            self.get_logger().warn(f"front_distance_limit : {front_distance_limit}, right_distance_limit: {right_distance_limit}")
            self.get_logger().info(f"linear_x: {linear_x}, angular_z: {angular_z}")


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