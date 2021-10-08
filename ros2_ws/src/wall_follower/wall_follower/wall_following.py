import sys, time
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

## import msg, srv, action
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rosky2_interfaces.action import OdomRecord 
from rosky2_interfaces.srv import FindWall


class WallFollowing(Node):

    def __init__(self):
        # Here we have the class constructor
        # call the class constructor
        super().__init__('wall_following_node')

        # parameter
        self.follow_wall = False
        self.cbg = ReentrantCallbackGroup()

        # create client
        ## action client
        #self.call_record_odometry_action_server()

        ## server client
        #self.call_find_wall_server()

        ## test
        self.follow_wall = True
        self.get_logger().warn("Test done!")

        # create the publisher object
        self.publisher_ = self.create_publisher(
            Twist, 
            'cmd_vel', 
            10,
            callback_group = self.cbg
            )
            
        # create the subscriber object
        self.subscriber = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.callback_scan, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group = self.cbg
            )
        # prevent unused variable warning
        self.subscriber

        # define the timer period
        self.timer_period = 0.1
        self.now =  time.time()

        # define the variable to save the received info
        self.laser_right = 0.0
        self.laser_front = 0.0
        self.laser_back = 0.0
        self.laser_left = 0.0


        # create a Twist message
        self.cmd = Twist()
        self.linear__speed_limit = 0.2
        self.angular_speed_limit = self.linear__speed_limit / 3
        self.cmd_multiple = 50 / self.angular_speed_limit
        

        # PID controller
        self.cu_error = 0
        self.last_error = 0
        self.standard = 0.25
        self.thres_hold_min = 0.05
        self.thres_hold_max = self.thres_hold_min + 0.02
        self.kp = -3
        self.kd = -5
        self.timer = self.create_timer(self.timer_period, self.motion)
        

    def call_find_wall_server(self):
        client = self.create_client(FindWall, "/find_wall")
        self.follow_wall = False
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waitting for Server /find_wall...")

        self.get_logger().warn("Call Service /find_wall")
        request = FindWall.Request()
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_find_wall))
    
    def callback_call_find_wall(self, future):
        self.get_logger().warn(f"future: {future}")
        try:
            response = future.result()
            if response.wallfound == True:
                self.follow_wall = True
            self.get_logger().info(f"I am ready! Can start to follow the wall")
        except Exception as error:
            self.get_logger().info(f"Service call fialed {error}")

    def call_record_odometry_action_server(self):
        client = ActionClient(self, OdomRecord, "/record_odometry")
        while not client.wait_for_server(1.0):
            self.get_logger().warn("Waitting for Action Server /record_odometry...")

        self.get_logger().warn("Call Action Server /record_odometry")
        goal_msg = OdomRecord.Goal()
        send_goal_future = client.send_goal_async(goal_msg, feedback_callback=self.callback_feedback_record_odometry_action_server)
        send_goal_future.add_done_callback(partial(self.callback_goal_response_record_odometry_action_server))

    def callback_goal_response_record_odometry_action_server(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected...")
            return
        self.get_logger().info("Goal accepted!")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(partial(self.callback_get_result_record_odometry_action_server))

    def callback_get_result_record_odometry_action_server(self, future):
        self.record_odom_lists = future.result().result.list_of_odoms
        self.get_logger().info(f"All odometry: {self.record_odom_lists}")
    

    def callback_feedback_record_odometry_action_server(self, feedback_msg):
        current_total = feedback_msg.feedback.current_total
        self.get_logger().info(f"I went {current_total} m")


    def callback_scan(self, msg):
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
        linear_x  = 0.2
        if self.laser_front > 0.2:
            self.cu_error = self.laser_right - self.standard
            if abs(self.cu_error) > self.thres_hold_min and abs(self.cu_error) < self.thres_hold_max:
                angular_z = (self.cu_error * self.kp) + ((self.cu_error - self.last_error) / (self.timer_period) * self.kd)
            elif abs(self.cu_error) > self.thres_hold_max:
                angular_z = -self.angular_speed_limit if self.cu_error > 0 else self.angular_speed_limit
            else:
                angular_z = 0.0
            angular_z = min(max(angular_z, -self.angular_speed_limit), self.angular_speed_limit)
        else:
            linear_x = 0.0
            angular_z = self.angular_speed_limit 
        self.cmd.linear.x = linear_x * self.cmd_multiple
        self.cmd.angular.z = angular_z * self.cmd_multiple
        self.last_error = self.cu_error
        self.publisher_.publish(self.cmd)

    def on_shutdown(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        self.get_logger().info('Shutdown!')
            
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