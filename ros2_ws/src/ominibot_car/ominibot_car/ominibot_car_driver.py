#!/bin/usr/python3
# coding=UTF-8
#Copyright (c) 2021 Wei-Chih Lin(weichih.lin@protonmail.com)
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#

# Import Libraries

from numpy.core.numeric import normalize_axis_tuple
import rclpy, threading, math, time, re
import numpy as np
from rclpy import parameter
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from tf2_ros import TransformBroadcaster
from .ominibot_car_com import OminibotCar
from .euler_from_quaternion import euler_from_quaternion
from .quaternion_from_euler import quaternion_from_euler

## tf2 library
from tf2_ros import TransformBroadcaster

# import msgs
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray 



class OminibotCarDriverNode(Node):
    """Get velocity and plan mobile path
    """
    def __init__(self):
        # Node name
        super().__init__("ominibot_car_driver")
        self.node_name = self.get_name()

        # declare parameter
        ## ominibot car
        self.parameter = {
            "port": "/dev/ominibot_car",
            "baud": 115200,
            "motor_encoder_count": 1170.0,
            "motor_output_rpm": 185.0,
            "motor_ratio": 90.0,
            "motor_axis_length": 125.0,
            "motor_axis_width": 150.0,
            "wheel_diameter": 75.0,
            "wheel_axis_length": 15.0,
            "wheel_width": 30.0,
            "odom_frequency": 20,
            "timeout": None,
            "platform": "mecanum",
            "pub_freq": 10,
            "odom_id": "odom",
            "child_frame_id": "base_link",
            "k": 1.6,} # coefficient of ROSKY2 itself for odometry

        self.get_ros2_parameter()
        self.platform = self.choose_platform(self.parameter["platform"])

        # initiallize driver
        self.driver = OminibotCar(self.parameter["port"], self.parameter["baud"], self.parameter["timeout"])
        self.initialize_driver()

        # Create subscriber
        self.twist_subscriber = self.create_subscription(
            Twist, '~/cmd_vel', self.callback_cmd_vel, 10)

        # Create publisher
        self.publisher = self.create_publisher(Odometry, "~/odom", 10)

        ## initiallize odometry
        self.initialize_odometry()

        # Create timer
        self.timer = self.create_timer(timer_period_sec=(1.0 / self.parameter["pub_freq"]), callback=self.callback_timer, callback_group=ReentrantCallbackGroup())



        # log
        self.get_logger().info(f'Start!')

    def get_ros2_parameter(self):
        """From ros2 parameter server to get parameter, and produce mecanum factor
        """
        for key in self.parameter.keys():
            self.declare_parameter(key, self.parameter[key])
            self.parameter[key] = self.get_parameter(key).value
            self.get_logger().info(f"Publish ros2 parameter, {key}: {self.parameter[key]}")
        
        ## use ros2 parameter to produce mecanum factor
        self.mecanum_factor()
              

    def mecanum_factor(self):
        """Factor for mecanum Drive kinematic
          Transform unit from meter to millimeter and add below parameter in self.parameter:
            wheel_radius: radius of wheel -> float
            wheel_perimeter: perimeter of wheel -> float
          Axis:
                x\n
                ^\n
                |\n
            y <--
        """ 
        for key in self.parameter.keys():
            if key[:10] == "motor_axis" or key[:5] == "wheel":
                self.parameter[key] *= math.pow(10, -3)
        self.parameter["wheel_radius"] = self.parameter["wheel_diameter"] / 2.0
        self.parameter["wheel_perimeter"] = self.parameter["wheel_diameter"] * math.pi

    def choose_platform(self, platform="mecanum"):
        """Define the kinematic model
        Args:
          platform: which platform you use, include \"differential\", \"omnibot\", \"mecanum\"
        Return:
          platform:
            kinematic_model: the kinematic model of the platform with numpy array
            inverse_kinematic_model: the inverse kinematic model of the platform with numpy array
            reverse_encoder: the reverse encoder of the platform with numpy array
        """
        platform = {"differential": 0, "omnibot": 1, "mecanum": 2}.get(platform, -1)
        if platform == -1:
            self.get_logger().info("Sorry, we just can use \"differential\", \"omnibot\" and \"mecanum\".\nShutdown the node now..")
            self.shutdown()
        else:
            if platform == 2:
                """
                _platform:
                  vehicle: which vehicle use 
                  kinematic model shape: (4, 3)
                  inverse kinematic_ model shape: (3, 4)
                  reverse_encoder shape: (4, 4)
                """
                _platform = {}
                _platform["vehicle"] = "mecanum"
                length = self.parameter["motor_axis_length"] / 2.0
                width = self.parameter["motor_axis_width"] / 2.0
                coefficient = length + width
                _platform["kinematic_model"] = np.array(
                    [ 
                        [coefficient, 1, 1],   # u2 -> v1 
                        [-coefficient, 1, -1], # u1 -> v2       
                        [coefficient, 1, -1],  # u3 -> v3
                        [-coefficient, 1, 1],  # u4 -> v4           
                    ]) / self.parameter["wheel_perimeter"] * self.parameter["motor_ratio"]

                coefficient = 1 / coefficient
                _platform["inverse_kinematic_model"] = np.array(
                    [
                        [coefficient, -coefficient, coefficient, -coefficient],
                        [1,           1,            1,            1          ],
                        [1,          -1,           -1,            1          ]
                       #u2,          u1,           u3,            u4
                    ]) * self.parameter["wheel_perimeter"] / self.parameter["motor_ratio"] / 4.0

                _platform["reverse_encoder"] = np.array(
                    [
                        [1, 0, 0, 0],
                        [0, -1, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, -1]
                    ])

        return _platform

    def initialize_driver(self, platform="individual_wheel"):
        """Start communciate with ominibot car 
        Args:
          platform: choose which platform you want to use, default is \"individual_wheel\" -> string
            1. omnibot 
            2. mecanum
            3. individual_wheel   
        """
        self.driver.set_system_mode(platform=platform)
        try:
            thread = threading.Thread(target=self.driver.serial_thread)
            thread.start()
        except:
            print("error")
            self.shutdown()

    def initialize_odometry(self):
        """Variable for ROS2 publisher
        Variable:
          odom: this represents an estimate of a position and velocity in free space. The pose in this message should be specified in the coordinate frame given by header.frame_id. The twist in this message should be specified in the coordinate frame given by the child_frame_id
            header: includes the frame id of the pose parent.
              stamp: current time
              fram_id: transform frame with which this data is associated, default is \"~/odom\"
            pose: this represents a pose in free space with uncertainty
              pose: a representation of pose in free space, composed of position and orientation
                position: this contains the position of a point in free space
                  x: float64
                  y: float64
                  z: float64
                orientation: this represents an orientation in free space in quaternion form.
                  x: float64
                  y: float64
                  z: float64
                  w: float64
            child_frame_id: frame id the pose points to. The twist is in this coordinate frame.
            twist: estimated linear and angular velocity relative to child_frame_id.
               twist: this expresses velocity in free space with uncertainty.
                  linear: this represents a vector in free space
                    x: float64 
                    y: float64
                    z: float64
                  angular: this represents a vector in free space
                    x: float64
                    y: float64
                    z: float64
          encoder: trick numbers of the encoder
            last: trick numbers of the encoder last moment
            current: trick numbers of the encoder this moment
            amount: how many trick numbers in hte motor
          odometry_time: how long the running time
            current: current time 
            last: last time
          position: this contains the position of a point in free space
            x: float64
            y: float64
            theta: float64
        """
        self.odom = Odometry()
        self.odom_broadcaster = TransformBroadcaster(self)
        self.encoder = {
            "last": np.array([0, 0, 0, 0]),
            "current": np.array([0, 0, 0, 0]),
            "amount": self.parameter["motor_ratio"]}

        self.odometry_time = {
            "current": self.get_clock().now(),
            "last": self.get_clock().now(), }

        self.position = {
            "x": 0.0,
            "y": 0.0,
            "theta": 0.0,}

        self.publish_transform_odometry()

    def publish_transform_odometry(self, velocity=[0.0, 0.0, 0.0]):
        """Publish the transform over tf2 and pulish topic \"~/odom\" with message odometry
        Args:
          velocity: twist for platform from encoder, order is [wz, vx, vy]
        """
        # publish the transform over tf2
        odom_quaternion = quaternion_from_euler(0.0, 0.0, self.position["theta"])
        _tf2 = TransformStamped()
        _tf2.header.stamp = self.odometry_time["current"].to_msg()
        _tf2.header.frame_id = self.parameter["odom_id"]
        _tf2.child_frame_id = self.parameter["child_frame_id"]
        _tf2.transform.translation.x = self.position["x"]
        _tf2.transform.translation.y = self.position["y"]
        _tf2.transform.translation.z = 0.0
        _tf2.transform.rotation.x = odom_quaternion[0]
        _tf2.transform.rotation.y = odom_quaternion[1]
        _tf2.transform.rotation.z = odom_quaternion[2]
        _tf2.transform.rotation.w = odom_quaternion[3]
        self.odom_broadcaster.sendTransform(_tf2)

        # publish the odometry message over ROS2
        self.odom.header.stamp = self.odometry_time["current"].to_msg()
        self.odom.header.frame_id = self.parameter["odom_id"]
        self.odom.pose.pose = Pose(position=Point(x=self.position["x"], y=self.position["y"], z=0.0), 
                                   orientation=Quaternion(x=odom_quaternion[0], y=odom_quaternion[1], z=odom_quaternion[2], w=odom_quaternion[3]))
        self.odom.child_frame_id = self.parameter["child_frame_id"]
        self.odom.twist.twist = Twist(linear=Vector3(x=velocity[1], y=velocity[2], z=0.0), angular=Vector3(x=0.0, y=0.0, z=velocity[0])) 
        self.publisher.publish(self.odom)
        self.encoder["last"] = self.encoder["current"]
        self.odometry_time["last"] = self.odometry_time["current"]

    def callback_cmd_vel(self, msg):
        """Receive msg Twist and send velocity to ominibot_car_com
        """
        self.linear_x = msg.linear.x 
        self.linear_y = msg.linear.y 
        self.angular_z = msg.angular.z
        #self.get_logger().info(f"I get velocity - linear x: {self.linear_x}, linear y: {self.linear_y}, angular z: {self.angular_z}")
        self.twist_to_wheels(self.linear_x, self.linear_y, self.angular_z)
        

    def twist_to_wheels(self, vx, vy, wz):
        """Calculate speed for each wheel, kinematic model dot [wz, vx, vy]^T\n
        Args:
          vx: m/s, linear speed for x-axis
          xy: m/s, linear speed for y-axis
          vz: rad/s, angular speed for z-axis
        """
        twist = np.array([wz, vx, vy])
        twist.shape = (3, 1)
        wheel_speed = np.dot(self.platform["kinematic_model"], twist)
        wheel_speed = wheel_speed.flatten()
        self.move(wheel_speed.tolist())

    def move(self, wheel_speed=[]):
        """Drive the car to move
        Args: 
            wheel_speed: list with each wheel's speed
              wheel_speed[0]: vrf
              wheel_speed[1]: vlf
              wheel_speed[2]: vrb
              wheel_speed[3]: vlb

        """
        if self.parameter["platform"] == "mecanum":
            self.driver.individual_wheel(V1=wheel_speed[0], V2=wheel_speed[1], V3=wheel_speed[2], V4=wheel_speed[3])

    def callback_timer(self):
        """Publisher odometry by pub_freq
        mecanum:
              x\n
              ^\n
              | 1+ | 2- |\n
              | 4- | 3+ |
        """
        self.odometry_time["current"] = self.get_clock().now()
        discrete_time = self.odometry_time["current"] - self.odometry_time["last"]
        discrete_time = discrete_time.nanoseconds * math.pow(10, -9)
        self.encoder["current"] = np.dot(np.array(self.driver.get_encoder_data()), self.platform["reverse_encoder"])
        delta = np.dot(self.platform["inverse_kinematic_model"], self.encoder["current"]) * discrete_time # [dphi, dx, dy]
        delta.shape = (3, 1)
        rotation_matrix = np.array(
            [
                [1, 0, 0],
                [0, np.cos(self.position["theta"]), -np.sin(self.position["theta"])],
                [0, np.sin(self.position["theta"]),  np.cos(self.position["theta"])],
            ])

        position = np.dot(rotation_matrix, delta) * self.parameter["k"] # np.array([dphi, dx, dy]).shape(3, 1)
        position = position.flatten()
        self.position["x"] += position[1]
        self.position["y"] += position[2]
        self.position["theta"] += position[0]
        self.position["theta"] = self.normalize_angle(self.position["theta"])
        velocity = delta / discrete_time
        self.get_logger().info(f"\nvx: {velocity[1]}")
        self.publish_transform_odometry(velocity.flatten().tolist())

    def normalize_angle(self, angle):
        """Let angle in range [-2pi, 2pi]
        Args:
          angle: over the radians [-2pi, 2pi]
        Return:
          normalized angle between [-2pi, 2pi]
        """
        normalize = 2 * math.pi
        if angle > normalize:
            angle = angle - normalize
        elif angle < -normalize :
            angle = angle + normalize
        return angle

    def shutdown(self):
        """Close ominibot car port and shutdown this node 
        """
        self.get_logger().info("Stop threading...")
        self.driver.stop_thread()
        self.get_logger().info("close ominibot car port...")
        self.driver.disconnect()
        self.get_logger().info("Done! Will shutdown this node.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = OminibotCarDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    node.shutdown()

if __name__ == '__main__':
    main()




