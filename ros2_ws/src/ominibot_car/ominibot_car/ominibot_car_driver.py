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
import rclpy, threading, math
from rclpy.node import Node
from .ominibot_car_com import OminibotCar

# import msgs
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray 


# tf library
import tf_transformations

# tf2 library
from tf2_ros import TransformBroadcaster



# defined constant 
PI = math.pi
MOTOR_ENCODER_COUNT = 1170 # per rotation
MOTOR_RPM = 139            # 6V / 20000, 1:120, we provide 5V


class OminibotCarDriverNode(Node):
    '''Get velocity and plan mobile path
    '''
    def __init__(self):
        # Node name
        super().__init__("ominibot_car_driver")
        self.node_name = self.get_name()

        # declare parameter
        ## ominibot car
        self.declare_parameter("port", "/dev/ominibot_car")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("time_out", None)
        self.declare_parameter("magnification.status", False)
        self.declare_parameter("magnification.value", 100) # self.declare_parameters(namespace="", parameters=[("status", False), ("value", 100)])

        ## mecanum
        self.declare_parameter("odom_frequency", 20)
        self.declare_parameter("wheel_diameter", 65)

        ##
        self.last_time = rclpy.time()
        self.current_time = rclpy.time()



        # get parameter
        ## from ros 2 parameter server
        self.port = self.get_parameter("port").value
        self.baud = self.get_parameter("baud").value
        self.time_out = self.get_parameter("time_out").value
        self.magnification_status = self.get_parameter("magnification.status").value
        self.magnification_value = self.get_parameter("magnification.value").value
        self.odom_frequency = self.get_parameter("odom_frequency").value
        self.wheel_diameter = self. get_parameter("wheel_diameter").value

        ## for mecanum
        self.scale = {
            "x": () / (),
            "y":,
            "theta",
        } 
        self.pose = {
            "x": 0.0,
            "y": 0.0,
            "theta": 0.0,
        }

        # initiallize driver
        self.driver = OminibotCar(self.port, self.baud, self.time_out)
        self.initialize_driver()

        # Create subscriber
        self.twist_subscriber = self.create_subscription(
            Twist, '~/cmd_vel', self.callback_cmd_vel, 10)

        # Create publisher
        self.odom_publisher = self.create_publisher(Float32MultiArray, "~/odom", 10)

        # log
        self.get_logger().info(f'Start!')

    def initialize_driver(self):
        '''Start communciate with ominibot car
        '''
        self.driver.set_system_mode(platform="mecanum")
        try:
            thread = threading.Thread(target=self.driver.serial_thread)
            thread.start()
        except:
            print("error")
            self.shutdown()


    def callback_cmd_vel(self, msg):
        '''Receive msg Twist and send velocity to ominibot_car_com  
        '''
        self.linear_x = msg.linear.x * self.magnification_value if self.magnification_status == True else msg.linear.x
        self.linear_y = msg.linear.y * self.magnification_value if self.magnification_status == True else msg.linear.y
        self.angular_z = msg.angular.z * self.magnification_value if self.magnification_status == True else msg.angular.z
        self.get_logger().info(f"I get velocity - linear x: {self.linear_x}, linear y: {self.linear_y}, angular z: {self.angular_z}")
        self.driver.mecanum(Vx=self.linear_x, Vy=self.linear_y,  Vz=self.angular_z)


    def encoder_callback(self):
        '''

        '''






    def shutdown(self):
        '''close ominibot car port and shutdown this node 
        '''
        self.get_logger().info("Stop threading...")
        self.driver.stop_thread()
        self.get_logger().info("close ominibot car port...")
        self.driver.disconnect()
        self.get_logger().info("Done! Will shutdown this node.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = OminibotCarDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    node.shutdown()

if __name__ == '__main__':
    main()




