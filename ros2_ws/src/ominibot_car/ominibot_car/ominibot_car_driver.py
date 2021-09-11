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
import rclpy, threading
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .ominibot_car_com import OminibotCar


class OminibotCarDriverNode(Node):
    '''Get velocity and plan mobile path
    '''
    def __init__(self):
        # Node name
        super().__init__("ominibot_car_driver")
        self.node_name = self.get_name()

        # declare parameter
        self.declare_parameter("port", "/dev/ominibot_car")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("time_out", None)
        self.declare_parameter("magnification_status", False)
        self.declare_parameter("magnification_value", 100)

        # get parameter
        self.port = self.get_parameter("port").value
        self.baud = self.get_parameter("baud").value
        self.time_out = self.get_parameter("time_out").value
        self.magnification_status = self.get_parameter("magnification_status").value
        self.magnification_value = self.get_parameter("magnification_value").value

        # initiallize driver
        self.driver = OminibotCar(self.port, self.baud, self.time_out)
        self.initialize_driver()

        # Create subscriber
        self.twist_subscriber = self.create_subscription(
            Twist, '~/cmd_vel', self.callback_cmd_vel, 10)

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




