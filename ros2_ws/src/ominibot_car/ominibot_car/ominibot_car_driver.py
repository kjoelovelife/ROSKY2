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

        # get parameter
        self.port = self.get_parameter("port").value
        self.baud = self.get_parameter("baud").value
        self.time_out = self.get_parameter("time_out").value

        # initiallize driver
        self.driver = OminibotCar(self.port, self.baud, self.time_out)
        self.initialize_driver()

        # Create subscriber
        self.twist_subscriber = self.create_subscription(
            Twist, '~/cmd_vel', self.callback_cmd_vel, 10)

        # log
        self.get_logger().info(f'Start!')

    def initialize_driver(self):
        self.driver.set_system_mode(platform="mecanum")
        try:
            thread = threading.Thread(target=self.driver.serial_thread)
            thread.start()
        except:
            print("error")
            self.shutdown()


    def callback_cmd_vel(self, msg):
        self.linear_x, self.linear_y, self.angular_z = msg.linear.x, msg.linear.y, msg.angular.z
        self.driver.mecanum(Vx=self.linear_x, Vy=self.linear_y,  Vz=self.angular_z)

    def shutdown(self):
        self.driver.stop_thread()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = OminibotCarDriverNode()
    rclpy.spin(node)
    node.shutdown()
    




if __name__ == '__main__':
    main()




