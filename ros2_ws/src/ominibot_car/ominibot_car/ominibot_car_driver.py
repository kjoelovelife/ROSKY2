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
import rclpy, threading, math, time, re
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
        }
        self.get_ros2_parameter()

        # initiallize driver
        self.driver = OminibotCar(self.parameter["port"], self.parameter["baud"], self.parameter["timeout"])
        self.initialize_driver()

        # Create subscriber
        self.twist_subscriber = self.create_subscription(
            Twist, '~/cmd_vel', self.callback_cmd_vel, 10)

        # Create publisher
        #self.odom_publisher = self.create_publisher(Float32MultiArray, "~/odom", 10)

        # log
        self.get_logger().info(f'Start!')

    def get_ros2_parameter(self):
        """from ros2 parameter server to get parameter, and produce mecanum factor
        """
        for key in self.parameter.keys():
            self.declare_parameter(key, self.parameter[key])
            self.parameter[key] = self.get_parameter(key).value
            self.get_logger().info(f"Publish ros2 parameter, {key}: {self.parameter[key]}")
        
        ## use ros2 parameter to produce mecanum factor
        self.mecanum_factor()
              

    def mecanum_factor(self):
        """factor for mecanum Drive kinematic
          Transform unit from meter to millimeter and add below parameter in self.parameter:
            wheel_radius: radius of wheel -> float
            wheel_perimeter: perimeter of wheel -> float
            wheel_k: abs(x_distance) + abs(y_distance)-> float
            x_distance: distance along to the x-axis from center of bot to motor axis -> float
            y_distance: distance along to the y-axis from center of bot to motor axis -> float
            left_front: coordinate(m) of the left-front wheel from center of bot -> tuple(float, float)
            left_back: coordinate(m) of the left-front wheel from center of bot -> tuple(float, float)
            right_front: coordinate(m) of the left-front wheel from center of bot -> tuple(float, float)
            right_back: coordinate(m) of the left-front wheel from center of bot -> tuple(float, float)
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

        # distance between center of bot and center of wheel
        self.parameter["x_distance"] = self.parameter["motor_axis_length"] / 2.0
        self.parameter["y_distance"] = (self.parameter["motor_axis_width"] / 2.0) + self.parameter["wheel_axis_length"] + (self.parameter["wheel_width"] / 2.0)
        self.parameter["left_front"] = (self.parameter["x_distance"], self.parameter["y_distance"])
        self.parameter["left_back"] = (-self.parameter["x_distance"], self.parameter["y_distance"])
        self.parameter["right_front"] = (self.parameter["x_distance"], -self.parameter["y_distance"])
        self.parameter["right_back"] = (-self.parameter["x_distance"], -self.parameter["y_distance"])
        self.parameter["wheel_k"] = self.parameter["x_distance"] + self.parameter["y_distance"]

    def wheel_speed(self, Vx, Vy, Vz, platform="mecanum"):
        """Calculate speed for each wheel\n
        Args:
          Vx: linear speed for x-axis -> float
          Vy: linear speed for y-axis -> float
          Vz: angular speed for z-axis -> float
          platform: which kinematic to use, default is \"mecanum\" -> string
        Return:
          (Vlf, Vlb, Vrf, Vrb):
            Vlf: speed for left-front wheel -> float
            Vlb: speed for left-back wheel -> float
            Vrf: speed for right-front wheel -> float
            Vrb: speed for right-back wheel -> float
        """
        if platform == "mecanum":
            Vz = self.parameter["wheel_k"] * Vz

            # Translate linear velocity for each wheel
            Vlf = Vx - Vy - Vz
            Vlb = Vx + Vy - Vz
            Vrb = Vx - Vy + Vz
            Vrf = Vx + Vy + Vz

            # Translate linear velocity for each wheel(rad/s)
            Vlf /= self.parameter["wheel_perimeter"]
            Vlb /= self.parameter["wheel_perimeter"]
            Vrb /= self.parameter["wheel_perimeter"]
            Vrf /= self.parameter["wheel_perimeter"]

            # Translate velocity for each motor(rpm)
            Vlf *= self.parameter["motor_ratio"]
            Vlb *= self.parameter["motor_ratio"]
            Vrb *= self.parameter["motor_ratio"]
            Vrf *= self.parameter["motor_ratio"]
            return (Vlf, Vlb, Vrf, Vrb)

    def initialize_driver(self, platform="individual_wheel"):
        """Start communciate with ominibot car 
        Args:
          platform: choose which platform you want to use:
            1. omnibot 
            2. mecanum
            3. individual_wheel
            default is \"individual_wheel\" -> string
        """
        self.driver.set_system_mode(platform=platform)
        try:
            thread = threading.Thread(target=self.driver.serial_thread)
            thread.start()
        except:
            print("error")
            self.shutdown()

    def callback_cmd_vel(self, msg):
        """Receive msg Twist and send velocity to ominibot_car_com
        """
        self.linear_x = msg.linear.x 
        self.linear_y = msg.linear.y 
        self.angular_z = msg.angular.z

        self.get_logger().info(f"I get velocity - linear x: {self.linear_x}, linear y: {self.linear_y}, angular z: {self.angular_z}")
        wheel_speed = self.wheel_speed(self.linear_x, self.linear_y, self.angular_z)
        self.driver.individual_wheel(V1=wheel_speed[3], V2=wheel_speed[0], V3=wheel_speed[2], V4=wheel_speed[1])


    def encoder_callback(self):
        """

        """
        pass

    def shutdown(self):
        """close ominibot car port and shutdown this node 
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




