# import the Empty module from std_servs service interface
import sys, time
path = "/opt/ros/noetic/lib/python3/dist-packages" 
if path in sys.path:
    sys.path.remove(path)

from custom_interface.srv import FindWall
from std_srvs.srv import Empty
import rclpy
from rclpy.node import Node

# reference: https://tw511.com/a/01/2598.html
from functools import partial

class TestClient(Node):

    def __init__(self):
        super().__init__("test_client")
        self.call_test_server()

    def call_test_server(self):
        client = self.create_client(FindWall, "/find_wall")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server find_wall...")

        request = FindWall.Request()
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_test_server))

    def callback_call_test_server(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Done!")
        except Exception as error:
            self.get_logger().error(f"Service call fialed {error}")

# import the Empty module from std_servs service interface
from std_srvs.srv import Empty
# import the ROS2 python client libraries
import rclpy
from rclpy.node import Node


def main(args=None):
    rclpy.init(args=args)
    node = TestClient()
    rclpy.spin(node)
    rclpy.shutdown()

class ClientAsync(Node):

    def __init__(self):
        # Here we have the class constructor

        # call the class constructor to initialize the node as server_client
        super().__init__('test_client')
        # create the service client object
        # defines the name and type of the service server we will work with.
        self.client = self.create_client(FindWall, "/find_wall")
        # checks once per second if a service matching the type and name of the client is available.
        while not self.client.wait_for_service(timeout_sec=1.0):
            # if it is not available, a message is displayed
            self.get_logger().info('service not available, waiting again...')
        
        # create a Empty request
        self.req = FindWall.Request()
        

    def send_request(self):
        
        # send the request
        self.future = self.client.call_async(self.req)
'''
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    client = ClientAsync()
    # run the send_request() method
    client.send_request()

    while rclpy.ok():
        # pause the program execution, waits for a request to kill the node (ctrl+c)
        rclpy.spin_once(client)
        if client.future.done():
            try:
                # checks the future for a response from the service
                # while the system is running. 
                # If the service has sent a response, the result will be written
                # to a log message.
                response = client.future.result()
            except Exception as e:
                # Display the message on the console
                client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                # Display the message on the console
                client.get_logger().info(
                    'the robot is moving' ) 
            break

    client.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()
'''

if __name__ == "__main__":
    main()