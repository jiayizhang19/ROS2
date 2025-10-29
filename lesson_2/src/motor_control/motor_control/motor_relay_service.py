# This node contains a very basic example for a service server
# This server receives a request to activate or deactivate the main relay enabling
# for motor control. A visualization of the node and service can be seen below:
#    /motor_relay    _______________
# <---------------->| motor_service |
#                   |_______________|
# 
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class MotorRelayService(Node):
    def __init__(self):
        super().__init__('motor_service')      # Initialize the node and name it
        # Create the server for the motor_relay service
        self.srv = self.create_service(
            SetBool,                            # of type SetBool
            'motor_relay',
            self.motor_relay_service_server     # Call this when a request is made
        )
        self.get_logger().info('Node initialized successfully')

    def motor_relay_service_server(self, request, response):
        '''Simple service server callback'''
        # Takes the request from the data field
        req = request.data
        # Create a string based on the request
        string = 'Activated motor' if req else 'Operation failed'
        # Log it
        self.get_logger().info(string)
        # And formulate the response to be returned
        response.success = True
        response.message = string
        return response

def main(args=None):
    rclpy.init(args=args)       # Start ROS2

    node = MotorRelayService()  # Initialize your node object
    rclpy.spin(node)            # Spin (loop) your node

    rclpy.shutdown()            # Once out of the loop, stop ROS2

if __name__ == "__main__":
    main()