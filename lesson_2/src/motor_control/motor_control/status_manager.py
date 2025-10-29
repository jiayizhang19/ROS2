# This node collects the status of different subsystems and decides if the system
# is ready to run. This kind of system is commonly seen in many robotic applications
#
# In this example, we simply have 2 subscribers (representing our subsystem feedbacks)
# and a service client, which is used for activating/deactivating the motor control.
# A visualization of the node communication design can be seen below:
#   /localization_valid    ________________
# ----------------------->|                |  service call: /motor_relay
#   /system_ready         | status_manager |<----------------------------->
# ----------------------->|________________|
# Something you should observe is that the system_ready callback updates at 0.5Hz, the
# localization_ready callback updates at 1Hz and the client calls are made at 0.33Hz.
# All of these work together without you having to do much (except for adding the flag
# variables)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool       # Standard messages
from std_srvs.srv import SetBool    # Standard services


class StatusManager(Node):
    def __init__(self):
        super().__init__('status_manager')
        # Flags for representing that the different subsystems are ready
        self.is_system_ready: bool = False
        self.is_localization_ready: bool = False
        # Subscriber for the localization subsystem
        self.sub_1 = self.create_subscription(
            Bool,
            'localization_valid',
            self.localization_validation_callback,
            10
        )
        # Subscriber for a general subsystem check before running
        self.sub_2 = self.create_subscription(
            Bool,
            'system_ready',
            self.system_ready_callback,
            10
        )

        # Create a client and wait for the client to connect to the server
        self.cli = self.create_client(SetBool, 'motor_relay')
        while not self.cli.wait_for_service(1.0):
            self.get_logger().info('Waiting for server...')
        # A timer that does the client calls at 0.33Hz
        self.timer = self.create_timer(3.0, self.control_motor)

        self.get_logger().info('Node initialized successfully')

    def control_motor(self):
        # If all the subsystems are ready
        if self.is_system_ready and self.is_localization_ready:
            # Request that the motors control is activated
            request = SetBool.Request()
            request.data = True
            # Send the request to the server
            future = self.cli.call_async(request)
            # And tell it to call 'motor_control_future_callback' with the
            # server service response
            future.add_done_callback(self.motor_control_future_callback)

    def motor_control_future_callback(self, future):
        '''Basic server service response -- Future'''
        self.get_logger().info('Calling from the future')

    def localization_validation_callback(self, msg: Bool):
        '''Simple subscriber updating a boolean flag based on message data'''
        self.is_localization_ready = msg.data

    def system_ready_callback(self, msg: Bool):
        '''Simple subscriber updating a boolean flag based on message data'''
        self.is_system_ready = msg.data

def main(args=None):
    rclpy.init(args=args)       # Start ROS2

    node = StatusManager()      # Initialize your node object
    rclpy.spin(node)            # Spin (loop) your node

    rclpy.shutdown()            # Once out of the loop, stop ROS2

if __name__ == "__main__":
    main()