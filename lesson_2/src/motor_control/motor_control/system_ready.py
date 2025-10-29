# This code represents a simple timer based publisher simulating a status update
# from a sensor or subsystem
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class SystemReady(Node):
    def __init__(self):
        super().__init__('system_ready')      # Initialize the node and name it

        self.pub = self.create_publisher(Bool, 'system_ready', 10)    # A boolean publisher
        self.timer = self.create_timer(2.0, self.check_robot_status)  # A 0.5Hz timer, means every 2 seconds
        # NOTE: At each instance at which the timer `ticks`, "check_robot_status" is called

        self.get_logger().info('Node initialized successfully')

    def check_robot_status(self):
        '''A simple boolean publisher -- Simulates that the subsystem is ready'''
        msg = Bool()
        msg.data = True
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)       # Start ROS2

    node = SystemReady()        # Initialize your node object
    rclpy.spin(node)            # Spin (loop) your node

    rclpy.shutdown()            # Once out of the loop, stop ROS2

if __name__ == "__main__":
    main()