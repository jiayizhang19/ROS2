import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point 
from std_msgs.msg import Bool
import os
import yaml
import math

class PointSafetyCheck(Node):
    def __init__(self):
        super().__init__('point_safety_checker')

        # Subscriber to Point messages
        self.point_input_sub = self.create_subscription(
            Point,
            '/point_input',
            self.point_safety_check_callback,
            10
        
        )
        # Publisher for safety status
        self.point_safety_check_pub = self.create_publisher(
            Bool, 
            '/point_safety_check', 
            10            
        )
        self.get_logger().info('Point Safety Checker Pub Node initialized successfully')

    def point_safety_check_callback(self, msg: Point):
        x, y, z = msg.x, msg.y, msg.z
        self.get_logger().info(f'Receiving point ({x}, {y}, {z}) for safety check')
        is_safe = True
        #Check Pick up and drop off location. Make sure it is not beyond arm's reach or inside base area
        safety_check_result = Bool()
        safety_check_result.data = is_safe
        self.point_safety_check_pub.publish(safety_check_result)
        
def main(args=None):
    rclpy.init(args=args)
    point_safety_check = PointSafetyCheck()
    rclpy.spin(point_safety_check)
    rclpy.shutdown()