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

        #Get Yaml file path to src
        self.config_file = os.path.join(
            os.path.dirname(__file__),
            '../../../../../../rx200_moveit_control/rx200_moveit_control/config/'
        )
        #if self.verbose: self.get_logger().info(f'Yaml Path1: {self.config_file}')
        self.yaml_dir = os.path.abspath(self.config_file)
        #if self.verbose: self.get_logger().info(f'Yaml Abs Path: {self.yaml_dir}')
        self.poses = self.load_from_yaml(self.yaml_dir, 'poses.yaml')
                
        #Default Base radius 
        self.base_radius = self.poses['no_go_zone']['base_radius']
        #Default Out reach circunsference
        self.out_radius = self.poses['no_go_zone']['out_radius']
        #Default Base Box (rectangle) Definition
        self.x1 = self.poses['no_go_zone']['x1']
        self.x2 = self.poses['no_go_zone']['x2']
        self.y1 = self.poses['no_go_zone']['y1']
        self.y2 = self.poses['no_go_zone']['y2']
        #Safe Z hight to move on top of base circle or base
        self.z_hieght_safe = 0.15


    def load_from_yaml(self, dir, file):
        path = os.path.join(dir, file)
        print(f'Loading poses from: {path}')
        with open(path, 'r') as file:
            return yaml.safe_load(file)

    def point_safety_check_callback(self, msg: Point):
        x, y, z = msg.x, msg.y, msg.z
        # self.get_logger().info(f'Receiving point ({x}, {y}, {z}) for safety check')
        is_safe = True
        #Check Pick up and drop off location. Make sure it is not beyond arm's reach or inside base area
        if self.base_circle_box_inside(x, y, z) or self.out_arm_reach_area(x, y, z):
            self.get_logger().info(f'Point ({x}, {y}, {z}) is either inside arm base or beyond arm reach')
            is_safe = False
        safety_check_result = Bool()
        safety_check_result.data = is_safe
        self.point_safety_check_pub.publish(safety_check_result)

        #This checks are True if coordinates are inside the base areas and False of outside the areas.   
    def base_circle_box_inside(self, x, y, z):        
        point_radius = math.sqrt(x**2 + y**2)        
        base_circle_check = point_radius <= self.base_radius
        
        if x <= self.x1 and x >= self.x2 and y <= self.y2 and y >= self.y1:  
            base_box_check = True
        else:
            base_box_check = False
        z_hieght_check = z < self.z_hieght_safe  

        #Any is true if inside the base circle or box
        if (base_circle_check or base_box_check) and z_hieght_check:
            return True
        else:
            return False 

    def out_arm_reach_area(self, x, y, z):
        point_radius = math.sqrt(x**2 + y**2)        
        if point_radius > self.out_radius:
            return True
        else:
            return False
        
def main(args=None):
    rclpy.init(args=args)
    point_safety_check = PointSafetyCheck()
    rclpy.spin(point_safety_check)
    rclpy.shutdown()