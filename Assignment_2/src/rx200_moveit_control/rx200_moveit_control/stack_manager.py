import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class StackManager(Node):
    def __init__(self):
        super().__init__('stack_manager')

        # Subscribe to camera node to get cube locations
        self.create_subscription(
            String,
            '/object_detection',
            self.vision_callback,
            10
        )

        # Publish cube locations to moveit node
        self.stack_suq_pub = self.create_publisher(
            String,
            '/stack_sequence',
            10
        )

        # Create a dictionary to hold cubes' coordinates
        self.cubes_detected = {}

        # Get color sequence from user input
        self.declare_parameter('color_seq', '')
        color_seq = self.get_parameter('color_seq').value
        if color_seq:
            self.color_seq = [color.strip() for color in color_seq.split(',')]
        else:
            self.color_seq = []

        self.get_logger().info("Stack_Manager node has been initialized successfully!")
        self.get_logger().info(f'Color sequence received: {self.color_seq}')


    def vision_callback(self, msg: String):
        self.parse_vision_output(msg)
        stack_seq = self.arrange_stack_order()
        if stack_seq:
            # Convert dictionary to string message type use json.dumps()
            stack_seq = json.dumps(stack_seq)
            msg = String()
            msg.data = stack_seq
            self.stack_suq_pub.publish(msg)
        self.get_logger().info(f'Sending stack sequence to moveit node: {msg.data}')


    def parse_vision_output(self, msg: String):
        """
        Example input: "red: 1,2,3; blue: 1,2,3; yellow: 1,2,3"
        Expected output: {'red': (5.0, 20.0, 3.0), 'blue': (5.0, 0.2, 0.3), 'yellow': (5.0, 10.0, 10.0)}
        """
        self.cubes_detected = {}
        # Example msg received from realsense camera: "red: 1,2,3; blue: 1,2,3; yellow: 1,2,3"
        try:
            cubes = msg.data.split(';')
            for cube in cubes:
                cube = cube.strip()
                if ":" in cube:
                    color, position = cube.split(':')
                    color = color.strip().lower()
                    x, y, z = map(float, position.strip().split(','))
                    self.cubes_detected[color] = (x, y, z)
            self.get_logger().info(f"Receiving {len(self.cubes_detected)} cubes from camera: {self.cubes_detected}")
        except Exception as e:
            self.get_logger().error(f'Error in parsing coordinates from camera: {e}')
        

    def arrange_stack_order(self):
        '''
        Reverse color_seq from user input to get stack_seq for robot 
        '''
        if not self.cubes_detected:
            self.get_logger().error("No cubes detected!")
            return None
        if not self.color_seq:
            return self.cubes_detected
        self.stack_seq = {}
        for color in self.color_seq[::-1]:
            self.stack_seq[color] = self.cubes_detected[color]
        return self.stack_seq
        


def main():
    rclpy.init()
    node = StackManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()