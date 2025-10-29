import rclpy
from rclpy.node import Node
from std_msgs import String


class MinNode(Node):
    def __init__(self):
        super().__init__('min_node')
        self.get_logger().info('My nice node')
        
        self.sub = self.create_subscription(String,'input', self.sub_callback, 10)

    def sub_callback(self, msg:String):
        modified = msg.data + '_LALA'
        self.get_logger().info(modified)
        
def main (args=None):
    rclpy.init(args=args)

    node = MinNode()
    rclpy.spin(node)
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()