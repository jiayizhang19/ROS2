import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64


class ExampleNode(Node):
    def __init__(self, default_msg='My nice node'):
        super().__init__('example_node')

        self.count = 0

        # ros2 topic pub /name_of_topic std_msgs/msg/Float64 {"data: 10.0"}

        self.sub = self.create_subscription(String,'input', self.sub_callback, 10)
        self.pub = self.create_publisher(Int64, 'lala_count', 10)

        self.get_logger().info(default_msg)

    def sub_callback(self, msg: String):
        modified = msg.data + 'LALA'
        self.get_logger().info(modified)
        self.count += 1
        count_pub = Int64()
        count_pub.data = self.count
        self.pub.publish(count_pub)
        
        # self.get_logger().warning()


def main (args=None):
    rclpy.init(args=args)

    node = ExampleNode(default_msg='Node initialised successfully!')
    rclpy.spin(node) # Spin (loop) your node
    
    rclpy.shutdown() # Once out of the loop, stop ROS2


if __name__ == "__main__":
    main()