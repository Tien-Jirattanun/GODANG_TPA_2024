import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class Planing(Node):

    def __init__(self):
        super().__init__('planing_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.01  # 100 hz
        
    def stateChecker(self):
        pass

    def timer_callback(self):
        msg = String()
        msg.data = 'test'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    planing = Planing()
    rclpy.spin(planing)
    
    planing.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()