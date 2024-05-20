import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


class Planing(Node):

    def __init__(self):
        super().__init__('planing_node')
        
        # variable
        self.pos = []
        self.way = []
        
        # way point publisher
        self.publisher_point = self.create_publisher(
            Float32MultiArray, 'way_point', 10)
        
        # pos subscriber
        self.subscription_pos = self.create_subscription(
            Float32MultiArray, 'pos', self.pos_callback, 10)
        self.subscription_pos
        
        # button subscriber
        self.subscription_but = self.create_subscription(
            Float32MultiArray, 'button', self.button_callback, 10)
        self.subscription_but


    def pos_callback(self, msg):
        self.pos = msg.data
        
    def button_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


    def timer_callback(self):
        msg = Float32MultiArray()
        msg.data = [1, 2, 3, 4]
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
