import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


class Planing(Node):

    def __init__(self):
        super().__init__('planing_node')
        
        # variable
        
        # Contain position of the robot
        self.pos = []
        # Contain the button status
        self.but = []
        # Contain the field side
        # 0 = right, 1 = left
        self.field = 0
        # Contain the robot state
        self.state = 0
        
        # Way point publisher
        self.publisher_point = self.create_publisher(
            Float32MultiArray, 'way_point', 10)
        
        # Position subscriber
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
        self.but = msg.data;


    def timer_callback(self):
        
        way_point = Float32MultiArray()
        
        # wait for lunch button
        if self.but == [0,1,1] and self.state == 0:
            self.state = 1
            self.field = 0
        elif self.but == [1,0,1] and self.state == 0:
            self.state = 1
            self.field = 1
        
        if self.state == 0:
            way_point = self.pos
        elif self.state == 1:
            if self.field == 0:
                # right side of the field 
                pass
            elif self.field == 1:
                # left side of the field
                pass
        elif self.state == 2:
            pass
        
        self.publisher_.publish(way_point)


def main(args=None):
    rclpy.init(args=args)
    planing = Planing()
    rclpy.spin(planing)

    planing.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
