import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import MultiArrayDimension


class StateNode(Node):

    def __init__(self):
        # Node init
        super().__init__('state_node')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'state_data', 10)
        self.subscription = self.create_subscription(Int32, 'butt_data',self.listener_callback,10)
        self.subscription
        # Timer
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Variable
        self.button = 0
        # state [state, field, retired]
        self.state = [0,0]
        
        
    def listener_callback(self, msg):
        self.button = msg.data

    def timer_callback(self):
        msg = Int32MultiArray()
        
        # on start
        if self.state[0] == 0 and self.button == 1:
            self.state = [1, 0, 0]
        elif self.state[0] == 0 and self.button == 2:
            self.state = [1, 1, 0]
        elif self.state[0] == 0 and self.button == 3:
            self.state = [1, 0, 1]
        elif self.state[0] == 0 and self.button == 4:
            self.state = [1, 1, 1]
        
        msg.data = self.state    
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    state_node = StateNode()

    rclpy.spin(state_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    state_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()