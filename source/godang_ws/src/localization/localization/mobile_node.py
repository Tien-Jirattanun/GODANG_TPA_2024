import sys
sys.path.append("/home/godang/BoutToHackNASA/source/godang_ws/src/localization/localization")
from Function import PositionController

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

class MobileNode(Node):

    def __init__(self):
        super().__init__("mobile_node")
        self.publisher_vel = self.create_publisher(Float32MultiArray, "vel_data", 10)
        self.subscription_pos = self.create_subscription(Float32MultiArray, "pos_data",self.listener_pos_callback, 10)
        self.subscription_pos   
        self.subscription_state = self.create_subscription(Int32, "state_data" ,self.listener_state_callback, 10)
        self.subscription_state   
        timer_period = 0.01  # 100 hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.pos_control = PositionController() 
        
        # variable zone
        #========================

        # [vx,vy,vz]
        self.vel_array = [0.0, 0.0, 0.0]
        
        # [pos_x,pos_y,pos_z]
        self.pos_array = [0.0, 0.0, 0.0]
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0

        # state
        self.state = 0
        # way point
        self.way_point = 0
        #========================

    def listener_state_callback(self, msg):
        self.state = msg.data

    def listener_pos_callback(self, msg):
        self.pos_array = msg.data
        self.pos_x = self.pos_array[0]
        self.pos_y = self.pos_array[1]
        self.pos_z = self.pos_array[2]

    def timer_callback(self):
        
        msg = Float32MultiArray()
        msg.layout.dim.append(MultiArrayDimension(label='rows', size=3, stride=3))
        msg.layout.dim.append(MultiArrayDimension(label='columns', size=1, stride=1))
        
        if self.state == 0:
            self.vel_array = [0.0, 0.0, 0.0]
                   
        elif self.state == 1:
            # waypoint 1
            if self.way_point == 0:
                self.vel_array = self.pos_control.go_to_position(6, 0, 0, self.pos_x, self.pos_y, self.pos_z)
                if self.vel_array[0] < 0.01 and self.vel_array[0] > -0.01 and self.vel_array[1] < 0.01 and self.vel_array[1] > -0.01 and self.vel_array[2] < 0.01 and self.vel_array[2] > -0.01:
                    self.vel_array[3] += 1.0
                    self.way_point += 1
            # waypoint 2        
            elif self.way_point == 1:
                self.vel_array[3] = 0.0
                self.vel_array = self.pos_control.go_to_position(0, -2, 0, self.pos_x, self.pos_y, self.pos_z)
                if self.vel_array[0] < 0.01 and self.vel_array[0] > -0.01 and self.vel_array[1] < 0.01 and self.vel_array[1] > -0.01 and self.vel_array[2] < 0.01 and self.vel_array[2] > -0.01:
                    self.vel_array[3] += 1.0
                    self.way_point += 1
            # waypoint 3    
            # elif self.way_point == 2:
            #     self.vel_array[3] = 0.0
            #     self.vel_array = self.pos_control.go_to_position(0.7, 0, 0, self.pos_x, self.pos_y, self.pos_z)
            #     if self.vel_array == [0.0, 0.0, 0.0, 0.0]:
            #         self.vel_array[3] += 1.0
            #         self.way_point += 1    
            else:
                self.vel_array = [0.0, 0.0, 0.0] 

        
        # sent data here
        msg.data = self.vel_array
        self.publisher_vel.publish(msg) 


def main(args=None):
    rclpy.init(args=args)

    mobile_node = MobileNode()

    rclpy.spin(mobile_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mobile_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    
