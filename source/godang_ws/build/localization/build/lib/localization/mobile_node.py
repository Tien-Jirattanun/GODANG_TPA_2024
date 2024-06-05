import sys
sys.path.append("/home/tien/Documents/GitHub/BoutToHackNASA/source/godang_ws/src/localization/localization")
from Function import PositionController
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

class MobileNode(Node):

    def __init__(self):
        super().__init__("mobile_node")
        self.publisher_vel = self.create_publisher(Float32MultiArray, "vel_data", 10)
        self.subscription_pos = self.create_subscription(Float32MultiArray, "pos_data",self.listener_pos_callback, 10)
        self.subscription_pos   
        self.subscription_state = self.create_subscription(Int32MultiArray, "state_data" ,self.listener_state_callback, 10)
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
        self.state = [0, 0, 0]
        # way point
        self.way_point = 0
        self.startX = 0
        self.startY = 0
        #========================

    def listener_state_callback(self, msg):
        self.state = msg.data

    def listener_pos_callback(self, msg):
        # before rotate
        self.pos_array = msg.data
        self.pos_x = self.pos_array[0]
        self.pos_y = self.pos_array[1]
        self.pos_z = self.pos_array[2]
        # # after rotate
        # self.pos_operate_array = self.rotate_vector([self.pos_x, self.pos_y], self.pos_z)
        # self.pos_x = self.pos_operate_array[0]
        # self.pos_y = self.pos_operate_array[1]
                
    def rotate_vector(self, vector, theta_degrees):
        theta = np.radians(theta_degrees)  # Convert degrees to radians
        rotation_matrix = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])
        return np.matmul(rotation_matrix, vector)
        
    def resetStart(self):
        self.startX = self.pos_x
        self.startY = self.pos_y

    def timer_callback(self):
        
        # print(self.state)
        
        msg = Float32MultiArray()
        msg.layout.dim.append(MultiArrayDimension(label='rows', size=3, stride=3))
        msg.layout.dim.append(MultiArrayDimension(label='columns', size=1, stride=1))

        if self.state[0] == 0:
            self.vel_array = [0.0, 0.0, 0.0]
                   
        elif self.state[0] == 1 and self.state[1] == 0 and self.state[2] == 0:
            # # waypoint 1
            # if self.way_point == 0:
            #     self.vel_array = self.pos_control.go_to_position(6, 0, 0, self.pos_x, self.pos_y, self.pos_z, self.startX, self.startY)
            #     if self.vel_array[0] < 0.01 and self.vel_array[0] > -0.01 and self.vel_array[1] < 0.01 and self.vel_array[1] > -0.01 and self.vel_array[2] < 0.01 and self.vel_array[2] > -0.01:
            #         self.way_point += 1
            #         self.resetStart()
            # # waypoint 2        
            # elif self.way_point == 1:
            #     self.vel_array = self.pos_control.go_to_position(6, 2, 0, self.pos_x, self.pos_y, self.pos_z, self.startX, self.startY)
            #     if self.vel_array[0] < 0.01 and self.vel_array[0] > -0.01 and self.vel_array[1] < 0.01 and self.vel_array[1] > -0.01 and self.vel_array[2] < 0.01 and self.vel_array[2] > -0.01:
            #         self.way_point += 1
            #         self.resetStart()
            # cwaypoint 3      
            if self.way_point == 0:
                self.vel_array = self.pos_control.rotate(90, self.pos_z)
                if self.vel_array[0] == 0.0 and self.vel_array[1] == 0.0 and self.vel_array[2] == 0.0:
                    self.way_point += 1
                    self.resetStart()   
            # waypoint 4    
            elif self.way_point == 1:
                self.vel_array = self.pos_control.go_to_position(1, 0, 90, self.pos_x, self.pos_y, self.pos_z, self.startX, self.startY)
                if self.vel_array[0] == 0.0 and self.vel_array[1] == 0.0 and self.vel_array[2] == 0.0:
                    self.way_point += 1
                    self.resetStart()   
            else:
                print(2)
                self.vel_array = [0.0, 0.0, 0.0] 
        elif self.state[0] == 1 and self.state[1] == 1 and self.state[2] == 0:
            pass
        elif self.state[0] == 1 and self.state[1] == 0 and self.state[2] == 0:
            pass
        elif self.state[0] == 1 and self.state[1] == 1 and self.state[2] == 1:
            pass
        
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
    
