import sys
sys.path.append(
    "/home/tien/Documents/GitHub/BoutToHackNASA/source/godang_ws/src/localization/localization")
from std_msgs.msg import Int32
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from rclpy.node import Node
import rclpy
import math
import numpy as np
from Function import PositionController



class MobileNode(Node):

    def __init__(self):
        super().__init__("mobile_node")
        self.publisher_vel = self.create_publisher(
            Float32MultiArray, "vel_data", 10)
        self.publisher_done = self.create_publisher(Int32, "done_data", 10)
        self.publisher_reset = self.create_publisher(Int32, "reset_data", 10)
        self.publisher_mani = self.create_publisher(Int32, "mani_com_data", 10)
        self.subscription_pos = self.create_subscription(
            Float32MultiArray, "pos_data", self.listener_pos_callback, 10)
        self.subscription_pos
        self.subscription_state = self.create_subscription(
            Int32MultiArray, "state_data", self.listener_state_callback, 10)
        self.subscription_state
        self.subscription_ball = self.create_subscription(
            Float32MultiArray, "ball_data", self.listener_ball_callback, 10)
        self.subscription_mani = self.create_subscription(
            Int32MultiArray, "mani_sensor_data", self.listener_sensor_callback, 10)
        self.subscription_mani
        timer_period = 0.01  # 100 hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.pos_control = PositionController()

        # variable zone
        # ========================

        # [vx,vy,vz]
        self.vel_array = [0.0, 0.0, 0.0]

        # [pos_x,pos_y,pos_z]
        self.pos_array = [0.0, 0.0, 0.0]
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0

        # ball_stable
        self.ball_x = 0
        self.ball_y = 0
        self.ball_z = 0
        self.ball_x_stable = 0
        self.ball_y_stable = 0
        self.ball_z_stable = 0

        # mani
        self.mani_sensor = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.mani_com = 0

        # state
        self.state = [0, 0, 0]
        # way point
        self.way_point = 0
        self.startX = 0
        self.startY = 0

        # loop control
        self.counter = 0
        # ========================

    def listener_state_callback(self, msg):
        self.state = msg.data

    def listener_sensor_callback(self, msg):
        self.mani_sensor = msg.data

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

    def listener_ball_callback(self, msg):
        self.ball_pos = msg.data
        self.ball_x = self.ball_pos[0]
        self.ball_y = self.ball_pos[1]
        self.ball_z = self.ball_pos[2]

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

        vel_msg = Float32MultiArray()
        done_msg = Int32()
        mani_msg = Int32()
        vel_msg.layout.dim.append(
            MultiArrayDimension(label='rows', size=3, stride=3))
        vel_msg.layout.dim.append(MultiArrayDimension(
            label='columns', size=1, stride=1))

        if self.state[0] == 0:
            self.vel_array = [0.0, 0.0, 0.0]
            self.way_point = 0
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
            # waypoint 3
            if self.way_point == 0:
                self.vel_array = self.pos_control.rotate(90, self.pos_z)
                if self.vel_array[0] == 0.0 and self.vel_array[1] == 0.0 and self.vel_array[2] == 0.0:
                    self.way_point += 1
                    self.resetStart()
            # waypoint 4
            elif self.way_point == 1:
                self.vel_array = self.pos_control.go_to_position(
                    1, 0, 90, self.pos_x, self.pos_y, self.pos_z, self.startX, self.startY)
                if self.vel_array[0] == 0.0 and self.vel_array[1] == 0.0 and self.vel_array[2] == 0.0:
                    self.way_point += 1
                    self.resetStart()
            else:
                print(2)
                self.vel_array = [0.0, 0.0, 0.0]
                self.way_point = 0
                done_msg.data = 2
        elif self.state[0] == 1 and self.state[1] == 1 and self.state[2] == 0:
            pass
        elif self.state[0] == 1 and self.state[1] == 0 and self.state[2] == 0:
            pass
        elif self.state[0] == 1 and self.state[1] == 1 and self.state[2] == 1:
            pass
        elif self.state[0] == 2:
            pass
        elif self.state[0] == 3:
            self.vel_array = [0.0, 0.0, 0.0]
            if self.ball_x != 0.0 or self.ball_y != 0.0 or self.ball_z != 0.0:
                self.ball_x_stable = self.ball_x
                self.ball_y_stable = self.ball_y
                self.ball_z_stable = self.ball_z
                self.way_point = 0
                done_msg.data = 4
        elif self.state[0] == 4:
            if self.way_point == 0:
                print(0)
                self.vel_array = self.pos_control.go_to_position((self.ball_x_stable - 1.0), 0, self.ball_z_stable, self.pos_x, self.pos_y, self.pos_z, self.startX, self.startY)
                # self.vel_array = self.pos_control.go_to_position(3, 0, 0, self.pos_x, self.pos_y, self.pos_z, self.startX, self.startY)
                if self.vel_array[0] == 0.0 and self.vel_array[1] == 0.0 and self.vel_array[2] == 0.0:
                    if self.counter < 100:
                        self.mani_com = 1
                    else:
                        self.resetStart()
                        self.counter = 0
                        self.way_point += 1
                    self.counter += 1
            elif self.way_point == 1:
                if (self.mani_sensor[0] + self.mani_sensor[1]) == 0:
                    self.vel_array = [0.1, 0.0,0.0]
                    self.mani_com = 2
                    self.counter = 0
                    self.way_point += 1
                else:
                    self.vel_array = [0.1, 0.0,0.0]
            elif self.way_point == 2:
                if self.counter < 100:
                    self.vel_array = [0.1, 0.0,0.0]
                elif self.counter < 250:
                    self.vel_array = [0.0, 0.0, 0.0]
                else:
                    self.vel_array = self.pos_control.go_to_position(0, 0, 0, self.pos_x, self.pos_y, self.pos_z, self.startX, self.startY)
                    if self.vel_array[0] == 0.0 and self.vel_array[1] == 0.0 and self.vel_array[2] == 0.0:
                        self.way_point += 1
                        self.resetStart()
                self.counter+=1

            else:
                self.vel_array = [0.0, 0.0, 0.0]
                done_msg.data = 5

        # sent data here

        vel_msg.data = self.vel_array
        mani_msg.data = self.mani_com
        self.publisher_vel.publish(vel_msg)
        self.publisher_done.publish(done_msg)
        self.publisher_mani.publish(mani_msg)


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
