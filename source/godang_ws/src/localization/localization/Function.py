import math
from PID import PIDController
import numpy as np

class PositionController:
    def __init__(self):
        self.x, self.y, self.theta = 0, 0, 0
        self.PosX = PIDController(kP=0.1, kI=0.05, kD=0)
        self.PosY = PIDController(kP=0.1, kI=0.05, kD=0)
        self.StraightZ = PIDController(kP=0.01, kI=0.001, kD=0)
        self.RotateZ = PIDController(kP=0.05, kI=0.05, kD=0.1)
        self.vx, self.vy, self.vz = 0, 0, 0
        self.Max_speed = 0.9
        self.Min_Speed_fac = 0.15
        self.reset = 0
        self.counter = 0
        
    def position_reset(self):
        self.reset = 1

    def update_position(self, x, y, theta):
        self.x, self.y, self.theta = x, y, theta
        # self.PosX.reset()
        # self.PosY.reset()
        # self.StraightZ.reset()
        # self.RotateZ.reset()
        # self.reset = 0

    def update_velocity(self, vx, vy, vz):
        self.vx, self.vy, self.vz = vx, vy, vz


    def world2robot(self, X_w, Y_w):
        theta_rad = np.deg2rad(self.theta)
        transformation_matrix = np.array([[np.cos(theta_rad), -np.sin(theta_rad), self.x],
                                          [np.sin(theta_rad), np.cos(theta_rad), self.y],
                                          [0, 0, 1]])
        world_coords_homogeneous = np.array([X_w, Y_w, 1])  
        robot_coords_homogeneous = np.dot(np.linalg.inv(transformation_matrix), world_coords_homogeneous)
        return robot_coords_homogeneous[0], robot_coords_homogeneous[1]


    def angular_difference(self, target, current):
        difference = target - current
        if difference < -180:
            difference += 360
        if difference > 180:
            difference -= 360
        return difference
    
    def mapf(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def clamp_speed(self, speed, max_speed):
        return max(min(speed, max_speed), -max_speed)
    
    def rotate_vector(self, vector, theta_degrees):
        theta = float(np.radians(theta_degrees))  # Convert degrees to radians
        rotation_matrix = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])
        return np.matmul(rotation_matrix, vector)
    
    def go_to_world_position(self, target_x, target_y,target_yaw, offset_distance = 1):
        if self.reset == 1:
            pass
        else:
            self.counter += 1
            max_local_vel = min(self.counter * 0.05, self.Max_speed)
            error_x, error_y = self.world2robot(target_x, target_y)
            error_magnitude = math.sqrt(error_x**2 + error_y**2) - offset_distance
            error_direction = math.atan2(error_y, error_x)

            error_z = -error_direction #(self.angular_difference(target_yaw, self.theta))
            error_z = self.angular_difference(target_yaw, self.theta)
            vz = self.clamp_speed(self.RotateZ.update(error_z), self.Max_speed) 
            # vz = 0.
            vx = min(max_local_vel, error_magnitude) * math.cos(error_direction)
            vy = min(max_local_vel, error_magnitude) * math.sin(error_direction)
            
            if self.counter >= 20 and abs(vx) <= 0.05 and abs(vy) <= 0.05 and abs(vz) <= 1:
                return [0.0,0.0,0.0]
            
            print("error_x", error_x)
            print("error_y", error_y)
            print("current pos", self.x, self.y, self.theta)
            
            
            return [vx, vy, vz]        
        

    def go_to_position(self, target_x, target_y, target_z, pos_x, pos_y, pos_z, start_x, start_y):        
        total_distance = math.sqrt((target_x - start_x)**2 + (target_y - start_y)**2)
        accel_distance = total_distance / 5   
        decel_distance = total_distance / 5
        const_distance = total_distance - accel_distance - decel_distance    
        
        if self.reset == 1:
            pass
        else:           
            current_x, current_y, current_z = pos_x, pos_y, pos_z
            error_x = target_x - current_x
            error_y = target_y - current_y
            error_z = self.angular_difference(target_z, current_z)
            current_distance = math.sqrt((current_x - start_x)**2 + (current_y - start_y)**2)
            
            if current_distance < accel_distance:
                velocity_scale = self.mapf(current_distance, 0, accel_distance, self.Min_Speed_fac, 1)
                # print(1)
            elif current_distance < accel_distance + const_distance:
                velocity_scale = 1
                # print(2)
            elif current_distance < total_distance:
                # velocity_scale = self.mapf(current_distance - (accel_distance + const_distance), 0, decel_distance, 1, self.Min_Speed_fac)
                velocity_scale = self.mapf(total_distance - current_distance, 0, decel_distance, self.Min_Speed_fac, 1)
                # print(3)
            else:
                velocity_scale = self.Min_Speed_fac
                # print(4)
            
            velocity_scale = self.clamp_speed(velocity_scale,self.Max_speed)
            
            # print(velocity_scale)
            
            error_magnitude = math.sqrt(error_x**2 + error_y**2)
            error_direction = math.atan2(error_y, error_x)
            
            base_vx = velocity_scale * math.cos(error_direction)
            base_vy = velocity_scale * math.sin(error_direction)
            
            vx = self.clamp_speed(base_vx, self.Max_speed)
            vy = self.clamp_speed(base_vy, self.Max_speed)
            vz = self.clamp_speed(self.StraightZ.update(error_z), self.Max_speed)

            # if abs(target_x - start_x) < 0.05:
            #     vx = 0.0
            
            # if abs(target_y - start_y) < 0.05:
            #     vy = 0.0

            vx, vy = self.rotate_vector([vx, vy], -pos_z)

            error_x, error_y = self.rotate_vector([error_x,error_y], -pos_z)
            
            # print(vx,vy,vz)

            if abs(error_x) <= 0.05 and abs(error_y) <= 0.05 and abs(error_z) <= 3:
                return [0.0,0.0,0.0]
            else:
                return [vx,vy,vz]
        
    def rotate(self, target_z, pos_z):
        if self.reset == 1:
            pass
        else:
            current_z = pos_z
            error_z = self.angular_difference(target_z, current_z)
            vz = self.clamp_speed(self.RotateZ.update(error_z), self.Max_speed)
        if abs(error_z) <= 3:
            return [0.0,0.0,0.0]
        else:
            return [0.0,0.0,vz]
    
    def speed(self,vx,vy,target_z,pos_z):
        current_z = pos_z
        error_z = self.angular_difference(target_z, current_z)
        vz = self.clamp_speed(self.RotateZ.update(error_z), self.Max_speed)
        return [vx,vy,vz]
    
        