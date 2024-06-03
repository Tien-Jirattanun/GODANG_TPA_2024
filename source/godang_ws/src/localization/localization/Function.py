import math
from PID import PIDController

class PositionController:
    def __init__(self):
        self.x, self.y, self.theta = 0, 0, 0
        self.PosX = PIDController(kP=0.1, kI=0.05, kD=0)
        self.PosY = PIDController(kP=0.1, kI=0.05, kD=0)
        self.PosZ = PIDController(kP=0.01, kI=0.001, kD=0)
        self.vx, self.vy, self.vz = 0, 0, 0
        self.Max_speed = 0.9
        self.Min_Speed_fac = 0.1
        self.reset = 0;
        
    def position_reset(self):
        self.reset = 1;

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

    def go_to_position(self, target_x, target_y, target_z, pos_x, pos_y, pos_z):
        
        total_distance = math.sqrt(target_x**2 + target_y**2)
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
            current_distance = math.sqrt((current_x)**2 + (current_y)**2)
            
            if current_distance < accel_distance:
                velocity_scale = self.mapf(current_distance, 0, accel_distance, self.Min_Speed_fac, 1)
                # print(1)
            elif current_distance < accel_distance + const_distance:
                velocity_scale = 1
                # print(2)
            elif current_distance < total_distance:
                velocity_scale = self.mapf(current_distance - (accel_distance + const_distance), 0, decel_distance, 1, self.Min_Speed_fac)
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
            
            # vx_pid = self.PosX.update(base_vx)
            # vy_pid = self.PosY.update(base_vy)
            
            # vx = self.clamp_speed(velocity_scale * vx_pid, self.Max_speed)
            # vy = self.clamp_speed(velocity_scale * vy_pid, self.Max_speed)

            # vx = self.clamp_speed(self.PosX.update(round(velocvx = self.clamp_speed(vx_pid, self.Max_speed)ity_scale * math.cos(error_direction),1)), self.Max_speed)
            # vy = self.clamp_speed(self.PosY.update(round(velocity_scale * math.sin(error_direction),1)), self.Max_speed)
            vx = self.clamp_speed(base_vx, self.Max_speed)
            vy = self.clamp_speed(base_vy, self.Max_speed)
            vz = self.clamp_speed(self.PosZ.update(error_z), self.Max_speed)
            
            # print(vx,vy,vz)

            if abs(error_x) <= 0.05 and abs(error_y) <= 0.05 and abs(error_z) <= 3:
                return [0.0,0.0,0.0]
            else:
                return [vx,vy,vz]